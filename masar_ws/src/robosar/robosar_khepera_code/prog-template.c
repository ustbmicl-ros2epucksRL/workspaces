#include <khepera/khepera.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <arpa/inet.h>
#include <math.h>
#include <ifaddrs.h>
#include "robosar.pb.h"
#include <pb_encode.h>


/** Declaring parameters as global variables
 * 
 * Run this file as ./binary [SERVER_IP] [CONTROL_PORT] [FEEDBACK_PORT] [FEEDBACK_FREQUENCY_HZ] [CONTROL_TIMEOUT_MS]
*/
#define NUM_PARAMETERS 5
#define TRUE 1
#define FALSE 0
#define epsilon 1e-7
int feedback_port;
int control_port;
int feedback_frequency;
long long int control_timeout;
char* server_ip;
char status_str[200];


#define MAX_WHEEL_SPEED_MM_S 810
#define MAXLINE 1024 
#define KH4_GYRO_DEG_S   (66.0/1000.0)
#define LRF_DEVICE "/dev/ttyACM0" 
// Thresholds for avoiding collisions
#define obstacleThreshold 500
#define obstacleThresholdOblique 500
#define obstacleNumThreshold 1

static knet_dev_t * dsPic;
static int quitReq = 0; // quit variable for loop

//Velocity timeout related variables
int timer_started = FALSE;

struct velo_cmd_s {
	double W;
	double V;
} velo_cmd = {0.0,0.0};
double override_flag = 0.0;

/*--------------------------------------------------------------------*/
/* Make sure the program terminate properly on a ctrl-c */
static void ctrlc_handler( int sig ) 
{
  quitReq = 1;
}

static void pkill_handler( int sig ) 
{
  quitReq = 1;
}

/*------------------- Time Value Difference -----------*/
/* Compute time difference

 * \param difference difference between the two times, in structure timeval type
 * \param end_time end time
 * \param start_time start time
 *
 * \return difference between the two times in [us] */
long long timeval_diff(struct timeval *difference, struct timeval *end_time, struct timeval *start_time)
{
	// timeval is a time structure that is commonly used in low level c
	struct timeval temp_diff;

	if(difference == NULL) {
		difference =& temp_diff;
	}

	difference -> tv_sec  = end_time -> tv_sec  - start_time -> tv_sec ;
	difference -> tv_usec = end_time -> tv_usec - start_time -> tv_usec;

	/* Using while instead of if below makes the code slightly more robust. */

	while(difference -> tv_usec < 0) {
		difference -> tv_usec += 1000000;
    	difference -> tv_sec  -= 1;
	}

	return 1000000LL * difference -> tv_sec + difference -> tv_usec;
}



/*--------velocity to pulse-------*/
int v2p(double v) {
	return (int)v / 0.678181;
}

int getSign(double x) {

	return x<0 ? -1 : 1;
}


/*---------Angular and linear velocity control of the robot----------*/
/** Ang_Vel_Control
 * @brief : Convert bot centre velocities from mm/s to wheel velocities in encoder tick/s

 * @param : double ang - rad/s
 * @param : double vel - mm/s


int kh4_set_speed 	( 	int  	left,
		int  	right,
		knet_dev_t *  	hDev 
	) 		
Parameters
    left	left motor speed (units: encoder)
    right	right motor speed (units: encoder)
    hDev	is a handle to an openned knet socket (Khepera4:dsPic).
*/
void Ang_Vel_Control(double ang, double vel) {
	ang = -ang;

	double wheel_base = 105.4;
	double left_wheel_speed = (2*vel + wheel_base*ang ) / 2;
	double right_wheel_speed = (2*vel - wheel_base*ang) / 2;

	// put limits
	left_wheel_speed = fabs(left_wheel_speed) > MAX_WHEEL_SPEED_MM_S ? MAX_WHEEL_SPEED_MM_S*getSign(left_wheel_speed) : left_wheel_speed;
	right_wheel_speed = fabs(right_wheel_speed) > MAX_WHEEL_SPEED_MM_S ? MAX_WHEEL_SPEED_MM_S*getSign(right_wheel_speed) : right_wheel_speed;


	int PL = v2p(left_wheel_speed);
	int PR = v2p(right_wheel_speed);
	//printf("\nL encoder input: %d", PL);
	//printf("\nR encoder input: %d", PR);
	//printf("\n");
	kh4_set_speed(PL, PR, dsPic);
}

float accel_convert(char byte_high, char byte_low){
    // Converts 2's compliment from Khepera IMU accel to m/s^2
    // Data is 12 bits, split over 16 bits for +/- 2g range
    // Byte high all has data
    // Byte low has 4 lowest bits set to 0
    int32_t val = -(byte_high & (0x80));
    val += byte_high & 0x7F;
    val <<= 8;
    val += byte_low;
    val >>= 4;
    float acceleration = (float)val * 2.0/(2048); // in g's
    acceleration *= 9.8066; // convert to m/s^2
    return acceleration;
}

/*-----------Get Acceleration----------*/
void getAcc(char * acc_Buffer, double * acc_X, double * acc_Y, double * acc_Z) {
	kh4_measure_acc((char *)acc_Buffer, dsPic);

	double dmean = 0;
	double dval = 0;
	int i;
    char byte_high, byte_low;
    int32_t accel_bytes;

	// Acceleration on X axis
	//printf("\nAcceleration sensor on X axis: ");
	for (i = 0; i < 10; i++) {
        dval=accel_convert(acc_Buffer[i * 2 + 1], acc_Buffer[i * 2]);
        dmean += dval;
	}

	*acc_X = dmean / 10.0;
	//printf(" %5.2f", *acc_X);

	// Acceleration on Y axis
	//printf("\nAcceleration sensor on Y axis: ");

	dmean = 0;

	for (i = 10; i < 20; i++) {
		dval=accel_convert(acc_Buffer[i * 2 + 1], acc_Buffer[i * 2]);
		dmean += dval;
	}

	*acc_Y = dmean / 10.0;
	//printf(" %5.2f", *acc_Y);

	// Acceleration on Z axis
	//printf("\nAcceleration sensor on Z axis: ");

	dmean = 0;

	for (i = 20; i < 30; i++) {
        dval=accel_convert(acc_Buffer[i * 2 + 1], acc_Buffer[i * 2]);
        dmean += dval;
	}

	*acc_Z = dmean / 10.0;
	//printf(" %5.2f", *acc_Z);
	//printf("\n");
}

/*---------------Get Ultrasonic Sensor Readings--------------*/
void getUS(char * us_Buffer, short * usValues) {
	kh4_measure_us((char *)us_Buffer, dsPic);
	int i;
	for (i = 0; i < 5; i++) {
		*(usValues + i) = (short)(us_Buffer[i * 2] | us_Buffer[i * 2 + 1] << 8);
		//printf("\nUltrasonic sensor %d: %d", i + 1, *(usValues + i));
	}
	//printf("\n");
}

/*---------------Get Infrared Sensor Readings--------------*/
void getIR(char * ir_Buffer, int * irValues) {
	kh4_proximity_ir((char *)ir_Buffer, dsPic);
	int i;
	for(i = 0; i < 12; i++) {
		*(irValues + i) = (ir_Buffer[i * 2] | ir_Buffer[i * 2 + 1] << 8);
		//printf("\nInfrared sensor %d: %d", i + 1, *(irValues + i));
	}
	//printf("\n");
}

/*------------------- Get gyroscope readings -------------------*/
void getGyro(char * gyro_Buffer, double * gyro_X, double * gyro_Y, double * gyro_Z) {
	kh4_measure_gyro((char *)gyro_Buffer, dsPic);

	int i;
	double dmean = 0;
	double dval;
	// Angular rate in X axis
	//printf("\nGyro on X axis: ");
	for (i = 0; i < 10; i++) {
		dval = ((short)(gyro_Buffer[i * 2] | gyro_Buffer[ i * 2 + 1] << 8));
		dmean += dval;
	}
	*gyro_X = dmean * KH4_GYRO_DEG_S / 10.0; // KH4_GYRO_DEG_S converts the reading value to deg/s
	//printf(" %5.2f deg/s", *gyro_X);

	// Angular rate on Y axis
	//printf("\nGyro on Y axis: ");
	dmean = 0;
	for (i = 10; i < 20; i++) {
		dval = ((short)(gyro_Buffer[i * 2] | gyro_Buffer[ i * 2 + 1] << 8));
		dmean += dval;
	}
	*gyro_Y = dmean * KH4_GYRO_DEG_S / 10.0; // KH4_GYRO_DEG_S convertsthe reading value to deg/s
	//printf(" %5.2f deg/s", *gyro_Y);

	// Angular rate on Z axis
	//printf("\nGyro on Z axis: ");
	dmean = 0;
	for (i = 20; i < 30; i++) {
		dval = ((short)(gyro_Buffer[i * 2] | gyro_Buffer[ i * 2 + 1] << 8));
		dmean += dval;
	}
	*gyro_Z = dmean * KH4_GYRO_DEG_S / 10.0; // KH4_GYRO_DEG_S convertsthe reading value to deg/s
	//printf(" %5.2f deg/s", *gyro_Z);

	//printf("\n");
}

/*------------------- Get encoder readings -------------------*/
void getEC(unsigned int * posL, unsigned int * posR) {
	kh4_get_position(posL, posR, dsPic);
	//printf("\nEncoder left: %d", *posL);
	//printf("\nEncoder right: %d", *posR);
	//printf("\n");
}

/*------------------- Get encoder speed readings -------------------*/
void getSPD(unsigned int * spdL, unsigned int * spdR) {
	kh4_get_speed(spdL, spdR, dsPic);
	//printf("\nEncoder rotation speed left: %d", *spdL);
	//printf("\nEncoder rotation speed right: %d", *spdR);
	//printf("\n");
}

/*-----------Get LRF readings----------*/
void getLRF(int LRF_DeviceHandle, long * LRF_Buffer) {
    // Get distance measurements
    int result = kb_lrf_GetDistances(LRF_DeviceHandle);
    if(result < 0){
        // Failure
        printf("\nERROR: Could not read LRF!\n");
        return;
    }
    // Copy data from global to local buffer
    memcpy(LRF_Buffer, kb_lrf_DistanceData, sizeof(long)*LRF_DATA_NB);
}



/*-------------------Establish UDP socket communication as client-------------------*/
void UDP_Client(int * sockfd, struct sockaddr_in * servaddr, struct sockaddr_in * clientaddr) {    

	// For getting own (Khepera) IP address
	/*
	struct ifaddrs *id;
	int val;
	val = getifaddrs(&id);
	id->ifa_addr
    */

    // Creating socket file descriptor 
    if ( (*sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 

    // Clear servaddr just in case
    memset(servaddr, 0, sizeof(*servaddr)); 
    
	// Convert IPv4 and IPv6 addresses from text to binary form 
	// Give the client the server's address to send to
    //if(inet_pton(AF_INET, "192.168.1.142", &(*servaddr).sin_addr)<=0)  
    if(inet_pton(AF_INET, server_ip, &(*servaddr).sin_addr)<=0)  
    { 
        printf("\nInvalid address/ Address not supported \n"); 
        return; 
    } 

    // Set a timeout time for the UDP socket when receiving
  	// timeval is a common structure for time when dealing with low level c
  	// it stores the time in both seconds and microseconds
  	/*
  	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 50000; // 50 ms
    if (setsockopt(*sockfd, SOL_SOCKET, SO_RCVTIMEO,&tv,sizeof(tv)) < 0) {
    	perror("Error");
	}
	*/

    // Filling server information 
    servaddr -> sin_family = AF_INET; 
    servaddr -> sin_port = htons(feedback_port); 

    memset(clientaddr, 0, sizeof(*clientaddr));
    clientaddr -> sin_family = AF_INET;
    (clientaddr -> sin_addr).s_addr = htonl(INADDR_ANY);
    clientaddr -> sin_port = htons(control_port);

    if (bind(*sockfd, (struct sockaddr *) clientaddr, sizeof(*clientaddr)) < 0) {
        perror("bind");
        exit(1);
    }
}


/*------------Sending sensor values to UDP server in one big string-------------*/
void UDPsendSensor(int UDP_sockfd, struct sockaddr_in servaddr, long double T, double acc_X, double acc_Y, double acc_Z, double gyro_X, double gyro_Y, double gyro_Z, unsigned int posL, unsigned int posR, unsigned int spdL, unsigned int spdR, short usValues[], int irValues[], long LRFValues[]) {
	char text[25000];
	uint8_t proto_buffer[25000];
	static unsigned long int seq_id = 0;

	// Separate sensor readings with "tags"
	// EX: "-----AY2.5AY-------"
	// The python server can do: AY = data.split('AY')[1]
	// Which splits the data into [-----, 2.5, -------]
	// then it gets the second index, [1], which is 2.5

	/** Create empty protobuf messages */
	robosar_fms_SensorData proto_data_all;	

	// Time stamp
	sprintf(text, "T");
	sprintf(text + strlen(text), "%2.4f", T);
	sprintf(text + strlen(text), "T\n");
	proto_data_all.timestamp_ns = 0; // TODO @indraneel later

	// seq id
	proto_data_all.seq_id = seq_id;
	seq_id++;

	// Accelerometer
	robosar_fms_Accelerometer proto_accel_data;
	proto_accel_data.acc_x = velo_cmd.V; // send received lin vel cmd
	proto_accel_data.acc_y = velo_cmd.W; // send received ang vel cmd
	proto_accel_data.acc_z = override_flag;
	proto_data_all.accel_data = proto_accel_data;

	// Gyroscope
	robosar_fms_Gyroscope proto_gyro_data;
	proto_gyro_data.gyro_x = gyro_X;
	proto_gyro_data.gyro_y = gyro_Y;
	proto_gyro_data.gyro_z = gyro_Z;
	proto_data_all.gyro_data = proto_gyro_data;

	// Encoders
	robosar_fms_Encoder_count proto_enc_count_data;
	proto_enc_count_data.left = posL;
	proto_enc_count_data.right = posR;
	proto_data_all.count_data = proto_enc_count_data;

	robosar_fms_Encoder_speed proto_enc_speed_data;
	proto_enc_speed_data.left = spdL;
	proto_enc_speed_data.right = spdR;
	proto_data_all.speed_data = proto_enc_speed_data;

	// Ultrasonic sensor
	robosar_fms_Ultrasonic proto_us_data;
	proto_us_data.sensor_a = usValues[0];
    proto_us_data.sensor_b = usValues[1];
    proto_us_data.sensor_c = usValues[2];
    proto_us_data.sensor_d = usValues[3];
    proto_us_data.sensor_e = usValues[4];
	proto_data_all.us_data = proto_us_data;

	// Infrared sensor
	robosar_fms_Infrared proto_ir_data;
	proto_ir_data.sensor_a = irValues[0];
    proto_ir_data.sensor_b = irValues[1];
    proto_ir_data.sensor_c = irValues[2];
    proto_ir_data.sensor_d = irValues[3];
    proto_ir_data.sensor_e = irValues[4];
    proto_ir_data.sensor_f = irValues[5];
    proto_ir_data.sensor_g = irValues[6];
    proto_ir_data.sensor_h = irValues[7];
    proto_ir_data.sensor_i = irValues[8];
    proto_ir_data.sensor_j = irValues[9];
    proto_ir_data.sensor_k = irValues[10];
    proto_ir_data.sensor_l = irValues[11];
	proto_data_all.ir_data = proto_ir_data;

	 // LRF sensor
	robosar_fms_LaserScanner proto_lrf_data;
    int i;
    for(i=0;i<LRF_DATA_NB;i++){
		proto_lrf_data.values[i] = LRFValues[i];
    }
	proto_lrf_data.values_count = LRF_DATA_NB;
	proto_data_all.lrf_data = proto_lrf_data;

	pb_ostream_t stream = pb_ostream_from_buffer(proto_buffer, sizeof(proto_buffer));
	bool status = pb_encode(&stream, robosar_fms_SensorData_fields, &proto_data_all);
	size_t proto_msg_length = stream.bytes_written;


	// Have char pointer p point to the whole text, send it to the client
	char *p = text;
	int len = strlen(p);

	// Send the big chunk of sensor data string to server
	/* Check for any protobuf encoding errors */
	if (!status)
	{
		printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
	}
	else 
	{
		//printf("Sending... %ld\n",proto_msg_length);
		sendto(UDP_sockfd, proto_buffer, proto_msg_length, MSG_CONFIRM, (const struct sockaddr *) &servaddr, sizeof(servaddr)); 
		//printf("Send completed.\n");
	}

}

/**
 * 	Helper function for double comparison
 * 
  */
bool is_velocity_non_zero(struct velo_cmd_s cur_cmd)
{
	bool result = true;
	if( (cur_cmd.V < 0.0 + epsilon && cur_cmd.V > 0.0-epsilon) &&
		(cur_cmd.W < 0.0 + epsilon && cur_cmd.W > 0.0-epsilon))
		{
			result = false;
		}
	return result;
}

/*---------------- Receiving and parsing from sever -----------------*/

struct timeval UDPrecvParseFromServer(int UDP_sockfd, struct sockaddr_in servaddr) {
	char sock_buffer[1024];
	char *pch;
	double recv[2];
	int i = 0;
	int n, len;
	static struct timeval start_v;
	static struct timeval end_v;
	static struct timeval elapsed_time;

	// Receive data string from server 
	n = recvfrom(UDP_sockfd, (char *)sock_buffer, MAXLINE, MSG_DONTWAIT, (struct sockaddr *) &servaddr, &len); 

	// Parsing the string
	// The angular velocity (W) and linear velocity (V) are sent in the same string, separated by an 'x'
	if(n>0)
	{
		pch = strtok (sock_buffer,"x");
		while (pch != NULL)
		{
			recv[i] = atof(pch);
			i++;
			pch = strtok (NULL, "x");
		}
        // Update commands and flag
		velo_cmd.W = recv[0];
		velo_cmd.V = recv[1];
        override_flag = 0.0;
        sprintf(status_str, "No override;\n");
			

		// Clear buffer
		memset(sock_buffer, 0, sizeof sock_buffer);

		// Reset time
		elapsed_time.tv_sec = 0;
		elapsed_time.tv_usec = 0;
		timer_started = FALSE;
	}
	else
	{
        // Comment out safety feature
		if(timer_started == FALSE && is_velocity_non_zero(velo_cmd))
		{
			gettimeofday(&start_v,NULL);
			timer_started = TRUE;
		}
		else if(timer_started == TRUE)
		{
			gettimeofday(&end_v,NULL);
			elapsed_time.tv_usec = timeval_diff(NULL,&end_v,&start_v);
		}
		else
		{
			// Robot is idle do nothing
			elapsed_time.tv_sec = 0;
			elapsed_time.tv_usec = 0;
		}
	}
	return elapsed_time;
}

void display_battery_status(knet_dev_t *hDev){
    char bat_buffer[100];
    kh4_battery_status(bat_buffer,dsPic);
    int battery_charge = bat_buffer[3];
    if(battery_charge > 75){
        // Green
        kh4_SetRGBLeds(
            0x00, 0x08, 0x00,
            0x00, 0x08, 0x00,
            0x00, 0x08, 0x00, hDev);
    }else if(battery_charge > 50){
        // Yellow
        kh4_SetRGBLeds(
            0x08, 0x08, 0x00,
            0x08, 0x08, 0x00,
            0x08, 0x08, 0x00, hDev);
    }else if(battery_charge > 25){
        // Orange
        kh4_SetRGBLeds(
            0x14, 0x04, 0x00,
            0x14, 0x04, 0x00,
            0x14, 0x04, 0x00, hDev);
    }else{
        // Red
        kh4_SetRGBLeds(
            0x20, 0x00, 0x00,
            0x20, 0x00, 0x00,
            0x20, 0x00, 0x00, hDev);
    }
}

int collision_detection(char *ir_Buffer, int *irValues, int *obstacle_found){
    // Get values of proximity sensors
    getIR(ir_Buffer, irValues);
    //checking the values of the front left, front right and the front IR sensor
    if((*(irValues + 2)>obstacleThresholdOblique || *(irValues + 4)>obstacleThresholdOblique) || *(irValues + 3)>obstacleThreshold){
        (*obstacle_found)++;
    }
    else{
        *obstacle_found = 0;
    }
    return *obstacle_found;
}



/*----------------Main Program-----------------*/
#define FOR_SPD 1000
#define SPIN_SPD 150
#define FOR_DEV_SPD 850

int main(int argc, char *argv[]) {
	int i;
	long int main_loop_delay;
	/* Check arguments */
	if(argc<NUM_PARAMETERS+1)
	{
		printf("Please enter %d arguments in the format [SERVER_IP] [CONTROL_PORT] [FEEDBACK_PORT] [FEEDBACK_FREQUENCY_HZ] [CONTROL_TIMEOUT_MS] \n",NUM_PARAMETERS);
		return 0;
	}

	/* Parse arguments */
	for(i=0;i<argc;i++)
	{
		if(i==1)
		{
			server_ip = argv[i];
		}
		else if(i==2)
		{
			control_port = strtol(argv[i],NULL,10);
		}
		else if(i==3)
		{
			feedback_port = strtol(argv[i],NULL,10);
		}
		else if(i==4)
		{
			feedback_frequency = strtol(argv[i],NULL,10);
			main_loop_delay = (int)(1e6*(1.0/(double)(feedback_frequency)));
			// Put lower limit on main loop delay
			main_loop_delay = main_loop_delay<100000 ? 100000 : main_loop_delay;
		}
		else if(i==5)
		{
			control_timeout = 1000LL*(strtol(argv[i],NULL,10));
		}
	}

	printf("[RoboSAR] Received arguments are server ip : %s control port: %d feedback port: %d\n",server_ip,control_port,feedback_port);
	printf("Main loop delay is set to %ld\n",main_loop_delay);


	/* Initial Template Setup by LinKhepera */
	int rc;

	/* Set the libkhepera debug level - Highly recommended for development. */
	kb_set_debug_level(2);

    /* Init the khepera library */
	if((rc = kb_init( argc , argv )) < 0 )
		return 1;


	/* Main Code */
  
  	// dsPIC is the microcontroller of khepera
  	// It handles all the inputs and outputs
  	dsPic  = knet_open( "Khepera4:dsPic" , KNET_BUS_I2C , 0 , NULL );

    // Blue LED for booting
    kh4_SetRGBLeds(
        0x00, 0x00, 0x08,
        0x00, 0x00, 0x08,
        0x00, 0x00, 0x08, dsPic);

  	// This is for the ctrl-C handler
  	signal( SIGINT , ctrlc_handler );
    // To handle PKILL
    signal( SIGTERM , pkill_handler );

  	// Setting the term mode to 1 will return the pressed key immediately!
  	kb_change_term_mode(1);

  	// Set to Normal Motor Control Mode
  	kh4_SetMode(kh4RegSpeed,dsPic);
  
  	// Reset Encoders
  	kh4_ResetEncoders(dsPic);

    // Get handle for Laser Rangefinder (LRF)
    int LRF_DeviceHandle;
    // Power LRF
    kb_lrf_Power_On();

    // Initialize LRF
    char LRF_device[] = LRF_DEVICE;
    char LRF_device_id;
    for(LRF_device_id = '0'; LRF_device_id <= '9'; LRF_device_id++){
        LRF_device[strlen(LRF_device)-1] = LRF_device_id;
        if ((LRF_DeviceHandle = kb_lrf_Init(LRF_device))<0){
            printf("ERRR: port %s could not initialise LRF!\n",LRF_device);
        } else{
            printf("SUCC: port %s initialised for LRF!\n",LRF_device);
            break;
        }
    }

  	// Establish socket communication
  	int new_socket;
  	int UDP_sockfd;
  	char sock_buffer[1024] = {0};
  	struct sockaddr_in     servaddr; 
  	struct sockaddr_in     clientaddr; 
  	UDP_Client(&UDP_sockfd, &servaddr, &clientaddr);

  	
    // Initialize a Buffer to store all the data collected from
    // the sensors by the dsPIC
    char acc_Buffer[100]; // Buffer for accelerometer
    char us_Buffer[100]; // Buffer for ultra-sonic sensors
    short usValues[5]; // Values of the 5 ultrasonic sensor readings from sensor No.1 - 5
    char ir_Buffer[256]; // Buffer for infrared sensors
    int irValues[12]; // Values of the 12 IR sensor readings from sensor No.1 - 12
    int obstacles_detected = 0; // number of times obstacles detected near Khepera
    char gyro_Buffer[100]; // Buffer for Gyroscope
    long LRF_Buffer[LRF_DATA_NB]; // Buffer for LIDAR readings

    double acc_X, acc_Y, acc_Z;
    double gyro_X, gyro_Y, gyro_Z;

    unsigned int posL, posR;
    unsigned int spdL, spdR;


    // Variables for time stamps
    struct timeval cur_time, old_time;
    long long elapsed_time_us;

    // Get the starting time stamp
    gettimeofday(&cur_time,0x0);
    old_time = cur_time;

    // For blinking LED
    char led_cnt = 0;

    while(quitReq == 0) {
		// Receive linear and angular velocity commands from the server
		struct timeval time_elapsed_v = UDPrecvParseFromServer(UDP_sockfd, servaddr);
		struct timeval control_timeout_s;
		control_timeout_s.tv_usec = control_timeout;
		long long int control_full;
		long long int time_elapsed_full;

		control_full = 1000000LL*control_timeout_s.tv_sec + control_timeout_s.tv_usec;
		time_elapsed_full = 1000000LL*time_elapsed_v.tv_sec + time_elapsed_v.tv_usec;
		// Check for override due to timeout
		if(timer_started==TRUE && time_elapsed_full >= control_full)
		{
			velo_cmd.V = 0.00;
			velo_cmd.W = 0.00;
            override_flag = 1.0;
			timer_started = FALSE;
			time_elapsed_full = 0;
            kh4_SetRGBLeds(
                0xFF, 0x00, 0x00,
                0xFF, 0x00, 0x00,
                0xFF, 0x00, 0x00, dsPic);
            sprintf(status_str, "Override,timeout;\n");
		}
        // Check and recheck for override due to imminent collision
        while(collision_detection(ir_Buffer, irValues, &obstacles_detected)){
            if(obstacles_detected > obstacleNumThreshold){
                velo_cmd.V = (velo_cmd.V > 0) ? 0.00 : velo_cmd.V;
                velo_cmd.W = 0.00;
                override_flag = 1.0;
                kh4_SetRGBLeds(
                    0xFF, 0x00, 0xFF,
                    0xFF, 0x00, 0xFF,
                    0xFF, 0x00, 0xFF, dsPic);
                sprintf(status_str, "Override,infrared;\n");
                break;
            }
        }
        Ang_Vel_Control(velo_cmd.W, velo_cmd.V);
		// if the velocity is non zero and last received velocity timestamp is mreo than control time out, set v = 0
		// Update time
		gettimeofday(&cur_time,0x0);
		elapsed_time_us = timeval_diff(NULL, &cur_time, &old_time);


		if(elapsed_time_us > main_loop_delay){
            led_cnt++;
            if(led_cnt > feedback_frequency){
                led_cnt = 0;
                // Turn LED off to cause blinking
                kh4_SetRGBLeds(
                    0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, dsPic);
            }
            old_time = cur_time;
            
            //----------------- All sensor readings ------------------//
    		// Receive accelerometer readings
    		// getAcc(acc_Buffer, &acc_X, &acc_Y, &acc_Z);

    		// Receive ultrasonic sensor readings
    		// getUS(us_Buffer, usValues);
    		
    		// Receive infrared sensor readings
    		// getIR(ir_Buffer, irValues);
    		
    		// Receive gyroscope readings
    		// getGyro(gyro_Buffer, &gyro_X, &gyro_Y, &gyro_Z);
    		
    		// Receive encoder readings
    		getEC(&posL, &posR);
    		
    		// Receive encoder speed readings
    		// getSPD(&spdL, &spdR);

            // Receive LRF readings if available
            if(!(LRF_DeviceHandle < 0))
                getLRF(LRF_DeviceHandle, LRF_Buffer);
            else
                memset(LRF_Buffer, 0, sizeof(long)*LRF_DATA_NB);

    		//TCPsendSensor(new_socket, T, acc_X, acc_Y, acc_Z, gyro_X, gyro_Y, gyro_Z, posL, posR, spdL, spdR, usValues, irValues);
    		UDPsendSensor(UDP_sockfd, servaddr, 0, acc_X, acc_Y, acc_Z, gyro_X, gyro_Y, gyro_Z, posL, posR, spdL, spdR, usValues, irValues, LRF_Buffer);
    		//printf("Sleeping...\n");

            // Display battery status
            display_battery_status(dsPic);

		}
  	}	


  	// Close UDP scoket
  	close(UDP_sockfd);

    // Close the lrf device
    kb_lrf_Close(LRF_DeviceHandle);

    // Red when not doing anything
    kh4_SetRGBLeds(
        0x08, 0x00, 0x00,
        0x08, 0x00, 0x00,
        0x08, 0x00, 0x00, dsPic);

  	// switch to normal key input mode
  	// This is important, if we don't switch the term mode back to zero
  	// It will still return the pressed key immediately
  	// even at the root@r1:~/tests#
  	// resulting in no characters showing up even if you press any keys on keyboard
  	kb_change_term_mode(0);

  	// stop robot
  	kh4_set_speed(0, 0, dsPic);

  	// set to regular idle mode!
  	kh4_SetMode(kh4RegIdle, dsPic);



 	return 0;  
}
