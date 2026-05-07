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
#include<stdio.h> 
#include<fcntl.h>

// Nanopb related headers
#include "robosar.pb.h"
#include <pb_encode.h>

// AprilTag related headers
#include "apriltag/common/getopt.h"
#include "apriltag/common/image_u8.h"
#include "apriltag/common/pjpeg.h"
#include "apriltag/common/zarray.h"
#ifdef __linux__
    #include <unistd.h>
#endif

#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
#include "apriltag/apriltag_pose.h"

static knet_dev_t * dsPic;
static int quitReq = 0; // quit variable for loop
int feedback_frequency = 10;
// Camera image dimensions
#define IMG_WIDTH 752 // max width
#define IMG_HEIGHT 480  // max height
#define FOV_HORIZONTAL 131
#define FOCAL_LENGTH 2.1 //mm
#define TAG_SIZE 0.128 //m
#define PI 3.1417
#define SENSOR_WIDTH 2.88 //mm
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

int start_camera(unsigned int dWidth, unsigned int dHeight) {
    // start camera and stream
    // Initialize camera
    int ret;
    if ((ret=kb_camera_init(&dWidth, &dHeight))<0){
        fprintf(stderr,"camera init error %d\r\n",ret);
        return -1;
    }else {
        switch(ret) {
            case 1:
                printf("width adjusted to %d\r\n",dWidth);
                break;
            case 2:
                printf("height adjusted to %d\r\n",dHeight);
                break;
            case 3:
                printf("width adjusted to %d and height adjusted to %d !\r\n",dWidth,dHeight);
                break;
            default:
                break;
        }
    }
    // Start stream
    if(kb_captureStart()<0){
        kb_camera_release();
        fprintf(stderr,"ERROR: capture start error in mutli frames!\r\n");
        return -3;
    }
    // 100ms startup
    usleep(100000);
    printf("Successfully started camera and stream\n");
    return 0;
}
int stop_camera(){
    // stops camera and stream
    // Stop video stream
    if (kb_captureStop()<0){
        fprintf(stderr,"ERROR: capture stop error in mutli frames!\r\n");
    }
    // releasing camera
    kb_camera_release();
    printf("Shut down camera and stream\n");
    return 0;
}
void getImg(unsigned char* buffer){
    int ret;
    // Get frame
    if ((ret=kb_frameRead(buffer))<0){
        fprintf(stderr,"ERROR: frame capture error %d!\r\n",ret);
    }
    return;
}

bool rgb_2_gray_scale(unsigned char* original, unsigned char* result) {

    if(into_greyscale(original)!=0) {
        printf("Failed to convert to greyscale!");
        return false;
    }

    for(int i=0; i<IMG_WIDTH*IMG_HEIGHT*3;i+=3) {
        result[(int)(i/3)]= original[i];
    }

    return true;
}   



bool processImageFrame(unsigned char* buffer, apriltag_detector_t *td, int fifo_client) {
    bool result = false;
    image_u8_t im = { .width = IMG_WIDTH, .height = IMG_HEIGHT, .stride = IMG_WIDTH, .buf = buffer };

    zarray_t *detections = apriltag_detector_detect(td, &im);
   
    int i;
    // Create protobuf message
    robosar_fms_AllDetections proto_detections;
    proto_detections.tag_detections_count = zarray_size(detections);
    for (i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        // printf("detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
        //                 i, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);
        // Do stuff with detections here.
        robosar_fms_AprilTagDetection detection;
        detection.tag_id =  det->id;

        double fx = (FOCAL_LENGTH / SENSOR_WIDTH) * IMG_WIDTH;
        double fy = (FOCAL_LENGTH / SENSOR_WIDTH) * IMG_WIDTH;

        apriltag_detection_info_t info;
        apriltag_pose_t pose;
        info.det = det;
        info.tagsize = TAG_SIZE;
        info.fx = fx;
        info.fy = fy;
        info.cx = 0;
        info.cy = 0;
        // TODO Handle error return from estimate tag pose
        double err = estimate_tag_pose(&info, &pose);
        // printf("Pose: Rotation matrix size: %3d X %3d Translation matrix size: %3d X %3d \n \
        //                                             Rotation matrix: %lf %lf %lf \n %lf %lf %lf \n %lf %lf %lf \n \
        //                                             Translation matrix: %lf %lf %lf \n",
        //                                             pose.R->nrows, pose.R->ncols, pose.t->nrows, 
        //                                             pose.t->ncols,pose.R->data[0], pose.R->data[1], 
        //                                             pose.R->data[2], pose.R->data[3], pose.R->data[4],
        //                                                 pose.R->data[5], pose.R->data[6], pose.R->data[7], 
        //                                                 pose.R->data[8], pose.t->data[0], pose.t->data[1], pose.t->data[2]);
        
        detection.pose.R.r11 = pose.R->data[0];
        detection.pose.R.r12 = pose.R->data[1];
        detection.pose.R.r13 = pose.R->data[2];
        detection.pose.R.r21 = pose.R->data[3];
        detection.pose.R.r22 = pose.R->data[4];
        detection.pose.R.r23 = pose.R->data[5];
        detection.pose.R.r31 = pose.R->data[6];
        detection.pose.R.r32 = pose.R->data[7];
        detection.pose.R.r33 = pose.R->data[8];
        detection.pose.t.x = pose.t->data[0];
        detection.pose.t.y = pose.t->data[1];
        detection.pose.t.z = pose.t->data[2];

        // Transfer this detection to proto message
        proto_detections.tag_detections[i] = detection;
        result = true;
    }

    if(result) {
        // Send detections to pipe
        uint8_t proto_buffer[25000];
        pb_ostream_t stream = pb_ostream_from_buffer(proto_buffer, sizeof(proto_buffer));
        bool status = pb_encode(&stream, robosar_fms_AllDetections_fields, &proto_detections);
        size_t proto_msg_length = stream.bytes_written;

        if (!status)
        {
            printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        }
        else 
        {
            //printf("Sending... %ld\n",proto_msg_length);
            //printf("Send completed.\n");
            write(fifo_client, proto_buffer,proto_msg_length+1);
        }
    }
    
    return result;
}


/*----------------Main Program-----------------*/
#define FOR_SPD 1000
#define SPIN_SPD 150
#define FOR_DEV_SPD 850

int main(int argc, char *argv[]) {
	long int main_loop_delay = 10000;

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

  	// This is for the ctrl-C handler
  	signal( SIGINT , ctrlc_handler );
    // To handle PKILL
    signal( SIGTERM , pkill_handler );

  	// Setting the term mode to 1 will return the pressed key immediately!
  	kb_change_term_mode(1);

    // Variables for time stamps
    struct timeval cur_time, old_time;
    long long elapsed_time_us;

    // Get the starting time stamp
    gettimeofday(&cur_time,0x0);
    old_time = cur_time;

    // Start camera
    unsigned char img_buffer[IMG_WIDTH*IMG_HEIGHT*3*sizeof(char)] = {0};
    unsigned char img_buffer_gray_scale[IMG_WIDTH*IMG_HEIGHT*sizeof(char)] = {0};
    start_camera(IMG_WIDTH, IMG_HEIGHT);

    // Set up Apriltag
    // image_u8_t* im = image_u8_create_from_pnm("original.jpg");
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_add_family(td, tf);

    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;
    td->nthreads = 1;
    td->debug = false;
    td->refine_edges = true;

     // Open IPC Pipe
    char * myfifo = "/tmp/myfifo";
    mkfifo(myfifo, 0666);
    // open for write only
    int fifo_client = open(myfifo, O_WRONLY | O_NONBLOCK);

    while(quitReq == 0) {
        
		// Update time
		gettimeofday(&cur_time,0x0);
		elapsed_time_us = timeval_diff(NULL, &cur_time, &old_time);


		if(elapsed_time_us > main_loop_delay){
            old_time = cur_time;
            

            // Get camera frame
            getImg(img_buffer);
            if(rgb_2_gray_scale(img_buffer, img_buffer_gray_scale)) {
                processImageFrame(img_buffer_gray_scale, td, fifo_client);
            }
            else {
                printf("Error in converting RGB to gray scale\n");
            }
            // saving image
            // int ret;
            // if ((ret=save_buffer_to_jpg("original.jpg",100,img_buffer_gray_scale))<0)
            // {
            //     fprintf(stderr,"save image error %d\r\n",ret);
            //     kb_camera_release();
            //     return -4;
            // }

    		//printf("Sleeping...\n");

            // Display battery status
            display_battery_status(dsPic);
		  }
  	}	

  	// switch to normal key input mode
  	// This is important, if we don't switch the term mode back to zero
  	// It will still return the pressed key immediately
  	// even at the root@r1:~/tests#
  	// resulting in no characters showing up even if you press any keys on keyboard
  	kb_change_term_mode(0);

  	// set to regular idle mode!
  	kh4_SetMode(kh4RegIdle, dsPic);

    // // Cleanup.
    tag36h11_destroy(tf);
    apriltag_detector_destroy(td);

    // close IPC pipe
    close(fifo_client);

 	return 0;  
}