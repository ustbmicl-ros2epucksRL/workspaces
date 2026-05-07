/*--------------------------------------------------------------------
 * lrf.c - KoreBot Library - LedRangeFinder Support
 *--------------------------------------------------------------------
 * $Id: kb_lrf.c,v 1.5 2005/07/03 07:37:55 pbureau Exp $
 *--------------------------------------------------------------------	
 * $Author: pbureau $
 * $Date: 2005/07/03 07:37:55 $
 * $Revision: 1.5 $
 *--------------------------------------------------------------------*/


/*! 
 * \file   kb_lrf.c Support for KoreBot Led Range Finder.             
 *
 * \brief 
 *         This module provides useful basic facilities to use the
 *         KoreBot with the LedRangeFinder card
 *
 * \author   Arnaud Maye (K-Team SA), 2011.10.31 J. Tharin : added support for LRF Hokuyo URG-04LX-UG01
 *
 * \note     Copyright (C) 2011 K-TEAM SA
 * \bug      none discovered.                                         
 * \todo     
 */

#include "kb_lrf.h"
#include "khepera.h" 

#include <sys/types.h>
#include <sys/mman.h>

// parameters for serial transmission
enum {
  Timeout = 800, // ms
  LineLength = 16 + 64 + 1 + 1 + 1,
};


#define GPIO_POWER 16 // power switch for the LRF module

#define BAUDRATE 19200 // not used for USB devices

static int HComm=-1; // handle to the port where the sensor is connected

static char* ErrorMessage = "no error.";

   
/************************
** URG DATA            **
************************/

typedef struct {
  enum parameters_t {
    MODL = 0,		//!< Sensor Model
    DMIN,			//!< Min detection range [mm]
    DMAX,			//!< Man detection range [mm]
    ARES,			//!< Angular resolution (division of 360degree)
    AMIN,			//!< Min Measurement step
    AMAX,			//!< Max Measurement step
    AFRT,			//!< Front Step 
    SCAN,			//!< Standard scan speed
  } parameter;
  char model[128];		//!< Obtained Sensor Model,  MODL
  long distance_min;		//!< Obtained DMIN 
  long distance_max;		//!< Obtained DMAX 
  int area_total;		//!< Obtained ARES 
  int area_min;			//!< Obtained AMIN 
  int area_max;			//!< Obtained AMAX 
  int area_front;		//!< Obtained AFRT 
  int scan_rpm;			//!< Obtained SCAN 

  int first;			//!< Scan Start position
  int last;			//!< Scan end position
  int max_size;			//!< Max. size of data
  long last_timestamp; //!< Time stamp of the last obtained data
} urg_state_t;

urg_state_t urg_state; //  urg parameter variable


long kb_lrf_DistanceData[LRF_DATA_NB];

long kb_lrf_DistanceDataSensor[LRF_DATA_NB];	// Current Data from LRF
long kb_lrf_DistanceDataSum[LRF_DATA_NB];		// Summed Data from LRF
int kb_lrf_DistanceGoodCounter[LRF_DATA_NB];		// Counter for good readings

// RoboSAR: Moved to global from urg_addRecvData for resetting
int urg_addRecvData_remain_byte = 0;


/*****************************************************************************/
/* Internal functions ********************************************************/
/*****************************************************************************/
/* connect to serial port
 *
 * \param device device name
 * \param baudrate baudrate
 * \return 0: Ok; -1 : cannot open serial port
 *
*/ 
static int com_connect(const char* device, long baudrate) {

	struct termios options;

	if ((HComm=open(device,O_RDWR | O_NOCTTY | O_NDELAY))<0) {
		printf("ERROR: can't open serial port %s\n",device);
		return -1;
	}

	fcntl(HComm, F_SETFL, 0); // blocking input

	/* get the current options */
	tcgetattr(HComm, &options);

	/* set raw input, 1 second timeout */
	options.c_cflag     |= (CLOCAL | CREAD);
	options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag     &= ~OPOST;
	options.c_cc[VMIN]  = 0;
	options.c_cc[VTIME] = Timeout/100; // timeout in 1/10 of second

	/* set the options */
	tcsetattr(HComm, TCSANOW, &options);



  // Baud Rate setting
  // !!! Not done : Baud rate setting is not required for USB connection and thus not performed.

  return 0;
}


/*****************************************************************************/
/*!
 * write to serial port
 *
 * \param data data to be sent to the port
 * \param size size of the data
 * \
*/
static int com_send(const char* data, int size) {
  int n;

   tcflush (HComm, TCIFLUSH); // clear the read/write buffer
   n=write( HComm , data , size );

  return n;
}


/*****************************************************************************/
/*!
 * receive data from serial port
 *
 * \param data data to be sent to the port
 * \param max_size max size of the data variable
 * \param timeout timeout in [ms]
 * \return number of data read
*/
static int com_recv(char* data, int max_size, int timeout) {

  int filled = 0;
  int readable_size = 0;
 	struct termios options;
 
  fcntl(HComm, F_SETFL, FNDELAY); // read will return immediately if no data

  memset (data, 0,max_size);

   do {
   // DWORD dwErrors;
   // COMSTAT ComStat;
   // ClearCommError(HComm, &dwErrors, &ComStat);

    ioctl(HComm, FIONREAD, &readable_size); // get number of bytes available to be read

    int read_n = (readable_size > max_size) ? max_size : readable_size;

    int n;
    
    n=read( HComm , &data[filled] ,read_n );
    filled += n;
    readable_size -= n;

    if (filled >= max_size) {
      return filled;
    }
  } while (readable_size > 0);

  // tcflush (HComm, TCIFLUSH); // clear the read buffer


  

  if (timeout > 0) {

	 /* get the current options */
		tcgetattr(HComm, &options);
		options.c_cc[VTIME] = timeout/100; // timeout in tenth of second

		/* set the options */
		tcsetattr(HComm, TCSANOW, &options);


		fcntl(HComm, F_SETFL, 0); // blocking with timeout

		int n;
		while (1) {

			n=read(HComm, &data[filled], 1);
			if (n < 1) {

				tcflush (HComm, TCIFLUSH); // clear the read buffer

				return filled;
			}
			filled += n;
			if (filled >= max_size) {
				return filled;
			}
		
		}
  }
}



/*****************************************************************************/

/*!
 * Send data(Commands) to URG
 * \param  tag command to be sent
 * \return size of the message sent
 */
static int urg_sendTag(const char* tag) {

  char send_message[LineLength];
  sprintf(send_message, "%s\n", tag);
  int send_size = strlen(send_message);
  com_send(send_message, send_size);

  return send_size;
}


/*****************************************************************************/
/*
/*!
 * Read data (Reply) from URG until the termination
 *  \param buffer read data
 *  \return return the number of the count of the read data
*/
static int urg_readLine(char *buffer) {

  int i;
  for (i = 0; i < LineLength -1; ++i) {
    char recv_ch;
    int n = com_recv(&recv_ch, 1, Timeout);
    if (n <= 0) {
      if (i == 0) {
				return -1;		// timeout
      }
      break;
    }
    if ((recv_ch == '\r') || (recv_ch == '\n')) {
      break;
    }
    buffer[i] = recv_ch;
  }
  buffer[i] = '\0';

  return i;
}

/*****************************************************************************/

/*!
 * Send data (Commands) to URG and wait for reply
 *
 * \param command command to be sent
 * \param timeout for waiting the data back (ack)
 * \param recv_n number of bytes received
 * \return return received data in hexa 
 *			 or -1 : received data size does not match
 *					-2 : command does not match	
 */
static int urg_sendMessage(const char* command, int timeout, int* recv_n) {

  int send_size = urg_sendTag(command);
  int recv_size = send_size + 2 + 1 + 2;
  char buffer[LineLength];
 
  sleep(1);
  int n = com_recv(buffer, recv_size, timeout);
  *recv_n = n;

  if (n < recv_size) {
    printf("Error:received size did not matched!\n");
    return -1;
  }

  if (strncmp(buffer, command, send_size -1)) {
    printf("Error:command not matched!\n");
    return -2;
  }

  // !!! If possible do calculate check-sum to verify data

  // Convert the received data to Hex and return data.
  char reply_str[3] = "00";
  reply_str[0] = buffer[send_size];
  reply_str[1] = buffer[send_size + 1];
  return strtol(reply_str, NULL, 16);
}


/*****************************************************************************/

/*!
 * Read URG parameters
 *
 * \param state parameters of the current LRF laser
 * \return 0 if OK, <0 if an error occured
 */
static int urg_getParameters(urg_state_t* state) {

 
  char buffer[LineLength];
  int line_index = 0;
  enum {
    TagReply = 0,
    DataReply,
    Other,
  };
  int line_length;
  
  // Parameter read and processing (use)
  urg_sendTag("PP");
  
  sleep(1);
  for (; (line_length = urg_readLine(buffer)) > 0; ++line_index) {

    if (line_index == Other + MODL) {
      buffer[line_length - 2] = '\0';
      
      strcpy(state->model,&buffer[5]);

			printf("model: %s\n",state->model);

    } else if (line_index == Other + DMIN) {
      state->distance_min = atoi(&buffer[5]);

    } else if (line_index == Other + DMAX) {
      state->distance_max = atoi(&buffer[5]);

    } else if (line_index == Other + ARES) {
      state->area_total = atoi(&buffer[5]);

    } else if (line_index == Other + AMIN) {
      state->area_min = atoi(&buffer[5]);
      state->first = state->area_min;

    } else if (line_index == Other + AMAX) {
      state->area_max = atoi(&buffer[5]);
      state->last = state->area_max;

    } else if (line_index == Other + AFRT) {
      state->area_front = atoi(&buffer[5]);

    } else if (line_index == Other + SCAN) {
      state->scan_rpm = atoi(&buffer[5]);
      printf("scan_rpm: %d\n",state->scan_rpm);
    }
  }

 if (line_index <= Other + SCAN) {
    return -1;
  }
  // Calculate size of the data if necessary
  state->max_size = state->area_max +1;

  return 0;
}


/*****************************************************************************/

/*!
 * Process to connect with URG
 *
 * \param state parameters of the current LRF laser 
 * \param port serial port device
 * \param baudrate baudrate
 * \return 0: no error
 *      -1: cannot open serial port
 *			-2: cannot switch to SCIP2.0 communication protocol
 *			-3: get parameters failed
 */ 
static int urg_connect(urg_state_t* state,
		       const char* port, const long baudrate) {

  if (com_connect(port, baudrate) < 0) {
    ErrorMessage = "Cannot connect COM device!";
    return -1;
  }
  
  // !!! Not done : Baud rate setting is not required for USB connection and thus not performed.
  // const long try_baudrate[] = { 19200, 115200, 38400 };
  //for (size_t i = 0; i < sizeof(try_baudrate)/sizeof(try_baudrate[0]); ++i) {
  
  // Change baud rate to detect the compatible baud rate with sensor
  // !!! com_changeBaudrate(try_baudrate[i]);

  // Change to SCIP2.0 mode
  int recv_n = 0;
  urg_sendMessage("SCIP2.0", Timeout, &recv_n);
  if (recv_n <= 0) {
    printf("If there is no reply it is considered as baud rate incompatibility, try with different baud rate!\n");
    close(HComm);
    ErrorMessage = "Cannot change to SCIP2.0 mode!";
    return -2;
    //continue;
  }

	// !!! Not done : Baud rate setting is not required for USB connection and thus not performed.
  // Change the baud rate if not match the setting
  // if (try_baudrate[i] != baudrate) {
  // !!! urg_changeBaudrate(baudrate);
  // !!! The change will be implemented after 1 scan.
  // !!! Thus, Host or PC should wait to send the command

  // !!! com_changeBaudrate(baudrate);
  //}

  // Read parameter
   if (urg_getParameters(state) < 0) {
    ErrorMessage = "PP command fail.";
    close(HComm);
    return -3;
  }
  
	state->last_timestamp = 0;
	
	printf("URG is detected, port %s\n\n",port);
	
	// URG is detected
	return 0;
}

/*****************************************************************************/

/*!
 * Data read using GD-Command : read lastest measurement; sensor should be switched on before.
 *
 * \param state parameters of the current LRF laser
 * \return 0 : OK ; <0 error
*/ 
static int urg_captureByGD(const urg_state_t* state) {

  char send_message[LineLength];
  sprintf(send_message, "GD%04d%04d%02d", state->first, state->last, 0);

  return urg_sendTag(send_message);
}

/*****************************************************************************/

/*!
 * Data read using MD-Command: captures number of scans defined by capture_times (0 = infinite)
 * Laser is switched on and off automatically.
 *
 * \param state parameters of the current LRF laser 
 * \param capture_times number of times scans should be captured
 * \return 0 : OK ; <0 error
*/
static int urg_captureByMD(const urg_state_t* state, int capture_times) {

  char send_message[LineLength];
  sprintf(send_message, "MD%04d%04d%02d%01d%02d",
	  state->first, state->last, 0, 0, capture_times);

  return urg_sendTag(send_message);
}


/*****************************************************************************/
/*!
 * Decode 6 bit data from URG
 *
 * \param data data
 * \param data_byte size of data
 * \return decoded value
 */
static long urg_decode(const char* data, int data_byte) {
  long value = 0;
  int i;
  
  for (i=0; i < data_byte; ++i) {
    value <<= 6;
    value &= ~0x3f;
    value |= data[i] - 0x30;
  }
  return value;
}

/*****************************************************************************/
/*!
 * Receive and decode URG distance data
 * \param buffer buffer
 * \param data array of long containing the measurement values
 * \param filled ?
 * \return 0 no error
*/
static int urg_addRecvData(const char buffer[], long data[], int* filled) {
  static char remain_data[3];
  const int data_byte = 3;

  const char* pre_p = buffer;
  const char* p = pre_p;

  if (urg_addRecvData_remain_byte > 0) {
    memmove(&remain_data[urg_addRecvData_remain_byte], buffer, data_byte - urg_addRecvData_remain_byte);
    data[*filled] = urg_decode(remain_data, data_byte);
    ++(*filled);
    pre_p = &buffer[data_byte - urg_addRecvData_remain_byte];
    p = pre_p;
    urg_addRecvData_remain_byte = 0;
  }

  do {
    ++p;
    if ((int)(p - pre_p) >= data_byte) {
      data[*filled] = urg_decode(pre_p, data_byte);
      ++(*filled);
      pre_p = p;
    }
  } while (*p != '\0');
  urg_addRecvData_remain_byte = p - pre_p;
  memmove(remain_data, pre_p, urg_addRecvData_remain_byte);

  return 0;
}

/*****************************************************************************/
/*!
 * Receive URG data
 * \param state state variable of the parameters of the laser
 * \param data array of long containing the measurement values
 * \param max_size maximum size of the array (can be determinated by urg_state.max_size )
 * \return an error code
*/
 
static int urg_receiveData(urg_state_t* state, long data[], size_t max_size) {

  int filled = 0;
  int i;
  int ret;
  size_t sti;
  char message_type = 'M';
  char buffer[LineLength];
  int line_length;

  // Fill the positions up to first or min by 19 (non-measurement range)
  /*for (i = state->first -1; i >= 0; --i) {
    data[filled++] = 19;
  }*/


  for (i = 0; (line_length = urg_readLine(buffer)) >= 0; ++i) {

    // Verify the checksum
    if ((i >= 6) && (line_length == 0)) {

      // End of data receive
      for (sti = filled; sti < max_size; ++sti) {
				// Fill the position upto data end by 19 (non-measurement range)
				data[filled++] = 19;
      }
      return filled;

    } else if (i == 0) {
      // Judge the message (Command) by first letter of receive data
      if ((buffer[0] != 'M') && (buffer[0] != 'G')) {
				return -1;
      }
      message_type = buffer[0];

    } else if (! strncmp(buffer, "99b", 3)) {
      // Detect "99b" and assume [time-stamp] and [data] to follow
      i = 4;

    } else if ((i == 1) && (message_type == 'G')) {
      i = 4;

    } else if (i == 4) {
      // "99b" Fixed
      if (strncmp(buffer, "99b", 3)) {
				return -2;
      }

    } else if (i == 5) {
      state->last_timestamp = urg_decode(buffer, 4);

    } else if (i >= 6) {
      // Received Data
      if (line_length > (64 + 1)) {
	line_length = (64 + 1);
      }
      buffer[line_length -1] = '\0';
      ret = urg_addRecvData(buffer, data, &filled);
      if (ret < 0) {
				return ret;
      }
    }
  }
  return -3;
}


/*****************************************************************************/
/* Exported functions ********************************************************/
/*****************************************************************************/


/*****************************************************************************/
/*!
 * kb_lrf_get_timestamp : get the last timestep in [ms]
 *
 * \return the timestamp of last set of data
 */
long kb_lrf_Get_Timestamp(void)
{
  return urg_state.last_timestamp;
}



/*****************************************************************************/
/*!
 * kb_lrf_Laser_On : set LRF laser beam ON
 */
void kb_lrf_Laser_On(int LRF_DeviceHandle)
{
  int recv_n;
  
  if (LRF_DeviceHandle>=0) {
    urg_sendMessage("BM", Timeout, &recv_n); 
  }
}
 
/*****************************************************************************/
/*!
 * kb_lrf_Laser_Off : set LRF laser beam OFF
 */
void kb_lrf_Laser_Off(int LRF_DeviceHandle)
{
    int recv_n;
  
  if (LRF_DeviceHandle>=0) {
    urg_sendMessage("QT", Timeout, &recv_n); 
  } 
}  


/*****************************************************************************/
/*!
 * read_write_memory read/write data on memory
 * \param address memory address to read/write to
 * \param access_type type of access: b: byte h: half word, w: word
 * \param read_result read value
 * \param writeval value to write
 * \param write 1 for write, 0 for read
 * \return <0 an error code, or 0 OK
*/
int read_write_memory(off_t address,char access_type, unsigned long *read_result,unsigned long  writeval,int write) {
  int fd;
  void *map_base, *virt_addr; 
	#define MAP_SIZE 4096UL
	#define MAP_MASK (MAP_SIZE - 1)

  if (address <=0) return -1;
  	

  if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) -1;
 // printf("/dev/mem opened.\n"); 
 // fflush(stdout);
  
  /* Map one page */
  map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, address & ~MAP_MASK);
  if(map_base == (void *) -1) return -3;
  //printf("Memory mapped at address %p.\n", map_base); 
 // fflush(stdout);
  
  virt_addr = map_base + (address & MAP_MASK);
  switch(access_type) {
		case 'b':
			*read_result = *((unsigned char *) virt_addr);
			break;
		case 'h':
			*read_result = *((unsigned short *) virt_addr);
			break;
		case 'w':
			*read_result = *((unsigned long *) virt_addr);
			break;
		default:
			//fprintf(stderr, "Illegal data type '%c'.\n", access_type);
			//exit(2);
			munmap(map_base, MAP_SIZE);
			close(fd);
			return -4;	
	}
    //printf("Value at address 0x%X (%p): 0x%X\n", target, virt_addr, read_result); 
   // fflush(stdout);

	if(write) {
		switch(access_type) {
			case 'b':
				*((unsigned char *) virt_addr) = writeval;
				*read_result = *((unsigned char *) virt_addr);
				break;
			case 'h':
				*((unsigned short *) virt_addr) = writeval;
				*read_result = *((unsigned short *) virt_addr);
				break;
			case 'w':
				*((unsigned long *) virt_addr) = writeval;
				*read_result = *((unsigned long *) virt_addr);
				break;
		}
		//printf("Written 0x%X; readback 0x%X\n", writeval, read_result); 
		//fflush(stdout);
	}
	
	if(munmap(map_base, MAP_SIZE) == -1) return -5;
    close(fd);
    
  return 0;
}



/*****************************************************************************/
/*!
 * kb_lrf_Power_On : power on the LRF battery supply module. use pwm0 as GPIO 145 
 */
void kb_lrf_Power_On(void)
{
	off_t address;
	char access_type;
	unsigned long read_result;
	unsigned long  writeval;

	// configure pwm0 as gpio as mode M4, PULLUDENABLE and INPUTENABLE	
	read_write_memory(0x48002176,'h',&read_result,0x10C,1);
 
 	// configure as ouput
 	read_write_memory(0x49056036,'h',&read_result,0,0);
 	writeval=read_result & 0xFFFFFFFD;
 	read_write_memory(0x49056036,'h',&read_result,writeval,1);
 	
 	
 	// set to 1
 	read_write_memory(0x4905603E,'h',&read_result,0,0);
 	writeval=read_result | 0x2;
 	read_write_memory(0x4905603E,'h',&read_result,writeval,1);
 	

	usleep(1500000); // wait for the lrf to initialize
}

/*****************************************************************************/
/*!
 * kb_lrf_Power_Off : power off the LRF battery supply module
 */
void kb_lrf_Power_Off(void)
{

	off_t address;
	char access_type;
	unsigned long read_result;
	unsigned long  writeval;

	// configure pwm0 as gpio as mode M4, PULLUDENABLE and INPUTENABLE	
	read_write_memory(0x48002176,'h',&read_result,0x10C,1);
 
 	// configure as ouput
 	read_write_memory(0x49056036,'h',&read_result,0,0);
 	writeval=read_result & 0xFFFFFFFD;
 	read_write_memory(0x49056036,'h',&read_result,writeval,1);
 	
 	
 	// set to 0
 	read_write_memory(0x4905603E,'h',&read_result,0,0);
 	writeval=read_result & 0xFFFFFFFD;
 	read_write_memory(0x4905603E,'h',&read_result,writeval,1);
	
}

/*****************************************************************************/
/*!
 * kb_lrf_Init does openning port, turn power on the LRF then certify the link,
 * \param LRF_DeviceName String name of the serial port device where the laser is connected 
 * \return 	- < 0 : Any error code 
 * 		- > 0 : Handle to the device
 */
int kb_lrf_Init(char *LRF_DeviceName)
{
  // RoboSAR: reset global variable at init
  urg_addRecvData_remain_byte = 0;
  int LRF_DeviceHandle=-1;
  int recv_n = 0; 
  int ret=0;
  
  
  // Connect with URG 
  ret = urg_connect(&urg_state, LRF_DeviceName, BAUDRATE);
  if (ret < 0) {
    // Show error message and end
    printf("Error: %s\n", ErrorMessage);
		KB_ERROR("kb_lrf_Init",KB_ERROR_OPENLRF, LRF_DeviceName);
	// If not terminated immidiately. (Remove if not necessary)
    //	getchar();
    return -1;
  }
 
  LRF_DeviceHandle=HComm;
 

  // Laser on
  kb_lrf_Laser_On(LRF_DeviceHandle);
  

  return(LRF_DeviceHandle);
}


/*****************************************************************************/
/*!
 * kb_lrf_Close does powering off the LRF, the close the port device
 *
 * \param LRF_DeviceHandle	Handle to the lrf port which as been openned
 *				by kb_lrf_openComPort or kb_lrf_init
 */
void kb_lrf_Close(int LRF_DeviceHandle)
{
  if (LRF_DeviceHandle>=0)
  {
	  kb_lrf_Laser_Off(LRF_DeviceHandle);
	  close(LRF_DeviceHandle);
  }
}



/*****************************************************************************/
/*!
 * kb_lrf_GetDistances is a high level function that does whole mechanism
 * certify the link, get some distances. 
 *
 * \param LRF_DeviceHandle	Handle to the lrf port which as been openned
 *				by kb_lrf_openComPort or kb_lrf_init
 * \return 	
 * 		-  0 : Sucess
 *		-	 -1 : serial port not open
 *    -  -2 : could not send command capture
 * 		-	 -3 : could not received data 			
 */
int kb_lrf_GetDistances(int LRF_DeviceHandle)
{
	if (LRF_DeviceHandle<0)
	{
		return -1;
	}	

	if(urg_captureByGD(&urg_state)<0)
	{
		return -2;
	}
	
	if(urg_receiveData(&urg_state, kb_lrf_DistanceData, LRF_DATA_NB)<0)
	{
		return -3;
	}

	return 0;
	
	
}

/*****************************************************************************/
/*!
 * kb_lrf_GetDistances is a high level function that does whole mechanism
 * certify the link, get some distances, averages then receive back the data from LRF,
 *
 * \param LRF_DeviceHandle	Handle to the lrf port which as been openned
 *				by kb_lrf_openComPort or kb_lrf_init
 * \param average number of acquisitions needed for averaging
 * \return 	
 * 		-  0 : Sucess
 *		-	 -1 : serial port not open
 *    -  -2 : could not send command capture
 * 		-	 -3 : could not received data 			
 */
int kb_lrf_GetDistances_Averaged(int LRF_DeviceHandle, int average)
{
  int i, j,ret;

	memset (kb_lrf_DistanceDataSum, 0,LRF_DATA_NB*sizeof(long)); // long
	memset (kb_lrf_DistanceGoodCounter, 0,LRF_DATA_NB*sizeof(int)); // int

  /* Get several readings to average */
  for (i=0; i<average; i++)
  {
    if ((ret=kb_lrf_GetDistances(LRF_DeviceHandle))<0)
    	return ret;
    for (j=0; j<LRF_DATA_NB; j++)
    {
     // if (kb_lrf_DistanceData[j] > )		/* otherwise error! */
     // {
        kb_lrf_DistanceDataSum[j] += kb_lrf_DistanceData[j];	/* add 'good' values */
        kb_lrf_DistanceGoodCounter[j]++;				/* increase counter for good values */
     // }
    }
  }

  /* averaging */
  for (j=0; j<LRF_DATA_NB; j++)
  {
    if (kb_lrf_DistanceGoodCounter[j] > 0)
    {
      kb_lrf_DistanceData[j] = kb_lrf_DistanceDataSum[j]/kb_lrf_DistanceGoodCounter[j];
    } else {
      kb_lrf_DistanceData[j] = 0;
    }
  }

  return 0;
}

