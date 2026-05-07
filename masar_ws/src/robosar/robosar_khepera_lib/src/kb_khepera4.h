/*--------------------------------------------------------------------
 * kb_khepera4.h - KoreBot Library - Khepera4 functions
 *--------------------------------------------------------------------
 * $Id: kb_khepera4.h,v 1.0 2013/05/25 12:33:46 jtharin Exp $
 *--------------------------------------------------------------------
 * $Author: jtharin $
 * $Date: 2013/05/25 12:33:46 $
 * $Revision: 1.0 $
 *-------------------------------------------------------------------*/ 

#ifndef __khepera4__
#define __khepera4__

#include "knet.h"
#include <math.h>


/*! 
 *  Khepera4 Error codes
 */ 
#define KH4_ERROR_SUCCESS	0		/* No errors */
#define KH4_ERROR_KBINIT	-1		/* Unable to initialize the khepera library */

#define KH4_ERROR_FRMSZ   	-2		/* Frame received doesnt have the expected size */
#define KH4_ERROR_SZFMT   	-3		/* Frame size received should be 1 byte wide, wrong format */
#define KH4_ERROR_FRMSND  	-4		/* Unable to send a command frame to the khepera4 */
#define KH4_ERROR_GPIO 		-5    // Unable to initialise the GPIO


// robot hardware constants

#define KH4_WHEELS_DISTANCE 105.4 // distance between wheel, for rotation calculus [mm]

#define KH4_PULSE_TO_MM  0.006781 // motor position factor to convert from pulse to mm

#define KH4_TIME_BTWN 10 // [ms] time for speed computation

#define KH4_SPEED_TO_MM_S  0.6781 // motor speed factor to convert from speed units to mm/s 

#define KH4_GYRO_DEG_S (66.0/1000.0) // [deg/s] for gyroscope 

#define KH4_MAX_IR_VALUE 1024  // 10 bits

#define KH4_MAX_OPENLOOP_SPEED_PWM_100 2940 // maximum of the pwm value

#define KH4_US_DISABLED_SENSOR 2000  // disabled sensor
#define KH4_US_NO_OBJECT_IN_RANGE 1000  // no object in range 25..250cm
#define KH4_US_OBJECT_NEAR	0 // object at less 25cm


/*!
 *  Status led
 */

#define KH4_ST_LED_RED_ON		0x01
#define KH4_ST_LED_GREEN_ON	0x10
#define KH4_ST_LED_ALL_ON		0x00
#define KH4_ST_LED_ALL_OFF	0x11	
#define KH4_ST_TOGGLE_RED		1
#define KH4_ST_TOGGLE_GREEN 0
 

/*--------------------------------------------------------------------
 *! Khepera4 Regulation Types 
 */
enum kh4RegType 
{
	kh4RegIdle     		= 0 , /*! Idle					*/
  kh4RegSpeed     	= 1 , /*! Speed					*/
  kh4RegSpeedProfile	= 2 , /*! Speed Profile	*/
  kh4RegPosition		= 3 , /*! Position			*/
  kh4RegSOpenLoop		= 4 , /*! Open Loop			*/
};


#define GPIO_KH4_RESET 90  /* Khepera4 reset if low*/
#define GPIO_KH4_CHARGE 65   /* Khepera4 detected charge plug if low*/


/*!--------------------------------------------------------------------
 * Prototypes Declaration
 */
/* Need to be called before any other functions */
extern int kh4_init(int argc , char * argv[]);

extern void kh4_release( void );


/* "Low level" function to communicate with the khepera3 via i2c */
extern int kh4_getcommand( knet_dev_t *hDev,char ,
		char * buf ,	unsigned int len);
extern int kh4_sendcommand( knet_dev_t *hDev, char ,
		 const char *, unsigned int);

/* "High level" function that let user to simply retrieves various informations from the robot */

/* status */
extern int kh4_battery_status(char *,knet_dev_t *);
extern int kh4_battery_charge(knet_dev_t *);

extern int kh4_revision(char *, knet_dev_t *);

extern int kh4_SetStatusLeds(char,knet_dev_t *);
extern int kh4_ToggleStatusLed(int,knet_dev_t *);


extern int kh4_SetRGBLeds(char,char,char,char,char,char,char,char,char,knet_dev_t *);

/* motor control */
extern int kh4_SetMode(int,knet_dev_t *);
extern int kh4_SetPositionMargin(int,knet_dev_t *);
extern int kh4_ConfigurePID(char,char,char,knet_dev_t *);
extern int kh4_SetSpeedProfile(char,char,char,char,int,knet_dev_t *);

extern int kh4_get_position(int *,int *, knet_dev_t *);
extern int kh4_set_position(int,int, knet_dev_t *);
extern int kh4_get_speed(int *,int *, knet_dev_t *);
extern int kh4_set_speed(int ,int , knet_dev_t *);
extern int kh4_ResetEncoders(knet_dev_t *);

/* sensoring */ 
extern int kh4_proximity_ir(char *, knet_dev_t *);
extern int kh4_ambiant_ir(char *, knet_dev_t *);
extern int kh4_measure_us(char *, knet_dev_t *);	
extern int kh4_measure_acc(char *, knet_dev_t *);
extern int kh4_measure_gyro(char *, knet_dev_t *);

extern int kh4_activate_us(char, knet_dev_t *);	

/* emergency function */
extern int kh4_reset(knet_dev_t *);

#endif /* __khepera4__ */
