/*! 
 * \file   kb_khepera4.c Kheper4 layer              
 *
 * \brief 
 *         This module is layer for communication with various devices 
 *         which are part of the khepera4 robot. It offers simple interface 
 *         to the user.
 *        
 * \author   J Tharin (K-Team SA)                               
 *
 * \note     Copyright (C) 2013 K-TEAM SA
 * \bug      none discovered.           
 * \todo     nothing.
 */

#include "khepera.h"


// I2c Address register
#define I2C_REVISION                      0x00

#define I2C_LED                           0x01    // ON/OFF and ERR Led value

#define I2C_RGB_FL_R                      0x02    // Front Left Red led
#define I2C_RGB_FL_G                      0x03    // Front Left Green led 
#define I2C_RGB_FL_B                      0x04    // Front Left Blue led 
#define I2C_RGB_FR_R                      0x05    // Front Right Red led 
#define I2C_RGB_FR_G                      0x06    // Front Right Green led 
#define I2C_RGB_FR_B                      0x07    // Front Right Blue led 
#define I2C_RGB_B_R                       0x08    // Back Red led 
#define I2C_RGB_B_G                       0x09    // Back Green led 
#define I2C_RGB_B_B                       0x0A    // Back Blue led 

#define I2C_MOT_KP                        0x0B    // Kp value of the controller
#define I2C_MOT_KI                        0x0C    // Ki value of the controller
#define I2C_MOT_KD                        0x0D    // Kd value of the controller

#define I2C_MOT_ACC_INC                   0x0E    // Acceleration increment of the motor 
#define I2C_MOT_ACC_DIV                   0x0F    // Acceleration divider of the motor 

#define I2C_MOT_MIN_SPEED_ACC             0x10    // Minimum speed used during the acceleration
#define I2C_MOT_MIN_SPEED_DEC             0x11    // Minimum speed used during the deceleration

#define I2C_MOT_POS_MAR_L                 0x12    // position margin used in the position control
#define I2C_MOT_POS_MAR_H                 0x13


#define I2C_MOT_SPEED_POS_L               0x14    // Consign of speed using during the position control
#define I2C_MOT_SPEED_POS_H               0x15     


#define I2C_MOT_POSITION_RESET            0x16    // register to reset the position counter of the two motors

#define I2C_MOT_CONTROL_TYPE              0x17    // Type of the control use for the motor

#define I2C_MOT_LEFT_SPEED_CONSIGN_L      0x18    // Consign for the left motor speed
#define I2C_MOT_LEFT_SPEED_CONSIGN_H      0x19 


#define I2C_MOT_RIGHT_SPEED_CONSIGN_L     0x1A    // Consign for the right motor speed
#define I2C_MOT_RIGHT_SPEED_CONSIGN_H     0x1B 


#define I2C_MOT_LEFT_PWM_CONSIGN_L        0x1C    // PWM Consign used in open loop for the left motor
#define I2C_MOT_LEFT_PWM_CONSIGN_H        0x1D


#define I2C_MOT_RIGHT_PWM_CONSIGN_L       0x1E    // PWM Consign used in open loop for the right motor
#define I2C_MOT_RIGHT_PWM_CONSIGN_H       0x1F


#define I2C_MOT_LEFT_POS_CONSIGN_LL       0x20    // Position Consign of the left motor
#define I2C_MOT_LEFT_POS_CONSIGN_LH       0x21
#define I2C_MOT_LEFT_POS_CONSIGN_HL       0x22
#define I2C_MOT_LEFT_POS_CONSIGN_HH       0x23


#define I2C_MOT_RIGHT_POS_CONSIGN_LL      0x24    // Position Consign of the right motor
#define I2C_MOT_RIGHT_POS_CONSIGN_LH      0x25
#define I2C_MOT_RIGHT_POS_CONSIGN_HL      0x26
#define I2C_MOT_RIGHT_POS_CONSIGN_HH      0x27


#define I2C_MOT_LEFT_SPEED_L              0x28    // Actual speed of the left motor
#define I2C_MOT_LEFT_SPEED_H              0x29          


#define I2C_MOT_RIGHT_SPEED_L             0x2A    // Actual speed of the right motor
#define I2C_MOT_RIGHT_SPEED_H             0x2B          


#define I2C_MOT_ON_TARGET                 0x2C    // Contain the two OnTarget flag (bit0 = left, bit1 = right)


#define I2C_MOT_LEFT_POS_LL               0x2D    // Actual position of the left motor
#define I2C_MOT_LEFT_POS_LH               0x2E
#define I2C_MOT_LEFT_POS_HL               0x2F
#define I2C_MOT_LEFT_POS_HH               0x30


#define I2C_MOT_RIGHT_POS_LL              0x31    // Actual position of the right motor
#define I2C_MOT_RIGHT_POS_LH              0x32
#define I2C_MOT_RIGHT_POS_HL              0x33
#define I2C_MOT_RIGHT_POS_HH              0x34



#define I2C_IR_AMB_BL_L                   0x40    // Ambiant value of the Back left sensor
#define I2C_IR_AMB_BL_H                   0x41
#define I2C_IR_AMB_L_L                    0x42    // Ambiant value of the Left sensor
#define I2C_IR_AMB_L_H                    0x43
#define I2C_IR_AMB_FL_L                   0x44    // Ambiant value of the Front left sensor
#define I2C_IR_AMB_FL_H                   0x45
#define I2C_IR_AMB_F_L                    0x46    // Ambiant value of the Front sensor
#define I2C_IR_AMB_F_H                    0x47
#define I2C_IR_AMB_FR_L                   0x48    // Ambiant value of the Front Right sensor
#define I2C_IR_AMB_FR_H                   0x49
#define I2C_IR_AMB_R_L                    0x4A    // Ambiant value of the Right sensor
#define I2C_IR_AMB_R_H                    0x4B
#define I2C_IR_AMB_BR_L                   0x4C    // Ambiant value of the Back Right sensor
#define I2C_IR_AMB_BR_H                   0x4D
#define I2C_IR_AMB_B_L                    0x4E    // Ambiant value of the Back sensor
#define I2C_IR_AMB_B_H                    0x4F
#define I2C_IR_AMB_GL_L                   0x50    // Ambiant value of the Ground Left sensor
#define I2C_IR_AMB_GL_H                   0x51
#define I2C_IR_AMB_GFL_L                  0x52    // Ambiant value of the Ground Front Left sensor
#define I2C_IR_AMB_GFL_H                  0x53
#define I2C_IR_AMB_GFR_L                  0x54    // Ambiant value of the Ground Front Right sensor
#define I2C_IR_AMB_GFR_H                  0x55
#define I2C_IR_AMB_GR_L                   0x56    // Ambiant value of the Ground Right sensor
#define I2C_IR_AMB_GR_H                   0x57

#define I2C_IR_DIST_BL_L                  0x58    // Distance value of the Back left sensor
#define I2C_IR_DIST_BL_H                  0x59
#define I2C_IR_DIST_L_L                   0x5A    // Distance value of the Left sensor
#define I2C_IR_DIST_L_H                   0x5B
#define I2C_IR_DIST_FL_L                  0x5C    // Distance value of the Front left sensor
#define I2C_IR_DIST_FL_H                  0x5D
#define I2C_IR_DIST_F_L                   0x5E    // Distance value of the Front sensor
#define I2C_IR_DIST_F_H                   0x5F
#define I2C_IR_DIST_FR_L                  0x60    // Distance value of the Front Right sensor
#define I2C_IR_DIST_FR_H                  0x61
#define I2C_IR_DIST_R_L                   0x62    // Distance value of the Right sensor
#define I2C_IR_DIST_R_H                   0x63
#define I2C_IR_DIST_BR_L                  0x64    // Distance value of the Back Right sensor
#define I2C_IR_DIST_BR_H                  0x65
#define I2C_IR_DIST_B_L                   0x66    // Distance value of the Back sensor
#define I2C_IR_DIST_B_H                   0x67
#define I2C_IR_DIST_GL_L                  0x68    // Distance value of the Ground Left sensor
#define I2C_IR_DIST_GL_H                  0x69
#define I2C_IR_DIST_GFL_L                 0x6A    // Distance value of the Ground Front Left sensor
#define I2C_IR_DIST_GFL_H                 0x6B
#define I2C_IR_DIST_GFR_L                 0x6C    // Distance value of the Ground Front Right sensor
#define I2C_IR_DIST_GFR_H                 0x6D
#define I2C_IR_DIST_GR_L                  0x6E    // Distance value of the Ground Right sensor
#define I2C_IR_DIST_GR_H                  0x6F

#define I2C_US_L_L                        0x70    // Distance of the Left US sensor
#define I2C_US_L_H                        0x71  
#define I2C_US_FL_L                       0x72    // Distance of the Front Left US sensor
#define I2C_US_FL_H                       0x73  
#define I2C_US_F_L                        0x74    // Distance of the Front US sensor
#define I2C_US_F_H                        0x75  
#define I2C_US_FR_L                       0x76    // Distance of the Front Right US sensor
#define I2C_US_FR_H                       0x77  
#define I2C_US_R_L                        0x78    // Distance of the Right US sensor
#define I2C_US_R_H                        0x79  

#define I2C_US_ACTIVE											0x7A		// US activation

#define I2C_ACC_X                         0x80    // Start of the X value of the accelerometer
#define I2C_ACC_Y                         0x94    // Start of the Y value of the accelerometer
#define I2C_ACC_Z                         0xA8    // Start of the Z value of the accelerometer

#define I2C_GYR_X                         0xC0    // Start of the X value of the Gyro
#define I2C_GYR_Y                         0xD4    // Start of the X value of the Gyro
#define I2C_GYR_Z                         0xE8    // Start of the X value of the Gyro

#define I2C_BAT_STATUS										0xFD // Batterie Status (raffraichi les autres valeurs)



static int currentRegType;

/*! kh4_init initializes some things like the GPIO40 pin.
 * This function needs to be called BEFORE any other functions.
 * 
 *
 * \return A value:
 *       - <0 on error 
 *       - =0 on success 
 *
 */
int kh4_init( int argc , char * argv[] )
{
	int rc;
	char label[10];

	/* First of all this function initializes the khepera library */

	if((rc = kb_init( argc , argv )) < 0 )
	{
		/* Unable to initialize the khepera library */
		KB_ERROR("kb_kh4_init",KB_ERROR_KH4KBINIT);
		return KH4_ERROR_KBINIT;
	}


	/* Init gpio module : already initialised in kb_init */
	//if ((rc=kb_gpio_init())<0)  
//	{
	
		// request GPIO for reset (it reset when 0)
		sprintf(label,"GPIO_%d",GPIO_KH4_RESET);
		
		
		if(gpio_request(GPIO_KH4_RESET, label ))
		{
			gpio_free(GPIO_KH4_RESET); // reset if to be sure to not already be taken
			
			if(gpio_request(GPIO_KH4_RESET, label ))
			{
				KB_ERROR("kb_gpio_init",KB_ERROR_INVALID,"Could not request io for KH4 reset");				
			}
			else
			{
				kb_gpio_dir_val(GPIO_KH4_RESET,0,1); // set to output and to 1 (not reset)
			}
				
		}
		else
		{
			kb_gpio_dir_val(GPIO_KH4_RESET,0,1); // set to output and to 1 (not reset)
		}
	
		// request GPIO for battery charge test
		sprintf(label,"GPIO_%d",GPIO_KH4_CHARGE);
		
		
		if(gpio_request(GPIO_KH4_CHARGE, label ))
		{
			gpio_free(GPIO_KH4_CHARGE); // reset if to be sure to not already be taken
			
			if(gpio_request(GPIO_KH4_CHARGE, label ))
			{
				KB_ERROR("kb_gpio_init",KB_ERROR_INVALID,"Could not request io for KH4 charge");				
			}
			else
			{
				kb_gpio_dir(GPIO_KH4_CHARGE,1); // set to input
			}
				
		}
		else
		{
			kb_gpio_dir(GPIO_KH4_CHARGE,1); // set to input
		}
	
	
		// Unable to initiliase gpio
	//	return KH4_ERROR_GPIO;
//	}

	atexit(kh4_release);

		 
 return KH4_ERROR_SUCCESS;	 
} 


/*--------------------------------------------------------------------*/
/*! This function is called automatically on exit. 
 *
 * \remark This function is called automatically at the terminaison 
 *         of the application.
 */
void  kh4_release( void )
{
	gpio_free(GPIO_KH4_RESET);
	gpio_free(GPIO_KH4_CHARGE);
}


/*!
 * kh4_getcommand read a robot register
 * 
 *  
 * Normally an end user don't want to use these function as they are
 * assumed as "low level functions".
 * 
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * \param reg_address address of the register to write to
 * \param buf Pointer to the buffer that will be written.
 * \param len Number of bytes to write
 
 * \return A value:
 *       - <0 on error (KB_ERROR_KH4RCVERR)
 *       - >=0 on success 
 *
 * \remark This function requires that kb_kh4_init has been called
 */
int kh4_getcommand( knet_dev_t *hDev,char reg_address,
		char * buf ,	unsigned int len)
{
	int rc;

	if( (rc=knet_lltransfer(hDev,&reg_address,1,buf,len)) <0 ) 
	{	
		KB_ERROR("knet_getCommand",KB_ERROR_KH4RCVERR,rc);
		return KB_ERROR_KH4RCVERR; 
	}
	
	return 0;
}


/*!
 * kh4_sendcommand sets a command frame to a given khepera3 device.
 *
 * Normally and end user don't want to use these function as they are
 * assumed as "low level functions".
 *   
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * \param reg_address address of the register to write to
 * \param buf Pointer to the buffer that will be written.
 * \param len Number of bytes to write
 *
 * \return A value:
 *       - <0 on error (KB_ERROR_KH4SENERR)
 *       - >=0 on success
 *
 * \remark This function requires that kb_kh4_init has been called
 */
int kh4_sendcommand( knet_dev_t *hDev, char reg_address,
		 const char * buf , unsigned int len)
{
  int rc;
  int i;
  char *bufreg;
  
	// add reg address
	len+=1; 
	
  bufreg=(char*)malloc(sizeof(char)*len);

  if (bufreg==NULL)
	return -ENOMEM;
	
	bufreg[0]=reg_address;

	for (i=1;i<len;i++)
		bufreg[i]=buf[i-1];

  rc = knet_llwrite(hDev,bufreg,len);

	free(bufreg);
	

	if (rc<0)
	{
	 KB_ERROR("knet_sendCommand", KB_ERROR_KH4SENERR,rc);
		return KB_ERROR_KH4SENERR;
	}
		
	return 0;
}

/*!
 * kh4_revision retrieves the current OS version/revision
 *
 * \param outbuf is a buffer where the data will be stored on.
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success
 */
int kh4_revision(char *outbuf, knet_dev_t *hDev)
{
  int rc;

	if(hDev)
  {
	
		rc = kh4_getcommand( hDev, I2C_REVISION,outbuf,1 );
		
		return rc;
  
  }
    
  return KB_ERROR_KH4KBNOTINIT;
}

/*!
 * kh4_proximity_ir retrieves an instant IR measure.
 *
 * \param outbuf is a buffer where the data will be stored on.
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 *
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success (returns should be the size of frame) 
 */
int kh4_proximity_ir(char *outbuf, knet_dev_t *hDev)
{

  int rc;
 

  if(hDev)
  {
 
		rc = kh4_getcommand( hDev,I2C_IR_DIST_BL_L, outbuf, 24);
	
		return rc;
	
  }
 
  return KB_ERROR_KH4KBNOTINIT;
 
}

/*!
 * kh4_ambiant_ir retrieves an instant IR measure.
 *
 * \param outbuf is a buffer where the data will be stored on.
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success (returns should be the size of frame) 
 *
 */
int kh4_ambiant_ir(char *outbuf, knet_dev_t *hDev)
{
  int rc;

  if(hDev)
  {
     
		rc = kh4_getcommand( hDev,I2C_IR_AMB_BL_L, outbuf, 24);

		return rc;
  }

  return KB_ERROR_KH4KBNOTINIT;

}

/*!
 * kh4_battery_status retrieves the actual battery status.
 *
 * \param outbuf is a buffer where the data will be stored on.
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success (returns should be the size of frame) 
 *
 */
int kh4_battery_status(char *outbuf,knet_dev_t *hDev)
{
  int rc;

  if(hDev)
  {
		rc = kh4_getcommand( hDev,I2C_BAT_STATUS, outbuf, 12);
		return rc;
  }
  return KB_ERROR_KH4KBNOTINIT;
}


/*!
 * kh4_battery_charge detetcs if a charger is plugged
 *
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 *
 * \return A value:
 *       - <0 on error
 *       - 1: charger plugged
 *       - 0: charger unplugged  
 *
 */
int kh4_battery_charge(knet_dev_t *hDev)
{
	int rc;
  if(hDev)
  {		
		rc=kb_gpio_get(GPIO_KH4_CHARGE);	
		
		if (rc<0)
		{
			return rc;
		}
		else
			return !rc;	
  }
  return KB_ERROR_KH4KBNOTINIT;
}

/*!
 * kh4_activate_us activate US sensors (default: all on).
 *
 * \param outbuf is a buffer where the data will be stored on. bit0:left, bit1:front left, bit2:front, bit3:front right, bt4:right
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success (returns should be the size of frame) 
 *
 */
int kh4_activate_us(char outbuf, knet_dev_t *hDev)
{
  if(hDev)
  {
		return kh4_sendcommand(hDev,I2C_US_ACTIVE,&outbuf,1);	
  }
  return KB_ERROR_KH4KBNOTINIT;
}


/*!
 * kh4_measure_us retrieves measure from  US transmitters. 
 *
 * \param outbuf is a buffer where the data will be stored on.
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * 
 * \return A value:
 *       - <0 on error
 *       - >=0 on success
 */
int kh4_measure_us(char *outbuf, knet_dev_t *hDev)
{
  int rc;

  if(hDev)
  {
 		rc = kh4_getcommand( hDev,I2C_US_L_L, outbuf, 10);
	
		return rc;
  }
  return KB_ERROR_KH4KBNOTINIT;
}


/*!
 * kh4_measure_ac retrieves measure from accelerometer
 *
 * \param outbuf is a buffer where the data will be stored on.
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * 
 * \return A value:
 *       - <0 on error
 *       - >=0 on success
 */
int kh4_measure_acc(char *outbuf, knet_dev_t *hDev)
{
  int rc;

  if(hDev)
  {
 		rc = kh4_getcommand( hDev,I2C_ACC_X, outbuf, 60);
	
		return rc;
  }
  return KB_ERROR_KH4KBNOTINIT;
}

/*!
 * kh4_measure_gyro retrieves measure from gyroscope
 *
 * \param outbuf is a buffer where the data will be stored on.
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * 
 * \return A value:
 *       - <0 on error
 *       - >=0 on success
 */
int kh4_measure_gyro(char *outbuf, knet_dev_t *hDev)
{
  int rc;

  if(hDev)
  {
 		rc = kh4_getcommand( hDev,I2C_GYR_X, outbuf, 60);
	
		return rc;
  }
  return KB_ERROR_KH4KBNOTINIT;
}


/*!
 * kh4_reset reset the robot microcontroller
 *
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 *
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success 
 */
int kh4_reset(knet_dev_t *hDev)
{
  if(hDev)
  { 		
		kb_gpio_clear(GPIO_KH4_RESET);
		usleep(1000); // wait 1ms
		kb_gpio_set(GPIO_KH4_RESET);			
		return 0;	
  }
  return KB_ERROR_KH4KBNOTINIT; 
}



/*!
 * kh4_SetMode set the control mode of the motors
 *
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * \param regtype type of control
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success 
 */
int kh4_SetMode(int regtype,knet_dev_t *hDev)
{
  char outbuf[1];
  if(hDev)
  {	
  	outbuf[0]=regtype;
  	currentRegType=regtype;	
		return kh4_sendcommand(hDev,I2C_MOT_CONTROL_TYPE,outbuf,1);	
  }
  return KB_ERROR_KH4KBNOTINIT; 
}


/*!
 * kh4_ResetEncoders reset the encoders positions
 *
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success 
 */
int kh4_ResetEncoders(knet_dev_t *hDev)
{
  char outbuf[1];
  if(hDev)
  {	
  	outbuf[0]=0;	 // any value
		return kh4_sendcommand(hDev,I2C_MOT_POSITION_RESET,outbuf,1);	
  }
  return KB_ERROR_KH4KBNOTINIT; 
}


/*!
 * kh4_SetPositionMargin set position margin for control
 *
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * \param margin margin value
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success 
 */
int kh4_SetPositionMargin(int margin,knet_dev_t *hDev)
{
	char outbuf[2];
  if(hDev)
  {	
  	outbuf[0]=margin&0xff;
  	outbuf[1]=margin>>8;
  		
		return kh4_sendcommand(hDev,I2C_MOT_POS_MAR_L,outbuf,2);	
  }
  return KB_ERROR_KH4KBNOTINIT; 
}


/*!
 * kh4_ConfigurePID configure motors PID
 * 
 * \param kp kp parameter
 * \param ki ki parameter
 * \param kd kd parameter
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success 
 */
int kh4_ConfigurePID(char kp,char ki, char kd,knet_dev_t *hDev)
{
	char outbuf[3];
  if(hDev)
  {	
  	outbuf[0]=kp;
  	outbuf[1]=ki;
  	outbuf[2]=kd;
  		
		return kh4_sendcommand(hDev,I2C_MOT_KP,outbuf,3);	
  }
  return KB_ERROR_KH4KBNOTINIT; 
}


/*!
 * kh4_SetSpeedProfile configure motor control speed profile
 * 
 * \param accinc Acceleration increment
 * \param accdiv Acceleration divider
 * \param mindacc Minimum speed acc
 * \param mindec  Minimum speed dec
 * \param max_speed maximum speed
 * 
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success 
 */
int kh4_SetSpeedProfile(char accinc,char accdiv, char mindacc,char mindec,int max_speed,knet_dev_t *hDev)
{
	char outbuf[4];
	int rc;
  if(hDev)
  {	
  	outbuf[0]=accinc;
  	outbuf[1]=accdiv;
  	outbuf[2]=mindacc;
  	outbuf[3]=mindec;  		
		if ((rc=kh4_sendcommand(hDev,I2C_MOT_ACC_INC,outbuf,4))<0)
		{
			return rc;
		}		
		outbuf[0]=max_speed&0xff;
		outbuf[1]=max_speed>>8;
		return kh4_sendcommand(hDev,I2C_MOT_SPEED_POS_L,outbuf,2);				
  }
  return KB_ERROR_KH4KBNOTINIT; 
}

/*!
 * kh4_get_position get motors position
 *
 * \param left left motor position (units: encoder)
 * \param right right motor position (units: encoder)
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * 
 * \return A value:
 *       - <0 on error
 *       - >=0 on success
 */
int kh4_get_position(int *left,int *right, knet_dev_t *hDev)
{
  int rc;
  char outbuf[8];

  if(hDev)
  {
 		rc = kh4_getcommand( hDev,I2C_MOT_LEFT_POS_LL, outbuf, 8);
	
		*left=outbuf[0] | (outbuf[1] | (outbuf[2] |outbuf[3]<<8)<<8)<<8;	
		*right=outbuf[4] | (outbuf[5] | (outbuf[6] |outbuf[7]<<8)<<8)<<8;;
	
		return rc;
  }
  return KB_ERROR_KH4KBNOTINIT;
}

/*!
 * kh4_set_position set motors position
 *
 * \param left left motor position (units: encoder)
 * \param right right motor position (units: encoder)
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * 
 * \return A value:
 *       - <0 on error
 *       - >=0 on success
 */
int kh4_set_position(int left,int right, knet_dev_t *hDev)
{
  int rc;
  char outbuf[8];

  if(hDev)
  {
		outbuf[0]=left & 0xff; 
		outbuf[1]=(left & 0xff00)>>8; 
		outbuf[2]=(left & 0xff0000)>>16;
		outbuf[3]=(left & 0xff000000)>>24;
		outbuf[4]=right & 0xff;
		outbuf[5]=(right & 0xff00)>>8;
		outbuf[6]=(right & 0xff0000)>>16; 
		outbuf[7]=(right & 0xff000000)>>24;

	
		return kh4_sendcommand(hDev,I2C_MOT_LEFT_POS_CONSIGN_LL,outbuf,8);	
  }
  return KB_ERROR_KH4KBNOTINIT;
}

/*!
 * kh4_get_speed get motors speed
 *
 * \param left left motor pspeed (units: encoder)
 * \param right right motor speed (units: encoder)
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * 
 * \return A value:
 *       - <0 on error
 *       - >=0 on success
 */
int kh4_get_speed(int *left,int *right, knet_dev_t *hDev)
{
  int rc;
  char  outbuf[4];

	short int l,r;

  if(hDev)
  {
 		rc = kh4_getcommand( hDev,I2C_MOT_LEFT_SPEED_L, outbuf, 4);
		
		l=(short int)(outbuf[0] | (outbuf[1]<<8));	// casting for getting sign
		r=(short int)(outbuf[2] | (outbuf[3]<<8));  // casting for getting sign
		*left=(int)l;
		*right=(int)r;
		return rc;
  }
  return KB_ERROR_KH4KBNOTINIT;
}

/*!
 * kh4_set_speed set motors position
 *
 * \param left left motor speed (units: encoder)
 * \param right right motor speed (units: encoder)
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * 
 * \return A value:
 *       - <0 on error
 *       - >=0 on success
 */
int kh4_set_speed(int left,int right, knet_dev_t *hDev)
{
  int rc;
  char outbuf[4];

  if(hDev)
  {
		outbuf[0]=left & 0xff; 
		outbuf[1]=(left & 0xff00)>>8; 
		outbuf[2]=right & 0xff;
		outbuf[3]=(right & 0xff00)>>8;

		
		if (currentRegType==kh4RegSOpenLoop)		
		{	// openloop : 2940 units = 100 %)
			return kh4_sendcommand(hDev,I2C_MOT_LEFT_PWM_CONSIGN_L,outbuf,4);
				
		}	
		else
		{
			return kh4_sendcommand(hDev,I2C_MOT_LEFT_SPEED_CONSIGN_L,outbuf,4);	
		}
  }
  return KB_ERROR_KH4KBNOTINIT;

}

/*!
 * kh4_SetStatusLeds set the status leds
 *
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * \param status  0x01 red on, 0x10 green on, 0x00 both on, 0x11 both off
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success 
 */
int kh4_SetStatusLeds(char status,knet_dev_t *hDev)
{
  char outbuf[1];
  if(hDev)
  {	
  	
  	outbuf[0]=status;	
		return kh4_sendcommand(hDev,I2C_LED,outbuf,1);	
  }
  return KB_ERROR_KH4KBNOTINIT; 
}

/*!
 * kh4_ToggleStatusLed set the status led
 *
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * \param led to toggle: 1 red, 0 green
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success 
 */
int kh4_ToggleStatusLed(int led,knet_dev_t *hDev)
{
  char outbuf[1];
  if(hDev)
  {	
  	kh4_getcommand( hDev,I2C_LED, outbuf, 1);
  	outbuf[0]=led?(0x20 | (outbuf[0] & 0x0F)):(0x02 | (outbuf[0] & 0xF0)); // keep other led status as before
		return kh4_sendcommand(hDev,I2C_LED,outbuf,1);	
  }
  return KB_ERROR_KH4KBNOTINIT; 
}

/*!
 * kh4_SetRGBLeds set the RGB leds
 *
 * \param hDev is a handle to an openned knet socket (Khepera4:dsPic).
 * \param left_R left led, R color on 6 bits
 * \param left_G left led, G color on 6 bits
 * \param left_B left led, B color on 6 bits
 * \param right_R right led, R color on 6 bits
 * \param right_G right led, G color on 6 bits
 * \param right_B right led, B color on 6 bits
 * \param back_R back led, R color on 6 bits
 * \param back_G back led, G color on 6 bits
 * \param back_B back led, B color on 6 bits
 *
 * \return A value:
 *       - <0 on error
 *       - >=0 on success 
 */
int kh4_SetRGBLeds(char left_R,char left_G,char left_B,char right_R,char right_G,char right_B,char back_R,char back_G,char back_B ,knet_dev_t *hDev)
{
  char outbuf[9];
  if(hDev)
  {		
  	outbuf[0]=left_R & 0x3F;	// on 6 first bits only
  	outbuf[1]=left_G& 0x3F;
  	outbuf[2]=left_B& 0x3F;
  	outbuf[3]=right_R& 0x3F;	
  	outbuf[4]=right_G& 0x3F;
  	outbuf[5]=right_B& 0x3F;
  	outbuf[6]=back_R& 0x3F;	
  	outbuf[7]=back_G& 0x3F;
  	outbuf[8]=back_B& 0x3F;
		return kh4_sendcommand(hDev,I2C_RGB_FL_R,outbuf,9);	
  }
  return KB_ERROR_KH4KBNOTINIT; 
}
