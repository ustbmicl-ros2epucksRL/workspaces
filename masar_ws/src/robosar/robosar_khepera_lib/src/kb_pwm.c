/*-------------------------------------------------------------------------------
 * Project: Khepera Library	
 * $Author: pbureau $
 * $Date: 2006/10/27 08:53:21 $
 * $Revision: 1.2 $
 * 
 * 
 * $Header: /home/cvs/libkhepera/src/kb_pwm.c,v 1.2 2006/10/27 08:53:21 pbureau Exp $
 */


#include "khepera.h"
//#include "overo-pwm_ioctl.h"
#include "pwm_ioctl.h"


/*! 
 * \file   kb_pwm.c Support for Khepera General Purpose IO.             
 *
 * \brief 
 *         This module provides useful basic facilities to use the
 *         Khepera PWM and configure them.
 *
 * \author   Julien Tharin (K-Team SA)
 *
 * \note     Copyright (C) 2012 K-TEAM SA
 * \bug      none discovered.                                         
 * \todo     nothing.
 */

//#define DEBUG

/* ---- Private Variables ------------------------------------------------ */

#define NUMBER_OF_PWM 2

static int  gFd[NUMBER_OF_PWM]= {-1,-1}; // file device descriptors

static int  pwmi[NUMBER_OF_PWM]= {10,9}; // pwm real numbers



/* ---- Internal Functions ----------------------------------------------- */
/*!
 * Request a PWM
 *
 * \param pwm pwm requested
 *
 * \return
 * 	- 0 no error occured
 * 	- an error code if the PWM is invalid
 */
int  pwm_request( unsigned pwm)
{
   /* if ( ioctl( gFd[pwm], PWM_IOCTL_REQUEST) != 0 )
    {
    	#ifdef DEBUG
			printf("Error pwm_request() pwm %d, \r\n  -> errno %d : error: %s\r\n",pwm,errno,strerror(errno));
			#endif
    
      return -errno;
    }*/
    
    #ifdef DEBUG
    //else
    {
			printf("pwm_request() pwm %d\r\n",pwm);
		}
		#endif
    
    
    return 0;
}


/*!
 * Release a PWM
 *
 * \param pwm pwm to be released
 *
*/
void pwm_free( unsigned pwm )
{
	#ifdef DEBUG
	printf("pwm_free() pwm %d\r\n",pwm);
	#endif
  //ioctl( gFd[pwm], PWM_IOCTL_FREE, pwm );
}

/* ---- Exported Functions  ------------------------------------------------------- */


/*!
 * This function initialises the PWMs.
 *
 *
 * \return A value indicating if the device could be open:
 *         - 0 devices open
 *				 - -1 /dev/pwm0 couldn't be open
 *				 - -2 /dev/pwm1 couldn't be open
 *				 - -3 pwm0 couldn't be initialiased
 *				 - -4 pwm1 couldn't be initialiased
 */
int kb_pwm_init()
{
  char devstr[12];

  int pwm,ret;

  for (pwm=0;pwm<NUMBER_OF_PWM;pwm++)
  {
  	
   
    if ( gFd[pwm] < 0 )
    {
			#ifdef DEBUG
			printf("kb_pwm_init: Opening pwm %d\r\n",pwm);
			#endif
			
      sprintf(devstr,"/dev/pwm%d",pwmi[pwm]);

      if (( gFd[pwm] = open( devstr, O_RDWR )) < 0 )
      {
      	#ifdef DEBUG
				printf("kb_pwm_init: open error %d\r\n",gFd[pwm]);
				#endif
        return -(pwm)-1;
      }
      
      if(ret=pwm_request(pwm)<0)
      {
      	#ifdef DEBUG
				printf("kb_pwm_init: pwm_request error %d\r\n",ret);
				#endif
				gFd[pwm]=-1;
      	return -(pwm)-3;
      }
     
      
    }
  }

  return 0;
}



/*!
 * Initialize the two PWM module. This function must be called before any
 * other call the pwm functions.
 *
 * \return 
 * 	- 0 on success
 *	- a negative error code on failure
 */
int kb_pwm_cleanup()
{
  int pwm;

  for (pwm=0;pwm<NUMBER_OF_PWM;pwm++)
  {
    if ( gFd[pwm] >= 0 )
    {
        #ifdef DEBUG
				printf("kb_pwm_cleanup: Closing pwm %d\r\n",pwm);
				#endif
        pwm_free(pwm);
     		// release device
        close( gFd[pwm] );
        gFd[pwm] = -1;

    }
  }
  
  return 0;
}



/*! CURRENTLY NOT IMPLEMENTED
 * Activate the PWM. 
 *
 * \param pwm pwm to set (0 or 1)
 *
 * \return
 * 	- 0 if OK
 * 	- an error code if the PWM couldn't be set
 */
int kb_pwm_activate(unsigned pwm)
{
    /*if ( ioctl( gFd[pwm], PWM_IOCTL_ON, pwm ) != 0 )
    {
        return -errno;
    }*/
    return 0;
}

/*!
 * Desactivate the PWM.
 *
 * \param pwm pwm to set (0 or 1)
 *
 * \return
 * 	- 0 if OK
 * 	- an error code if the PWM couldn't be set
 */
int kb_pwm_desactivate(unsigned pwm)
{
    /*if ( ioctl( gFd[pwm], PWM_IOCTL_OFF, pwm ) != 0 )
    {
        return -errno;
    }*/
    if ( ioctl( gFd[pwm], PWM_PULSE_RESET) != 0 )
    {
        return -errno;
    }
    
    return 0;
}



/*!
 * Set the duty cyle to the PWM
 * must be called after setting the frequency; otherwise the previous frequency will be used!
 *
 * \param pwm pwm to set  (0 or 1)
 * \param duty duty cycle in %
 * \return
 * 	- 0 if OK
 * 	- an error code if the PWM couldn't be set
 */
int kb_pwm_duty(unsigned pwm, unsigned duty)
{

    if ( ioctl( gFd[pwm], PWM_PULSE_SET, duty ) != 0 )
    {
        return -errno;
    }
    return 0;
}


/*! CURRENTLY NOT IMPLEMENTED
 * Set the frequency to the PWM; must be set before the duty cycle!
 * 
 *
 * \param pwm pwm to set (0 or 1)
 * \param frequency frequency cycle in Hz
 * \return
 * 	- 0 if OK

 * 	- an error code if the PWM couldn't be set
 */
int kb_pwm_period(unsigned pwm, unsigned frequency)
{
 /*   PWM_Value_t    value;

    value.frequency = frequency;
    value.pwm = pwm;

    if ( ioctl( gFd[pwm], PWM_IOCTL_SET_FREQUENCY, &value ) != 0 )
    {
        return -errno;
    }*/
    return 0;
}
