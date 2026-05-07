/*-------------------------------------------------------------------------------
 * Project: KoreBot Library	
 * $Author: flambercy $
 * $Date: 2006/01/26 14:50:58 $
 * $Revision: 1.2 $
 * 
 * 
 * $Header: /home/cvs/libkhepera/src/kb_gpio.c,v 1.2 2006/01/26 14:50:58 flambercy Exp $
 */

#include <sys/mman.h>
#include "khepera.h"
#include "user-gpio-drv.h"

/*! 
 * \file   kb_gpio.c Support for KoreBot General Purpose IO.             
 *
 * \brief 
 *         This module provides useful basic facilities to use the
 *         KoreBot GPIO and configure them.
 *
 * \author   Julien Tharin (K-Team SA)
 *
 * \note     Copyright (C) 2012 K-TEAM SA
 * \bug      none discovered.                                         
 * \todo     nothing.
 */


static int  gFd = -1; // file descriptor

/* ---- Internal Functions ----------------------------------------------- */
/*!
 * Request a GPIO
 *
 * \param gpio gpio requested
 * \param label name to set to the gpio
 *
 * \return
 * 	- 0 no error occured
 * 	- an error code if the GPIO is invalid
 */
int gpio_request( unsigned gpio, const char *label )
{
  GPIO_Request_t  request;
	
  request.gpio = gpio;
  strncpy( request.label, label, sizeof( request.label ));
  request.label[ sizeof( request.label ) - 1 ] = '\0';

  if ( ioctl( gFd, GPIO_IOCTL_REQUEST, &request ) != 0 )
  {
      return -errno;
  }
  return 0;
}



/*!
 * Release GPIO
 *
 * \param gpio gpio to be released
 *
 * \return
 * 	- 0 no error occured
 * 	- an error code if the GPIO is invalid
*/
int gpio_free( unsigned gpio )
{
   
  if ( ioctl( gFd, GPIO_IOCTL_FREE, gpio )!= 0 )
  {
      return -errno;
  }
  return 0;
}

/* ---- Exported Functions ----------------------------------------------- */

/*!
 * Initialize the GPIO module. This function must be called before any
 * other call the gpio functions.
 *
 * \return 
 *  - 0 device open successfully
 * - -1 gpio device couldn't be open
 *  -i where i is the gpio that could not be requested
 */
 
int kb_gpio_init()
{
	unsigned int i;
	
	char label[10];
	
  if ( gFd < 0 )
  {
      if (( gFd = open( "/dev/user-gpio", O_RDWR )) < 0 )
      {
        return -1;
      }
	
	
		// request GPIO for /MUTE (it unmutes because, sets it to 0 at request initialisation)
		sprintf(label,"GPIO_%d",GPIO_SMUTE);
		if(gpio_request( GPIO_SMUTE, label ))
		{
		
			// perhaps already taken
			gpio_free(GPIO_SMUTE);	// release GPIO for /MUTE

			if(gpio_request( GPIO_SMUTE, label ))
			{
		
				KB_ERROR("kb_gpio_init",KB_ERROR_INVALID,"Could not request GPIO 64 for /MUTE!");
				close(gFd);
				gFd = -1;
				return -GPIO_SMUTE;
			} else{
				kb_gpio_dir(GPIO_SMUTE,0);		
			}
				
		}
		else
		{
			kb_gpio_dir(GPIO_SMUTE,0);
		}
		
	
		// initialize the gpio Korebot/ compatible
		for (i=GPIO_FIRST;i<=GPIO_LAST;i++)
		{	
	
			sprintf(label,"GPIO_%d",i);
			if (gpio_request( i, label )<0)
			{
				gpio_free(i); // perhaps already taken: free it
				
				if (gpio_request( i, label )<0) // and request again
				{		
					KB_ERROR("kb_gpio_init",KB_ERROR_INVALID,label);
					close(gFd);
					gFd = -1;
					return -i;
				}
				
			}	
		}

  }

  return 0;
}

/*!
 * Cleanup the GPIO module. Must be called to free the module ressources.
 */
int kb_gpio_cleanup()
{
	unsigned int i;
	
  if ( gFd >= 0 )
  {
  
  	gpio_free(GPIO_SMUTE);	// release GPIO for /MUTE

		// release the gpio Korebot/ compatible
		for (i=GPIO_FIRST;i<=GPIO_LAST;i++)
		{	
			
			if (gpio_free(i)<0)
			{
				KB_ERROR("kb_gpio_free",KB_ERROR_INVAL);	
				
			}	
		}

	 	// release device
    close( gFd );
    gFd = -1;
		return 0;
  }
  
  return -1;
}

int kb_gpio_mode()
{
}

/*!
 * Configure the given GPIO pin function. Check first if the pin is freely
 * available on the KoreBot.
 *
 * \param gpio A gpio number between 0 and 84
 * \param function 	0:IO mode 
 * 			1:Alternate function 1 
 * 			2:Alternate function 2
 * 			3:Alternate function 3
 */
void kb_gpio_function(unsigned gpio, unsigned function)
{
	// not used; GPIO_FIRST to GPIO_LAST are always in GPIO mode
}

/*!
 * Configure the given GPIO as input or output. The GPIO function must be 
 * configured as well to use the pin as an IO.
 * 
 * \param gpio A gpio number between 0 and 84
 * \param dir The direction 0:output (initial value=0) 1:input
 *
 * \return
 * 	- 0 no error occured
 * 	- an error code if the GPIO is invalid
 */
int kb_gpio_dir(unsigned gpio, unsigned dir)
{
	GPIO_Value_t    setDirOutput;
	
	
	
	if(((gpio > GPIO_LAST) && (gpio != GPIO_KH4_RESET)) || ((gpio < GPIO_FIRST) && ((gpio != GPIO_SMUTE)&&(gpio != GPIO_KH4_CHARGE))))
    KB_ERROR("kb_gpio_dir",KB_ERROR_OUTOFRANGE,gpio,"GPIO",GPIO_FIRST,GPIO_LAST);
	
	
	if (dir)
	{
  	if ( ioctl( gFd, GPIO_IOCTL_DIRECTION_INPUT, gpio ) != 0 )
  	{
	    return -errno;
	  }
	}
	else
	{  

		setDirOutput.gpio = gpio;
		setDirOutput.value = 0;

		if ( ioctl( gFd, GPIO_IOCTL_DIRECTION_OUTPUT, &setDirOutput ) != 0 )
		{
		    return -errno;
		}
  }
 
	return 0;
}


/*!
 * Configure the given GPIO as input or output (with initial value). The GPIO function must be 
 * configured as well to use the pin as an IO.
 * 
 * \param gpio A gpio number between 0 and 84
 * \param dir The direction 0:output 1:input
 * \param value Initial output value
 *
 * \return
 * 	- 0 no error occured
 * 	- an error code if the GPIO is invalid
 */
int kb_gpio_dir_val(unsigned gpio, unsigned dir, unsigned value)
{
	GPIO_Value_t    setDirOutput;
	
	
	
	if(((gpio > GPIO_LAST) && (gpio != GPIO_KH4_RESET)) || ((gpio < GPIO_FIRST) && ((gpio != GPIO_SMUTE)&&(gpio != GPIO_KH4_CHARGE))))
    KB_ERROR("kb_gpio_dir",KB_ERROR_OUTOFRANGE,gpio,"GPIO",GPIO_FIRST,GPIO_LAST);
	
	
	if (dir)
	{
  	if ( ioctl( gFd, GPIO_IOCTL_DIRECTION_INPUT, gpio ) != 0 )
  	{
	    return -errno;
	  }
	}
	else
	{  

		setDirOutput.gpio = gpio;
		setDirOutput.value = value;

		if ( ioctl( gFd, GPIO_IOCTL_DIRECTION_OUTPUT, &setDirOutput ) != 0 )
		{
		    return -errno;
		}
  }
 
	return 0;
}

/*! 
 * Set a given GPIO. The GPIO must be first configured to ouput mode.
 * If the GPIO is configured as input, it will be set when 
 * configured as ouput.
 *
 * \param gpio A gpio number between 0 and 84
 */
void kb_gpio_set(unsigned gpio)
{
  GPIO_Value_t    setValue;
  
	if(((gpio > GPIO_LAST) && (gpio != GPIO_KH4_RESET)) || ((gpio < GPIO_FIRST) && ((gpio != GPIO_SMUTE)&&(gpio != GPIO_KH4_CHARGE))))
    KB_ERROR("kb_gpio_set",KB_ERROR_OUTOFRANGE,gpio,"GPIO",GPIO_FIRST,GPIO_LAST);

	
  setValue.gpio = gpio;
  setValue.value = 1;

  ioctl( gFd, GPIO_IOCTL_SET_VALUE, &setValue );
}

/*! 
 * Clear a given GPIO. The GPIO must be first configured to ouput mode.
 * If the GPIO is configured as input, it will be cleared when 
 * configured as ouput.
 *
 * \param gpio A gpio number between 0 and 84
 */
void kb_gpio_clear(unsigned gpio)
{
  GPIO_Value_t    setValue;
  
	if(((gpio > GPIO_LAST) && (gpio != GPIO_KH4_RESET)) || ((gpio < GPIO_FIRST) && ((gpio != GPIO_SMUTE)&&(gpio != GPIO_KH4_CHARGE))))
    KB_ERROR("kb_gpio_clear",KB_ERROR_OUTOFRANGE,gpio,"GPIO",GPIO_FIRST,GPIO_LAST);

  setValue.gpio = gpio;
  setValue.value = 0;

  ioctl( gFd, GPIO_IOCTL_SET_VALUE, &setValue );
}

/*!
 * Get the current level for a given GPIO. It can be used regardless 
 * of the current GPIO configuration.
 *
 * \param gpio A gpio number between 0 and 84
 *
 * \return 
 * 	- 1 if the GPIO is set
 * 	- 0 if the GPIO is cleared
 * 	- a negative error code if the GPIO is invalid
 */
int kb_gpio_get(unsigned gpio)
{
  GPIO_Value_t    getValue;
  
	if(((gpio > GPIO_LAST) && (gpio != GPIO_KH4_RESET)) || ((gpio < GPIO_FIRST) && ((gpio != GPIO_SMUTE)&&(gpio != GPIO_KH4_CHARGE))))
    KB_ERROR("kb_gpio_get",KB_ERROR_OUTOFRANGE,gpio,"GPIO",GPIO_FIRST,GPIO_LAST);

    

  getValue.gpio = gpio;

  if ( ioctl( gFd, GPIO_IOCTL_GET_VALUE, &getValue ) != 0 )
  {
      return -errno;
  }
  return getValue.value;
}
