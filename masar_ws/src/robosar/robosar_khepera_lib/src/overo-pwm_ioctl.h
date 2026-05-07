/*
 Copyright (c) 2010-2012, Scott Ellis
 All rights reserved.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#ifndef PWM_IOCTL_H
#define PWM_IOCTL_H

#include <linux/ioctl.h>

#define PWM_IOC_MAGIC  0x6b

#define PWM_CMD_ON	                   0x6b02
#define PWM_CMD_OFF							       0x6b03
#define PWM_CMD_SET_FREQUENCY          0x6b04
#define PWM_CMD_SET_DUTY							 0x6b05

typedef struct
{
    unsigned    pwm;
    int         frequency;
    int 				duty;

} PWM_Value_t;

#define PWM_IOCTL_ON        		_IO( PWM_IOC_MAGIC, PWM_CMD_ON )  /* arg is int */
#define PWM_IOCTL_OFF						_IO( PWM_IOC_MAGIC, PWM_CMD_OFF ) /* arg is int */

#define PWM_IOCTL_SET_FREQUENCY	_IOW( PWM_IOC_MAGIC, PWM_CMD_SET_FREQUENCY,int )   /* arg is PWM_Value_t * */
#define PWM_IOCTL_SET_DUTY      _IOW( PWM_IOC_MAGIC, PWM_CMD_SET_DUTY, int ) /* arg is PWM_Value_t * */


#define PWM_IOC_MAXNR 3

#endif
