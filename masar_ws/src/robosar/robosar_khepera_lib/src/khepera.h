/*--------------------------------------------------------------------
 * khepera.h - Khepera 4 Library	
 *--------------------------------------------------------------------
 * $Id: khepera.h,v 1.0 2014/02/13 08:40:58 jtharin Exp $
 *--------------------------------------------------------------------
 * $Author: flambercy $
 * $Date: 2014/02/13 08:40:58 $
 * $Revision: 1.0 $
 *--------------------------------------------------------------------*/ 

#ifndef __khepera__
#define __khepera__

#ifdef __cplusplus
extern "C"{
#endif

/* see LIBVER environment variable defined Makefile for version */
#define KB_VERSION  2
#define KB_REVISION 1


#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <termio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <getopt.h>

#include "kb_init.h"
#include "kb_error.h"
#include "kb_memory.h"
#include "kb_time.h"
#include "kb_cmdparser.h"
#include "kb_symbol.h"
#include "kb_config.h"
#include "kb_socket.h"
#include "kb_sound.h"
//#include "kb_wav.h"
#include "kb_gpio.h"
#include "kb_pwm.h"
#include "kb_lrf.h"
#include "kb_gripper.h"
#include "kb_utils.h"
#include "kb_stargazer.h"
#include "kb_camera.h"
#include "kb_khepera4.h"


#include "knet.h"

#include "kmot.h"
#include "koreio.h"

#ifdef __cplusplus
}
#endif

#endif /* __khepera__ */
