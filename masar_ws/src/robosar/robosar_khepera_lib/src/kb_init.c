/*--------------------------------------------------------------------
 * kb_init.c - KoreBot Library -  Initialization
 *--------------------------------------------------------------------
 * $Id: kb_init.c,v 1.17 2011/11/03 09:45:14 jtharin Exp $
 *--------------------------------------------------------------------
 * $Author: pbureau $
 * $Revision: 1.17 $
 * $Date: 2011/11/03 09:45:14 $
 *--------------------------------------------------------------------*/

#include "khepera.h"

static int kb_init_done = 0;

struct option long_opts[] = {
  { "debug" ,   1 , 0 , 'd' } ,
  { "version" , 0 , 0 , 'v' } ,
  { NULL      ,    0 , 0 , 0    }
};


char short_opts[] = "d:v";

/*--------------------------------------------------------------------*/
/*! This function initializes the KoreBot Library
 * 
 */
int kb_init( int argc , char * argv[] )
{
  int opt, rc;

  if ( kb_init_done == 0 ) {

    for (;;) {
		  opt = getopt_long( argc , argv , short_opts , long_opts , NULL );
		  if ( opt == -1 )
				break;
				  
			switch( opt ) {
					/* --kb-debug <level> */
					case 'd':
						kb_set_debug_level(atoi(optarg));
						break;

					/* --kb-version */
					case 'v':
						kb_msg( "libkhepera version %u.%u [%s  %s]\n" , 
						KB_VERSION , KB_REVISION , __TIME__,__DATE__ );
						break;

					default:
						break;
			}
    }
      
    if ((rc = kb_config_init( argc, argv )) < 0 ) {
    	fprintf(stderr,"\nkb_config_init ERROR!\n\n");
      return rc;
    }
    
    if ((rc = knet_init( argc , argv )) <  0) {
    	fprintf(stderr,"\nknet_init ERROR!\n\n");
      return rc;
    }
      
   
		if ((rc = kb_pwm_init()) <  0) {
			fprintf(stderr,"\nkb_pwm_init ERROR!\n\n");
		 return rc;
		}
      
		if ((rc = kb_gpio_init()) <  0) {
   	 fprintf(stderr,"\nkb_gpio_init ERROR!\n\n");	
     return rc;
    }
    
    atexit( kb_exit );
    
    kb_init_done = 1;
  }

  return 1;
}

/*--------------------------------------------------------------------*/
/*! This function is called automatically on exit and call the 'exit'
 *  of all sub layer. 
 *
 * \remark This function is called automatically at the terminaison 
 *         of the application.
 */
void kb_exit(void)
{
  if ( kb_init_done ) {
    knet_exit();
    kb_config_exit();
    kb_gpio_cleanup();
    kb_pwm_cleanup();
    kb_init_done = 0;
  }
}
