
#ifndef __kb_stargazer__
#define __kb_stargazer__

/* ***************************************************************************************** */
/* ** Stargazer library                                                                   ** */
/* ***************************************************************************************** */

#include "khepera.h" 

// landmark types
#define NB_MARK_TYPES 6
#define HLD1S 0
#define HLD1L 1
#define HLD2S	2				 
#define HLD2L 3
#define HLD3S 4
#define HLD3L 5
extern const char *kb_gazer_landmark_types[];

// landmark modes
#define NB_MARK_MODES 2
#define MARK_ALONE 0
#define MARK_MAP 1
extern const char *kb_gazer_landmark_modes[];


// height fix
#define NB_HEIGHT_FIX_MODES 2
#define HEIGHT_FIX_NO 0
#define HEIGHT_FIX_YES 1
extern const char *kb_gazer_height_fix_modes[];

#define ANGLE_CORRECTION 100.0 // angle correction

#define CALIB_STDEV_MAX 5.0 // maximum standard deviation before calibration error [cm]


/* ***************************************************************************************** */
/* ** Prototypes                                                                          ** */
/* ***************************************************************************************** */

extern int kb_stargazer_init(char *DeviceName); // initialisation of the Stargazer

extern int kb_gazer_get_version(char *version); // return firmware version of the Stargazer

extern int kb_gazer_set_landmark_number(int number); // Set the number of landmarks
extern int kb_gazer_get_landmark_number(int *number); // Get the number of landmarks

extern int kb_gazer_set_ref_id(int refid); // Set the landmark id as reference
extern int kb_gazer_get_ref_id(int *refid); // Get the landmark id as reference

extern int kb_gazer_set_landmark_type(int type); // Set the landmark type
extern int kb_gazer_get_landmark_type(int *type); // Get the landmark type

extern int kb_gazer_set_landmark_mode(int mode); // Set the landmark mode
extern int kb_gazer_get_landmark_mode(int *mode); // Get the landmark mode

extern int kb_gazer_set_height_fix_mode(int mode); // Set the height fix mode
extern int kb_gazer_get_height_fix_mode(int *mode); // Get the height fix mode

extern int kb_gazer_start_map_mode(); // Start the map building mode

extern int kb_gazer_set_end_command(); // Set end of commands for update

extern int kb_gazer_start_computation(); // Start computation of position

extern int kb_gazer_stop_computation();  // Stop computation of position

extern int kb_gazer_wait_stop_computation(); //  Stop computation of position and wait until stopped

extern int kb_stargazer_read_data(double *x,double *y,double *z,double *angle, int *idnum, char *mode, int corr); // read data

extern void kb_stargazer_Close(); // release the Stargazer

// calibrate the Stargazer
extern int kb_gazer_calibration(knet_dev_t * dsPic,double *_center_x0,double *_center_y0,double *_angle_rot,double *_a_axis, double *_b_axis, double *_stddev_x, double *_stddev_y);

#endif /*  #ifndef __kb_stargazer__ */
