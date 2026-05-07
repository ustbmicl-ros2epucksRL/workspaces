
#ifndef __lrf__
#define __lrf__

/* ***************************************************************************************** */
/* ** LRF library                                                                         ** */
/* ***************************************************************************************** */



#define LRF_DATA_NB 682 // number of LRF data

// Array contains distance data after successfull call of LRF_GetDistances
extern long kb_lrf_DistanceData[LRF_DATA_NB];		
						 


/* ***************************************************************************************** */
/* ** Prototypes                                                                          ** */
/* ***************************************************************************************** */

extern int kb_lrf_Init(char *LRF_DeviceName);		// returns LRF_DeviceHandle
                                  
extern int kb_lrf_GetDistances(int LRF_DeviceHandle);	// return <0 on failure, 0 on success

extern int kb_lrf_GetDistances_Averaged(int LRF_DeviceHandle, int average);	// return <0 on failure, 0 on success

extern void kb_lrf_Power_On(void);
extern void kb_lrf_Power_Off(void);

extern void kb_lrf_Laser_On(int LRF_DeviceHandle);
extern void kb_lrf_Laser_Off(int LRF_DeviceHandle);


extern void kb_lrf_Close(int LRF_DeviceHandle);		// no return value

extern long kb_lrf_Get_Timestamp(void);

#endif /* __lrf__ */
