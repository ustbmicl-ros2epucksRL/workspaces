/*! 
 * \file   kb_stargazer.c Support for KoreBot Stargazer             
 *
 * \brief 
 *         This module provides useful basic facilities to use the
 *         KoreBot with the Stargazer sensor
 *
 * \author  J. Tharin  (K-Team SA)
 *
 * \note     Copyright (C) 2012 K-TEAM SA
 * \bug      none discovered.                                         
 * \todo     
 */

#include "kb_stargazer.h"
#include "khepera.h" 

#include <termio.h>
#include <math.h>

// serial port where the Stargazer is connected
#define PORT_NAME "/dev/ttyO0"

// port of the battery on/off switch
#define BATTERY_PORT 0  // PWM 0

// Stargazer commands
#define CMD_CALC_STOP "~#CalcStop`"
#define CMD_CALC_STOP_ACK "~!CalcStop`"
#define CMD_CALC_START "~#CalcStart`"
#define CMD_CALC_START_ACK "~!CalcStart`"
#define CMD_VERSION "~@Version`"
#define CMD_VERSION_ACK "~$Version|"
#define CMD_DEADZONE_ACK "DeadZone"
#define CMD_MAPID_ACK "MAPID"

// commands for map building
#define CMD_LANDMARK_NB "IDNum"
#define CMD_REFERENCE_ID "RefID"
#define CMD_HEIGHT_FIX "HeightFix"
#define CMD_MARK_TYPE "MarkType"
#define CMD_MARK_MODE "MarkMode"
#define CMD_SET_END "SetEnd"
#define CMD_PARAM_UPDATE "~!ParameterUpdate`"
#define CMD_MAP_MODE_START "~#MapMode|Start`"

#define ACK_CHAR '!'

static int SerialDeviceHandle=-1; // handle to the port where the sensor is connected



const char *kb_gazer_landmark_types[] = {"HLD1S","HLD1L","HLD2S","HLD2L","HLD3S","HLD3L"};
const char *kb_gazer_landmark_modes[] = {"Alone","Map"};
const char *kb_gazer_height_fix_modes[] = {"No","Yes"};


#define MAX_STOP 10 // number times of tyring to stop the Stargazer of sending data

#define MAX_BUF 2048 // buffer size for char array

// calibration variables
double a_axis,b_axis,center_x0,center_y0,angle_rot;

// for debugging (if defined)
//#define DEBUG 1
//#define DEBUG_FULL 1
//#define DEBUG_ERROR 1

#ifdef DEBUG
static FILE *debug_file=NULL; // debug file pointer
#endif


// types of fitting
#define BOOKSTEIN  0
#define TAUBIN  1
#define FPF  2


// calibration parameters
#define CONFIGURE_TIMEOUT 60		// timeout until calibration error [s]
#define STOP_THRESHOLD_ANGLE 5	// difference threshold of angle when the robot stop after overpassing it
#define CALIBRATION_SPEED 40  // calibration speed
#define MAX_CALIB_DATA 2048 		// maximum number of data for calibration

/*****************************************************************************/
/* Internal functions ********************************************************/
/*****************************************************************************/

/*! write text to file fd
 *
 *  \param fd file descriptor
 *  \param text char array to write
 *
 *  \return none 
 */
void write_serial(int fd, char *text)
{
	int i;

	for(i = 0; i < (int)strlen(text) ; i++) {
		write( fd , &text[i], 1 );	
		usleep(30000); // needed between EACH char (see Stargazer User's Guide)
	}


}

/*****************************************************************************/
/*! Open serial port
 *
 *  \param fd file descriptor
 *  \param name serial device name
 *
 *  \return 0 OK, <0 ERROR 
 */
int openserial(int *fd, const char *name)
{
	struct termios tios;


	if ((*fd=open(name,O_RDWR | O_NOCTTY | O_NDELAY))<0) {
     return -1;
  }

  tcgetattr(*fd , &tios);
  tios.c_cflag = ( CS8 | CLOCAL | CREAD | B115200 );
  tios.c_iflag = IGNPAR;
  tios.c_oflag = 0;
  tios.c_lflag = 0;
  tios.c_cc[VMIN] = 0;
  tios.c_cc[VTIME] = 0;
  tcsetattr( *fd , TCSANOW , &tios );
  
  fcntl(*fd, F_SETFL, 0);

	#ifdef DEBUG
	debug_file=fopen("debug.dat","w");
	#endif
  return 0;
}


// 
/*****************************************************************************/
/*!
 * compute rotation in matrix A of Aij and Akl:
 *
 * \param A input/ouput matrix pointer to be rotated
 * \param i index   
 * \param j index
 * \param k index   
 * \param l index
 * \param tau parameter
 * \param s parameter 
 * \param n matrix size  
 *
 */
void rotate(double *A, int i, int j, int k, int l, double tau, double s,int n) 
{
	double g,h;
	g=A[i*(n+1)+j];
	h=A[k*(n+1)+l];
	A[i*(n+1)+j]=g-s*(h+g*tau);
	A[k*(n+1)+l]=h+s*(g-h*tau);
 }

/*****************************************************************************/
/*!
 *  Perform Jacobi eigenvalue algorithm 
 * 
 * \param a input matrix pointer
 * \param n size of the matrix 
 * \param d eigenvalue matrix pointer
 * \param v output eigenvalue vector pointer
 * \param nrot number of rotation
 * 
 * \return 0 Ok, -1 : Too many iterations
 *
*/
int jacobi(double *a, int n, double *d , double *v, int *nrot)      
{
	int j,iq,ip,i;
	double tresh,theta,tau,t,sm,s,h,g,c;

	double b[6+1]={0};
	double z[6+1]={0};

	if (n>6)
		return;

	for (ip=1;ip<=n;ip++) {
		for (iq=1;iq<=n;iq++) v[ip*(n+1)+iq]=0.0;
		v[ip*(n+1)+ip]=1.0;
	}
	
	for (ip=1;ip<=n;ip++) {
		b[ip]=d[ip]=a[ip*(n+1)+ip];
		z[ip]=0.0;
	}
	
	*nrot=0;
	for (i=1;i<=50;i++) {
		sm=0.0;
		for (ip=1;ip<=n-1;ip++) {
			for (iq=ip+1;iq<=n;iq++)
			  sm += fabs(a[ip*(n+1)+iq]);
		}
		if (sm == 0.0) {
			//   free_vector(z,1,n);
			//free_vector(b,1,n);
			return 0;
		}
		if (i < 4)
			tresh=0.2*sm/(n*n);
		else
			tresh=0.0;
		for (ip=1;ip<=n-1;ip++) {
			for (iq=ip+1;iq<=n;iq++) {
			  g=100.0*fabs(a[ip*(n+1)+iq]);
			  if (i > 4 && fabs(d[ip])+g == fabs(d[ip])
			&& fabs(d[iq])+g == fabs(d[iq]))
					a[ip*n+iq]=0.0;
				else if (fabs(a[ip*(n+1)+iq]) > tresh) {
					h=d[iq]-d[ip];
					if (fabs(h)+g == fabs(h))
						t=(a[ip*(n+1)+iq])/h;
					else {
						theta=0.5*h/(a[ip*(n+1)+iq]);
						t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
						if (theta < 0.0) t = -t;
					}
					c=1.0/sqrt(1+t*t);
					s=t*c;
					tau=s/(1.0+c);
					h=t*a[ip*(n+1)+iq];
					z[ip] -= h;
					z[iq] += h;
					d[ip] -= h;
					d[iq] += h;
					a[ip*(n+1)+iq]=0.0;
					for (j=1;j<=ip-1;j++) {
					rotate(&a[0],j,ip,j,iq,tau,s,n);
					}
				for (j=ip+1;j<=iq-1;j++) {
					rotate(&a[0],ip,j,j,iq,tau,s,n);
					}
				for (j=iq+1;j<=n;j++) {
					rotate(&a[0],ip,j,iq,j,tau,s,n);
					}
				for (j=1;j<=n;j++) {
					rotate(&v[0],j,ip,j,iq,tau,s,n);
					}
				(*nrot)++;
				}
			}
		}
		for (ip=1;ip<=n;ip++) {
			b[ip] += z[ip];
			d[ip]=b[ip];
			z[ip]=0.0;
		}
	}

	return -1;
}


/*****************************************************************************/
/*!
 *  Perform the Cholesky decomposition    
 *  the lower triangular L  such that L*L'=A  
 * 
 * \param a input matrix pointer
 * \param n size of the matrix 
 * \param l output matrix pointer
 * \return 0 Ok, else not positive definite
*/
int choldc(double *a, int n, double *l)
{
		int i,j,k;
		double sum;
		double p[6+1]; 

		if (n>6)
			return;

		for (i=1; i<=n; i++)  {
			for (j=i; j<=n; j++)  {
				for (sum=a[i*(n+1)+j],k=i-1;k>=1;k--) sum -= a[i*(n+1)+k]*a[j*(n+1)+k];
				if (i == j) {
					if (sum<=0.0)  							
					{// printf("\nA is not positive definite!");
						return -1;
					}
					else 
						p[i]=sqrt(sum);
				}
				else 
				{
					a[j*(n+1)+i]=sum/p[i];
				}
			}
		}       
		for (i=1; i<=n; i++)  
			for (j=i; j<=n; j++)  
				if (i==j)
					l[i*(n+1)+i] = p[i];
				else
				{
					l[j*(n+1)+i]=a[j*(n+1)+i];  
					l[i*(n+1)+j]=0.0;
				} 
				
	return 0;			   
}

/*****************************************************************************/
/*!
 *  Compute the inverse of the matrice TB in InvB
 *  It uses Gauss-Jordan. 
 *
 * \param TB input matrix pointer
 * \param InvB ouput matrix pointer
 * \param N size of the matrix 
 *
 * \return 0 Ok, else singular
*/
int inverse(double *TB, double *InvB, int N) {  
  int k,i,j,p,q;
  double mult;
  double D,temp;
  double maxpivot;
  int npivot;
  double B[(6+1)*(6+2)]={0};
  double A[(6+1)*(2*6+2)]={0};
  double C[(6+1)*(6+1)]={0};
  double eps = 10e-20;
    
    
  if (N>6)
  	return -2;
      
  for(k=1;k<=N;k++)
		for(j=1;j<=N;j++)
			B[k*(N+2)+j]=TB[k*(N+1)+j];
  
  

      
  for (k=1;k<=N;k++)
	{
		for (j=1;j<=N+1;j++)
			A[k*(2*N+2)+j]=B[k*(N+2)+j];

			
		for (j=N+2;j<=2*N+1;j++)
			A[k*(2*N+2)+j]=0;
		A[k*(2*N+2)+k-1+N+2]=1;
	}
  

   
  for (k=1;k<=N;k++)
	{
	  maxpivot=fabs(A[k*(2*N+2)+k]);
	  npivot=k;

	  
	  for (i=k;i<=N;i++) {

	    if (maxpivot<fabs(A[i*(2*N+2)+k]))
	    {
				maxpivot=fabs(A[i*(2*N+2)+k]);
				npivot=i;

	    }
	  }  
	  if (maxpivot>=eps)
	  {  
      if (npivot!=k)
    	for (j=k;j<=2*N+1;j++)
			{

				 temp=A[npivot*(2*N+2)+j];
				 A[npivot*(2*N+2)+j]=A[k*(2*N+2)+j];
				 A[k*(2*N+2)+j]=temp;
			} ;
				


			D=A[k*(2*N+2)+k];
			for (j=2*N+1;j>=k;j--)
				A[k*(2*N+2)+j]=A[k*(2*N+2)+j]/D;
			 

			 
			for (i=1;i<=N;i++)
			{
				if (i!=k)
				{
				 mult=A[i*(2*N+2)+k];
				 for (j=2*N+1;j>=k;j--)
					 A[i*(2*N+2)+j]=A[i*(2*N+2)+j]-mult*A[k*(2*N+2)+j] ;
				}
			}

		}
		else
		{  // printf("\n The matrix may be singular !!") ;
			return(-1);
		};
	}
  
  //   Copy result in InvB 
  for (k=1,p=1;k<=N;k++,p++)
		for (j=N+2,q=1;j<=2*N+1;j++,q++)
	 	 InvB[p*(N+1)+q]=A[k*(2*N+2)+j];
  return(0);
}   //  End of INVERSE
    

/*****************************************************************************/
/*!
 *  
*/
void AperB(double *_A, double *_B, double *_res, 
       int _righA, int _colA, int _righB, int _colB) 
{
	int p,q,l;                                      
	for (p=1;p<=_righA;p++)                        
		for (q=1;q<=_colB;q++)                        
		{ 
			_res[p*(_colB+1)+q]=0.0;                            
			for (l=1;l<=_colA;l++)                     
		  _res[p*(_colB+1)+q]=_res[p*(_colB+1)+q]+_A[p*(_colA+1)+l]*_B[l*(_colB+1)+q];  
		}                                            
}                                                 

/*****************************************************************************/
/*! 
 * 
*/
void A_TperB(double *_A, double  *_B, double *_res,
	 int _righA, int _colA, int _righB, int _colB)
{
	int p,q,l;                                      
	for (p=1;p<=_colA;p++)                        
		for (q=1;q<=_colB;q++)                        
		{ _res[p*(_colB+1)+q]=0.0;                            
			for (l=1;l<=_righA;l++)                    
				_res[p*(_colB+1)+q]=_res[p*(_colB+1)+q]+_A[l*(_colA+1)+p]*_B[l*(_colB+1)+q];  
		}                                            
}

/*****************************************************************************/
/*!
 *  
*/
void AperB_T(double *_A, double *_B, double *_res,
	 int _righA, int _colA, int _righB, int _colB)
{
	int p,q,l;                                      
	for (p=1;p<=_colA;p++)                         
		for (q=1;q<=_colB;q++)                        
		{ 
			_res[p*(_colB+1)+q]=0.0;                            
		  for (l=1;l<=_righA;l++)                    
    		_res[p*(_colB+1)+q]=_res[p*(_colB+1)+q]+_A[p*(_righA+1)+l]*_B[q*(_righA+1)+l];  
		}                                            
}

   
/*****************************************************************************/
/*! 
 * 
 * compute the best ellipse fitting data (x,y)
 * 
 * \param x x coordinates of data
 * \param y y coordinates of data
 * \param np number of data
 * \param *_x0 x of ellipse center
 * \param *_y0 y of ellipse center
 * \param *_angle angle between x axis and ellipse biggest axis
 * \param *_a half major axis of the ellipse
 * \param *_b half minor axis of the ellipse
 * 
 * \return 0 for OK, -1 error (scalar matrix while inverting)
*/
int compute_ellipse(double x[], double y[],int np,double *_x0,double *_y0,double *_angle,double *_a,double *_b)
{

		double D[MAX_CALIB_DATA+1][7]={0};
		double S[7][7]={0};
		double Const[7][7]={0};
		double temp[7][7]={0};
		double L[7][7]={0}; 
		double C[7][7]={0}; 

		double invL[7][7]={0};
		double dd[7]={0};
		double V[7][7]={0}; 
		double sol[7][7]={0};
		double tx,ty;
		int nrot=0;
		int npts=50;

		double XY[3][MAX_CALIB_DATA+1];
		double pvec[7];
		
	
	  int mode,i,j,solind;
	  
	  double mod, zero,minev;

	
		double x0,y0,a,b,c,d,f,g,a2,b2,angle;

	
		mode=BOOKSTEIN;

		switch (mode) {
			 case (FPF):
			    Const[1][3]=-2;
			    Const[2][2]=1;
			    Const[3][1]=-2;	
			    break;
			 case(TAUBIN):
			    break;
		   case(BOOKSTEIN):
			    Const[1][1]=2;
			    Const[2][2]=1;
			    Const[3][3]=2;	
			 break;
			 default:
			 break;	
		}

		if (np<6)
			return;

		// Now first fill design matrix
		for (i=1; i <= np; i++)
			{ 
			  tx = x[i-1];
			  ty = y[i-1];
			  D[i][1] = tx*tx;
			  D[i][2] = tx*ty;
			  D[i][3] = ty*ty;
			  D[i][4] = tx;
			  D[i][5] = ty;
			  D[i][6] = 1.0;
			}
	
		//	pm(&Const[0][0],"Constraint");
		
		// Now compute scatter matrix  S
		A_TperB(&D[0][0],&D[0][0],&S[0][0],np,6,np,6);
		//pm(&S[0][0],"Scatter");

		choldc(&S[0][0],6,&L[0][0]);    
		//pm(&L[0][0],"Cholesky");
	
		if (inverse(&L[0][0],&invL[0][0],6) !=0)
		{
			printf("ERROR inverting matrix L may be singular!\n");
			//pm(&L[0][0],"L");
			
			return -1;
		}
		//pm(&invL[0][0],"inverse");
	
		AperB_T(&Const[0][0],&invL[0][0],&temp[0][0],6,6,6,6);
		AperB(&invL[0][0],&temp[0][0],&C[0][0],6,6,6,6);
		//pm(&C[0][0],"The C matrix");
	
		jacobi(&C[0][0],6,&dd[0],&V[0][0],&nrot);
		//pm(&V[0][0],"The Eigenvectors");  // OK
		//pv(&dd[0],"The eigevalues");
	
		A_TperB(&invL[0][0],&V[0][0],&sol[0][0],6,6,6,6);
		//pm(&sol[0][0],"The GEV solution unnormalized");  // SOl

		// Now normalize them 
		for (j=1;j<=6;j++)  /// Scan columns
			{
			  mod = 0.0;
			  for (i=1;i<=6;i++)
			    mod += sol[i][j]*sol[i][j];
			  for (i=1;i<=6;i++)
			    sol[i][j] /=  sqrt(mod); 
			}

		//pm(&sol[0][0],"The GEV solution");  // SOl
	
		zero=10e-20;
		minev=10e+20;
		solind=0;
		
		switch (mode) {
			 case(BOOKSTEIN):  // smallest eigenvalue	 	   
			    for (i=1; i<=6; i++)
						 if (dd[i]<minev && fabs(dd[i])>zero)	
							 solind = i;
			    break;
		    case(FPF):
			    for (i=1; i<=6; i++)
						 if (dd[i]<0 && fabs(dd[i])>zero)	
							 solind = i;
		}
		        
		// Now fetch the right solution
		for (j=1;j<=6;j++)
			pvec[j] = sol[j][solind];
			
		//pv(&pvec[0],"the solution");
	
		// a*x²+2b*x*y+c*y²+2*d*x+2f*y+g=0
		g = pvec[6];
		d = pvec[4]/2;
		f = pvec[5]/2;
		a = pvec[1];
		c = pvec[3];
		b = pvec[2]/2;
	
	//	printf("equation solution: %6.5lf *x²+2* %6.5lf *x*y+ %6.5lf *y²+2* %6.5lf *x+*2* %6.5lf *y+ %6.5lf =0\n",a,b,c,d,f,g);
	
		// ellipse center
		x0=(c*d-b*f)/(b*b-a*c);
		y0=(a*f-b*d)/(b*b-a*c);
		
		// half axes
		a2=sqrt( 2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g)/((b*b-a*c)*(sqrt((a-c)*(a-c)+4*b*b)-(a+c))));
		b2=sqrt( 2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g)/((b*b-a*c)*(-sqrt((a-c)*(a-c)+4*b*b)-(a+c))));
	
	
		angle=0;	
		// angle in deg
		if((b== 0) && (a<c))
		{
			angle=0;
		} 
		else if((b==0) && (a>c))
		{
			angle= 90;
		} 
		else if((b!=0) && (a<c))
		{
			angle= (0.5*atan((2*b)/(a-c)))*180.0/M_PI;  // arccotan(z) = artan(1/z)
		}
		else if((b!=0) && (a>c))
		{
			angle= 90+(0.5*atan((2*b)/(a-c)))*180.0/M_PI;
		}
				
	
		
		*_x0=x0;
		*_y0=y0;
		*_a=a2;
		*_b=b2;
		*_angle=angle;
		
		return 0;
		
}

/*****************************************************************************/
/* Exported functions ********************************************************/
/*****************************************************************************/

/*! Get Stargazer firmware version
 *
 *  \param version version
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 						-3 : command not acknowledged
 */
int kb_gazer_get_version(char *version)
{
	int nbytes;
	char cread[MAX_BUF];
	char *dch, *ech;
	
	if (SerialDeviceHandle<0)
		return -1;
	
	write_serial(SerialDeviceHandle , CMD_VERSION  );
	

	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,cread,nbytes);
	cread[nbytes]='\0';
	
	dch = strstr(cread,CMD_VERSION_ACK);

	#ifdef DEBUG
  	printf("DEBUG: kb_gazer_get_version cread: %s\n",cread);
  #endif

	if (dch != NULL )
	{
		dch+=10;
		ech=strchr(dch,'`');
	
		if (ech != NULL)	{
		
			if (version == NULL)
			{				
				return -2;
			}
							
		  memcpy(version,dch,ech-dch);
		  version[ech-dch]='\0';
		  
		  #ifdef DEBUG
		  	printf("DEBUG: kb_gazer_get_version version: %s\n",version);
		  #endif
		  		
			return 0;		
		}	
	}	
	return -3;
}

/*****************************************************************************/
/*! Stop computation of position
 *
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 */
int kb_gazer_stop_computation()
{
	int nbytes;
	char cread[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	write_serial(SerialDeviceHandle , CMD_CALC_STOP );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,cread,nbytes);
	cread[nbytes]='\0';
	
	#ifdef DEBUG
  	printf("DEBUG: kb_gazer_stop_computation cread (nb chars: %d): %s\n",nbytes,cread);
  #endif
	
	dch = strstr(cread,CMD_CALC_STOP_ACK);

	if (dch != NULL )
	{  		
			return 0;			
	}	
	return -2;
}

/*****************************************************************************/
/*! Stop computation of position and wait until it stops (retry MAX_STOP)
 *
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 */
int kb_gazer_wait_stop_computation()
{
	int i,ret;
	
	// stop the computation and get acknowledge for testing communication
	// try maximum MAX_STOP times
	for(i=0;i<MAX_STOP;i++)
	{
		if ((ret=kb_gazer_stop_computation()) == 0)
		{
			#ifdef DEBUG
			printf("DEBUG: kb_gazer_wait_stop_computation stop after %d time(s)\n",i+1);
			#endif
			return 0;
		}
		
		usleep(100000);
	}
	
	return ret;
}


/*****************************************************************************/
/*! Start computation of position
 *
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 */
int kb_gazer_start_computation()
{
	int nbytes;
	char cread[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	write_serial(SerialDeviceHandle, CMD_CALC_START );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,cread,nbytes);
	cread[nbytes]='\0';
	
	dch = strstr(cread,CMD_CALC_START_ACK);

	if (dch == NULL )
	{  		
			return -2;		
	}	
	
	usleep(200000); // wait for the first data
	
	return 0;
}

/*****************************************************************************/
/*!
 * kb_gazer_Init does openning port, then certify the link
 *
 * \param DeviceName String name of the serial port device where the Stargazer is connected
 * 
 * \return 	0 : no error
 						-1 : error initialising gpio
 						-2 : cannot open serial port
 						-3 : cannot communicate with the Stargazer

 */
int kb_stargazer_Init(char *DeviceName)
{
  int recv_n = 0; 
  int ret=0,i;
  
  
  // start gpio for battery power on
  //if (kb_pwm_init()!=0)
  //{	
  //	return -1;
  //}
   
  // default on
  
   
  // battery power on
  //kb_pwm_activate(BATTERY_PORT);
 //kb_pwm_period(BATTERY_PORT,100);
  //kb_pwm_duty(BATTERY_PORT,100);
  
  
 // sleep(3); // wait 3.5s for the power on
 // usleep(500000);
   
  // open serial port
 	if (openserial(&SerialDeviceHandle,PORT_NAME)<0)
	{
		return -2;
	}

	if (kb_gazer_wait_stop_computation() == 0)
		return 0;
  
  return -3;
}

/*****************************************************************************/
/*! Set the number of landmarks
 *
 *  \param number number of landmarks
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 */
int kb_gazer_set_landmark_number(int number)
{
	int nbytes;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	
	sprintf(buf,"~#%s|%d`",CMD_LANDMARK_NB,number);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	
	
	buf[1]='!';
	
	dch = strstr(rep,buf);

	if (dch != NULL )
	{ 	
		return 0;			
	}	
	return -2;
}

/*****************************************************************************/
/*! Get the number of landmarks
 *
 *  \param number number of landmarks
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 */
int kb_gazer_get_landmark_number(int *number)
{
	int nbytes;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	
	sprintf(buf,"~@%s`",CMD_LANDMARK_NB);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	
	sprintf(buf,"~$%s|",CMD_LANDMARK_NB);
	
	dch = strstr(rep,buf);

	if (dch != NULL )
	{ 
		if (sscanf(dch,"~$IDNum|%d",number)==1)
		{
	 		return 0;			
	 	}	
	}	
	return -2;
}

/*****************************************************************************/
/*! Set the landmark id as reference
 *
 *  \param refid landmark id number to set as reference
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 */
int kb_gazer_set_ref_id(int refid)
{
	int nbytes;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	
	sprintf(buf,"~#%s|%d`",CMD_REFERENCE_ID,refid);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	
	
	buf[1]='!';
	
	dch = strstr(rep,buf);

	if (dch != NULL )
	{  		
		return 0;			
	}	
	return -2;
}

/*****************************************************************************/
/*! Get the landmark id reference
 *
 *  \param refid landmark id number reference
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 */
int kb_gazer_get_ref_id(int *refid)
{
	int nbytes;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	
	sprintf(buf,"~@%s`",CMD_REFERENCE_ID);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	
	sprintf(buf,"~$%s|",CMD_REFERENCE_ID);
	
	dch = strstr(rep,buf);

	if (dch != NULL )
	{ 
		if (sscanf(dch,"~$RefID|%d",refid)==1)
		{
	 		return 0;			
	 	}	
	}	
	return -2;
}

/*****************************************************************************/
/*! Set the landmark type
 *
 *  \param type landmark type
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 						-3 : type not valid	
 */
int kb_gazer_set_landmark_type(int type)
{
	int nbytes;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	
	if ((type < 0) || (type >5))
		return -3;
	
	sprintf(buf,"~#%s|%s`",CMD_MARK_TYPE,kb_gazer_landmark_types[type]);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	

	buf[1]='!';
		
	dch = strstr(rep,buf);

	if (dch != NULL )
	{  		
		return 0;			
	}	
	return -2;
}

/*****************************************************************************/
/*! Get the landmark type
 *
 *  \param type landmark type
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 						-3 : unknown landmark type
 */
int kb_gazer_get_landmark_type(int *type)
{
	int nbytes,i;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	
	sprintf(buf,"~@%s`",CMD_MARK_TYPE);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	
	sprintf(buf,"~$%s|",CMD_MARK_TYPE);
	
	dch = strstr(rep,buf);

	if (dch != NULL )
	{ 

		for (i=0;i<NB_MARK_TYPES;i++)
		{
			if (strstr(rep,kb_gazer_landmark_types[i]) != NULL)
			{
				*type=i;
				return 0;
			}
		}
	
		*type=0; // put any valid value
		return -3;
	 					

	}	
	return -2;
}

/*****************************************************************************/
/*! Set the landmark mode
 *
 *  \param mode landmark mode
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 						-3 : mode not valid	
 */
int kb_gazer_set_landmark_mode(int mode)
{
	int nbytes;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	
	if ((mode < 0) || (mode >1))
		return -3;
	
	sprintf(buf,"~#%s|%s`",CMD_MARK_MODE,kb_gazer_landmark_modes[mode]);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	

	buf[1]='!';
		
	dch = strstr(rep,buf);

	if (dch != NULL )
	{  		
		return 0;			
	}	
	return -2;
}

/*****************************************************************************/
/*! Get the landmark mode
 *
 *  \param mode landmark mode
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 						-3 : unknown landmark mode
 */
int kb_gazer_get_landmark_mode(int *mode)
{
	int nbytes,i;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	
	sprintf(buf,"~@%s`",CMD_MARK_MODE);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	
	sprintf(buf,"~$%s|",CMD_MARK_MODE);
	
	dch = strstr(rep,buf);

	if (dch != NULL )
	{ 
	
		for (i=0;i<NB_MARK_MODES;i++)
		{
			if (strstr(rep,kb_gazer_landmark_modes[i]) != NULL)
			{
				*mode=i;
				return 0;
			}
		}

		*mode=0; // put any valid value
		return -3;
	} 					
	
	return -2;
}


/*****************************************************************************/
/*! Set the height fix mode
 *
 *  \param mode height fix: 0 = NO, 1 = YES
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 						-3 : mode not valid	
 */
int kb_gazer_set_height_fix_mode(int mode)
{
	int nbytes;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	
	if ((mode < 0) || (mode >1))
		return -3;
	
	sprintf(buf,"~#%s|%s`",CMD_HEIGHT_FIX,kb_gazer_height_fix_modes[mode]);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	

	buf[1]=ACK_CHAR;
	
	
	dch = strstr(rep,buf);

	if (dch != NULL )
	{  		
		return 0;			
	}	
	return -2;
}

/*****************************************************************************/
/*! Get the height fix mode
 *
 *  \param mode height fix mode: 0 = NO, 1 = YES
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 						-3 : unknown height mode
 */
int kb_gazer_get_height_fix_mode(int *mode)
{
	int nbytes,i;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	
	sprintf(buf,"~@%s`",CMD_HEIGHT_FIX);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	
	sprintf(buf,"~$%s|",CMD_HEIGHT_FIX);
	
	dch = strstr(rep,buf);

	if (dch != NULL )
	{ 
	
		for (i=0;i<NB_HEIGHT_FIX_MODES;i++)
		{
			if (strstr(rep,kb_gazer_height_fix_modes[i]) != NULL)
			{
				*mode=i;
				return 0;
			}
		}

		*mode=0; // put any valid value
		return -3;
	} 					
	
	return -2;
}


/*****************************************************************************/
/*! Start the map building mode
 *
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : could not stop receiving position
 						-3 : command not acknowledged 
 * 						
 */
int kb_gazer_start_map_mode()
{
	int nbytes;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}

	if (kb_gazer_wait_stop_computation()!=0)
	{
		return -2;
	}
		

	sprintf(buf,"%s",CMD_MAP_MODE_START);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	

	buf[1]=ACK_CHAR;
	
	
	dch = strstr(rep,buf);

	if (dch != NULL )
	{ 	 		
		return 0;			
	}	
	return -4;
}

/*****************************************************************************/
/*! Set end of commands for update
 *
 *
 *  \return  0 : no error
 						-1 : serial port not open
 						-2 : command not acknowledged
 						-3 : data not upated
 */
int kb_gazer_set_end_command()
{
	int nbytes;
	char buf[MAX_BUF],rep[MAX_BUF];
	char *dch;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	

	sprintf(buf,"~#%s`",CMD_SET_END);
	
	write_serial(SerialDeviceHandle , buf );
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	

	buf[1]='!';
	
	dch = strstr(rep,buf);

	if (dch == NULL )
	{  		
		return -2;			
	}	
	
	
	// wait for update
	sleep(3); // 3s
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes);
	read(SerialDeviceHandle,rep,nbytes);
	rep[nbytes]='\0';
	
	dch = strstr(rep,CMD_PARAM_UPDATE);

	if (dch == NULL )
	{  		
		return -3;			
	}	
	
	
	return 0;
}

/*****************************************************************************/
/*!
 * Read data and interpret data from the Stargazer serial port
 * kb_stargazer_read_data must be called periodically (up to 10 times/s)
 *  kb_gazer_start_computation must be called once before
 *
 * \param x x position relative to the reference landmark in [cm], right direction  
 * \param y y position relative to the reference landmark in [cm], forward direction
 * \param z height to the landmark in [cm]
 * \param angle angle relative to the reference landmark orientation in [degree] (counterclockwise) in 0..360 range
 * \param idnum id number of the currently used landmark
 * \param cmode current mode: 'F' = map building mode, 'I' map mode, 'Z' height calculation mode
 * \param corr 1 = apply position correction; 0 do not apply it
 * 
 * \return 	0 : no error
 						-1 : cannot serial port not open
 						-2 : cannot communicate with the Stargazer
 						-3 : buffer overrun (try to call more often this function!)
 						-4 : read command not acknowledged
 						-5 : data error 
 						-6 : no landmark found
 						-7 : data error
 						-8 : no received data
 						-9 : mapid error
 						1  : update parameters after map building mode
 						> 1: MAPID in map building mode
 
 */
int kb_stargazer_read_data(double *x,double *y,double *z,double *angle, int *idnum, char *cmode, int corr)
{
	int nbytes,ret,id;
	static int deb=0;						// static: keep reading
	static char cread[MAX_BUF]; // static: keep reading
	char *dch;
	double xc=0,yc=0;
	
	if (SerialDeviceHandle<0)
	{
		return -1;
	}
	
	ioctl(SerialDeviceHandle, FIONREAD, &nbytes); // get number of bytes available to read
			
	if ((nbytes+deb)>=MAX_BUF)
	{
		
		tcflush(SerialDeviceHandle,TCIFLUSH); // empty serial buffer
		deb=0;
		ioctl(SerialDeviceHandle, FIONREAD, &nbytes); // get number of bytes available to read
		
		if (nbytes>=MAX_BUF)
		{		
			return -3;
		}
	}
	read(SerialDeviceHandle,cread+deb,nbytes);
	
	cread[nbytes+deb]='\0';
	#ifdef DEBUG_FULL
		kb_erase_line(4);
  	printf("DEBUG: kb_stargazer_read_data cread (nb chars: %d): %s",nbytes,cread);
  #endif

	

	if (nbytes>0)
	{			
		if (strstr(cread,CMD_DEADZONE_ACK) != NULL) 
		{	
			return -6;
		}
		else
		
		 		
		if ((strstr(cread,CMD_MAPID_ACK) != NULL) && (cread[nbytes+deb-1]=='`'))
		{
			
			if ((ret=sscanf(cread+8,"%d",&id)) != 1)
			{ 
				return -9;
			}
			
			return id;
		}
		else
	
		if (strstr(cread,CMD_PARAM_UPDATE) != NULL)
		{		
			return 1;
		}
		else
	
		if ((cread[0]=='~') && (cread[1]=='^') && (cread[nbytes+deb-1]=='`') )// && cread[2]=='I')
		{
		
			// parse data
			
			if ((ret=sscanf(cread,"%*c%*c%c%d%*c%lf%*c%lf%*c%lf%*c%lf",cmode,idnum,angle,x,y,z)) != 6)
			{
				/*#ifdef DEBUG
					printf("DEBUG: kb_stargazer_read_data sccanf field number: %d \n",ret);
					fflush(stdout);
				#endif	*/
				return -5;
			}
			
			/*#ifdef DEBUG
					printf("DEBUG: kb_stargazer_read_data angle: %4.1f \n",*angle);
					printf("DEBUG: kb_stargazer_read_data landmark id: %d \n",*idnum);
  				fflush(stdout);
 			 #endif*/

			// y is forward, x is right, angle is 0 when robot is forward, counterclockwise
			*angle = -(*angle-90);
	
			// keep angle in 0..360 deg range
			if (*angle <0)
				(*angle)+=360;
			else 
				if (*angle >360)
				 (*angle)-=360;
	
			#ifdef DEBUG
			fprintf(debug_file,"%.1f\t%.1f\t%.1f\t%.1f\t%d\n",*x,*y,*z,*angle,*idnum);
			#endif
			
			
			if (corr==1)
			{
				xc=a_axis*cos((*angle+ANGLE_CORRECTION-angle_rot)*M_PI/180.0)*cos(angle_rot*M_PI/180.0)-b_axis*sin((*angle+ANGLE_CORRECTION-angle_rot)*M_PI/180.0)*sin(angle_rot*M_PI/180.0);
	
				yc=a_axis*cos((*angle+ANGLE_CORRECTION-angle_rot)*M_PI/180.0)*sin(angle_rot*M_PI/180.0)+b_axis*sin((*angle+ANGLE_CORRECTION-angle_rot)*M_PI/180.0)*cos(angle_rot*M_PI/180.0);	
			}
			
			(*x)-=xc;
			(*y)-=yc;
			
			deb=0;
			return 0;
		} 
		else
		{
			#ifdef DEBUG_ERROR
					printf("DEBUG: kb_stargazer_read_data error -7, cread: %s\n",cread);
					fflush(stdout);
			#endif
			return -7;
		}
	}	// if (nbytes>0)
	
	
	return -8;		
}

/*****************************************************************************/
/*!
 * kb_stargazer_Close release the Stargazer
 *
 */
void kb_stargazer_Close()
{
  if (SerialDeviceHandle>=0)
  {
	  close(SerialDeviceHandle);
  }
  #ifdef DEBUG
  if (debug_file != NULL)
  	fclose(debug_file);
  #endif
  
  // stop battery module
//  kb_pwm_duty(BATTERY_PORT,0);
  //kb_pwm_cleanup();
  
}			

/*****************************************************************************/
/*! 
 * configure the Stargazer rotation compensation
 * 
 *	\param mot1 left motor pointer
 *	\param mot2 right motor pointer
 *	\param _center_x0 x center of the fitted ellipse
 *	\param _center_y0 y center of the fitted ellipse
 *	\param _angle_rot angle of rotation of the fitted ellipse
 *	\param _a_axis half major axis of the fitted ellipse
 *	\param _b_axis half minor axis of the fitted ellipse
 *	\param _stddev_x standard deviation error of x
 *	\param _stddev_y standard deviation error of y
 *   
 *	\return 0 no error
 * 
 * 				-1 timeout while computing calibration
 * 				-2 data buffer too short
 * 				-3 error computing ellipse parameters
 *				-4 error: data are too scattered
 * 				< -4+ (return kb_stargazer_read_data)	error getting Stargazer data	  
 * 
*/
int kb_gazer_calibration(knet_dev_t * dsPic,double *_center_x0,double *_center_y0,double *_angle_rot,double *_a_axis, double *_b_axis, double *_stddev_x, double *_stddev_y)
{
	double x,y,z,angle, angle_start;
	int idnum,stop=0,ret,nb_data=0;
	char cmode;
	

	int i;
	double xc,yc,stddevx,stddevy;
	
	struct timeval startTime,endTime;

	#ifdef DEBUG
	FILE *file;
	#endif 

	double data_x[MAX_CALIB_DATA],data_y[MAX_CALIB_DATA],angle_t[MAX_CALIB_DATA];
	
  
 // make the robot rotate one revolution and take Stargazer data
	
	
	ret=kb_stargazer_read_data(&x,&y,&z,&angle_start,&idnum,&cmode,0);
	
	
	if (ret!=0)
	{	
		//printf("\nDEBUG: calibration: ret = %d\n",ret);
		return ret-4;
	}
	
	#ifdef DEBUG
	file=fopen("revolution.dat","w");
	#endif
	
	// turn left
	kh4_SetMode(kh4RegSpeed,dsPic );
	kh4_set_speed(-CALIBRATION_SPEED ,CALIBRATION_SPEED ,dsPic);
	
	usleep(500000); // wait some time for stabilisation
	
	gettimeofday(&startTime, NULL);
	
	while (!stop)
	{
		ret=kb_stargazer_read_data(&x,&y,&z,&angle,&idnum,&cmode,0);
		/*if (ret!=0)
		{	
		//printf("\nDEBUG: calibration: ret = %d\n",ret);
			return ret-4;
		}		*/		
		if (nb_data>MAX_CALIB_DATA)	
		{
			#ifdef DEBUG
			fclose(file);
			#endif
			kh4_set_speed(0,0,dsPic);
			return -2;
		}
		
		data_x[nb_data]=x;
		data_y[nb_data]=y;
		angle_t[nb_data]=angle;
		
		#ifdef DEBUG
		fprintf(file,"%.2f\t%.2f\t%.2f\t%.2f\t%d\n",x,y,z,angle,idnum);
		#endif
		
		nb_data++;
		
		gettimeofday(&endTime, NULL);

		// if more than one revolution (test also to not have any rebonce at the beginning: 2s of delay)
		if ( ( (endTime.tv_sec  - startTime.tv_sec) > 2) && ( abs( (angle<angle_start?360+angle:angle)- angle_start ) < STOP_THRESHOLD_ANGLE )  )
		{
			stop = 1;
		}

		// check timeout
    if ((endTime.tv_sec  - startTime.tv_sec) > CONFIGURE_TIMEOUT)
    {
			kh4_set_speed(0,0,dsPic);
			#ifdef DEBUG
			fclose(file);
			#endif
    	return -1;
    }
	
		printf("\033[%d`",1); // go to first column of current line
		printf("\033[K"); //erase to end of line
		
		
		printf("  start angle: %5.1f current angle: %5.1f    done: %4.1f %%",angle_start,angle,((angle<angle_start?360:0)-angle_start+angle)/3.60);
		fflush(stdout);
	
		usleep(100000); // update of the Stargazer is max 10 measures/s only
		
	}
	
	#ifdef DEBUG
	fclose(file);
	#endif

	kh4_set_speed(0,0,dsPic);
	
	
	printf("\033[%d`",1); // go to first column of current line
	printf("\033[K"); //erase to end of line
		
		
	printf("  start angle: %5.1f current angle: %5.1f\n",angle_start,angle);
	
	// compute the ellipse parameters
	if ((ret=compute_ellipse(data_x,data_y,nb_data,&center_x0,&center_y0,&angle_rot,&a_axis,&b_axis))!=0)
	{
			printf("\nERROR: computing ellipse parameters (error : %d)!\n",ret);
			return -3;
	}
	
	// compute the adjustment parameters
	
	
	
	// compute statistics errors
	stddevx=0;
	stddevy=0;
	for (i=0;i<nb_data;i++)
	{
		xc=-data_x[i]+center_x0+a_axis*cos((angle_t[i]+ANGLE_CORRECTION-angle_rot)*M_PI/180.0)*cos(angle_rot*M_PI/180.0)-b_axis*sin((angle_t[i]+ANGLE_CORRECTION-angle_rot)*M_PI/180.0)*sin(angle_rot*M_PI/180.0);
	
		stddevx+=xc*xc;

		yc=-data_y[i]+center_y0+a_axis*cos((angle_t[i]+ANGLE_CORRECTION-angle_rot)*M_PI/180.0)*sin(angle_rot*M_PI/180.0)+b_axis*sin((angle_t[i]+ANGLE_CORRECTION-angle_rot)*M_PI/180.0)*cos(angle_rot*M_PI/180.0);	
		stddevy+=yc*yc;
	}

	stddevx=sqrt(stddevx/nb_data);
	stddevy=sqrt(stddevy/nb_data);
		
	#ifdef DEBUG
		printf("DEBUG: ellipse parameters: centre: (%4.1f,%4.1f)  angle: %4.1f  major axe: %4.1f  minor axe: %4.1f  stddevx = %4.2f  stddevy = %4.2f\n",center_x0,center_y0,angle_rot,2*a_axis,2*b_axis,stddevx,stddevy);
	#endif
	
	 *_angle_rot=angle_rot;
	 *_a_axis=a_axis;
	 *_b_axis=b_axis;
	 *_center_x0=center_x0;
	 *_center_y0=center_y0;
	 *_stddev_x=stddevx;
	 *_stddev_y=stddevy;
	 
	 if ( (stddevx>CALIB_STDEV_MAX) || (stddevy>CALIB_STDEV_MAX) || isnan(stddevx) || isnan(stddevy))
	 {
	 	return -4;
	 }
	 
	 
	return 0;
}	

