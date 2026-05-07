//--------------------------------------------------------------------------------//
//-                   (  Camera access on the Khepera	 )                         -//
//                                                                               -//
//-  Copyright (C) Julien Tharin, K-Team S.A. 2011                               -//
//-  This library is free software; you can redistribute it and/or               -//
//-  modify it under the terms of the GNU Lesser General Public                  -//
//-  License as published by the Free Software Foundation; either                -//
//-  version 2.1 of the License, or any later version.                           -//
//-                                                                              -//
//-  This library is distributed in the hope that it will be useful,             -//
//-  but WITHOUT ANY WARRANTY; without even the implied warranty of              -//
//-  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU           -//
//-  Lesser General Public License for more details.                             -//
//-                                                                              -//
//-  You should have received a copy of the GNU Lesser General Public            -//
//-  License along with this library; if not, write to the Free Software         -//
//-  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA   -//
//-                                                                              -//
//-                               __  __  ________                               -//
//- K-Team S.A.                  |  |/  /|__    __|___  _____  ___  ___          -//
//- Rue Galilee 9. Y-Park,       |     / __ |  | _____|/  _  \|   \/   |         -//
//- 1400 Yverdon-les-Bains       |  |  \    |  | ____|/  /_\  |        |         -//
//- Switzerland                  |__|\__\   |__|______|_/   \_|__|\/|__|         -//
//- jtharin@k-team.com   tel:+41 24 423 89 75 fax:+41 24 423 8960                -//
//-                                                                              -//
/***************************************************************************
 *   based on v4l2grab Version 0.3                                         *
 *                                                                         *
 *   Copyright (C) 2012 by Tobias Müller                                   *
 *   Tobias_Mueller@twam.info                                              *
 *                                                                         *
 *   based on V4L2 Specification, Appendix B: Video Capture Example        *
 *   (http://v4l2spec.bytesex.org/spec/capture-example.html)               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
 
 /**************************************************************************
 *   Modification History                                                  *
 *                                                                         *
 *   Matthew Witherwax      21AUG2013                                      *
 *      Added ability to change frame interval (ie. frame rate/fps)        *
 ***************************************************************************/
//----------------------------------------------------------------------------//

////////////////////////////////////////////////////////////////////////////////
/*!   \file kb_camera.c
      \brief Functions for accessing camera
*/
////////////////////////////////////////////////////////////////////////////////



/* ---- Include Files ---------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>
#include <linux/videodev2.h>
#include <jpeglib.h>
#include <libv4l2.h>

#include "kb_camera.h"

/* ---- Private Constants and Types -------------------------------------- */

// use ITU-R float conversion for YUV422toRGB888 by default
#if !defined(ITU_R_FLOAT) && !defined(ITU_R_INT) && !defined(NTSC)
#define ITU_R_FLOAT
#endif

#if ((defined(ITU_R_FLOAT)) && (defined(ITU_R_INT)) && (defined(NTSC))) || ((defined(ITU_R_FLOAT)) && (defined(ITU_R_INT))) || ((defined(ITU_R_FLOAT)) && (defined(NTSC))) || ((defined(ITU_R_INT)) && (defined(NTSC)))
#error Only one conversion for YUV422toRGB888 is allowed!
#endif

// compile with all three access methods
#if !defined(IO_READ) && !defined(IO_MMAP) && !defined(IO_USERPTR)
#define IO_READ
#define IO_MMAP
#define IO_USERPTR
#endif


#define CLEAR(x) memset (&(x), 0, sizeof (x))

#if defined(IO_MMAP) || defined(IO_USERPTR)
// minimum number of buffers to request in VIDIOC_REQBUFS call
#define VIDIOC_REQBUFS_COUNT 2
#endif

typedef enum {
#ifdef IO_READ
        IO_METHOD_READ,
#endif
#ifdef IO_MMAP
        IO_METHOD_MMAP,
#endif
#ifdef IO_USERPTR
        IO_METHOD_USERPTR,
#endif
} io_method;

// global settings
static unsigned int dWidth = 752;         // default image width
static unsigned int dHeight = 480;        // default image height
static unsigned char jpegQuality = 70;    // jpeg default quality
static char* jpegFilename;         				// for saving image
static unsigned int fps = -1;							// frame rate 

static char* deviceName = "/dev/video6";  // device name

static io_method io = IO_METHOD_MMAP;     // device acces method

static int fd = -1;                       // device file descriptor
static int initialised=0;                 // camera initialisation

struct buffer {                           // buffer structure
        void * start;
        size_t length;
};

struct buffer * buffers = NULL;           // buffers for images

static unsigned int n_buffers = 0;        // number of buffers for images

/* ---- Internal Functions ----------------------------------------------- */

/**
	Convert from YUV420 format to YUV444.

	\param width width of image
	\param height height of image
	\param src source
	\param dst destination
*/
void YUV420toYUV444(int width, int height, unsigned char* src, unsigned char* dst) {
	int line, column;
	unsigned char *py, *pu, *pv;
	unsigned char *tmp = dst;

	// In this format each four bytes is two pixels. Each four bytes is two Y's, a Cb and a Cr.
	// Each Y goes to one of the pixels, and the Cb and Cr belong to both pixels.
	unsigned char *base_py = src;
	unsigned char *base_pu = src+(height*width);
	unsigned char *base_pv = src+(height*width)+(height*width)/4;

	for (line = 0; line < height; ++line) {
		for (column = 0; column < width; ++column) {
			py = base_py+(line*width)+column;
			pu = base_pu+(line/2*width/2)+column/2;
			pv = base_pv+(line/2*width/2)+column/2;

			*tmp++ = *py;
			*tmp++ = *pu;
			*tmp++ = *pv;
		}
	}
}


/**
Convert from YUV420 format to RGB888. Formulae are described on http://en.wikipedia.org/wiki/YUV

\param width width of image
\param height height of image
\param src source
\param dst destination
*/
static void YUV420toRGB888(int width, int height, unsigned char *src, unsigned char *dst)
{
	int line, column;
	unsigned char *py, *pu, *pv;
	unsigned char *tmp = dst;

	// In this format each four bytes is two pixels. Each four bytes is two Y's, a Cb and a Cr.
	// Each Y goes to one of the pixels, and the Cb and Cr belong to both pixels.
	unsigned char *base_py = src;
	unsigned char *base_pu = src+(height*width);
	unsigned char *base_pv = src+(height*width)+(height*width)/4;

	#define CLIP(x) ( (x)>=0xFF ? 0xFF : ( (x) <= 0x00 ? 0x00 : (x) ) )

	for (line = 0; line < height; ++line) {
		for (column = 0; column < width; ++column) {
			py = base_py+(line*width)+column;
			pu = base_pu+(line/2*width/2)+column/2;
			pv = base_pv+(line/2*width/2)+column/2;

			/**tmp++ = *py;
			*tmp++ = *pu;
			*tmp++ = *pv; */
			#ifdef ITU_R_FLOAT
			// ITU-R float
			*tmp++ = CLIP((double)*py + 1.402*((double)*pv-128.0));
			*tmp++ = CLIP((double)*py - 0.344*((double)*pu-128.0) - 0.714*((double)*pv-128.0));
			*tmp++ = CLIP((double)*py + 1.772*((double)*pu-128.0));
			#endif

			#ifdef ITU_R_INT
			// ITU-R integer
			*tmp++ = CLIP( *py + (*pv-128) + ((*pv-128) >> 2) + ((*pv-128) >> 3) + ((*pv-128) >> 5) );
			*tmp++ = CLIP( *py - (((*pu-128) >> 2) + ((*pu-128) >> 4) + ((*pu-128) >> 5)) - (((*pv-128) >> 1) + ((*pv-128) >> 3) + ((*pv-128) >> 4) + ((*pv-128) >> 5)) ); // 52 58
			*tmp++ = CLIP( *py + (*pu-128) + ((*pu-128) >> 1) + ((*pu-128) >> 2) + ((*pu-128) >> 6) );
			#endif

			#ifdef NTSC
			// NTSC integer
			*tmp++ = CLIP( (298*(*py-16) + 409*(*pv-128) + 128) >> 8 );
			*tmp++ = CLIP( (298*(*py-16) - 100*(*pu-128) - 208*(*pv-128) + 128) >> 8 );
			*tmp++ = CLIP( (298*(*py-16) + 516*(*pu-128) + 128) >> 8 );
			#endif
			
		}
	}
}

/**
Convert from YUV422 format to RGB888. Formulae are described on http://en.wikipedia.org/wiki/YUV

\param width width of image
\param height height of image
\param src source
\param dst destination
*/
static void YUV422toRGB888(int width, int height, unsigned char *src, unsigned char *dst)
{
	int line, column;
	unsigned char *py, *pu, *pv;
	unsigned char *tmp = dst;

	/* In this format each four bytes is two pixels. Each four bytes is two Y's, a Cb and a Cr.
	Each Y goes to one of the pixels, and the Cb and Cr belong to both pixels. */
	py = src;
	pu = src + 1;
	pv = src + 3;

	#define CLIP(x) ( (x)>=0xFF ? 0xFF : ( (x) <= 0x00 ? 0x00 : (x) ) )

	for (line = 0; line < height; ++line) {
		for (column = 0; column < width; ++column) {
			#ifdef ITU_R_FLOAT
			// ITU-R float
			*tmp++ = CLIP((double)*py + 1.402*((double)*pv-128.0));
			*tmp++ = CLIP((double)*py - 0.344*((double)*pu-128.0) - 0.714*((double)*pv-128.0));
			*tmp++ = CLIP((double)*py + 1.772*((double)*pu-128.0));
			#endif

			#ifdef ITU_R_INT
			// ITU-R integer
			*tmp++ = CLIP( *py + (*pv-128) + ((*pv-128) >> 2) + ((*pv-128) >> 3) + ((*pv-128) >> 5) );
			*tmp++ = CLIP( *py - (((*pu-128) >> 2) + ((*pu-128) >> 4) + ((*pu-128) >> 5)) - (((*pv-128) >> 1) + ((*pv-128) >> 3) + ((*pv-128) >> 4) + ((*pv-128) >> 5)) ); // 52 58
			*tmp++ = CLIP( *py + (*pu-128) + ((*pu-128) >> 1) + ((*pu-128) >> 2) + ((*pu-128) >> 6) );
			#endif

			#ifdef NTSC
			// NTSC integer
			*tmp++ = CLIP( (298*(*py-16) + 409*(*pv-128) + 128) >> 8 );
			*tmp++ = CLIP( (298*(*py-16) - 100*(*pu-128) - 208*(*pv-128) + 128) >> 8 );
			*tmp++ = CLIP( (298*(*py-16) + 516*(*pu-128) + 128) >> 8 );
			#endif
			// increase py every time
			py += 2;

			// increase pu,pv every second time
			if ((column & 1)==1) {
				pu += 4;
				pv += 4;
			}
		}
	}
}


/**
	Do ioctl and retry if error was EINTR ("A signal was caught during the ioctl() operation."). Parameters are the same as on ioctl.

	\param fd file descriptor
	\param request request
	\param argp argument
	\returns result from ioctl
*/
static int xioctl(int fd, int request, void* argp)
{
	int r;

	do r = v4l2_ioctl(fd, request, argp);
	while (-1 == r && EINTR == errno);

	return r;
}



#ifdef IO_READ
/*initialize io_read

 return:  -1 : out of memory allocating all buffers
          -2 : out of memory allocating buffer
           0 : on success

*/
int readInit(unsigned int buffer_size)
{
	buffers = calloc(1, sizeof(*buffers));

	if (!buffers) {
    //fprintf(stderr, "Out of memory\n");
    //exit(EXIT_FAILURE);
    return -1;
	}

	buffers[0].length = buffer_size;
	buffers[0].start = malloc(buffer_size);

	if (!buffers[0].start) {
    //fprintf (stderr, "Out of memory\n");
    //exit(EXIT_FAILURE);
    return -2;
	}

	return 0;
}
#endif


#ifdef IO_MMAP
/*initialize memory mapping

 return:  -1 : does not support memory mapping
          -2 : insufficient buffer memory
          -3 : out of memory
          -x : standard error number

           0 : on success
*/
int mmapInit(void)
{
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count = VIDIOC_REQBUFS_COUNT;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      //fprintf(stderr, "%s does not support memory mapping\n", deviceName);
      //exit(EXIT_FAILURE);
      return -1;
    } else {
      //errno_exit("VIDIOC_REQBUFS");
      return -errno;
    }
	}

	if (req.count < 2) {
    //fprintf(stderr, "Insufficient buffer memory on %s\n", deviceName);
    //exit(EXIT_FAILURE);
    return 2;
	}

	buffers = calloc(req.count, sizeof(*buffers));

	if (!buffers) {
    //fprintf(stderr, "Out of memory\n");
    // exit(EXIT_FAILURE);
    return 3;
	}

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
      //errno_exit("VIDIOC_QUERYBUF");
      return -errno;

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = v4l2_mmap (NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, fd, buf.m.offset);

    if (MAP_FAILED == buffers[n_buffers].start)
      //errno_exit("mmap");
      return -errno;
	}

	return 0;
}
#endif


#ifdef IO_USERPTR
/*initialize USERPTR

 return:  -1 : does not support user pointer
          -2 : Out of memory
          -3 : error aligning memory
           0 : on success
*/
int userptrInit(unsigned int buffer_size)
{
	struct v4l2_requestbuffers req;
	unsigned int page_size;

	page_size = getpagesize();
	buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

	CLEAR(req);

	req.count = VIDIOC_REQBUFS_COUNT;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      //fprintf(stderr, "%s does not support user pointer i/o\n", deviceName);
      //exit(EXIT_FAILURE);
      return -1;
    } else {
      //errno_exit("VIDIOC_REQBUFS");
      return -errno;
    }
	}

	buffers = calloc(4, sizeof (*buffers));

	if (!buffers) {
    //fprintf(stderr, "Out of memory\n");
    //exit(EXIT_FAILURE);
    return -2;
	}

	for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
    buffers[n_buffers].length = buffer_size;
    buffers[n_buffers].start = memalign (/* boundary */ page_size, buffer_size);

    if (!buffers[n_buffers].start) {
      //fprintf(stderr, "Out of memory\n");
      //exit(EXIT_FAILURE);
      return -3;

    }
	}

	return 0;
}
#endif

/**
initialize device

 return:  -1 : no device /dev/video0
          -2 : no video capture device
          -3 : capabilities error
           0 : on success
           1 : image width adjusted
           2 : image height adjusted
           3 : image height + width adjusted

*/
int deviceInit(void)
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	struct v4l2_streamparm frameint;
	unsigned int min;

	int ret=0; // return value

  // get device capabilities
	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			//fprintf(stderr, "%s is no V4L2 device\n",deviceName);
			//exit(EXIT_FAILURE);
			return -1;
		} else {
			// errno_exit("VIDIOC_QUERYCAP");
			return -errno;
		}
	}


	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		//fprintf(stderr, "%s is no video capture device\n",deviceName);
		return -2;
	}

	switch (io) {
		#ifdef IO_READ
		case IO_METHOD_READ:
			if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        //fprintf(stderr, "%s does not support read i/o\n",deviceName);
        //exit(EXIT_FAILURE);
        return -3;
			}
			break;
		#endif

		#ifdef IO_MMAP
		case IO_METHOD_MMAP:
		#endif
		#ifdef IO_USERPTR
		case IO_METHOD_USERPTR:
			#endif
			#if defined(IO_MMAP) || defined(IO_USERPTR)
			if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
			//fprintf(stderr, "%s does not support streaming i/o\n",deviceName);
			return -3;
			}
			break;
		#endif
	}


	/* Select video input, video standard and tune here. */
	CLEAR(cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
		{
			switch (errno) {
				case EINVAL:
					/* Cropping not supported. */
					break;
				default:
					/* Errors ignored. */
				break;
				}
			}
		} else {
			/* Errors ignored. */
		}

		CLEAR(fmt);

		// v4l2_format
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fmt.fmt.pix.width = dWidth;
		fmt.fmt.pix.height = dHeight;
		fmt.fmt.pix.field = V4L2_FIELD_INTERLACED; //V4L2_FIELD_INTERLACED; V4L2_FIELD_ANY
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420; //V4L2_PIX_FMT_YUV420;//V4L2_PIX_FMT_MJPEG;  //V4L2_PIX_FMT_YUYV;


		if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
      //errno_exit("VIDIOC_S_FMT");
      return -errno;

		/*if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUV420) {
			//fprintf(stderr,"Libv4l didn't accept YUV420 format. Can't proceed.\n");
			//exit(EXIT_FAILURE);
			return -3;
		}*/

		/* Note VIDIOC_S_FMT may change width and height. */
		if (dWidth != fmt.fmt.pix.width) {
		dWidth = fmt.fmt.pix.width;
		//fprintf(stderr,"Image width set to %i by device %s.\n",width,deviceName);
		 ret = 1;
		}

		if (dHeight != fmt.fmt.pix.height) {
		dHeight = fmt.fmt.pix.height;
		//fprintf(stderr,"Image height set to %i by device %s.\n",height,deviceName);
      ret+= 2;
		}
		
		/* If the user has set the fps to -1, don't try to set the frame interval */
		if (fps != -1)
		{
		  CLEAR(frameint);
		  
		  /* Attempt to set the frame interval. */
		  frameint.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		  frameint.parm.capture.timeperframe.numerator = 1;
		  frameint.parm.capture.timeperframe.denominator = fps;
		  if (-1 == xioctl(fd, VIDIOC_S_PARM, &frameint))
		    fprintf(stderr,"Unable to set frame interval.\n");
		}
		

		/* Buggy driver paranoia. */
		/*min = fmt.fmt.pix.width * 2;
		if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
		min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
		if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;*/

		switch (io) {
		#ifdef IO_READ
		case IO_METHOD_READ:
		readInit(fmt.fmt.pix.sizeimage);
		break;
		#endif

		#ifdef IO_MMAP
		case IO_METHOD_MMAP:
		mmapInit();
		break;
		#endif

		#ifdef IO_USERPTR
		case IO_METHOD_USERPTR:
		userptrInit(fmt.fmt.pix.sizeimage);
		break;
		#endif
	}


  initialised=1;
  if (ret!=0)
   return ret;

	return 0;
}


/* uninitialise device
*/
int deviceUninit(void)
{
	unsigned int i;

	switch (io) {
    #ifdef IO_READ
    case IO_METHOD_READ:
      free(buffers[0].start);
      break;
    #endif

    #ifdef IO_MMAP
    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i)
        if (-1 == v4l2_munmap (buffers[i].start, buffers[i].length))
          //errno_exit("munmap");
          return -errno;
      break;
    #endif

    #ifdef IO_USERPTR
    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i)
        free (buffers[i].start);
      break;
    #endif
	}

	free(buffers);
	initialised=0;

	return 0;
}


/* open device
 return:   0 : no error, device open
          -1 : cannot state the device
          -2 : not a character device file
          -3 : cannot open the device
*/
int deviceOpen(void)
{
	struct stat st;

	// stat file
	if (-1 == stat(deviceName, &st)) {
		//fprintf(stderr, "Cannot identify '%s': %d, %s\n", deviceName, errno, strerror (errno));
		//exit(EXIT_FAILURE);
		return -1;
	}

	// check if its device is a character device file
	if (!S_ISCHR (st.st_mode)) {
		//fprintf(stderr, "%s is no device\n", deviceName);
	//	exit(EXIT_FAILURE);
	 return -2;
	}

	// open device
	fd = v4l2_open(deviceName, O_RDWR /* required */ | O_NONBLOCK, 0);

	// check if opening was successfull
	if (-1 == fd) {
		//fprintf(stderr, "Cannot open '%s': %d, %s\n", deviceName, errno, strerror (errno));
		//exit(EXIT_FAILURE);
		return -3;
	}

	return 0;
}

/**
close device
*/
static void deviceClose(void)
{
  if (-1 == v4l2_close(fd))
		//errno_exit("close");
	 {
	 }
	fd = -1;

}

/**
  Write image to jpeg file.

  \param img image to write
  \return A value
     -1 : cannot open file
      0 : success
*/
int jpegWrite(unsigned char* img)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;

	JSAMPROW row_pointer[1];
	FILE *outfile = fopen( jpegFilename, "wb" );

	// try to open file for saving
	if (!outfile) {
		//errno_exit("jpeg");
		return -1;
	}

	// create jpeg data
	cinfo.err = jpeg_std_error( &jerr );
	jpeg_create_compress(&cinfo);
	jpeg_stdio_dest(&cinfo, outfile);

	// set image parameters
	cinfo.image_width = dWidth;
	cinfo.image_height = dHeight;
	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;

	// set jpeg compression parameters to default
	jpeg_set_defaults(&cinfo);
	// and then adjust quality setting
	jpeg_set_quality(&cinfo, jpegQuality, TRUE);

	// start compress
	jpeg_start_compress(&cinfo, TRUE);

	// feed data
	while (cinfo.next_scanline < cinfo.image_height) {
		row_pointer[0] = &img[cinfo.next_scanline * cinfo.image_width * cinfo.input_components];
		jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}

	// finish compression
	jpeg_finish_compress(&cinfo);

	// destroy jpeg data
	jpeg_destroy_compress(&cinfo);

	// close output file
	fclose(outfile);

	return 0;
}

/**
process image read: convert to RGB 8 bit
  \return A value
  					0 : success
            -1 : buffer not initialised
            -2 : not enough memory into buffer
*/
int imageProcess(const void* p , unsigned char* buffer)
{
	unsigned char* src = (unsigned char*)p;
	//unsigned char* dst = malloc(dWidth*dHeight*3*sizeof(char));


  if (buffer == NULL)
    return -1;


  //if (buffer))
    //return -2;

	// convert from YUV422 to RGB888
	//YUV422toRGB888(dWidth,dHeight,src,buffer);
	
	YUV420toRGB888(dWidth,dHeight,src,buffer);

	// write jpeg
	//jpegWrite(dst);

	// free temporary image
//	free(dst);

  return 0;
}

/*
 *! Read single frame
 *
 * \param output RGB888 buffer
 * \return A value
 *    0 : no error
 *   -1 : read error
 *   -2 : VIDIOC_DQBUF error
 *   -3 : xioctl error
 *   -4 : try again
 *   -5 : buffer size error
 *
*/
int frameRead(unsigned char *output)
{
	struct v4l2_buffer buf;
	#ifdef IO_USERPTR
	unsigned int i;
	#endif

	switch (io) {
    #ifdef IO_READ
    case IO_METHOD_READ:
      if (-1 == v4l2_read (fd, buffers[0].start, buffers[0].length)) {
      switch (errno) {
        case EAGAIN: // Try again
        return -4;

        case EIO:
        // Could ignore EIO, see spec.
        // fall through

        default:
          //errno_exit("read");
          return -1;
        }
      }

       if (imageProcess(buffers[0].start,output))
       {
        return -5;
       }

      break;
    #endif

    #ifdef IO_MMAP
    case IO_METHOD_MMAP:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
          return -4;

          case EIO:
          // Could ignore EIO, see spec
          // fall through

          default:
          //errno_exit("VIDIOC_DQBUF");
            return -2;
        }
      }

      assert(buf.index < n_buffers);

      if(imageProcess(buffers[buf.index].start,output)<0)
      {
        return -5;
      }

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
      //errno_exit("VIDIOC_QBUF");
        return -3;

      break;
    #endif

    #ifdef IO_USERPTR
      case IO_METHOD_USERPTR:
      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
          return -4;

          case EIO:
          // Could ignore EIO, see spec.
          // fall through

          default:
            //errno_exit("VIDIOC_DQBUF");
            return -2;
        }
      }

      for (i = 0; i < n_buffers; ++i)
        if (buf.m.userptr == (unsigned long) buffers[i].start && buf.length == buffers[i].length)
          break;

      assert (i < n_buffers);

      if (imageProcess((void *)buf.m.userptr,output)<0)
      {
        return -5;
      }

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        //errno_exit("VIDIOC_QBUF");
        return -3;
      break;
    #endif
  }

  return 0;
}

/* ---- Exported Functions------------------------------------------------ */


/*
 *! Read single frame
 *
 * \param output RGB888 buffer
 * \return A value
 *    0 : no error
 *   -5 : buffer size error
 *   -1 : read error
 *   -2 : VIDIOC_DQBUF error
 *   -3 : xioctl error
 *   -4 : try again
 *   -5 : select error
 *   -6 : timeout error
 *
*/
int kb_frameRead(unsigned char *output)
{	
	unsigned int count;
	unsigned int numberOfTimeouts;
	int ret=0;

	numberOfTimeouts = 0;
	count = 3;

	while (count-- > 0) {
		for (;;) {
			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(fd, &fds);

			/* Timeout. */
			tv.tv_sec = 1;
			tv.tv_usec = 0;

			r = select(fd + 1, &fds, NULL, NULL, &tv);

			if (-1 == r) {
				if (EINTR == errno)
					continue;

				//errno_exit("select");
				return -5;
			}

			if (0 == r) {
				if (numberOfTimeouts <= 0) {
					count++;
				} else {
					//fprintf(stderr, "select timeout\n");
					//exit(EXIT_FAILURE);
					return -6;
				}
			}

			ret=frameRead(output);
			
			if (ret==0)
				break;

			
			if (ret!=-4)
				return ret;
			
			/* EAGAIN - continue select loop. */
		}
	}
	
	return ret;
}




/*! Stop capturing multiple frames
 *
 *  \param (none)
 *  \return A value
 *  				0 : success
 *  				-1 : VIDIOC_STREAMOFF error
 *
*/
int kb_captureStop(void)
{
	enum v4l2_buf_type type;

	switch (io) {
	#ifdef IO_READ
	case IO_METHOD_READ:
	/* Nothing to do. */
	break;
	#endif

	#ifdef IO_MMAP
	case IO_METHOD_MMAP:
	#endif
	#ifdef IO_USERPTR
	case IO_METHOD_USERPTR:
	#endif
	#if defined(IO_MMAP) || defined(IO_USERPTR)
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
    //errno_exit("VIDIOC_STREAMOFF");
    return -1;

	break;
	#endif
	}

	return 0;
}


/*!  Start capturing multiple frames
 *
 *  \param (none)
 *  \return A value
 *  				0 : success
 *    			-1 : VIDIOC_QBUF error
 *		 	  	-2 : VIDIOC_STREAMON error
 *
*/
int kb_captureStart(void)
{
	unsigned int i;
	enum v4l2_buf_type type;

	switch (io) {
    #ifdef IO_READ
    case IO_METHOD_READ:
    /* Nothing to do. */
    break;
    #endif

    #ifdef IO_MMAP
    case IO_METHOD_MMAP:
    for (i = 0; i < n_buffers; ++i) {
      struct v4l2_buffer buf;

      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        //errno_exit("VIDIOC_QBUF");
        return -1;
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
      //errno_exit("VIDIOC_STREAMON");
      return -2;

    break;
    #endif

    #ifdef IO_USERPTR
    case IO_METHOD_USERPTR:
    for (i = 0; i < n_buffers; ++i) {
      struct v4l2_buffer buf;

      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = i;
      buf.m.userptr = (unsigned long) buffers[i].start;
      buf.length = buffers[i].length;

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
          //errno_exit("VIDIOC_QBUF");
          return -1;
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
      //errno_exit("VIDIOC_STREAMON");
      return -2;

    break;
    #endif
	}

	return 0;
}






/*! convert to greyscale
 *
 * \param src image
 * \return A value:
 *        0 : on success
 */

int into_greyscale(unsigned char *src)
{
	int i;
	
	
	for(i=0; i<dWidth*dHeight*3;i+=3)
	{
	  src[i]=src[i+1]=src[i+2]= (unsigned char) (0.3*src[i] + 0.59*src[i+1] + 0.11*src[i+2]);
	}
	
	return 0;
}
 

/*! apply array filter on greyscale image
 *
 * \param src source of image
 * \param dst destination of processed image
 * \param filter_x_array filter square array for column dimension
 * \param filter_y_array filter square array for line dimension
 * \param filter_dim_x size of filter for column dimension; must be odd
 * \param filter_dim_y size of filter for line dimension; must be odd
 *
 * \return A value:
 *			 -1 : source image memory is NULL
 *			 -2 : destination image memory is NULL
 *			 -3 : filter x size is not an odd number >0
 *			 -4 : filter x size is not an odd number >0
 *        0 : on success
 */
int apply_filter(unsigned char *src, unsigned char *dst,int *filter_x_array,int *filter_y_array,int filter_dim_x,int filter_dim_y )
{
  int line, column,i,j,ind,ind2;
  int sumx,sumy,sum;
  
  int dx,dy;
  
  if (src == NULL)
  	return -1;
  	
  if (dst == NULL)
	  return -2;
  
  if ((filter_dim_x<1) || (filter_dim_x % 2 == 0))
  {
    // not odd
    return -3;
  }
  
    if ((filter_dim_y<1) || (filter_dim_y % 2 == 0))
  {
    // not odd
    return -4;
  }
  
  dx=(filter_dim_x-1)/2;
  dy=(filter_dim_y-1)/2;
  
  for (column=0; column<dWidth;column++)
	{
		for (line=0; line<dHeight;line++)
		{
		
			sumx=sumy=0;
		
		  // image boundaries
		  
		  if (column == 0 || column == dWidth-1)
		     sum=0;
		  else if (line == 0 || line == dHeight-1)
		     sum=0;  
		     
		  else
		  {
		     // convolution
		  		
		  	 // gradient approximation
	    	for(i=-dx; i<=dx; i++)  {
		  		for(j=-dy; j<=dy; j++)  {
		  		  ind=3*(column+i+(line+j)*dWidth);
		  		  
		  		  ind2=i+1+(j+1)*filter_dim_y;
		      	sumx += (int)src[ind]  * filter_x_array[ind2];
		      	sumy +=  (int)src[ind] * filter_y_array[ind2];
		  		 }
	       }

    	  sum=abs(sumx)+abs(sumy); // gradient magnitude  
  
		  }   
		  
		  if (sum>255)
		  	sum=255;
		  if (sum<0)	
		  	sum=0;
		  
		  ind=3*(column+line*dWidth); // compute linear index
			dst[ind] = 255 - (unsigned char)(sum);
			dst[ind+1] = 255 - (unsigned char)(sum);
			dst[ind+2] = 255 - (unsigned char)(sum);
		}
	}
	
	return 0;
}



/*! initializes.
 * This function needs to be called BEFORE any other functions.
 *
 * \param width width of image; may be autonomoulsy adjusted and returned
 * \param height height of image; may be autonomoulsy adjusted and returned
 *
 * \return A value:
 *			 -1 : no device /dev/video0
 *       -2 : no video capture device
 *       -3 : capabilities error
 *       -4 : cannot open the device
 *       -5 : cannot call system for media-ctl pipes
 *       -6 : media-ctl pipes command exited with error
 *       -7 : cannot call system for media-ctl formats
 *       -8 : media-ctl formats command exited with error
 *        0 : on success
 *        1 : image width adjusted
 *        2 : image height adjusted
 *        3 : image height + width adjusted
 */

int kb_camera_init(unsigned int *width, unsigned int *height)
{
  int ret=0;

	char pipe_cmd[]="media-pipes.sh";
	char default_format_cmd[]="media-formats.sh";
  char format_cmd[256];


  int ret2;
  
  if ((ret2=system(pipe_cmd))==-1)
  {
  	return -5;
  }   
  
  if (WEXITSTATUS(ret2)!=0)
	{
	  return -6;
  } 
  
	sprintf(format_cmd,"%s %d %d",default_format_cmd,*width,*height);

	
	if ((ret2=system(format_cmd))==-1)
  {
  	return -7;
  } 
	
	if (WEXITSTATUS(ret2)!=0)
	{
	  return -8;
  } 

  // open and initialize device
	if (deviceOpen()!=0)
    return -4;

	dWidth=*width;
	dHeight=*height;


	if ((ret=deviceInit())!=0)
  {
  	*width=dWidth;
		*height=dHeight;
  	return ret;
  }




	return 0;
}

/*! release the camera and device
 *
 * \param none
 *
 * \return A value:
 *       - -1 cannot open the device
 *       - 0 on success
 *
 */
void kb_camera_release( void )
{
  if (fd !=-1) {
    deviceUninit();
  }

  deviceClose();

}


/*! take an image in format : rgb 8 bit (RGB888)
 *
 * \param buffer returned image
 *
 * \return A value:
 *       - 0 on success
 *       - -1 device not open
 *       - -2 device not initialised
 *       - -3 capture start error
 *       - -5 error stopping capture
 *       - -6 : read error into frameRead
 *       - -7 : VIDIOC_DQBUF error into frameRead
 *       - -8 : xioctl error into frameRead
 *       - -9 : try again into frameRead
 *       - -10 : buffer size error into frameRead
 */

int take_one_image(unsigned char *buffer){

 int ret,i;

  if (buffer==NULL)
    return -1;

  if (fd==-1)
  {
    return -1;
  }

  if (!initialised)
  {
    return -2;
  }

  if(kb_captureStart()<0)
  {
    return -3;
  }

	// take some images for auto adjust
	/*for (i=0;i<10;i++)
	{
	  usleep(100000); // wait 100ms for initialisation
	  kb_frameRead(buffer);
	}*/

  

  if ((ret=kb_frameRead(buffer))<0)
  {

    if (ret==-4)
    {
    	printf("trying again...\r\n");


		/*for (i=0;i<10;i++)
		{
		  usleep(100000); // wait 100ms for initialisation
		  ret=kb_frameRead(buffer);
		  
		  printf("DEBUG : i %d   return %d\n",i,ret);
		  
		}*/


     ret=kb_frameRead(buffer);

    }


    if (ret<0)
    {
    	return -5+ret;
    }
  }

  if (kb_captureStop()<0)
  {
    return -5;
  }

  return 0;
}

/*! take an image buffer to jpeg file
 *
 * \param filename filename of the image where is is saved
 * \param quality  jpeg quality in range [0-100]
 * \param buffer image buffer
 *
 * \return A value:
 *       - -1 quality range error
 *       - -2 buffer error
 *       - -3 file write error
 *       - 0 on success
 */
int save_buffer_to_jpg(const char *filename,int quality,unsigned char *buffer)
{

  if (quality<0 || quality > 100)
    return -1;

  jpegQuality=quality;


  if (buffer==NULL)
    return -2;

  jpegFilename=(char *)filename;

  if (jpegWrite(buffer)<0)
  {
    return -3;
  }

  return 0;
}

