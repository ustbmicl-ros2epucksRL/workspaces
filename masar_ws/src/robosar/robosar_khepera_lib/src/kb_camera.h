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
//--------------------------------------------------------------------------------//

////////////////////////////////////////////////////////////////////////////////
/*!   \file kb_camera.h
      \brief header of Funtions for camera access
*/
////////////////////////////////////////////////////////////////////////////////


#ifndef __kb_camera__
#define __kb_camera__

extern int kb_camera_init(unsigned int *width, unsigned int *height);
extern void kb_camera_release( void );

extern int take_one_image(unsigned char *buffer);


extern int kb_captureStart(void);
extern int kb_captureStop(void);
extern int kb_frameRead(unsigned char *output);


extern int save_buffer_to_jpg(const char *filename,int quality,unsigned char *buffer);

extern int apply_filter(unsigned char *src, unsigned char *dst,int *filter_x_array,int *filter_y_array,int filter_dim_x,int filter_dim_y );

extern int into_greyscale(unsigned char *src);


#endif /* __kb_camera__ */
