//--------------------------------------------------------------------------------//
//-                   (  Sound access on the Khepera	 )                         -//
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
/*!   \file kb_sound.h
      \brief header of Funtions for sound access
*/
////////////////////////////////////////////////////////////////////////////////


#ifndef __kb_sound__
#define __kb_sound__



int kb_sound_init( void );
int kb_sound_release( void );

int kb_sound_configure(int sampling_frequency,int sample_size,int sign,int endian, int channels);
int set_speakers_volume(unsigned int left_volume,unsigned  int right_volume);
int set_microphones_volume(unsigned int left_volume,unsigned  int right_volume);
int switch_speakers_ON_OFF(int on_off);
int mute_micros(int mute);
int mute_speaker(int mute);

int stop_record_play(void);
int wait_end_of_play(void);

int play_buffer(char *buffer, unsigned long length);
int record_buffer(char *buffer, unsigned long length);


int load_wav_file(char *fname, char *sound_buffer[],int *data_size, short *channels, short *bits_per_sample, int *sample_rate);
int save_wav_file(char *fname, char *sound_buffer,int data_size, short channels, short bits_per_sample, int sample_rate);

#endif /* __kb_sound__ */
