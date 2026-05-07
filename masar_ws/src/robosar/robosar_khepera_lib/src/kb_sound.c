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
/*!   \file kb_sound.c
      \brief Functions for accessing sound
*/
////////////////////////////////////////////////////////////////////////////////



/* ---- Include Files ---------------------------------------------------- */

#include <fcntl.h>
#include <linux/soundcard.h>

#include <alsa/asoundlib.h>

#include "kb_sound.h"
#include "kb_gpio.h"



/* ---- Private Constants and Types -------------------------------------- */

static int dsp_fd=-1; // device file descriptor fo the dsp
//static int mixer_fd=-1; // device file descriptor fo the mixer

static snd_mixer_t *mixer_handle; // mixer handle
static snd_mixer_selem_id_t *mixer_sid; // mixer sid

#define SOUND_CARD_INDEX 0 // index of the sound card used

#define SOUND_DEVICE_DSP_STRING "/dev/dsp"
//#define SOUND_DEVICE_MIXER_STRING "/dev/mixer"

#define SOUND_CARD_NAME "hw:0"

const char *MIXER_ATTACH = "default";

#define MIXER_SPEAKERS_FINE_NAME "DAC2 Digital Fine"
#define MIXER_SPEAKERS_ANALOG_NAME "DAC2 Analog"

#define MIXER_MICRO_NAME "Analog"
// electronically left is right and vice versa
#define LEFT_MICRO_NAME "Analog Right Sub Mic"
#define RIGHT_MICRO_NAME "Analog Left Main Mic"

#define MAX_SAMPLING_FREQUENCY 100000
#define MIN_SAMPLING_FREQUENCY 1000


/* WAVE fmt block constants from Microsoft mmreg.h header */
#define WAV_FMT_PCM             0x0001


/* ---- Internal Functions ----------------------------------------------- */

/*! mixer initializes.
 * This function needs to be called BEFORE any other mixing functions.
 *
 * \param none
 *
 * \return A value:
 *       - -1  mixer open error
 *       - -2  mixer attach error
 *       - -3  mixer register error
 *       - -4  mixer load error
 *       - 0 on success
 *
 */
int mixer_init(void)
{

  int err;


  mixer_handle=NULL;



  if ((err = snd_mixer_open (&mixer_handle, 1)) < 0) {
    //fprintf(stderr,"alsa-control: mixer open error: %s\n",strerror(err));
    return -1;
  }

  if ((err = snd_mixer_attach (mixer_handle, MIXER_ATTACH)) < 0) {
   // fprintf(stderr,"alsa-control: mixer attach %s error: %s\n",card_name,strerror(err));
    snd_mixer_close(mixer_handle);
    return -2;
  }

  if ((err = snd_mixer_selem_register (mixer_handle, NULL, NULL)) < 0) {
    //fprintf(stderr,"alsa-control: mixer register error:  %s\n",strerror(err));
    snd_mixer_close(mixer_handle);
    return -3;
  }


  if ((err = snd_mixer_load(mixer_handle)) < 0) {
  //  fprintf(stderr,"alsa-control: mixer load error:  %s\n",strerror(err));
    snd_mixer_close(mixer_handle);
    return -4;
  }


  // unmute micros
  mute_micros(0);

  return 0;
}

/* ---- Exported Functions------------------------------------------------ */

/*! initializes.
 * This function needs to be called BEFORE any other functions.
 *
 * \param none
 *
 * \return A value:
 *       - -1 dsp open error
 *       - -2 mixer initialization error
 *       - 0 on success
 *
 */
int kb_sound_init( void )
{
  int rc;

  if ((dsp_fd=open(SOUND_DEVICE_DSP_STRING, O_RDWR))< 0)
  {
    rc= -1;
  }
  else
  {
  /*  if ((mixer_fd=open(SOUND_DEVICE_MIXER_STRING, O_RDWR))< 0)
    {
      rc= -2;
      dsp_fd=-1;
    }*/
   if ((mixer_init())< 0)
    {
      rc= -2;
      dsp_fd=-1;
    }
    else
    {
      rc= 0;
    }
  }



  return rc;

}


/*! release the sound devices
 * This function needs to be called AFTER any other functions.
 *
 * \param none
 *
 * \return A value:
 *       - <0 on error
 *       - 0 on success
 *
 */
int kb_sound_release( void )
{
  int rc;


  if (dsp_fd < 0)
  {
    rc= -1;
  }
  else
  {
    close(dsp_fd);
    dsp_fd=-1;

    if (mixer_handle==NULL)
    {
      rc=2;
    }
    else
    {
    	// mute micros
		  mute_micros(1);
      snd_mixer_close(mixer_handle);
      mixer_handle=NULL;
    }

  }

  return rc;

}

/*! configure the sound device (default: 8kHz, unsigned 8-bit, mono)
 *
 * \param sampling_frequency sampling frequency of the sound signal [Hz] range=1000..100000
 *
 * \param sample_size sample size : 8 or 16 bits
 *
 * \param sign signed = 1, unsigned = 0
 *
 * \param endian for 16 bit: little = 0 (LSB MSB), big = 1 (MSB LSB)
 *
 * \param channels 1=mono; 2 = stereo
 *
 *
 *
 * \return A value:
 *       - -1 sound not initialised
 *       - -2 sampling_frequency out or range or error setting parameter
 *       - -3 sample_size out or range
 *       - -4 sign out of range
 *       - -5 endian out ofrange
 *       - -6 number of channels out of range
 *       - -7 IOCTL error: sampling_frequency set error
 *       - -8 IOCTL error: format (8/16 bit, endianness, sign) set error
 *       - -9 IOCTL error: channels set error
 *       - 0 on success
 *
 */
int kb_sound_configure(int sampling_frequency,int sample_size,int sign,int endian, int channels)
{
  int rc=0;
  int format=AFMT_U8;

  if (dsp_fd < 0)
  {
    return -1;
  }

  if ((sampling_frequency<MIN_SAMPLING_FREQUENCY) || (sampling_frequency>MAX_SAMPLING_FREQUENCY))
  {
    return -2;
  }

  if ((sample_size!=8) && (sample_size!=16))
  {
    return -3;
  }


  if ((sign!=0) && (sign!=1))
  {
    return -4;
  }

  if ((sample_size==16)  && ((endian!=0) && (endian!=1)))
  {
    return -5;
  }

  if ((channels!=1) && (channels!=2))
  {
    return -6;
  }




  if (ioctl(dsp_fd, SOUND_PCM_WRITE_RATE, &sampling_frequency)==-1)
  {
    return -7;
  }

 if (sample_size == 8)
 {
    if (sign)
    {
      format=AFMT_S8;
    }
    else
    {
      format=AFMT_U8;
    }
 }
 else
 {
    if (sign)
    {
      if (endian)
      {
        format=AFMT_S16_BE;
      }
      else
      {
        format=AFMT_S16_LE;
      }
    }
    else
    {
      if (endian)
      {
        format=AFMT_U16_BE;
      }
      else
      {
        format=AFMT_U16_LE;
      }
    }
 }


   if (ioctl(dsp_fd, SNDCTL_DSP_SETFMT, &format)==-1)
  {
    return -8;
  }

  if (ioctl(dsp_fd, SOUND_PCM_WRITE_CHANNELS, &channels)==-1)
  {
    return -9;
  }


  return rc;

}

/*! mute micros.
 * 
 *
 * \param mute 1 = mute, 0 = unmute
 *
 * \return A value:
 *       - -1 sound not initialised
 *       - -2 cannot mute/unmute left micro
 *			 - -3 cannot mute/unmute right micro
 *       - 0 on success
 *
 */
int mute_micros(int mute)
{
  snd_mixer_elem_t *mixer_elem; // mixer element




  if (mixer_handle == NULL)
  {
    return -1;
  }


	//allocate simple id
  snd_mixer_selem_id_alloca (&mixer_sid);
  
	//sets simple-mixer index
  snd_mixer_selem_id_set_index (mixer_sid, 0);


	// left micro
  snd_mixer_selem_id_set_name (mixer_sid, LEFT_MICRO_NAME);
  mixer_elem = snd_mixer_find_selem (mixer_handle, mixer_sid); 
  if (snd_mixer_selem_set_capture_switch_all(mixer_elem, !mute)<0)
  	return -2;

	// right micro
	snd_mixer_selem_id_set_name (mixer_sid, RIGHT_MICRO_NAME);
  mixer_elem = snd_mixer_find_selem (mixer_handle, mixer_sid);
	if (snd_mixer_selem_set_capture_switch_all(mixer_elem,!mute)<0)
  	return -3;

  return 0;
} 

/*! mute speaker

 * 
 *
 * \param mute 1 = mute, 0 = unmute
 *
 * \return A value:
 *       - -1 sound not initialised
 *       - -2 cannot mute/unmute speaker
 *       - 0 on success
 *
 */
int mute_speaker(int mute)
{
  snd_mixer_elem_t *mixer_elem; // mixer element


  if (mixer_handle == NULL)
  {
    return -1;
  }


	//allocate simple id
  snd_mixer_selem_id_alloca (&mixer_sid);
  
	//sets simple-mixer index
  snd_mixer_selem_id_set_index (mixer_sid, 0);


	snd_mixer_selem_id_set_name (mixer_sid, MIXER_SPEAKERS_ANALOG_NAME);
  

  mixer_elem = snd_mixer_find_selem (mixer_handle, mixer_sid);

  if (snd_mixer_selem_set_capture_switch_all(mixer_elem, !mute)<0)
  	return -2;



  return 0;
}

/*! set the volume of the speakers
 *
 * \param left_volume in % range= 0..100
 *
 * \param right_volume in % range= 0..100
 *
 * \return A value:
 *       - -1 sound not initialised
 *       - -2 left_volume out or range or error setting parameter
 *       - -3 right_volume out or range or error setting parameter
 *       - -4 
 *       - 0 on success
 *
 */
int set_speakers_volume(unsigned int left_volume, unsigned int right_volume)
{
  snd_mixer_elem_t *mixer_elem; // mixer element
  long pmin = 0, pmax = 0;
  long int vol = 0;


  if (mixer_handle == NULL)
  {
    return -1;
  }

  if ((left_volume<0) || (left_volume>100))
  {
    return -2;
  }

  if ((right_volume<0) || (right_volume>100))
  {
    return -3;
  }

	//allocate simple id
  snd_mixer_selem_id_alloca (&mixer_sid);

  //sets simple-mixer index
  snd_mixer_selem_id_set_index (mixer_sid, 0);

  snd_mixer_selem_id_set_name (mixer_sid, MIXER_SPEAKERS_FINE_NAME);
  

  mixer_elem = snd_mixer_find_selem (mixer_handle, mixer_sid);
  
  if (mixer_elem == NULL)
  {
    //printf("DEBUG: snd_mixer_find_selem ERROR: \"%s\" not found\n",MIXER_SPEAKERS_FINE_NAME);
    return -4;
  }
  


  snd_mixer_selem_get_playback_volume_range (mixer_elem, &pmin, &pmax);
  

  snd_mixer_selem_get_playback_volume (mixer_elem, SND_MIXER_SCHN_FRONT_LEFT, &vol);
  

  
  
  vol = (left_volume * (pmax?pmax:31)) / 100;

 // fprintf(stderr,"vol l %d\n",vol);
  if (snd_mixer_selem_set_playback_volume (mixer_elem, SND_MIXER_SCHN_FRONT_LEFT, vol)<0)
  {
    return -2;
  }

  vol = (right_volume * (pmax?pmax:31)) / 100;
 // fprintf(stderr,"vol r %d\n",vol);
  if(snd_mixer_selem_set_playback_volume (mixer_elem, SND_MIXER_SCHN_FRONT_RIGHT, vol)<0)
  {
    return -3;
  }

  return 0;

}


/*! set the input volume of microphones
 *
 * \param left_volume in % range= 0..100
 *
 * \param right_volume in % range= 0..100
 *
 * \return A value:
 *       - -1 sound not initialised
 *       - -2 left_volume out or range or error setting parameter
 *       - -3 right_volume out or range or error setting parameter
 *       - 0 on success
 *
 */
int set_microphones_volume(unsigned int left_volume,unsigned  int right_volume)
{
  snd_mixer_elem_t *mixer_elem; // mixer element
  long pmin = 0, pmax = 0;
  long int vol = 0;



  if (mixer_handle == NULL)
  {
    return -1;
  }


  if ((left_volume<0) || (left_volume>100))
  {
    return -2;
  }

  if ((right_volume<0) || (right_volume>100))
  {
    return -3;
  }

	//allocate simple id
  snd_mixer_selem_id_alloca (&mixer_sid);
  
	//sets simple-mixer index
  snd_mixer_selem_id_set_index (mixer_sid, 0);


  snd_mixer_selem_id_set_name (mixer_sid, MIXER_MICRO_NAME);
  
  
  mixer_elem = snd_mixer_find_selem (mixer_handle, mixer_sid);
  

  snd_mixer_selem_get_capture_volume_range(mixer_elem, &pmin, &pmax);
  
   
  
  vol = (left_volume * (pmax?pmax:31)) / 100;

  //fprintf(stderr,"vol l %d\n",vol);
  if (snd_mixer_selem_set_capture_volume (mixer_elem, SND_MIXER_SCHN_FRONT_LEFT, vol)<0)
  {
    return -2;
  }

  vol = (right_volume * (pmax?pmax:31)) / 100;
 // fprintf(stderr,"vol r %d\n",vol);
  if (snd_mixer_selem_set_capture_volume (mixer_elem, SND_MIXER_SCHN_FRONT_RIGHT, vol)<0)
  {
    return -3;
  }

  return 0;

}


/*! wait for playback to complete
 *
 * \param none
 *
 * \return A value:
 *       - -1 sound not initialised
 *       - -2 I/O error
 *       - 0 on success
 *
 */
int wait_end_of_play(void)
{
  if (dsp_fd < 0)
  {
    return -1;
  }

  if (ioctl(dsp_fd, SOUND_PCM_SYNC, 0)==-1)
  {
    return -2;
  }

  return 0;
}

/*! stop record/play
 *
 * \param none
 *
 * \return A value:
 *       - -1 sound not initialised
 *       - -2 I/O error
 *       - 0 on success
 *
 */
int stop_record_play(void)
{
  if (dsp_fd < 0)
  {
    return -1;
  }

  if (ioctl(dsp_fd, SOUND_PCM_RESET, 0)==-1)
  {
    return -2;
  }

  return 0;
}


/*! play a sound buffer
 *
 * \param buffer sound buffer (format: unsigned 8 or 16 bit, see function "kb_sound_configure"; if stereo,
 *        left and right are alternate )
 *
 * \param length number of samples
 *
 * \return A value:
 *       - -1 sound not initialised
 *       - -2 play error
 *       - 0 on success
 *
 */
int play_buffer(char *buffer, unsigned long length)
{
  if (dsp_fd < 0)
  {
    return -1;
  }

  if (write(dsp_fd, buffer, length)==-1)
  {
    return -2;
  }

  return 0;

}

/*! record a sound
 *
 *
 * \param buffer sound buffer (format: unsigned 8 or 16 bit, see function "kb_sound_configure"; if stereo,
 *        left and right are alternate )
 *
 * \param length number of samples
 *
 *
 * \return A value:
 *       - -1 sound not initialised
 *       - -2 record error
 *       - 0 on success
 *
 */
int record_buffer(char *buffer, unsigned long length)
{
	int read_len=0;
  if (dsp_fd < 0)
  {
    return -1;
  }

  if ((read_len=read(dsp_fd, buffer, length))==-1)
  {
    return -2;
  }

	//printf("DEBUG: length %d | currently read length : %d\n",length,read_len);

  return 0;

}

/*! switch by hardware the speakers on/off
 *
 *
 * \param on_off on = 1; off = 0
 *
 * \return A value:
 *       - -1 gpio could not be initialised
 *       - -2 couldn't switch speaker
 *       - 0 on success
 *
 */
int switch_speakers_ON_OFF(int on_off)
{

  if (kb_gpio_init()<0)
  {
    return -1;
  }

  if (kb_gpio_dir_val(GPIO_SMUTE,GPIO_OUTPUT,on_off)<0)
  {
    return -2;
  }

  return 0;
}



/*! load a PCM wave file (allocate memory and load file into)
 *
 *
 * \param fname filename
 * \param sound_buffer
 * \param data_size length of the soundbuffer
 * \param channels number of channels (1 mono, 2 stereo)
 * \param bits_per_sample number of bits per sample (8 or 16)
 * \param sample_rate sample rate (44100,22050,...) 
 *
 * \return A value:
 *       - -1 cannot open file 
 *       - -2 file is not a RIFF file
 *			 - -3 file is RIFF but not a WAV file	
 *       - -4 not a PCM wav file 
 *			 - -5 cannot allocate memory in the buffer
 *			 - -6 could not read all the WAV data in file; file truncated??
 *       - 0 on success
 */
int load_wav_file(char *fname, char *sound_buffer[],int *data_size, short *channels, short *bits_per_sample, int *sample_rate)
{
  FILE *fp;

  fp = fopen(fname,"rb");
  if (fp)
  {
    char id[4]; //four bytes to hold 'RIFF', 'fmt ', 'data'
    int size; //32 bit value to hold file size
    short format_tag, channels, block_align ; //our 16 bit values
    int format_length, avg_bytes_sec, i,loaded; //our 32 bit values

		char *sound_buffer_tmp=NULL;

    fread(id, sizeof(char), 4, fp); //read in first four bytes
    
    if (strncmp(id, "RIFF",4)==0) 
		{ 
      fread(&size, sizeof(int), 1, fp); //read in 32bit size value; size of remaining data
      fread(id, sizeof(char), 4, fp); //read in 4 byte string now
      if (strncmp(id,"WAVE",4)==0)
      { 
        fread(id, sizeof(char), 4, fp); //read in 4 bytes "fmt ";
        fread(&format_length, sizeof(int),1,fp);
        fread(&format_tag, sizeof(short), 1, fp); //check mmreg.h for other possible format tags like ADPCM
        
        if (format_tag !=WAV_FMT_PCM)
        { // not a PCM wav file
        	fclose(fp);
          return -4;
        }
        
        fread(&channels, sizeof(short),1,fp); //1 mono, 2 stereo
        fread(sample_rate, sizeof(int), 1, fp); //like 44100, 22050, etc...
        fread(&avg_bytes_sec, sizeof(int), 1, fp); //samplerate*channels*(bitspersample/8)
        fread(&block_align, sizeof(short), 1, fp); //block alignment (found by: (bitspersample/8)*channels)
        fread(bits_per_sample, sizeof(short), 1, fp); //8 bit or 16 bit file?
        fread(id, sizeof(char), 4, fp); //read in 'data'      
        fread(data_size, sizeof(int), 1, fp); // size of sound data
        
       // printf("DEBUG: realloc buffer for  data size %d | %c | sample_rate %d Hz | %d bits \n",*data_size,channels?'s':'m',*sample_rate,*bits_per_sample);
        
        sound_buffer_tmp = (char*) realloc (*sound_buffer, *data_size * sizeof(char)); //set aside sound buffer space
        
        if (sound_buffer_tmp == NULL)
        {
        	free(*sound_buffer);
        	*sound_buffer=NULL;
        	return -5;
        } else
        {
        	*sound_buffer=sound_buffer_tmp;
        }
        
        loaded=fread(*sound_buffer, sizeof(char), *data_size, fp); //read in our whole sound data chunk
        fclose(fp);
        
        
        if (loaded !=*data_size)
        {
          return -6;
        }
        
        
        
        
      }
      else
        {
         // RIFF file but not a wave file
         fclose(fp);
         return -3;	
        } 
    }
    else
     {
       // not a RIFF file
       fclose(fp); 
     	 return -2;
     }   
     
  } 
  else
  {
  	// cannot open file 
    return -1;    
  }
  
  
  return 0;
  
}

/*! save a PCM wave file
 *
 *
 * \param fname filename
 * \param sound_buffer
 * \param data_size length of the soundbuffer
 * \param channels number of channels (1 mono, 2 stereo)
 * \param bits_per_sample number of bits per sample (8 or 16)
 * \param sample_rate sample rate (44100,22050,...) 
 *
 * \return A value:
 *       - -1 cannot open file 
 *       - -2 file but not a wave file
 *			 - -3 cannot allocate memory in the buffer	
 *       - 0 on success
 */
int save_wav_file(char *fname, char *sound_buffer,int data_size, short channels, short bits_per_sample, int sample_rate)
{
  FILE *fp;
	char ids[5][4] = {"RIFF","WAVE","fmt ","data"};


  fp = fopen(fname,"wb");
  if (fp)
  {
    int size; //32 bit value to hold file size
    short format_tag, block_align; //our 16 bit values
    int format_length, avg_bytes_sec, i; //our 32 bit values

												

    fwrite(ids[0], sizeof(char), 4, fp); // write in first four bytes "RIFF"
    size=36+data_size;
    
  
    fwrite(&size, sizeof(int), 1, fp); // write in 32bit size value

    fwrite(ids[1], sizeof(char), 4, fp); //write in 4 byte "WAVE"
		
    fwrite(ids[2], sizeof(char), 4, fp); //write in 4 bytes "fmt ";
    format_length=16;
    fwrite(&format_length, sizeof(int),1,fp);
    format_tag=WAV_FMT_PCM;
    fwrite(&format_tag, sizeof(short), 1, fp); //check mmreg.h (i think?) for other
                                                  // possible format tags like ADPCM
    fwrite(&channels, sizeof(short),1,fp); //1 mono, 2 stereo
    fwrite(&sample_rate, sizeof(int), 1, fp); //like 44100, 22050, etc...
    
    avg_bytes_sec=sample_rate*channels*(bits_per_sample/8);
    fwrite(&avg_bytes_sec, sizeof(int), 1, fp); //samplerate*channels*(bitspersample/8)
    block_align=(bits_per_sample/8)*channels;
    fwrite(&block_align, sizeof(short), 1, fp); //block alignment (bitspersample/8)*channels
    fwrite(&bits_per_sample, sizeof(short), 1, fp); //8 bit or 16 bit file?
    fwrite(ids[3], sizeof(char), 4, fp); //write in 'data'      
    fwrite(&data_size, sizeof(int), 1, fp); //how many bytes of sound data we have     
    fwrite(sound_buffer, sizeof(char), data_size, fp); //read in our whole sound data chunk
    
    fclose(fp);
  }
  else
  {
    // cannot open file 
   return -1;
  } 
  
}
