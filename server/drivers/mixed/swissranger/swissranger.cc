/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2006 Radu Bogdan Rusu (rusu@cs.tum.edu) (SR3000) and
 *  Patrick Beeson (pbeeson@traclabs.com) (SR4000 and SR3000 merge)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 */

/*
 * $Id $
 */

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_swissranger swissranger
 * @brief SWISSRANGER

The swissranger driver controls the Swiss Ranger SR4000 and SR3000 cameras. A broad range of
camera option parameters are supported, via the libmesasr library. The driver
provides a @ref interface_pointcloud3d interface and two @ref interface_camera
interfaces for both distance and intensity images, *or* a @ref interface_stereo
interface.


@par Compile-time dependencies

- libmesaSR.so and libMesaSR.h 
- This version of the driver works for SR3000 and SR4000 using libmesa v1.0.12-583 (http://www.mesa-imaging.ch/drivers.php)

@par Provides

- @ref interface_pointcloud3d : the 3d point cloud generated by the SWISSRANGER
- @ref interface_camera : snapshot images (both distance and intensity) taken by
                          the SWISSRANGER
- @ref interface_stereo : intensity and distance images as left and right
                          channels, and the 3d point cloud generated by the SWISSRANGER

@par Requires

- none

@par Supported configuration requests

  - none

@par Properties provided

  - auto_exposure (integer): Set to 1 to turn auto exposure on.

  - integration_time (integer): Integration time.
    - SR3k (integration_time+1)*0.200 ms
    - SR4k 0.300ms+(integration_time)*0.100 ms
    - Note auto exposure adapts integration time online

  - modulation_freq (integer): The devices employ the following values:
                    0  == 40MHz,  SR3k: maximal range 3.75m
                    1  == 30MHz,  SR3k, SR4k: maximal range 5m
                    2  == 21MHz,  SR3k: maximal range 7.14m
                    3  == 20MHz,  SR3k: maximal range 7.5m
                    4  == 19MHz,  SR3k: maximal range 7.89m
                    5  == 60MHz,  SR4k: maximal range 2.5m
                    6  == 15MHz,  SR4k: maximal range 10m
                    7  == 10MHz,  SR4k: maximal range 15m
                    8  == 29MHz,  SR4k: maximal range 5.17m
                    9  == 31MHz,  SR4k: maximal range 4.84m
                    10 == 14.5MHz, SR4k: maximal range 10.34m
                    11 == 15.5MHz, SR4k: maximal range 9.68m

  - amp_threshold (integer): Amplification threshold.
    - Setting this value will set all distance values to 0 if their amplitude is lower than the amplitude threshold

@par Configuration file options

  - model (string)
    - "SR3000" or
    - "SR4000" (default)

  - cycle_freq (integer) 
    - cycle rate (in Hertz) (default is 30 Hz)

  - also see Properties above

@par Example

@verbatim
driver
(
  name "swissranger"

  provides ["pointcloud3d:0" "distance:::camera:0" "intensity:::camera:1"]

  # OR ...

  provides ["stereo:0"]
)
@endverbatim

@author Patrick Beeson (based on sr3000 driver by Radu Bogdan Rusu)
 */
/** @} */

#include <unistd.h>
#include <stdlib.h>
#include <libplayercore/playercore.h>
#include <libMesaSR.h>

#define SR4K_MODE (AM_COR_FIX_PTRN|AM_CONV_GRAY|AM_DENOISE_ANF)
#define SR3K_MODE (AM_COR_FIX_PTRN|AM_MEDIAN)
#define CAM_ROWS 144
#define CAM_COLS 176

class SWISSRANGER:public ThreadedDriver
{
  public:
	// constructor
    SWISSRANGER (ConfigFile* cf, int section);
    ~SWISSRANGER ();

    int MainSetup ();
    void MainQuit ();

    // MessageHandler
    virtual int ProcessMessage (QueuePointer &resp_queue,
                                player_msghdr * hdr,
                                void * data);

  private:

    // Camera MessageHandler
    int ProcessMessageCamera (QueuePointer &resp_queue,
                              player_msghdr * hdr,
                              void * data,
                              player_devaddr_t);
    virtual void Main ();
    void RefreshData  ();

    // device identifier
    CMesaDevice* srCam;

    // SR specific values
    unsigned int rows, cols, inr;

    bool use_SR4k;

    ImgEntry* imgEntryArray;
    float *buffer, *xp, *yp, *zp;

    // device bookkeeping
    player_devaddr_t stereo_addr, pcloud_addr, d_cam_addr, i_cam_addr;

    player_pointcloud3d_data_t pcloud_data;
    player_camera_data_t       d_cam_data, i_cam_data;
    player_stereo_data_t       stereo_data;

    int cycle_freq;  
 
  protected:
    // Properties
    IntProperty auto_exposure, integration_time, modulation_freq, amp_threshold;

    bool providePCloud, provideDCam, provideICam, provideStereo;
};

////////////////////////////////////////////////////////////////////////////////
//Factory creation function. This functions is given as an argument when
// the driver is added to the driver table
Driver*
    SWISSRANGER_Init (ConfigFile* cf, int section)
{
  return ((Driver*)(new SWISSRANGER (cf, section)));
}

////////////////////////////////////////////////////////////////////////////////
//Registers the driver in the driver table. Called from the
// player_driver_init function that the loader looks for
void
    swissranger_Register (DriverTable* table)
{
  table->AddDriver ("swissranger", SWISSRANGER_Init);
}

////////////////////////////////////////////////////////////////////////////////
// Constructor.  Retrieve options from the configuration file and do any
// pre-Setup() setup.
SWISSRANGER::SWISSRANGER (ConfigFile* cf, int section)
	: ThreadedDriver (cf, section),
	auto_exposure ("auto_exposure", -1, 0),
	integration_time ("integration_time", -1, 0),
	modulation_freq ("modulation_freq", -1, 0),
	amp_threshold ("amp_threshold", -1, 0)
{
  memset (&this->stereo_addr, 0, sizeof (player_devaddr_t));
  memset (&this->pcloud_addr, 0, sizeof (player_devaddr_t));
  memset (&this->d_cam_addr,  0, sizeof (player_devaddr_t));
  memset (&this->i_cam_addr,  0, sizeof (player_devaddr_t));

  this->RegisterProperty ("auto_exposure", &this->auto_exposure, cf, section);
  this->RegisterProperty ("integration_time", &this->integration_time, cf, section);
  this->RegisterProperty ("modulation_freq", &this->modulation_freq, cf, section);
  this->RegisterProperty ("amp_threshold", &this->amp_threshold, cf, section);

  cycle_freq = cf->ReadInt(section, "cycle_freq", 30);
  PLAYER_MSG1(2,"Running at %d Hz", cycle_freq);

  use_SR4k=true;

  const char* model = cf->ReadString(section, "model", "SR4000");
  if (!strcmp(model,"SR3000"))
    use_SR4k=false;

  if (use_SR4k)
    PLAYER_MSG0(2,"Using SR4000 device");
  else PLAYER_MSG0(2,"Using SR3000 device");

 
  providePCloud = FALSE; provideDCam = FALSE; provideICam = FALSE; provideStereo = FALSE;

  if (cf->ReadDeviceAddr (&(this->stereo_addr), section, "provides",
      PLAYER_STEREO_CODE, -1, NULL) == 0)
  {
    if (this->AddInterface (this->stereo_addr) != 0)
    {
      this->SetError (-1);
      return;
    }
    provideStereo = TRUE;
  }
  else
  {
    // Outgoing pointcloud interface
    if (cf->ReadDeviceAddr (&(this->pcloud_addr), section, "provides",
        PLAYER_POINTCLOUD3D_CODE, -1, NULL) == 0)
    {
      if (this->AddInterface (this->pcloud_addr) != 0)
      {
        this->SetError (-1);
        return;
      }
      providePCloud = TRUE;
    }

    // Outgoing distance::camera:0 interface
    if (cf->ReadDeviceAddr (&(this->d_cam_addr), section, "provides",
        PLAYER_CAMERA_CODE, -1, "distance") == 0)
    {
      if (this->AddInterface (this->d_cam_addr) != 0)
      {
        this->SetError (-1);
        return;
      }
      provideDCam = TRUE;
    }

    // Outgoing intensity::camera:1 interface
    if (cf->ReadDeviceAddr (&(this->i_cam_addr), section, "provides",
        PLAYER_CAMERA_CODE, -1, "intensity") == 0)
    {
      if (this->AddInterface (this->i_cam_addr) != 0)
      {
        this->SetError (-1);
        return;
      }
      provideICam = TRUE;
    }

    provideStereo = FALSE;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Destructor.
SWISSRANGER::~SWISSRANGER ()
{
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device.  Return 0 if things go well, and -1 otherwise.
int
    SWISSRANGER::MainSetup ()
{
  int res;
  // ---[ Open the camera ]---
  res = SR_OpenUSB (&srCam, 0);          //returns the device ID used in other calls

  PLAYER_MSG0 (1, "> Connecting to SWISSRANGER... [done]");

  if (auto_exposure == 1) {
    if (use_SR4k) {
      if (SR_SetAutoExposure (srCam, 1, 150, 5, 70)>=0)
	PLAYER_MSG0(2,"Turned on auto exposure mode");
      else PLAYER_MSG0(2,"Could not change auto exposure mode");
    }
    else {
      if (SR_SetAutoExposure (srCam, 2, 255, 10, 45)>=0)
	PLAYER_MSG0(2,"Turned on auto exposure mode");
      else PLAYER_MSG0(2,"Could not change auto exposure mode");
    }
  }
  else 
    if (auto_exposure == 0) {
      if (SR_SetAutoExposure (srCam, 255, 0, 0, 0)>=0)
	PLAYER_MSG0(2,"Turned off auto exposure mode");
      else PLAYER_MSG0(2,"Could not change auto exposure mode");
    }
    else PLAYER_MSG0(2,"Not changing current auto exposure mode");
  
  if (integration_time >= 0 && integration_time <= 255)
    if (SR_SetIntegrationTime (srCam, integration_time)>=0)
      PLAYER_MSG1(2,"Set integration time to %d", integration_time.GetValue());
    else PLAYER_MSG0(2,"Could not set integration time");
  else PLAYER_MSG0(2,"Not changing current integration time");

  if (modulation_freq >= 0 && modulation_freq <= 11)
    if (SR_SetModulationFrequency (srCam, (ModulationFrq)(modulation_freq.GetValue()))>=0) {
      PLAYER_MSG1(2,"Set modulation frequency to mode %d", modulation_freq.GetValue());
    }
    else PLAYER_MSG0(2,"Could not set modulation frequency");
  else PLAYER_MSG0(2,"Not changing current modulation frequency");
  PLAYER_MSG1(2,"Modulation Frequency: %d",(int)SR_GetModulationFrequency(srCam));


  if (amp_threshold >= 0)
    if (SR_SetAmplitudeThreshold (srCam, amp_threshold)>=0)
      PLAYER_MSG1(2,"Set amplitude threshold to %d", amp_threshold.GetValue());
    else PLAYER_MSG0(2,"Could not set amplitude threshold");
  else PLAYER_MSG0(2,"Not changing current amplitude threshold");


  // ---[ Get the number of rows, cols, ... ]---
  rows = SR_GetRows (srCam);
  cols = SR_GetCols (srCam);
  inr  = SR_GetImageList (srCam, &imgEntryArray);
  modulation_freq  = SR_GetModulationFrequency (srCam);
  integration_time = SR_GetIntegrationTime (srCam);


  if ( (cols != CAM_COLS) || (rows != CAM_ROWS) || (inr < 1) || (imgEntryArray == 0) )
  {
    PLAYER_ERROR ("> Error while connecting to camera!");
    SR_Close (srCam);
    return (-1);
  }

  // ---[ Set the acquisition mode ]---
  if (use_SR4k)
    SR_SetMode (srCam, SR4K_MODE);
  else SR_SetMode (srCam, SR3K_MODE);

  // Points array
  size_t buffer_size = rows * cols * 3 * sizeof (float);
  buffer = (float*)malloc (buffer_size);
  memset (buffer, 0xaf, buffer_size);

  xp = buffer;
  yp = &xp[rows*cols];
  zp = &yp[rows*cols];

  PLAYER_MSG0(2, ">> Ready");

  return (0);
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
void
    SWISSRANGER::MainQuit ()
{
  StopThread ();

  // ---[ Close the camera ]---
  int res = SR_Close (srCam);

  PLAYER_MSG1 (1, "> SWISSRANGER driver shutting down... %d [done]", res);

  // ---[ Free the allocated memory buffer ]---
  free (buffer);

}

////////////////////////////////////////////////////////////////////////////////
// Process messages from/for a camera interface
int
    SWISSRANGER::ProcessMessageCamera (QueuePointer &resp_queue,
                                  player_msghdr * hdr,
                                  void * data,
                                  player_devaddr_t stereo_addr)
{
  int res;

  // Check for properties
  if (Message::MatchMessage (hdr, PLAYER_MSGTYPE_REQ, PLAYER_SET_INTPROP_REQ, stereo_addr))
  {
    player_intprop_req_t req = *reinterpret_cast<player_intprop_req_t*> (data);
    if (auto_exposure.KeyIsEqual (req.key))
    {
      // ---[ Set Autoexposure
      if (req.value == 1) {
	if (use_SR4k)
	  res = SR_SetAutoExposure (srCam, 1, 150, 5, 70);
	else
	  res = SR_SetAutoExposure (srCam, 5, 255, 10, 45);
      }
      else
        res = SR_SetAutoExposure (srCam, 255, 0, 0, 0);
      
      // Check the error code
      if (res >= 0)
      {
        auto_exposure = req.value;
        Publish (stereo_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_SET_INTPROP_REQ, NULL, 0, NULL);
      }
      else
      {
        Publish (stereo_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK, PLAYER_SET_INTPROP_REQ, NULL, 0, NULL);
      }
      return (0);
    }
    else if (integration_time.KeyIsEqual (req.key))
    {
      // ---[ Set integration time
      res = SR_SetIntegrationTime (srCam, req.value);

      // Check the error code
      if (res >= 0)
      {
        integration_time = req.value;
        Publish (stereo_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_SET_INTPROP_REQ, NULL, 0, NULL);
      }
      else
      {
        Publish (stereo_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK, PLAYER_SET_INTPROP_REQ, NULL, 0, NULL);
      }
      return (0);
    }
    else if (modulation_freq.KeyIsEqual (req.key))
    {
      // ---[ Set modulation frequency
      if (req.value >= 0 && req.value <= 11)
	res = SR_SetModulationFrequency (srCam, (ModulationFrq)req.value);
      else res=1;
      
      // Check the error code
      if (res >= 0)
	{
	  modulation_freq = req.value;
	  Publish (stereo_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_SET_INTPROP_REQ, NULL, 0, NULL);
	}
      else
	{
	  Publish (stereo_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK, PLAYER_SET_INTPROP_REQ, NULL, 0, NULL);
	}
      return (0);
    }
    else if (amp_threshold.KeyIsEqual (req.key))
    {
      // ---[ Set amplitude threshold
      res = SR_SetAmplitudeThreshold (srCam, req.value);

      // Check the error code
      if (res >= 0)
      {
        amp_threshold = req.value;
        Publish (stereo_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_SET_INTPROP_REQ, NULL, 0, NULL);
      }
      else
      {
        Publish (stereo_addr, resp_queue, PLAYER_MSGTYPE_RESP_NACK, PLAYER_SET_INTPROP_REQ, NULL, 0, NULL);
      }
      return (0);
    }
    return (-1);    // Let the default property handling handle it
  }
  else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, PLAYER_GET_INTPROP_REQ, device_addr))
  {
    player_intprop_req_t req = *reinterpret_cast<player_intprop_req_t*> (data);
    if (modulation_freq.KeyIsEqual (req.key))
    {
      // ---[ Get modulation frequency
      modulation_freq.SetValue (SR_GetModulationFrequency (srCam));
      modulation_freq.GetValueToMessage (reinterpret_cast<void*> (&req));
      Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_GET_INTPROP_REQ, reinterpret_cast<void*> (&req), sizeof(player_intprop_req_t), NULL);
      return (0);
    }
    else if (integration_time.KeyIsEqual (req.key))
    {
      // ---[ Get integration time
      integration_time.SetValue (SR_GetIntegrationTime (srCam));
      integration_time.GetValueToMessage (reinterpret_cast<void*> (&req));
      Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_GET_INTPROP_REQ, reinterpret_cast<void*> (&req), sizeof(player_intprop_req_t), NULL);
      return (0);
    }
    return (-1);    // Let the default property handling handle it
  }

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// ProcessMessage
int SWISSRANGER::ProcessMessage (QueuePointer &resp_queue,
                            player_msghdr * hdr,
                            void * data)
{
  assert (hdr);

  if (provideStereo)
    ProcessMessageCamera (resp_queue, hdr, data, stereo_addr);
  else
  {
    ProcessMessageCamera (resp_queue, hdr, data, d_cam_addr);
    ProcessMessageCamera (resp_queue, hdr, data, i_cam_addr);
  }

  return (0);
}



////////////////////////////////////////////////////////////////////////////////
void SWISSRANGER::Main ()
{

  double duration_usecs = 1000000.0 / cycle_freq;

  memset (&stereo_data, 0, sizeof (stereo_data));
  memset (&pcloud_data, 0, sizeof (pcloud_data));
  memset (&d_cam_data,  0, sizeof (d_cam_data ));
  memset (&i_cam_data,  0, sizeof (i_cam_data ));

  double start_time;
  GlobalTime->GetTimeDouble(&start_time); // start of initial cycle  

  for (;;)
  {
    // handle commands and requests/replies --------------------------------
    pthread_testcancel ();
    ProcessMessages ();
    
    // get data ------------------------------------------------------------
    if ((rows != 1) && (cols != 1))
      RefreshData ();
    
    double now;
    GlobalTime->GetTimeDouble(&now);

    double elapsed_usecs = (now - start_time) * 1000000.0;

    if (elapsed_usecs < duration_usecs) 
      while (duration_usecs - elapsed_usecs >= 500) {
	
	useconds_t delay=(useconds_t) (duration_usecs - elapsed_usecs - 499);
	// Linux usleep() rounds up to the next millisecond	
	usleep(delay);
    	GlobalTime->GetTimeDouble(&now);
    	elapsed_usecs = (now - start_time) * 1000000.0;
      }
    
    GlobalTime->GetTimeDouble(&start_time);
  }
}

////////////////////////////////////////////////////////////////////////////////
// RefreshData function
void SWISSRANGER::RefreshData ()
{
  int res;
  unsigned int i;

  res = SR_Acquire (srCam);

  uint8_t *distance_image  = (unsigned char*)imgEntryArray->data;
  uint8_t *intensity_image = (unsigned char*)imgEntryArray->data + (imgEntryArray->width * imgEntryArray->height * 2);
//  buffer_size/2;

  // Points array
  res = SR_CoordTrfFlt (srCam, xp, yp, zp, sizeof (float), sizeof (float), sizeof (float));

  if (provideStereo)
  {
    stereo_data.points_count = rows * cols;
    stereo_data.points = new player_pointcloud3d_stereo_element_t[stereo_data.points_count];
    for (i = 0; i < rows*cols; i++)
    {
      player_pointcloud3d_stereo_element_t element;
      element.px = xp[i];
      element.py = yp[i];
      element.pz = zp[i];

      element.red   = intensity_image[i*2 + 1];
      element.green = intensity_image[i*2 + 1];
      element.blue  = intensity_image[i*2 + 1];
      stereo_data.points[i] = element;
    }

    // Prepare distance camera data
    d_cam_data.width       = cols;
    d_cam_data.height      = rows;
    d_cam_data.bpp         = 16;
    d_cam_data.format      = PLAYER_CAMERA_FORMAT_MONO16;
    d_cam_data.fdiv        = 1;
    d_cam_data.compression = PLAYER_CAMERA_COMPRESS_RAW;
    d_cam_data.image_count = rows*cols*2;
    d_cam_data.image       = distance_image;

    stereo_data.left_channel = d_cam_data;

    // Prepare intensity camera data
    i_cam_data.width       = cols;
    i_cam_data.height      = rows;
    i_cam_data.bpp         = 16;
    i_cam_data.format      = PLAYER_CAMERA_FORMAT_MONO16;
    i_cam_data.fdiv        = 1;
    i_cam_data.compression = PLAYER_CAMERA_COMPRESS_RAW;
    i_cam_data.image_count = rows*cols*2;
    i_cam_data.image       = intensity_image;

    stereo_data.right_channel = i_cam_data;

    // Publish the stereo data
    //  int total_size = 28+(i_cam_data.image_count) +
    //                   28+(d_cam_data.image_count) +
    //                   4 + pcloud_data.points_count*sizeof (player_pointcloud3d_element_t);

    Publish (stereo_addr, PLAYER_MSGTYPE_DATA, PLAYER_STEREO_DATA_STATE, &stereo_data);  //, total_size, NULL);

    delete [] stereo_data.points;
  }
  else
  {
    // Publish pointcloud3d data if subscribed
    if (providePCloud)
    {
      pcloud_data.points_count = rows * cols;
      pcloud_data.points = new player_pointcloud3d_element_t[pcloud_data.points_count];
      for (i = 0; i < rows*cols; i++)
      {
        player_pointcloud3d_element_t element;
        element.point.px = xp[i];
        element.point.py = yp[i];
        element.point.pz = zp[i];

        element.color.alpha = 255;
        element.color.red   = intensity_image[i*2 + 1];
        element.color.green = intensity_image[i*2 + 1];
        element.color.blue  = intensity_image[i*2 + 1];
        pcloud_data.points[i] = element;
      }

      // Write the Pointcloud3d data
      Publish (pcloud_addr, PLAYER_MSGTYPE_DATA, PLAYER_POINTCLOUD3D_DATA_STATE,
               &pcloud_data); //, 4 + pcloud_data.points_count*sizeof (player_pointcloud3d_element_t), NULL);

      delete [] pcloud_data.points;
    }

    // Publish distance camera data if subscribed
    if (provideDCam)
    {
      d_cam_data.width       = cols;
      d_cam_data.height      = rows;
      d_cam_data.bpp         = 16;
      d_cam_data.format      = PLAYER_CAMERA_FORMAT_MONO16;
      d_cam_data.fdiv        = 1;
      d_cam_data.compression = PLAYER_CAMERA_COMPRESS_RAW;
      d_cam_data.image_count = rows*cols*2;
      d_cam_data.image       = distance_image;

      // Write the distance camera data
      Publish (d_cam_addr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE,
               &d_cam_data); //, 28+(d_cam_data.image_count), NULL);
    }

    // Publish intensity camera data if subscribed
    if (provideICam)
    {
      i_cam_data.width       = cols;
      i_cam_data.height      = rows;
      i_cam_data.bpp         = 16;
      i_cam_data.format      = PLAYER_CAMERA_FORMAT_MONO16;
      i_cam_data.fdiv        = 1;
      i_cam_data.compression = PLAYER_CAMERA_COMPRESS_RAW;
      i_cam_data.image_count = rows*cols*2;
      i_cam_data.image       = intensity_image;

      // Write the intensity camera data
      Publish (i_cam_addr, PLAYER_MSGTYPE_DATA, PLAYER_CAMERA_DATA_STATE,
               &i_cam_data); //, 28+(i_cam_data.image_count), NULL);
    }
  } // if (Stereo) ... else

  return;
}
