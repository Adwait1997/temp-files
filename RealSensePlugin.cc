/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

/* Modified by Adwait Naik: Github- addy1997 */

#include <string>
#include <vector>

#include "RealSensePlugin.hh"

#include "Manager.hh"

#include "DepthCamera.hh"
#include "Camera.hh"

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/common/Image.hh>

#include <ignition/msgs.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include <ignition/transport/Node.hh>

// params

#define DEPTH_PUB_FREQ_HZ 60
#define COLOR_PUB_FREQ_HZ 60
#define IRED1_PUB_FREQ_HZ 60
#define IRED2_PUB_FREQ_HZ 60

#define DEPTH_CAMERA_NAME "depth"
#define COLOR_CAMERA_NAME "color"
#define IRED1_CAMERA_NAME "ired1"
#define IRED2_CAMERA_NAME "ired2"

#define DEPTH_CAMERA_TOPIC "depth"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "infrared"
#define IRED2_CAMERA_TOPIC "infrared2"

#define DEPTH_NEAR_CLIP_M 0.3
#define DEPTH_FAR_CLIP_M 10.0
#define DEPTH_SCALE_M 0.001

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::RealSensePluginPrivate
{

	/// \brief Callback that publishes a received Depth Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewDepthFrame() const;

    
    /// \brief Callback that publishes a received Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewFrame(const rendering::CameraPtr cam, const ignition::transport::Node::Publisher pub) const;
    
    
    /// \brief a model interface
	public: Model rsModel{kNullEntity};

	
	/// \brief Pointer to the Depth Camera Renderer
	public: ignition::rendering::DepthCameraPtr depthCam;


	/// \brief Pointer to the Color Camera Renderer.
	public: rendering::CameraPtr colorCam;


	/// \brief Pointer to the Infrared Camera Renderer.
	public: rendering::CameraPtr ired1Cam;


	/// \brief Pointer to the Infrared2 Camera Renderer.
	public: rendering::CameraPtr ired2Cam;


	/// \brief Pointer to the transport Node.
	public: ignition::transport::Node transportNode;


	/// \brief Store Real Sense depth map data.
	public: std::vector<uint16_t> depthMap;


	/// \brief Pointer to the Depth Publisher.
	public: transport::Node::Publisher depthPub;


	/// \brief Pointer to the Color Publisher.
	public: transport::Node::Publisher colorPub;


	/// \brief Pointer to the Infrared Publisher.
	public: transport::Node::Publisher ired1Pub;		


	/// \brief Pointer to the Infrared2 Publisher.
	public: transport::Node::Publisher ired2Pub;


	/// \brief sensor manager
	public: ignition::sensors::Manager smanager	

};

//////////////////////////////////////////////////////

RealSensePlugin::RealSensePlugin()
    : dataPtr(new RealSensePluginPrivate)
{
  this->dataPtr->depthCam = nullptr;
  this->dataPtr->ired1Cam = nullptr;
  this->dataPtr->ired2Cam = nullptr;
  this->dataPtr->colorCam = nullptr;
}

//////////////////////////////////////////////////////

RealSensePlugin::~RealSensePlugin()
{
}

//////////////////////////////////////////////////////

void RealSensePlugin::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)

{

	// Output the name of the model
  	std::cout << std::endl
              << "RealSensePlugin: The rs_camera plugin is attach to model "
              << model.Name(_ecm) << std::endl;


    // Store a pointer to the this model
    auto model = Model(_entity);

  	this->dataPtr->rsModel = model;

  	
  	
 	  // Check if camera renderers have been found successfuly
  	if (!this->dataPtr->depthCam)
  	{
   		std::cerr << "RealSensePlugin: Depth Camera has not been found"
              	  << std::endl;
    	return;
  	}
  	if (!this->dataPtr->ired1Cam)
  	{
    	std::cerr << "RealSensePlugin: InfraRed Camera 1 has not been found"
              	  << std::endl;
    	return;
 	  }
  	if (!this->dataPtr->ired2Cam)
  	{
    	std::cerr << "RealSensePlugin: InfraRed Camera 2 has not been found"
              	  << std::endl;
    	return;
  	}
  	if (!this->dataPtr->colorCam)
  	{
    	std::cerr << "RealSensePlugin: Color Camera has not been found"
              	  << std::endl;
    	return;
  	}

  	// Create renders
    sensors::Manager smanager;

    this->dataPtr->depthCam = _sdf->GetElement("depth");
    ignition::sensors::DepthCameraSensor *depthcam_ = smanager.CreateSensor<ignition::sensors::DepthCameraSensor>(this->dataPtr->depthCam);


    this->dataPtr->colorCam = _sdf->GetElement("color");
    ignition::sensors::CameraSensor *colorcam_ = smanager.CreateSensor<ignition::sensors::CameraSensor>(this->dataPtr->colorCam);


    this->dataPtr->ired1Cam = _sdf->GetElement("ired1");
    ignition::sensors::CameraSensor *ired1cam_ = smanager.CreateSensor<ignition::sensors::CameraSensor>(this->dataPtr->ired1Cam);


    this->dataPtr->ired1Cam = _sdf->GetElement("ired2");
    ignition::sensors::CameraSensor *ired2cam_ = smanager.CreateSensor<ignition::sensors::CameraSensor>(this->dataPtr->ire2Cam);

  	// Setup Publishers
  	// topic for publishing
  	std::string rsTopicRoot = "~/" + this->dataPtr->rsModel.Name(_ecm) + "/rs/stream/";

  	this->dataPtr->depthPub = this->dataPtr->transportNode.Advertise<msgs::Image>(rsTopicRoot + DEPTH_CAMERA_TOPIC, 1, DEPTH_PUB_FREQ_HZ);

  	this->dataPtr->colorPub = this->dataPtr->transportNode.Advertise<msgs::Image>(rsTopicRoot + COLOR_CAMERA_TOPIC, 1, COLOR_PUB_FREQ_HZ);

  	this->dataPtr->ired1Pub = this->dataPtr->transportNode.Advertise<msgs::Image>(rsTopicRoot + IRED1_CAMERA_TOPIC, 1, IRED1_PUB_FREQ_HZ);

  	this->dataPtr->ired2Pub = this->dataPtr->transportNode.Advertise<msgs::Image>(rsTopicRoot + IRED2_CAMERA_TOPIC, 1, IRED2_PUB_FREQ_HZ);

}

//////////////////////////////////////////////////////

void RealSensePluginPrivate::OnNewFrame(const rendering::CameraPtr cam, 
	                const ignition::transport::Node::Publisher pub) const;

{
	IGN_PROFILE("RealSensePluginPrivate::OnNewFrame");

	ignition::msgs::Image msg;

	ignition::gazebo::UpdateInfo _info;

	// Set Simulation Time
	msgs::Set(msg.mutable_time(), _info.simTime);

	// Set Image Dimensions
	msg.set_width(cam->ImageWidth());
	msg.set_height(cam->ImageHeight());

	// Set Image Pixel Format
	msg.set_pixel_format(common::Image::ConvertPixelFormat(cam->ImageFormat()));

	// Set Image Data
	msg.set_step(cam->ImageWidth() * cam->ImageHeight());
	msg.set_data(cam->ImageData(),
      cam->ImageDepth() * cam->ImageWidth() * cam->ImageHeight());

	// Publish realsense infrared stream
	//pub = this->dataPtr->transportNode.Advertise<msg::Image>("")\

	pub.Publish(msg);
}

//////////////////////////////////////////////////////

void RealSensePluginPrivate::OnNewDepthFrame() const
{
	IGN_PROFILE("RealSensePluginPrivate::OnNewDepthFrame");
  // Get Depth Map dimensions
  unsigned int imageSize = this->dataPtr->depthCam->ImageWidth() *
                           this->dataPtr->depthCam->ImageHeight();

  // Check if depthMap size is equivalent to imageSize
  if (this->dataPtr->depthMap.size() != imageSize)
  {
    try
    {
      this->dataPtr->depthMap.resize(imageSize);
    }
    catch (std::bad_alloc &e)
    {
      std::cerr << "RealSensePlugin: depthMap allocation failed: " << e.what()
                << std::endl;
      return;
    }
  }

  // Instantiate message
  msgs::Image msg;

  // Convert Float depth data to RealSense depth data
  const float *depthDataFloat = this->dataPtr->depthCam->DepthData();
  for (unsigned int i = 0; i < imageSize; ++i)
  {
    // Check clipping and overflow
    if (depthDataFloat[i] < DEPTH_NEAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_FAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depthDataFloat[i] < 0)
    {
      this->dataPtr->depthMap[i] = 0;
    }
    else
    {
      this->dataPtr->depthMap[i] =
          (uint16_t)(depthDataFloat[i] / DEPTH_SCALE_M);
    }
  }

  	// Pack realsense scaled depth map
  	// Set Simulation Time
	msgs::Set(msg.mutable_time(), _info.simTime);

	msg.set_width(this->dataPtr->depthCam->ImageWidth());
	msg.set_height(this->dataPtr->depthCam->ImageHeight());
	msg.set_pixel_format(common::Image::L_INT16);
	msg.set_step(this->dataPtr->depthCam->ImageWidth() *
                                this->dataPtr->depthCam->ImageDepth());

	msg.set_data(this->dataPtr->depthMap.data(), sizeof(*this->dataPtr->depthMap.data() * imageSize));

	// Publish realsense scaled depth map

	this->dataPtr->depthPub.Publish(msg);

}

//////////////////////////////////////////////////////////

void RealSensePlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
	IGN_PROFILE("RealSensePlugin::PreUpdate");

	if (_info.dt < std::chrono::steady_clock::duration::zero())
  	{
      ignwarn << "Detected jump back in time ["
        	  << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
              << "s]. System may not work properly." << std::endl;
  	}
}

IGNITION_ADD_PLUGIN(RealSensePlugin,
  ignition::gazebo::System,
  RealSensePlugin::ISystemConfigure,
  RealSensePlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RealSensePlugin,"ignition::gazebo::systems::RealSensePlugin")
