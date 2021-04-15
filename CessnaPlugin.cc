/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <functional>
#include <string>
#include <vector>

#include <sdf/sdf.hh>


#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Model.hh"

#include <ignition/plugin/Register.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/common/Profiler.hh>


#include <ignition/msgs/cessna.pb.h>
#include <ignition/msgs.hh>

#include "ignition/math/PID.hh"

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointForce.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocity.hh"


#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "CessnaPlugin.hh"

///////////////////////////////////////////////////////////////////////////////

using namespace ignition;
using namespace gazebo;
using namespace systems;



CessnaPlugin::CessnaPlugin(): System()
{

    std::array<float, 7> cmds;

    this->cmds.fill(0.0f);

    // default params for PID
    //Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);
    this->propellerPID.Init(50.0, 0.1, 1, 0.0, 0.0, 20000.0, -20000.0);
    this->propellerPID.SetCmd(0.0);

    for (auto &pid : this->controlSurfacesPID)
    {
        pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);
        pid.SetCmd(0.0);
    }
}
///////////////////////////////////////////////////////////////////////////////

CessnaPlugin::~CessnaPlugin()
{    
}

///////////////////////////////////////////////////////////////////////////////

void CessnaPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)

{
    auto model = Model(_entity);

    if (!model.Valid(_ecm))
    {
      ignerr << "Linear battery plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
      return;
    }

    // Read the required parameter for the propeller max RPMs.
    if (!_sdf->HasElement("propeller_max_rpm"))
    {
      ignerr << "Unable to find the <propeller_max_rpm> parameter." << std::endl;
      return;
    }
    this->propellerMaxRpm = _sdf->Get<int32_t>("propeller_max_rpm");

    if (this->propellerMaxRpm == 0)
    {
      ignerr << "Maximum propeller RPMs cannot be 0" << std::endl;
      return;
    }

    // Read the required joint name parameters. 

    this->jointName = _sdf->Get<std::string>("joint_name");

    if (this->jointName == "")
    {
      ignerr << "found an empty jointName parameter. "
             << "Failed to initialize.";
      return;
    }

    // Overload the PID parameter if they are available
    if (_sdf->HasElement("propeller_p_gain"))
      this->propellerPID.SetPGain(_sdf->Get<double>("propeller_p_gain"));


    if (_sdf->HasElement("propeller_i_gain"))
      this->propellerPID.SetIGain(_sdf->Get<double>("propeller_i_gain"));


    if (_sdf->HasElement("propeller_d_gain"))
      this->propellerPID.SetDGain(_sdf->Get<double>("propeller_d_gain"));

    
    if (_sdf->HasElement("surfaces_p_gain"))
    {
      for (auto &pid : this->controlSurfacesPID)
        pid.SetPGain(_sdf->Get<double>("surfaces_p_gain"));
    }

    if (_sdf->HasElement("surfaces_i_gain"))
    {
      for (auto &pid : this->controlSurfacesPID)
        pid.SetIGain(_sdf->Get<double>("surfaces_i_gain"));
    }

    if (_sdf->HasElement("surfaces_d_gain"))
    {
      for (auto &pid : this->controlSurfacesPID)
        pid.SetDGain(_sdf->Get<double>("surfaces_d_gain"));
    }

    std::string prefix = "~/" + this->model.Name(_ecm)+ "/";
    this->statePub = this->node.Advertise<msgs::Cessna>(prefix + "state");
    this->controlSub = this->node.Subscribe(prefix+"control", &CessnaPlugin::OnControl, this);

    ignlog <<"Cessna ready to fly. The force will be with you"<< std::endl;

}
////////////////////////////////////////////////////////////////////////////////

void CessnaPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{

	IGN_PROFILE("CessnaPlugin::PreUpdate");

    std::lock_guard<std::mutex> lock(this->mutex);

    if (_info.simTime > this->lastControllerUpdateTime)
    {
      // Update the control surfaces and publish the new state.
      IGN_PROFILE_BEGIN("PreUpdate");
      this->UpdatePIDs(std::chrono::duration_cast<std::chrono::duration<double> >(_info.simTime -
      this->lastControllerUpdateTime).count(), _ecm);

      IGN_PROFILE_END();
      IGN_PROFILE_BEGIN("Publish");
      this->PublishState();
      IGN_PROFILE_END();
  
    }

    this->lastControllerUpdateTime = _info.simTime;

}
////////////////////////////////////////////////////////////////////////////////

void CessnaPlugin::OnControl(const auto &_msg)
{

    IGN_PROFILE("CessnaPlugin::OnControl");

    std::lock_guard<std::mutex> lock(this->mutex);

    std::vector<bool> cmds;

    if (_msg->has_cmd_propeller_speed() &&
      std::abs(_msg->cmd_propeller_speed()) <= 1)
    {
      this->cmds[kPropeller] = _msg->cmd_propeller_speed();
    }

    if (_msg->has_cmd_left_aileron())
      this->cmds[kLeftAileron] = _msg->cmd_left_aileron();
    
    if (_msg->has_cmd_left_flap())
      this->cmds[kLeftFlap] = _msg->cmd_left_flap();
  
    if (_msg->has_cmd_right_aileron())
      this->cmds[kRightAileron] = _msg->cmd_right_aileron();
  
    if (_msg->has_cmd_right_flap())
      this->cmds[kRightFlap] = _msg->cmd_right_flap();
  
    if (_msg->has_cmd_elevators())
      this->cmds[kElevators] = _msg->cmd_elevators();
  
    if (_msg->has_cmd_rudder())
      this->cmds[kRudder] = _msg->cmd_rudder();

}
////////////////////////////////////////////////////////////////////////////////
void CessnaPlugin::UpdatePIDs(_dt, EntityComponentManager &_ecm)
{

	IGN_PROFILE("CessnaPlugin::UpdatePIDs");

    std::array<float, 7> cmds;

    // Velocity PID for the propeller
    double vel =  _ecm.ComponentData<ignition::gazebo::components::JointVelocity>(this->joint[6]);
    double maxVel = this->propellerMaxRpm*2.0*M_PI/60.0;
    double target = maxVel * this->cmds[kPropeller];
    double error = vel - target;
    double force = this->propellerPID.Update(error, _dt);
    _ecm.SetComponentData(this->joint[6], ignition::gazebo::components::JointForce({force}));

    //Position PID for the propeller
    for (size_t i=0; i<this->controlSurfacesPID.size(); i++)
    {
        double pos = _ecm.Component<components::JointPosition>(this->joint[i]);
        error = pos - this->cmds[i];
        force = this->propellerPID.Update(error, _dt);
        _ecm.SetComponentData(this->joint[i], ignition::gazebo::components::JointForce({force}));
    }

}
////////////////////////////////////////////////////////////////////////////////

void CessnaPlugin::PublishState(EntityComponentManager &_ecm)

{
    IGN_PROFILE("CessnaPlugin::PublishState");
  
    // Read current state

    auto propellerRpms = _ecm.ComponentData<components::JointVelocity>(this->joint[6])/(2.0*M_PI)*60.0;
    auto propellerSpeed = propellerRpms / this->propellerMaxRpm;
    auto leftAileron = _ecm.Component<components::JointPosition>(this->joint[2500]);
    auto leftFlap =  _ecm.Component<components::JointPosition>(this->joint[1]);
    auto rightAileron =  _ecm.Component<components::JointPosition>(this->joint[2]);
    auto rightFlap =  _ecm.Component<components::JointPosition>(this->joint[3]);
    auto elevators =  _ecm.Component<components::JointPosition>(this->joint[4]);
    auto rudder =  _ecm.Component<components::JointPosition>(this->joint[5]);

    
    msgs::Cessna msg;
    
    // Set Observed state
	  msg.set_propeller_speed(propellerSpeed);
    msg.set_left_aileron(leftAileron);
    msg.set_left_flap(leftFlap);
    msg.set_right_aileron(rightAileron);
    msg.set_right_flap(rightFlap);
    msg.set_elevators(elevators);
    msg.set_rudder(rudder);

    // Set target state

    std::array<float, 7> cmds;

    msg.set_cmd_propeller_speed(this->cmds[kPropeller]);
    msg.set_cmd_left_aileron(this->cmds[kLeftAileron]);
    msg.set_cmd_left_flap(this->cmds[kLeftFlap]);
    msg.set_cmd_right_aileron(this->cmds[kRightAileron]);
    msg.set_cmd_right_flap(this->cmds[kRightFlap]);
    msg.set_cmd_elevators(this->cmds[kElevators]);
    msg.set_cmd_rudder(this->cmds[kRudder]);

    this->statePub.Publish(msg);
    
}
////////////////////////////////////////////////////////////////////////////////

IGNITION_ADD_PLUGIN(CessnaPlugin,
                    ignition::gazebo::System,
                    CessnaPlugin::ISystemConfigure,
                    CessnaPlugin::ISystemPreUpdate)
                

IGNITION_ADD_PLUGIN_ALIAS(CessnaPlugin, "ignition::gazebo::systems::CessnaPlugin")
