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
 
#ifndef IGNITION_GAZEBO_SYSTEMS_CESSNA_PLUGIN_HH_
#define IGNITION_GAZEBO_SYSTEMS_CESSNA_PLUGIN_HH_

#include <array>
#include <mutex>
#include <string>
#include <chrono>

#include <sdf/sdf.hh>
#include <sdf/Element.hh>

#include <ignition/math/PID.hh>
#include <ignition/msgs.hh>

#include <ignition/transport/TransportTypes.hh>
#include <ignition/transport/Node.hh>


#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/common/Plugin.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{

   /// \brief Allow moving the control surfaces of a Cessna C-172 plane. This
   /// plugin might be used with other models that have similar control surfaces.
   
   /// Parameters
   /// The plugin requires the following parameters:
   /// <propeller>         Name of the joint controlling the propeller spin.
   /// <propeller_max_rpm> Maximum angular speed in rpm.
   /// <left_aileron>      Name of the joint controlling the left aileron.
   /// <left_flap>         Name of the joint controlling the left flap.
   /// <right_aileron>     Name of the joint controlling the right aileron.
   /// <right_flap>        Name of the joint controlling the right flap.
   /// <elevators>         Name of the joint controlling the rear elevators.
   /// <rudder>            Name of the joint controlling the rudder.
   ///
   /// The following parameters are optional:
   /// <propeller_p_gain> P gain for the PID that controls the propeller's speed.
   /// <propeller_i_gain> I gain for the PID that controls the propeller's speed.
   /// <propeller_d_gain> D gain for the PID that controls the propeller's speed.
   /// <surfaces_p_gain> P gain for the PID that controls the position of the
   ///                   control surfaces.
   /// <surfaces_i_gain> I gain for the PID that controls the position of the
   ///                   control surfaces.
   /// <surfaces_d_gain> D gain for the PID that controls the position of the
   ///                   control surfaces.
   ///	

   /// <----Include topic-related info here----------->


	class CessnaPlugin
	    : public System,
        public ISystemConfigure,
        public ISystemPreUpdate

    {

    	  /// \brief Constructor.
        public: CessnaPlugin();

        /// \brief Destructor.
        public: ~CessnaPlugin() override;


        // Documentation inherited
        public: void Configure(const Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               EntityComponentManager &_ecm,
                               EventManager &_eventMgr) override;


        // Documentation inherited
        public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                  ignition::gazebo::EntityComponentManager &_ecm) override;

     

        /// \brief Callback executed when a new message containing control commands
        /// is received.
        /// \param[in] _msg New message containing control commands.
        private: void OnControl(const auto &_msg);

        /// \brief Update PID Joint controllers.
        /// \param[in] _dt time step size since last update.
        private: void UpdatePIDs(_dt, EntityComponentManager &_ecm);

        /// \brief Publish Cessna state.
        private: void PublishState();

        /// \params values
        private: static const unsigned int kLeftAileron  = 0;
        

        private: static const unsigned int kLeftFlap     = 1;
        
        
        private: static const unsigned int kRightAileron = 2;
        
        
        private: static const unsigned int kRightFlap    = 3;
        
        
        private: static const unsigned int kElevators    = 4;
        
        
        private: static const unsigned int kRudder       = 5;
        

        private: static const unsigned int kPropeller    = 6;

        
        /// \brief Node used for using ignition communications.
        public: ignition::transport::Node node;


        public: std::string jointName;

        /// \brief Subscriber for subscribing to a topic.
        private: ignition::transport::Node controlSub;

        /// \brief Publisher for publishing on the topic
        private: ignition::transport::Node::Publisher statePub;


        public: std::array<ignition::gazebo::Entity, 7> joint;

        //public: const unsigned int joint;
        
        /// \brief Pointer to the model;
        public: Model model{ignition::gazebo::kNullEntity};


        /// \brief Max propeller RPM.
        private: int32_t propellerMaxRpm = 2500;


        /// \brief Next command to be applied to the propeller and control surfaces.
        private: std::array<float, 7> cmds;
         
        /// \brief Velocity PID for the propeller.
        private: ignition::math::PID propellerPID;

        /// \brief Position PID for the control surfaces.
        private: std::array<ignition::math::PID, 6> controlSurfacesPID;

        /// \brief keep track of controller update sim-time.
	      private: std::chrono::steady_clock::duration lastControllerUpdateTime{0};

        /// \brief Controller update mutex.
        private: std::mutex mutex;

        /// \brief Ignition node used for using Gazebo communications.
        private: ignition::transport::Node nodeIgn;

    };

}
}
}
}

#endif //IGNITION_GAZEBO_SYSTEMS_CESSNA_PLUGIN_HH_
