/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#ifndef SYSTEM_PLUGIN_MIMICJOINTS_HH_
#define SYSTEM_PLUGIN_MIMICJOINTS_HH_

#include <chrono>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <float.h>

namespace mimic_joint
{

  /// \brief Example showing different ways of commanding an actor.
  class MimicJoints:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemConfigurePriority
  {
    /// \brief Constructor
    public: MimicJoints();

    /// \brief Destructor
    public: ~MimicJoints() override;

    /// Documentation inherited
    public: void Configure(const gz::sim::Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override final;
    
    public: gz::sim::System::PriorityType ConfigurePriority() override;

    // Documentation inherited
    public: void Update(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
    
    /// \brief Joint entities to their set offset and multiplier
    private: std::map<gz::sim::Entity, std::map<std::string, double>> mimicJointsParams;

    /// \brief Main joint entity
    private: gz::sim::Entity mainJoint;

    private: gz::sim::Model model;

    private: double K_p;
    private: double K_d;
  };
}

#endif
