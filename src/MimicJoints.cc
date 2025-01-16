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
#include "MimicJoints.hh"
#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(
    mimic_joint::MimicJoints,
    gz::sim::System,
    mimic_joint::MimicJoints::ISystemConfigure,
    mimic_joint::MimicJoints::ISystemConfigurePriority,
    mimic_joint::MimicJoints::ISystemUpdate)
using namespace mimic_joint;

//////////////////////////////////////////////////
MimicJoints::MimicJoints()
{
}


//////////////////////////////////////////////////
MimicJoints::~MimicJoints()
{
}

gz::sim::System::PriorityType MimicJoints::ConfigurePriority(){
  return -2147483647;
}


//////////////////////////////////////////////////
void MimicJoints::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->model = gz::sim::Model(_entity);

  
  const auto _sdfClone = _sdf->Clone();

  if(!_sdfClone->HasElement("mainJoint")){
    gzerr << "No main joint specifcied, mimic plugin could not be loaded" << std::endl;
    return;
  }

  this->mainJoint = this->model.JointByName(_ecm, _sdfClone->Get<std::string>("mainJoint"));

  if(!this->mainJoint){
    gzerr << "Main joint not found in robot model, mimic plugin could not be loaded"<< std::endl;
    return;
  }

  double mainPos = 0.0;
  auto mainJointPosOpt = gz::sim::Joint(this->mainJoint).Position(_ecm);
  //Need to verify if this also happens if initial positions for main joint is provided
  if(mainJointPosOpt.has_value()){
    gzdbg << "Initial position of main joint found" << std::endl;
    mainPos = mainJointPosOpt.value()[0];
  }
  
  this->K_p = 10.0;
  if(_sdfClone->HasElement("Kp")){
    this->K_p = _sdfClone->Get<double>("Kp");
  }

  this->K_d = 0.001;
  if(_sdfClone->HasElement("Kd")){
    this->K_d = _sdfClone->Get<double>("Kd");
  }

  if(!_sdfClone->HasElement("mimicJoint")){
    gzerr << "No mimic joints specifcied, mimic plugin could not be loaded";
    return;
  }

  // Collect mimic joint names to reduces runtime of limit collection later on
  std::set<std::string> mimicJointNames = std::set<std::string>();

  for (auto mimicJoint = _sdfClone->GetElement("mimicJoint");
       mimicJoint != nullptr;
       mimicJoint = mimicJoint->GetNextElement("mimicJoint")){

    if(!mimicJoint->HasElement("joint")){
      gzerr <<  "No joint specifcied in mimic joint element, mimic plugin could not be loaded" << std::endl;
      return;
    }

    std::string jointName = mimicJoint->Get<std::string>("joint");
    mimicJointNames.insert(jointName);
    auto mimicJointEntity = model.JointByName(_ecm, jointName);

    // Initialize with default parameter values
    this->mimicJointsParams[mimicJointEntity] = {{"multiplier", 1.0},
                                            {"offset", 0.0},
                                            {"lower_limit", -DBL_MAX},
                                            {"uppper_limit", DBL_MAX},
                                            {"velocity_limit", DBL_MAX},
                                            {"effort_limit", DBL_MAX}};
                                            
    auto& params = mimicJointsParams[mimicJointEntity];

    if(mimicJoint->HasElement("multiplier")){
      params["multiplier"] = mimicJoint->Get<double>("multiplier");
    }

    if(!mimicJoint->HasElement("offset")){
      params["offset"] = mimicJoint->Get<double>("offset");
    }

    if(mimicJoint->HasElement("limit")){
      auto limitElement = mimicJoint->GetElement("limit");
      if(limitElement->HasAttribute("lower"))
        params["lower_limit"] = limitElement->Get<double>("lower");
      if(limitElement->HasAttribute("upper"))
        params["upper_limit"] = limitElement->Get<double>("upper");
      if(limitElement->HasAttribute("velocity"))
        params["velocity_limit"] = limitElement->Get<double>("velocity");
      if(limitElement->HasAttribute("effort"))
        params["effort_limit"] = limitElement->Get<double>("effort");
    }

    gzdbg << "Registered mimic joint [" << jointName << "] with multiplier ["
      << params["multiplier"] << "], offset [" 
      << params["offset"]  << "], pos limits ["
      << params["lower_limit"]  << ","
      << params["upper_limit"]  << "] velocity_limit ["
      << params["velocity_limit"]  << "] effort_limit ["
      << params["effort_limit"]  << "]"
      << std::endl;
    
    //Reset mimic joints to fit inital main joint position * multiplier + offset
    gz::sim::Joint(mimicJointEntity).ResetPosition(_ecm, std::vector<double>{mainPos * params["multiplier"] + params["offset"]});
  }



}

//////////////////////////////////////////////////
void MimicJoints::Update(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
  {
    gz::sim::Joint mainJoint = gz::sim::Joint(this->mainJoint);
    
    double mainPos = mainJoint.Position(_ecm).value()[0];
    double mainVel = mainJoint.Velocity(_ecm).value()[0];

    double cmdForce = 0.0;
    auto jointForceCmd = _ecm.Component<gz::sim::components::JointForceCmd>(this->mainJoint);
    if(jointForceCmd)
      cmdForce = jointForceCmd->Data()[0];
    
    auto jointVelCmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(this->mainJoint);
    if(jointVelCmd)
      mainVel = jointVelCmd->Data()[0];

    for (auto mimicJointEntry = mimicJointsParams.begin(); mimicJointEntry != mimicJointsParams.end(); mimicJointEntry++){
    
      gz::sim::Joint mimicJoint = gz::sim::Joint(mimicJointEntry->first);

      auto params = mimicJointsParams[mimicJointEntry->first];

      double targetPos = std::clamp(mainPos * params["multiplier"] + params["offset"], params["lower_limit"], params["upper_limit"]);
      double targetVel = std::clamp(mainVel * params["multiplier"], -params["velocity_limit"], params["velocity_limit"]);

      double posCorrectionForce = K_p * (targetPos - mimicJoint.Position(_ecm).value()[0]);
      double velCorrectionForce = K_d * (targetVel - mimicJoint.Velocity(_ecm).value()[0]);

      std::vector<double> mimicForceCmd = std::vector<double>{std::clamp(cmdForce + posCorrectionForce + velCorrectionForce, -params["effort_limit"], params["effort_limit"])};

      mimicJoint.SetForce(_ecm, mimicForceCmd);

      // Mark the component as processed
      _ecm.SetChanged(mimicJointEntry->first, gz::sim::components::JointForceCmd::typeId);    
    }
  }
  
