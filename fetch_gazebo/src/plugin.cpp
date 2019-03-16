 /*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015 Fetch Robotics Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Fetch Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Michael Ferguson

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_controllers_interface/controller_manager.h>
#include <fetch_gazebo/joint_handle.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <urdf/model.h>

namespace gazebo
{

class FetchGazeboPlugin : public ModelPlugin
{
public:
  FetchGazeboPlugin();
  ~FetchGazeboPlugin();
  virtual void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  virtual void Init();

private:
  void OnUpdate(const common::UpdateInfo& info);

  physics::ModelPtr model_;
  event::ConnectionPtr update_;
  common::Time prevUpdateTime;

  std::vector<JointHandlePtr> joints_;
  robot_controllers::ControllerManager controller_manager_;
  ros::Time last_update_time_;

  ros::Publisher joint_state_pub_;

  ros::NodeHandle nh_;
  ros::Time last_publish_;
};

FetchGazeboPlugin::FetchGazeboPlugin()
{
}

FetchGazeboPlugin::~FetchGazeboPlugin()
{
}

void FetchGazeboPlugin::Load(
  physics::ModelPtr parent,
  sdf::ElementPtr sdf)
{
  // Need to hang onto model
  model_ = parent;

  // Update each simulation iteration
  update_ = event::Events::ConnectWorldUpdateBegin(
              boost::bind(&FetchGazeboPlugin::OnUpdate, this, _1));
}

void FetchGazeboPlugin::Init()
{
  // Init time stuff
  prevUpdateTime = model_->GetWorld()->GetSimTime();
  last_publish_ = ros::Time(prevUpdateTime.Double());
  urdf::Model urdfmodel;
  if (!urdfmodel.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse URDF");  
  }

  // Init joint handles
  gazebo::physics::Joint_V joints = model_->GetJoints();
  for (physics::Joint_V::iterator it = joints.begin(); it != joints.end(); ++it)
  {
    //get effort limit and continuous state from URDF
    boost::shared_ptr<const urdf::Joint> urdf_joint = urdfmodel.getJoint((*it)->GetName());

    JointHandlePtr handle(new JointHandle(*it,
                                          urdf_joint->limits->velocity,
                                          urdf_joint->limits->effort,
                                          (urdf_joint->type == urdf::Joint::CONTINUOUS)));
    joints_.push_back(handle);
    robot_controllers::JointHandlePtr h(handle);
    controller_manager_.addJointHandle(h);
  }

  // Init controllers
  ros::NodeHandle pnh("~");
  controller_manager_.init(pnh);

  // Publish joint states only after controllers are fully ready
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

  ROS_INFO("Finished initializing FetchGazeboPlugin");
}

void FetchGazeboPlugin::OnUpdate(
  const common::UpdateInfo& info)
{
  if (!ros::ok())
    return;

  // Get time and timestep for controllers
  common::Time currTime = model_->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - prevUpdateTime;
  prevUpdateTime = currTime;
  double dt = stepTime.Double();
  ros::Time now = ros::Time(currTime.Double());

  // Update controllers
  controller_manager_.update(now, ros::Duration(dt));

  // Update joints back into Gazebo
  for (size_t i = 0; i < joints_.size(); ++i)
    joints_[i]->update(now, ros::Duration(dt));

  // Limit publish rate
  if (now - last_publish_ < ros::Duration(0.01))
    return;

  // Publish joint_state message
  sensor_msgs::JointState js;
  js.header.stamp = ros::Time(currTime.Double());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    js.name.push_back(joints_[i]->getName());
    js.position.push_back(joints_[i]->getPosition());
    js.velocity.push_back(joints_[i]->getVelocity());
    js.effort.push_back(joints_[i]->getEffort());
  }
  joint_state_pub_.publish(js);

  last_publish_ = now;
}

GZ_REGISTER_MODEL_PLUGIN(FetchGazeboPlugin)

}  // namespace gazebo
