#ifndef _GAZEBO_KINECT_TILT_PLUGIN_HH_
#define _GAZEBO_KINECT_TILT_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

//#include <gazebo/common/common.hh>

#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#define PI 3.1416

using namespace std;

namespace gazebo
{
	class GazeboROSKinectTilt : public ModelPlugin
	{
	public:
		ros::NodeHandle* nh;
		ros::Subscriber sub;
		ros::Publisher pub;
		std::string robot_namespace_;
		float tiltAngle;
		float inputAngle;
		
		
	public:
		GazeboRosPtr gazeboROS;
// 		Pointer to the model
		physics::ModelPtr model;
// 		Pointer to the update event connection
		event::ConnectionPtr updateConnection;
		physics::JointPtr joint;
		
	public:	
		void tiltCallback(const std_msgs::Float64::ConstPtr& msg)
		{
			//ROS_INFO("I heard: [%f]", msg->data);
			inputAngle = msg->data;
		}
		
		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{ 

// 			Store the pointer to the model
			this->model = _parent;
			gazeboROS = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "KinectTilt" ) );
			joint = gazeboROS->getJoint ( model, "tiltJoint", "kinect_joint" );
			//joint->SetForce ( 0, 0.1 );            // for gazebo 7
			joint->SetMaxForce ( 0, 0.1 );       // for gazebo 6

// 			Listen to the update event. This event is broadcast every
// 			simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			      boost::bind(&GazeboROSKinectTilt::OnUpdate, this, _1));
			

			if (!ros::isInitialized())
			{
				ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
				  << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
				int argc = 0;
				char** argv = NULL;
				ros::init(argc,argv,"kinect_tilt",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
			}
			this->robot_namespace_ = "";
			nh = new ros::NodeHandle(this->robot_namespace_);
			sub = nh->subscribe("/tilt_angle", 10, &GazeboROSKinectTilt::tiltCallback, this);
			pub = nh->advertise<std_msgs::Float64>("/cur_tilt_angle", 100);
			inputAngle = 0;
			tiltAngle = 0;
			

			double initTiltAngle;
			string key;
			if (nh->searchParam("tilt_angle", key))
			{
			    nh->getParam(key, initTiltAngle);
			    inputAngle = initTiltAngle;
			}

		}

// 		Called by the model update start event
		void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			tiltAngle = joint->GetAngle(0).Degree();
			if (tiltAngle < -inputAngle - 0.5)
			{		
				joint->SetVelocity(0, 0.25);
			}
			else if (tiltAngle > -inputAngle + 0.5)
			{
				joint->SetVelocity(0, -0.25);
			}
			else
			{
				joint->SetVelocity(0, 0);
			}
//			cout << joint->GetAngle(0).Degree() << endl;

			std_msgs::Float64 curtiltMsg;
// 			curtiltMsg.data = tiltAngle * 180 / PI;
			curtiltMsg.data = joint->GetAngle(0).Degree();
			pub.publish(curtiltMsg);
			ros::spinOnce();
		}
		

	};

	GZ_REGISTER_MODEL_PLUGIN(GazeboROSKinectTilt)
}

#endif
