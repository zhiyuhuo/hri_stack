#ifndef _GAZEBO_WIRELESS_RECEIVER_PLUGIN_HH_
#define _GAZEBO_WIRELESS_RECEIVER_PLUGIN_HH_
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#define PI 3.1416

namespace gazebo
{
	/// \brief Plugin for a wireless receiver sensor.
	class GazeboROSWirelessReceiver : public SensorPlugin
	{
	public:
		ros::NodeHandle* nh;
		ros::Subscriber sub;
		ros::Publisher pub;
		std::string robot_namespace_;
		float tiltAngle;
		float inputAngle;
	  
	public:
		/// \brief Constructor.
		GazeboROSWirelessReceiver(){}

		/// \brief Destructor.
		~GazeboROSWirelessReceiver(){}

		/// \brief Load the sensor plugin.
		/// \param[in] _sensor Pointer to the sensor that loaded this plugin.
		/// \param[in] _sdf SDF element that describes the plugin.
		void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
		{
			// Get the parent sensor.
			this->parentSensor = std::dynamic_pointer_cast<sensors::WirelessReceiver>(_sensor);

			// Make sure the parent sensor is valid.
			if (!this->parentSensor)
			{
			  gzerr << "GazeboROSWirelessReceiver requires a WirelessReceiver.\n";
			  return;
			}

			// Connect to the sensor update event.
			this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&GazeboROSWirelessReceiver::OnUpdate, this));

			// Make sure the parent sensor is active.
			this->parentSensor->SetActive(true);
			std::cout << "wirelress receiver active"; 
			
			if (!ros::isInitialized())
			{
				ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
				  << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
				int argc = 0;
				char** argv = NULL;
				ros::init(argc,argv,"wireless_receiver",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
			}
			this->robot_namespace_ = "";
			nh = new ros::NodeHandle(this->robot_namespace_);
		}

		/// \brief Callback that receives the wireless receiver sensor's update signal.
		void OnUpdate()
		{
			std::cout << "recieved wireless signal";
			double power = 0.0;
			power = this->parentSensor->GetPower();
			std::cout << power << " \n";
		}

		/// \brief Pointer to the contact sensor
		sensors::WirelessReceiverPtr parentSensor;

		/// \brief Connection that maintains a link between the contact sensor's
		/// updated signal and the OnUpdate callback.
		event::ConnectionPtr updateConnection;
	};
  
  
	GZ_REGISTER_SENSOR_PLUGIN(GazeboROSWirelessReceiver)
}

#endif

