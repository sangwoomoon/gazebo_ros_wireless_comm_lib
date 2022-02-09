#ifndef _GAZEBO_ROS_WIRELESS_RECEIVER_PLUGIN_HH_
#define _GAZEBO_ROS_WIRELESS_RECEIVER_PLUGIN_HH_

#include <string>
#include <ros/ros.h>
#include <sdf/Param.hh>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/util/system.hh"


namespace gazebo
{
  /// \brief Plugin for a wireless receiver sensor.
  class WirelessReceiverPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: WirelessReceiverPlugin();

    /// \brief Destructor.
    public: ~WirelessReceiverPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the wireless receiver sensor's update signal.
    public: void SensorUpdated();

    /// \brief Pointer to the contact sensor
    private: 

      // for signal identification: is it from targets? or for communication?
      float commMinFreq, commMaxFreq, measMinFreq, measMaxFreq;
      // for communication model configuration: Quadrature Phase Shift Keyring (QPSK)
      float commNoiseFloor;
      uint16_t commByteNum, commIdFreq;

      ros::NodeHandle* commRosNode;
      ros::NodeHandle* measRosNode;
      ros::Publisher commRosPub;
      ros::Publisher measRosPub;
      sensors::WirelessReceiverPtr parentSensor;
      physics::LinkPtr parentLink;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;
  };

}
#endif