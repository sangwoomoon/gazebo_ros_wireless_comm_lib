/* this code is based on https://answers.ros.org/question/334350/find-correct-tag-to-publish-a-ros-topic/ */
/* communication part is based on https://bitbucket.org/osrf/subt/src/default/subt-communication/subt_communication_model/src/subt_communication_model.cpp */
/* Receives all signals from targets and aircraft and make TWO Topics based on frequency-ID*/
#include "wireless_receiver_ros_plugin.h"

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <math.h>
#include <random>
#include <limits>

using namespace std;
using namespace gazebo;
using namespace sensors;


// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(WirelessReceiverPlugin)

// constructor
WirelessReceiverPlugin::WirelessReceiverPlugin():
    SensorPlugin ()
{}

// destructor
WirelessReceiverPlugin::~WirelessReceiverPlugin()
{
    this->commRosNode->shutdown();
    this->measRosNode->shutdown();
    delete this->commRosNode;
    delete this->measRosNode;
}

/* HELPER METHODS */
/////////////////////////////////////////////
inline double dbmToPow(double x) { return 0.001 * pow(10., x / 10.); }

/////////////////////////////////////////////
inline double QPSKPowerToBER(double P, double N) { return erfc(sqrt(P / N)); }


void WirelessReceiverPlugin::Load (SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    this->parentSensor = dynamic_pointer_cast<WirelessReceiver> (_sensor);
    this->parentLink = dynamic_pointer_cast<physics::Link> (physics::get_world ()->EntityByName (this->parentSensor->ParentName ()));
    // for topic naming
    string ns, measTopicName, commTopicName;


    if (!parentSensor) {
        gzerr << "WirelessReceiverRosPlugin must be instantiate within a Wireless Transmitter Sensor.\n";
        return;
    }

    // check whether ROS is initialized
    if (!ros::isInitialized ()) {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.\n");
        return;
    }
    
    // get robot namespace given by sdf file
    ns = _sdf->Get<string> ( "robotNamespace" );
    // set topic name based on robot namespace
    measTopicName = ns + "/wireless_receiver";
    commTopicName = ns + "/comm_receiver";

    // get filter range parameters
    this->commMinFreq = _sdf->Get<double> ( "commMinFreq" );
    this->commMaxFreq = _sdf->Get<double> ( "commMaxFreq" );

    this->measMinFreq = _sdf->Get<double> ( "measMinFreq" );
    this->measMaxFreq = _sdf->Get<double> ( "measMaxFreq" );

    this->commNoiseFloor = _sdf->Get<double> ( "commNoiseFloor" );
    this->commByteNum = _sdf->Get<uint64_t> ( "commByteNum" );
    this->commIdFreq = _sdf->Get<uint64_t> ( "commIdFreq" );

    // create ROS Node
    this->measRosNode = new ros::NodeHandle (measTopicName);
    this->measRosPub = this->measRosNode->advertise<std_msgs::Float32MultiArray> ("power", 1);

    this->commRosNode = new ros::NodeHandle (commTopicName);
    this->commRosPub = this->commRosNode->advertise<std_msgs::UInt16MultiArray> ("status", 1);


    updateConnection = this->parentSensor->ConnectUpdated (
                bind (&WirelessReceiverPlugin::SensorUpdated, this));
}

void WirelessReceiverPlugin::SensorUpdated ()
{
    std_msgs::Float32MultiArray measMsg;
    std_msgs::UInt16MultiArray commMsg;

    vector<pair<u_int16_t,u_int16_t>> commVector;
    vector<pair<u_int16_t,float>> measVector;
        
    vector<SensorPtr> currSensors;
    
    float measSignalStrength, commSignalStrength;
    u_int16_t measSignalFreq, commSignalFreq;
    
    float packetDropProb, ber, randDraw; //ber: Bit Error Rate
    bool bPacketReceived;

    // Gather all sensors in Gazebo
    currSensors = SensorManager::Instance ()->GetSensors ();

    // Find transmitters 
    for (int i = 0 ; i < currSensors.size () ; i++ ) {

        if (currSensors[i]->Type () == "wireless_transmitter") {
            WirelessTransmitterPtr transmitter = dynamic_pointer_cast<WirelessTransmitter> (currSensors[i]);
            if ( (this->measMinFreq <= transmitter->Freq ()) && (this->measMaxFreq >= transmitter->Freq ()) ) {
                // get signal strength and frequency by given Gazebo method
                measSignalStrength = transmitter->SignalStrength (parentLink->WorldPose (), parentSensor->Gain ());
                measSignalFreq = transmitter->Freq ();
                // RF power increased by multiple emittors
                measVector.push_back (make_pair (measSignalFreq, measSignalStrength) );
            }
            
            if ( (this->commMinFreq <= transmitter->Freq ()) && (this->commMaxFreq >= transmitter->Freq ()) ) {
                // get signal strength and frequency by given Gazebo method
                commSignalStrength = transmitter->SignalStrength (parentLink->WorldPose (), parentSensor->Gain ());
                commSignalFreq = transmitter->Freq ();
                
                // if detected transmitter is from the aircraft itself, {bPacketReceived} = 1 (successfully delivered)
                if ( commSignalFreq == commIdFreq ) {
                    ber = 0;
                    packetDropProb = 0;
                    bPacketReceived = true;
                }
                else {
                    // compute comm status based on QPSK configuration: please refer Sub-T communication model
                    // : https://bitbucket.org/osrf/subt/src/default/subt-communication/subt_communication_model/src/subt_communication_model.cpp
                    ber = QPSKPowerToBER( dbmToPow (commSignalStrength), dbmToPow (this->commNoiseFloor) );
                    packetDropProb = 1.0 - exp( this->commByteNum*log(1-ber) );
                    randDraw = ( rand () % 1000 ) / 1000.0;
                    bPacketReceived = randDraw > packetDropProb;
                }

                // commStatus increased by multiple aircraft
                commVector.push_back ( make_pair(commSignalFreq, bPacketReceived) );
            }
        }
    }

    // sort meas and comm data w.r.t. frequency
    sort(measVector.begin(), measVector.end());
    sort(commVector.begin(), commVector.end());

    // Build msg
    // please refer http://docs.ros.org/melodic/api/std_msgs/html/msg/Float32MultiArray.html for message format info    
    measMsg.layout.dim.resize (measVector.size ());
    // add signal information: see label for detail
    // measMsg.layout.dim[0].label = "size: signal freq (MHz), data: received powers";
    for (int i = 0 ; i < measVector.size () ; i++ ) {
        measMsg.layout.dim[i].size = measVector[i].first;
        measMsg.data.push_back (measVector[i].second);
    }

    // Build comm. status message: same way of building measurement signals
    // please refer http://docs.ros.org/melodic/api/std_msgs/html/msg/UInt8MultiArray.html for message format info    
    commMsg.layout.dim.resize (commVector.size ());
    for (int i = 0 ; i < commVector.size () ; i++ ) {
        commMsg.layout.dim[i].size = commVector[i].first;
        commMsg.data.push_back (commVector[i].second);
    }
    

    // publish data
    this->measRosPub.publish (measMsg);
    this->commRosPub.publish (commMsg);
}
