#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "gazebo/common/common.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "nail_sensor.pb.h"
#include "check_time.pb.h"

namespace gazebo
{

  class NailCollision : public SensorPlugin
  {
    
  public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) {

    // Get the parent sensor.
    this->parentSensor =
      std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

    // Create and initialize the node
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();

    // Init the topic that will be published - Nail/Distance topic and the
    // message will be of type NailCollision
    this->pub = this->node->Advertise<nail_sensor_msgs::msgs::NailCollision>("~/nail/collision");
        
    // Create and initialize the node
    this->node2 = transport::NodePtr(new transport::Node());
    this->node2->Init();
        
    this->pub2 = this->node2->Advertise<time_msgs::msgs::CheckTime>("~/nail/checktime");

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
      {
	gzerr << "NailCollision requires a ContactSensor.\n";
	return;
      }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(
								std::bind(&NailCollision::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
        
    this->check_time.set_aux(false);

  }

  public: void OnUpdate() {	
    // update the counter to decide whether to send messages			
    this->counter_msg++;			
    // Get all the contacts of this model
    msgs::Contacts contacts = this->parentSensor->Contacts();
    // If there is not collision with the nail:
    if(!this->nail_col.collision()) {					
      //check contacts
      if(contacts.contact_size() == 0)
	this->nail_col.set_collision(false);
      else {
	// check all collisions
	for ( int i = 0; i < contacts.contact_size(); ++i){
	  // if the current collison is with the nail:
	  if(contacts.contact(i).collision1() == "nail::nail::nail_collision") {
	    this->nail_col.set_collision(true);
	    contact_ix = i;				
	    break;
	  }
	  // If No collision with the nail, 
	  else
	    this->nail_col.set_collision(false);
	}
      }						
    }		
    // accordign to the message frequency,
    // publishes message containing collision with the nail
    // publishes message to check time
    /*
    if (this->counter_msg == this->MSG_FREQUENCY)
      {					
	if (contact_ix >= 0){
	  for (contact_ix=0; contact_ix < contacts.contact_size(); contact_ix++){			
	    for (int j=0; j < contacts.contact(contact_ix).wrench_size(); j++){
	      double force[3] = {contacts.contact(contact_ix).wrench(j).body_1_wrench().force().x(), contacts.contact(contact_ix).wrench(j).body_1_wrench().force().y(), contacts.contact(contact_ix).wrench(j).body_1_wrench().force().z()};
	      double force2[3] = {contacts.contact(contact_ix).wrench(j).body_2_wrench().force().x(), contacts.contact(contact_ix).wrench(j).body_2_wrench().force().y(), contacts.contact(contact_ix).wrench(j).body_2_wrench().force().z()};
	      double torque[3] = {contacts.contact(contact_ix).wrench(j).body_1_wrench().torque().x(), contacts.contact(contact_ix).wrench(j).body_1_wrench().torque().y(), contacts.contact(contact_ix).wrench(j).body_1_wrench().torque().z()};
	      double torque2[3] = {contacts.contact(contact_ix).wrench(j).body_2_wrench().torque().x(), contacts.contact(contact_ix).wrench(j).body_2_wrench().torque().y(), contacts.contact(contact_ix).wrench(j).body_2_wrench().torque().z()};
	      if (true) {//(force[1] > 0.01 || force[2] > 0.01 || force[3] > 0.01 || force2[1] > 0.01 || force2[2] > 0.01 || force2[3] > 0.01){						
		std::cout << contacts.contact(contact_ix).wrench(j).body_1_name() << " |     ";
		std::cout << "Force: [" << contact_ix << ", " << j << "] (" << force[1] << ", " << force[2] << ", " << force[3] << ")" << " | ";
		std::cout << "Torque: [" << contact_ix << ", " << j << "] (" << torque[1] << ", " << torque[2] << ", " << torque[3] << ")" << std::endl;
		std::cout << contacts.contact(contact_ix).wrench(j).body_2_name() << " | ";
		std::cout << "Force: [" << contact_ix << ", " << j << "] (" << force2[1] << ", " << force2[2] << ", " << force2[3] << ")" << " | ";
		std::cout << "Torque: [" << contact_ix << ", " << j << "] (" << torque2[1] << ", " << torque2[2] << ", " << torque2[3] << ")" << std::endl;
		std::cout << std::endl;
		std::cout << std::endl;
	      }
	    }
	  }
	}
    */
    this->pub2->Publish(this->check_time);
    this->pub->Publish(this->nail_col);	
    this->counter_msg = 0;		
  }
	
  private: int contact_ix = -1;	
  private: const int MSG_FREQUENCY = 1;
  private: int counter_msg = 0;
    // Pointer to the sensor
  private: sensors::ContactSensorPtr parentSensor;
    // Pointer to the connection
  private: event::ConnectionPtr updateConnection;
    // Pointer to the Node
  private: transport::NodePtr node;
    // Pointer to the Publisher
  private: transport::PublisherPtr pub;
    // Pointer to the msg
  private: nail_sensor_msgs::msgs::NailCollision nail_col;
    // Pointer to the Node
  private: transport::NodePtr node2;
    // Pointer to the Publisher
  private: transport::PublisherPtr pub2;
    // Pointer to the msg
  private: time_msgs::msgs::CheckTime check_time;

  };
  GZ_REGISTER_SENSOR_PLUGIN(NailCollision)
}
#endif
