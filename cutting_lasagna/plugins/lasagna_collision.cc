#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "gazebo/common/common.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "lasagna_sensor.pb.h"

namespace gazebo
{

  class LasagnaCollision : public SensorPlugin
  {
    
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) {
        // Get the parent sensor.
        this->parentSensor =
          std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
        // Create and initialize node
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();
        // Init the topic that will be published
        this->pub = this->node->Advertise<lasagna_sensor_msgs::msgs::LasagnaCollision>("~/lasagna/cut");

        // Make sure the parent sensor is valid.
        if (!this->parentSensor)
        {
          gzerr << "LasagnaCollision requires a ContactSensor.\n";
          return;
        }
        // Connect to the sensor update event.
        this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&LasagnaCollision::OnUpdate, this));
        // Make sure the parent sensor is active.
        this->parentSensor->SetActive(true);
    }

    public: void OnUpdate() {          
		// set collision FALSE as default
		this->lasagna_col.set_collision(false); 
		// Get all the contacts.
		msgs::Contacts contacts;
		contacts = this->parentSensor->Contacts();		    
		// if no contacts with other models
		if(contacts.contact_size() == 0) {
			if (VERBOSE) std::cout << "\tNo contact with other models, publish collision as FALSE" << std::endl;
		}
		// there are contacts with other models
		else {
			if (VERBOSE) std::cout << "\tThere is contact wiht other models" << std::endl;
			for ( int i = 0; i < contacts.contact_size(); ++i){
				std::string collision_prefix = "";
				if (contacts.contact(i).collision1().size() > 5)
					collision_prefix = contacts.contact(i).collision1().substr(0,6);					
				if (VERBOSE) std::cout << "\t\tContact " << collision_prefix << " " << i << " " << contacts.contact(i).collision1() << std::endl;
				// if the contact is with any of the lasagne elements, publish collision msg as true
				if(collision_prefix.compare("lasagn") == 0) {
					if (VERBOSE) std::cout << "\t\tCollision wiht lasagne found! Publishing collision as TRUE" << std::endl;
					this->lasagna_col.set_collision(true);	
					break;					
				}					
				// no lasagne contact
				else {
					if (VERBOSE) std::cout << "\t\tNo lasagne contact, publishing collision as FALSE" << std::endl;											
				}									
			} 				
		}
		this->pub->Publish(this->lasagna_col);
    }
    
    // verbose mode
	private: const bool VERBOSE = 0;

    // Pointer to the sensor
    private: sensors::ContactSensorPtr parentSensor;
    // Pointer to the connection
    private: event::ConnectionPtr updateConnection;
    // Pointer to the Node
    private: transport::NodePtr node;
    // Pointer to the Publisher
    private: transport::PublisherPtr pub;
    // Pointer to the msg
    private: lasagna_sensor_msgs::msgs::LasagnaCollision lasagna_col;

  };
  GZ_REGISTER_SENSOR_PLUGIN(LasagnaCollision)
}
#endif
