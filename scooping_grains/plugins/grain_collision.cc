#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "gazebo/common/common.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "grain_sensor.pb.h"

namespace gazebo
{

  class GrainCollision : public SensorPlugin
  {
    
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) {

        // Get the parent sensor.
        this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

        // Create and initialize node

        this->pub_node = transport::NodePtr(new transport::Node());
        this->pub_node->Init();

        // Init the topic that will be published
        this->pub = this->pub_node->Advertise<grain_sensor_msgs::msgs::GrainCollision>("~/grain/collision");

        // Make sure the parent sensor is valid.
        if (!this->parentSensor)
        {
			std::cout << "GrainCollision requires a ContactSensor.\n" << std::endl;
			gzerr << "GrainCollision requires a ContactSensor.\n";
			return;
        }

        // Connect to the sensor update event.
        this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&GrainCollision::OnUpdate, this));

        // Make sure the parent sensor is active.
        this->parentSensor->SetActive(true);


    }

    public: void OnUpdate() {	
		counter_steps_overall++;
		if (VERBOSE) std::cout << "Step #" << counter_steps_overall << std::endl;
		if (counter_steps_overall > MAX_STEPS_OVERALL) {
			if (VERBOSE) std::cout << "Max #steps (" << MAX_STEPS_OVERALL << ") reached :(" << std::endl;
			std::cout << -5 << std::endl;
			system("pkill gzserver");				
		}
		// Get all the contacts of the model.
		msgs::Contacts contacts;
		contacts = this->parentSensor->Contacts();
		// If no collision yet
		//if(!this->grain_col.collision()) {		  			
			// if no contacts with other models
			if(contacts.contact_size() == 0) {
				if (VERBOSE) std::cout << "\tNo contact, publish collision as FALSE" << std::endl;
				// publish collision msg as false
				this->grain_col.set_box_collision(false);
				this->grain_col.set_collision(false);
				this->pub->Publish(this->grain_col);
			}
			// there are contacts with other models
			else {
				if (VERBOSE) std::cout << "\tThere is contact" << std::endl;
				for ( int i = 0; i < contacts.contact_size(); ++i){
					std::string collision_prefix = "";
					if (contacts.contact(i).collision1().size() > 4)
						collision_prefix = contacts.contact(i).collision1().substr(0,5);					
					if (VERBOSE) std::cout << "\t\tContact " << collision_prefix << " " << i << " " << contacts.contact(i).collision1() << std::endl;
					// see if there is contact with the box
					if(collision_prefix.compare("box_g") == 0) {
						if (VERBOSE) std::cout << "\t\tCollision wiht the box found!" << std::endl;
						this->grain_col.set_box_collision(true);
					}					
					// if the contact is with any of the grains, publish collision msg as true
					if(collision_prefix.compare("grain") == 0) {
						if (VERBOSE) std::cout << "\t\tCollision wiht the grains found!" << std::endl;
						this->grain_col.set_collision(true);	
						break;					
					}					
					// contact is not with the box behind the grain, so publish collision msg as false
					else {
						if (VERBOSE) std::cout << "\t\tNo grain contact, publishing collision as FALSE" << std::endl;
						this->grain_col.set_collision(false);						
					}
					this->pub->Publish(this->grain_col);					
				} 				
			}
      //}
      /*
      // contact msg is true, so keep publishing it as true
      else {
		  if (VERBOSE) std::cout << "\tContinue publishing collision as TRUE" << std::endl;
          this->grain_col.set_collision(true);
          this->pub->Publish(this->grain_col);
      }
      * */       
    }
	// verbose mode
	private: const bool VERBOSE = 0;
	private: int counter_steps_overall = 0;
    private: const int MAX_STEPS_OVERALL = 20000;  
    // Pointer to the sensor
    private: sensors::ContactSensorPtr parentSensor;
    // Pointer to the connection
    private: event::ConnectionPtr updateConnection;
    // Pointer to the Node
    private: transport::NodePtr pub_node;
    // Pointer to the Publisher
    private: transport::PublisherPtr pub;
    // Pointer to the msg
    private: grain_sensor_msgs::msgs::GrainCollision grain_col;

  };
  GZ_REGISTER_SENSOR_PLUGIN(GrainCollision)
}
#endif
