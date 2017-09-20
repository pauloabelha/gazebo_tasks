#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "gazebo/common/common.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "end_sensor.pb.h"

namespace gazebo
{

  class EndCollision : public SensorPlugin
  {
    
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) {

        // Get the parent sensor.
        this->parentSensor =
          std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

        // Create and initialize node
        
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();

        // Init the topic that will be published
        this->pub = this->node->Advertise<end_sensor_msgs::msgs::EndCollision>("~/end/collision");

        // Make sure the parent sensor is valid.
        if (!this->parentSensor)
        {
          gzerr << "EndBox requires a ContactSensor.\n";
          return;
        }

        // Connect to the sensor update event.
        this->updateConnection = this->parentSensor->ConnectUpdated(
            std::bind(&EndCollision::OnUpdate, this));

        // Make sure the parent sensor is active.
        this->parentSensor->SetActive(true);
        
        this->end_col.set_end_collision(false);
        
        this->end_col.set_table_height(TABLE_HEIGHT);
        
        std::cout << "END COLLISION SENSOR OK" << std::endl;

    }

    public: void OnUpdate() {

       std::cout << "UPDATING END COLLISION SENSOR" << std::endl;   
      // Get all the contacts of this model
      msgs::Contacts contacts;
      contacts = this->parentSensor->Contacts();

      // If there is not collision with the end:
      std::cout << this->end_col.end_collision() << std::endl;
      if(!this->end_col.end_collision()) {
		std::cout << "COLLISION FALSE" << std::endl;	
        // if no collision at all, set msg as false
        if(contacts.contact_size() == 0) {

          this->end_col.set_end_collision(false);
          this->pub->Publish(this->end_col);

        }
        // there exist a collision
        else {
			std::cout << "COLLISION TRUE" << std::endl;	
          // check all collisions
          for ( int i = 0; i < contacts.contact_size(); ++i){
            // if the current collison is the one we are looking for
            std::cout << contacts.contact(i).collision1() << std::endl;
            if(contacts.contact(i).collision1().find("end_box") != std::string::npos) {
              // publish the message that there is collision with the nail
              this->end_col.set_end_collision(true);
              this->pub->Publish(this->end_col);
              break;
            }
            else {
              // publish the message that there is no collision with the nail
              this->end_col.set_end_collision(false);
              this->pub->Publish(this->end_col);
            }
          }
        }
      }
      else{
        this->end_col.set_end_collision(true);
        this->pub->Publish(this->end_col);
      }
    }
	
	private: const float TABLE_HEIGHT = 0.78;
    // Pointer to the sensor
    private: sensors::ContactSensorPtr parentSensor;
    // Pointer to the connection
    private: event::ConnectionPtr updateConnection;
    // Pointer to the Node
    private: transport::NodePtr node;
    // Pointer to the Publisher
    private: transport::PublisherPtr pub;
    // Pointer to the msg
    private: end_sensor_msgs::msgs::EndCollision end_col;

  };
  GZ_REGISTER_SENSOR_PLUGIN(EndCollision)
}
#endif
