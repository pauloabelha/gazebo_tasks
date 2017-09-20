#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "gazebo/common/common.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "pancake_sensor.pb.h"

namespace gazebo
{

  class PancakeCollision : public SensorPlugin
  {
    
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) {

        // Get the parent sensor.
        this->parentSensor =
          std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

        // Create and initialize node

        this->pub_node = transport::NodePtr(new transport::Node());
        this->pub_node->Init();

        // Init the topic that will be published
        this->pub = this->pub_node->Advertise<pancake_sensor_msgs::msgs::PancakeCollision>("~/pancake/collision");

        // Make sure the parent sensor is valid.
        if (!this->parentSensor)
        {
          gzerr << "PancakeCollision requires a ContactSensor.\n";
          return;
        }

        // Connect to the sensor update event.
        this->updateConnection = this->parentSensor->ConnectUpdated(
            std::bind(&PancakeCollision::OnUpdate, this));

        // Make sure the parent sensor is active.
        this->parentSensor->SetActive(true);

    }

    public: void OnUpdate() {

          
      // Get all the contacts of the model.
      msgs::Contacts contacts;
      contacts = this->parentSensor->Contacts();

      // If no collision yet
      if(!this->pancake_col.collision()) {

        // if no contacts with other models
        if(contacts.contact_size() == 0) {

          // publish collision msg as false
          this->pancake_col.set_collision(false);
          this->pub->Publish(this->pancake_col);

        }
        // there are contacts with other models
        else {

          // loop through all contacts
          for ( int i = 0; i < contacts.contact_size(); ++i){
          
            // if the contact is with the box behind the pancake, publish collision msg as true
            if(contacts.contact(i).collision1() == "boxBack::boxBack::box_collision") {
              this->pancake_col.set_collision(true);
              this->pub->Publish(this->pancake_col);
              break;
            }
            // contact is not with the box behind the pancake, so publish collision msg as false
            else {
              this->pancake_col.set_collision(false);
              this->pub->Publish(this->pancake_col);
            }
          }
        }      
      }
      // contact msg is true, so keep publishing it as true
      else {
          this->pancake_col.set_collision(true);
          this->pub->Publish(this->pancake_col);
      }
    }

    // Pointer to the sensor
    private: sensors::ContactSensorPtr parentSensor;
    // Pointer to the connection
    private: event::ConnectionPtr updateConnection;
    // Pointer to the Node
    private: transport::NodePtr pub_node;
    // Pointer to the Publisher
    private: transport::PublisherPtr pub;
    // Pointer to the msg
    private: pancake_sensor_msgs::msgs::PancakeCollision pancake_col;

  };
  GZ_REGISTER_SENSOR_PLUGIN(PancakeCollision)
}
#endif