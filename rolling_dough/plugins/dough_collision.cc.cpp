#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>
#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "gazebo/common/common.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "dough_sensor.pb.h"

namespace gazebo
{

  class DoughCollision : public SensorPlugin
  {
    
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) {

        // Get the parent sensor.
        this->parentSensor =
          std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

        // Create and initialize the node
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init();

        // Init the topic that will be published - Dough/Collision topic and the
        // message will be of type DoughCollision
        this->pub = this->node->Advertise<dough_sensor_msgs::msgs::DoughCollision>("~/dough/collision");

        // Make sure the parent sensor is valid.
        if (!this->parentSensor)
        {
          gzerr << "DoughCollision requires a ContactSensor.\n";
          return;
        }

        // Connect to the sensor update event.
        this->updateConnection = this->parentSensor->ConnectUpdated(
            std::bind(&DoughCollision::OnUpdate, this));

        // Make sure the parent sensor is active.
        this->parentSensor->SetActive(true);

    }

    public: void OnUpdate() {

          
      // Get all the contacts of this model
      msgs::Contacts contacts;
      contacts = this->parentSensor->Contacts();

      // If there is not collision with the dough:
      if(!this->dough_col.collision()) {

        // if no collision at all, set msg as false
        if(contacts.contact_size() == 0) {

          this->dough_col.set_collision(false);
          this->pub->Publish(this->dough_col);

        }
        // there exist a collision
        else {
          // check all collisions
          for ( int i = 0; i < contacts.contact_size(); ++i){
            // if the current collison is with the nail:
            if(contacts.contact(i).collision1().find("dough") != std::string::npos) {
              // publish the message that there is collision with the nail
              this->dough_col.set_collision(true);
              this->pub->Publish(this->dough_col);
              break;
            }
            // If No collision
            else {
              // publish the message that there is no collision with the nail
              this->dough_col.set_collision(false);
              this->pub->Publish(this->dough_col);
            }
          }
        }
      }
      else{
        this->dough_col.set_collision(true);
        this->pub->Publish(this->dough_col);
      }
    }

    // Pointer to the sensor
    private: sensors::ContactSensorPtr parentSensor;
    // Pointer to the connection
    private: event::ConnectionPtr updateConnection;
    // Pointer to the Node
    private: transport::NodePtr node;
    // Pointer to the Publisher
    private: transport::PublisherPtr pub;
    // Pointer to the msg
    private: dough_sensor_msgs::msgs::DoughCollision dough_col;

  };
  GZ_REGISTER_SENSOR_PLUGIN(DoughCollision)
}
#endif
