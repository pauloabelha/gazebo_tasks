#include <iostream>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "nail_sensor.pb.h"
#include "nail_distance.pb.h"

namespace gazebo {

    // Typedef to store a pointer to the defined msg type NailCollision that
    // will be received in the callback function 
    typedef const boost::shared_ptr<
        const nail_sensor_msgs::msgs::NailCollision>
            NailCollisionPtr;

    class Hammer : public ModelPlugin {

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {

            // Store the pointer to the model, link and joint
            this->model = _parent;
            this->tool = this->model->GetLink("tool");
            this->joint = this->model->GetJoint("joint");
            // Store the pointer to the nodes
            this->sub_node = transport::NodePtr(new transport::Node());
            this->pub_node = transport::NodePtr(new transport::Node());
            // Init the nodes
            this->sub_node->Init();
            this->pub_node->Init();
            // Init subsriber on this topic and callback ApplyTorque function
            this->sub = this->sub_node->Subscribe("~/nail/collision",
                &Hammer::ApplyTorque, this);
            // Init the topic that will be published - Nail/MeasureDistance topic and the
            // message will be of type NailDistance
            this->pub = this->pub_node->Advertise<nail_distance_msgs::msgs::NailDistance
            >("~/nail/distance");
		
		//calculate torque to be applied getting ang Y accel as max between 2m/s and inverse of mass
		double mass = this->tool->GetInertial()->GetMass();
		double ang_accel_y = std::min((double)MAX_ANG_ACCEL_Y,1/mass); //rad/s^2 		
		torque_y = std::max((double)1,mass * ang_accel_y);
		//std::cout << "Torque | Mass | Angular Acceleration: " << torque_y << "\t" << mass << "\t" << ang_accel_y << std::endl;            
        }

        /*public: void ControlJoint() {
            // Thread to sleep before setting the angles of the joint
            // to the origin 0, 0		
            usleep(1000000); // sleep in microseconds
            this->joint->SetHighStop(0, 0);
            this->joint->SetLowStop(0, 0);
        }*/


        public: void ApplyTorque(NailCollisionPtr &msg) {
			counter_steps_approach_stuck++;
			if (counter_steps_approach_stuck > MAX_STEPS_FOR_APPROACH_STUCK) {
				std::cout << -1;
				if (VERBOSE) std::cout << "\tTool never hit the nail" << std::endl;				
				system("pkill gzserver");							
			}
			// if collision with nail
			if(msg->collision()) {
				this->tool->SetTorque(TORQUE_AFTER_COLLISION);
				if (VERBOSE) std::cout << "\tApplying torque after collision (Torque)\t\t" << TORQUE_AFTER_COLLISION << std::endl;
			}
			else {
				this->tool->SetTorque(TORQUE_BEFORE_COLLISION);  
				if (VERBOSE) std::cout << "\tApproaching nail (Torque)\t\t" << TORQUE_BEFORE_COLLISION << std::endl; 				
			}
        }
        
        // verbose mode
        private: const bool VERBOSE = false;  
        
        private: int counter_steps_approach_stuck = 0;
        private: const int MAX_STEPS_FOR_APPROACH_STUCK = 5000; 
		private: const math::Vector3 TORQUE_BEFORE_COLLISION = math::Vector3(0, 2, 0);
		private: const math::Vector3 TORQUE_AFTER_COLLISION = math::Vector3(0, 0, 0);
        // Pointer to the model
        private: physics::ModelPtr model;
        // Pointer to a link
        private: physics::LinkPtr tool;
        // Pointer to the joint
        private: physics::JointPtr joint;
        // Pointer to the node
        private: transport::NodePtr sub_node, pub_node;
        // Pointer to the subscriber
        private: transport::SubscriberPtr sub;
        // Pointer to the Publisher
        private: transport::PublisherPtr pub;
        // Pointer to the msg
        private: nail_distance_msgs::msgs::NailDistance nail_dis;
        // A thread to wait before publishing to Nail Distance 
        private: boost::thread* JointThread;

	private: const double MAX_ANG_ACCEL_Y = 4; //rad/s
	private: double torque_y = 0; 
    };

    // Register plugin
    GZ_REGISTER_MODEL_PLUGIN(Hammer)
}
