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
#include "dough_variance.pb.h"
#include "dough_sensor.pb.h"
#include "dough_stuck.pb.h"

namespace gazebo {

    

    class Roll : public ModelPlugin {

        // typedef the message type that will be received
          typedef const boost::shared_ptr<
                        const dough_sensor_msgs::msgs::DoughCollision>
                            DoughCollisionPtr;

           // typedef the message type that will be received
          typedef const boost::shared_ptr<
                        const dough_stuck_msgs::msgs::DoughStuck>
                            DoughStuckPtr;                 

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {

            // Store the pointer to the model, links and joint
            this->model = _parent;
            this->tool = this->model->GetLink("tool");
            // Store the pointer to the node
            this->pub_node = transport::NodePtr(new transport::Node());
            this->sub_node1 = transport::NodePtr(new transport::Node());
            this->sub_node2 = transport::NodePtr(new transport::Node());
            // Init the node
            this->pub_node->Init();
            this->sub_node1->Init();
            this->sub_node2->Init();
            
            // get action centre link
            this->link_action_centre = this->model->GetWorld()->GetModel("action_centre_model")->GetLink("action_centre_link");

            // Init subsriber on this topic and callback ApplyVelocity function
            this->sub1 = this->sub_node1->Subscribe("~/dough/collision",
                &Roll::ApplyVelocity, this);


            this->sub2 = this->sub_node2->Subscribe("~/dough/stuck",
                &Roll::Stop, this);

            // Init the topic that will be published - ~/dough/variance topic and the
            // message will be of type DoughVariance
            this->pub = this->pub_node->Advertise<dough_variance_msgs::msgs::DoughVariance
            >("~/dough/variance");

			this->dough_var.set_dough_base_height(DOUGH_BASE_HEIGHT); 	
            
        }
        // callback fundtion, receives the message that the simulation will be stopped
        // sets a flag to stop the tool from rolling
        public: void Stop(DoughStuckPtr &msg) {

            this->dough_var.set_variance(true);
            this->pub->Publish(this->dough_var);
            this->variance_after = true;

        }


        public: void ApplyVelocity(DoughCollisionPtr &msg) {		
			// if tool is stuck, return -1 and kill Gazebo
			if (counter_steps_for_stuck > MAX_N_STEPS_FOR_STUCK) {
				std::cout << -1 << std::endl;
				if (VERBOSE) std::cout << "Tool got stuck :(" << std::endl;							
				system("pkill gzserver");					
			}				
			// if tool hasn't reached end point, keep moving it
			if( this->link_action_centre->GetWorldPose().pos.x <= END_POINT_X ) {
				// if there is no collision, move it with a certain angular vel
				if (!msg->collision()) {
					this->tool->SetAngularVel(math::Vector3(0, ANG_VEL_BEFORE_COLL, 0));
					if (VERBOSE) std::cout << "\tApproaching dough (Action Centre Pos; End Point)\t\t" << this->link_action_centre->GetWorldPose().pos.x << "\t\t" << END_POINT_X << std::endl;
				}
				// if there is collision, move it with the given angular vel and put force to press down
				else {
					this->tool->SetAngularVel(math::Vector3(0, ANG_VEL_AFTER_COLL, 0));
                    this->tool->SetForce(math::Vector3(0, 0, DOWN_FORCE_AFTER_COLL));
                    if (VERBOSE) std::cout << "\tRolling Dough (Action Centre Pos; End Point)\t\t" << this->link_action_centre->GetWorldPose().pos.x << "\t\t" << END_POINT_X << std::endl; 
				}				
			}
			// if tool reached end point, stop it and publish msg to measure dough variance
			else {
				this->tool->SetAngularVel(math::Vector3(0, 0, 0));
				this->tool->SetForce(math::Vector3(0, 0, 0));
				this->dough_var.set_variance(true);
				this->pub->Publish(this->dough_var);
				if (VERBOSE) std::cout << "\tTool reached end point; message published to measure dough variance" << std::endl;
			}           
        }
		
		private: const bool VERBOSE = false;
		
		private: const float CAFE_TABLE_HEIGHT = 0.78;		
		private: const float DOUGH_ELEM_THICKNESS = 0.003;
		private: const float DOUGH_BASE_HEIGHT = CAFE_TABLE_HEIGHT - DOUGH_ELEM_THICKNESS;		
		private: const float END_POINT_X = 0.9;
		private: const float ANG_VEL_BEFORE_COLL = 15;
		private: const float ANG_VEL_AFTER_COLL = 10;
		private: const float DOWN_FORCE_AFTER_COLL = -100;
		private: int counter_steps_for_stuck = 0;
		private: const int MAX_N_STEPS_FOR_STUCK = 5000;
        // Pointer to the model
        private: physics::ModelPtr model;
        // Pointer to a link
        private: physics::LinkPtr tool;
        // Pointer to the node
        private: transport::NodePtr sub_node1, sub_node2, pub_node;
        // Pointer to the Publisher
        private: transport::PublisherPtr pub;
        // Pointer to the subscriber
        private: transport::SubscriberPtr sub1, sub2;
        // Pointer to the msg that will be published
        private: dough_variance_msgs::msgs::DoughVariance dough_var;
        // Boolean to check if variance msg already published
        private: bool variance_after = false;
        // action centre link
        private: physics::LinkPtr link_action_centre;

    };

    // Register plugin
    GZ_REGISTER_MODEL_PLUGIN(Roll)
}
