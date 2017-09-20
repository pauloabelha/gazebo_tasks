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
#include "lasagna_sensor.pb.h"
#include "measure_cut.pb.h"


namespace gazebo {

    // Typedef to store a pointer to the defined msg type LasagnaCollision
    typedef const boost::shared_ptr<const lasagna_sensor_msgs::msgs::LasagnaCollision> LasagnaCollisionPtr;
    
    // Typedef to store a pointer to the defined msg type MeasureCut
    typedef const boost::shared_ptr<const measure_cut_msgs::msgs::MeasureCut> MeasureCutPtr;

    class Cut : public ModelPlugin {

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
            // Store the pointer to the model, links and joint
            this->model = _parent;
            this->tool = this->model->GetLink("tool");
            // Store the pointer to the node
            this->transport_node1 = transport::NodePtr(new transport::Node());
            this->transport_node2 = transport::NodePtr(new transport::Node());
            // Init transport nodes
            this->transport_node1->Init();
            this->transport_node2->Init();
            // Init subscriber on this topic and callback ApplyVelocity function
            this->transport_subscriber = this->transport_node1->Subscribe("~/lasagna/cut", &Cut::ApplyVelocity, this);
            // Init publisher on this topic
            this->transport_publisher = this->transport_node2->Advertise<measure_cut_msgs::msgs::MeasureCut>("~/measure/cut");
            // Get action bottom
            this->link_action_bottom = this->model->GetWorld()->GetModel("action_bottom_model")->GetLink("action_bottom_link");
        }

		public: void ApplyVelocity(LasagnaCollisionPtr &msg) {	
			if (!this->done_cutting) {
				if (!this->call_initial_measure) {
					this->measure_cut_msg.set_initial_measure(true);
					this->measure_cut_msg.set_lasagna_collision(false);
					this->measure_cut_msg.set_cut(false);
					this->measure_cut_msg.set_action_bottom_x(-1);
					this->transport_publisher->Publish(this->measure_cut_msg);
					this->call_initial_measure = true;
				}
				double action_bottom_pos = this->link_action_bottom->GetWorldPose().pos.x;		
				if (action_bottom_pos > STOP_CUTTING_X) {
					if (VERBOSE) std::cout << "Tool has reached final positioning; publishing message to measure (action bottom pos; stopping pos):\t\t" << action_bottom_pos << "\t\t" << STOP_CUTTING_X << std::endl;
					this->tool->SetLinearVel(math::Vector3(0, 0, 0));
					this->tool->SetForce(math::Vector3(0, 0, 0));
					// publish message
					this->measure_cut_msg.set_initial_measure(false);
					this->measure_cut_msg.set_cut(true);
					this->measure_cut_msg.set_action_bottom_x(action_bottom_pos);
					this->transport_publisher->Publish(this->measure_cut_msg);
					this->done_cutting = 1;
				}	
				else {			
					if(!msg->collision()) {
						if (VERBOSE) std::cout << "No Collision with lasagne; moving tool (vel; downward force; action bottom pos; stopping pos)\t\t" << VEL_B4_LASAGNE << "\t\t" << DOWN_FORCE_B4_LASAGNE << "\t\t" << action_bottom_pos << "\t\t" << STOP_CUTTING_X << std::endl;
						this->tool->SetLinearVel(math::Vector3(VEL_B4_LASAGNE, 0, 0));
						this->tool->SetForce(math::Vector3(0, 0, DOWN_FORCE_B4_LASAGNE));
					}
					else{
						if (VERBOSE) std::cout << "Collision with lasagne; moving tool (vel; downward force; action bottom pos; stopping pos)\t\t" << VEL_AFTER_LASAGNE << "\t\t" << DOWN_FORCE_AFTER_LASAGNE << "\t\t" << action_bottom_pos << "\t\t" << STOP_CUTTING_X << std::endl;
						this->measure_cut_msg.set_lasagna_collision(true);
						this->tool->SetLinearVel(math::Vector3(VEL_AFTER_LASAGNE, 0, 0));
						this->tool->SetForce(math::Vector3(DOWN_FORCE_AFTER_LASAGNE, 0, 0));
					}
				}
			}
        }
		
		// verbose mode
		private: const bool VERBOSE = 0;		
		// holds the state of being done cutting
		private: bool done_cutting = 0;
		// holds the state of initial measure of lasagna positions
		private: bool call_initial_measure = 0;
		// velocity before touching lasagne
		private: const double VEL_B4_LASAGNE = .5;
		// downward force before touching lasagne
		private: const double DOWN_FORCE_B4_LASAGNE = 0;
		// velocity after touching lasagne
		private: const double VEL_AFTER_LASAGNE = .25;
		// downward force after touching lasagne
		private: const double DOWN_FORCE_AFTER_LASAGNE = -5;
		// X of where to stop the cutting
		private: const double STOP_CUTTING_X = 1;
		// Pointer to the msg that will be published
        private: measure_cut_msgs::msgs::MeasureCut measure_cut_msg;
        // Pointer to the model
        private: physics::ModelPtr model;
        // Pointer to a link
        private: physics::LinkPtr tool;
        // Pointer to the node
        private: transport::NodePtr transport_node1, transport_node2;
        // Pointer to the subscriber
        private: transport::SubscriberPtr transport_subscriber;
        // Pointer to the publisher
        private: transport::PublisherPtr transport_publisher;
        // Link of action bottom
        private: physics::LinkPtr link_action_bottom;
    };

    // Register plugin
    GZ_REGISTER_MODEL_PLUGIN(Cut)
}
