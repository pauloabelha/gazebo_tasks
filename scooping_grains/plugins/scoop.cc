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
#include "grain_sensor.pb.h"
#include "measure_scoop.pb.h"

namespace gazebo {
    // Typedef to store a pointer to the defined msg type GrainCollision
    typedef const boost::shared_ptr<const grain_sensor_msgs::msgs::GrainCollision> GrainCollisionPtr;
    
    // Typedef to store a pointer to the defined msg type MeasureScoop
    typedef const boost::shared_ptr<const measure_scoop_msgs::msgs::MeasureScoop> MeasureScoopPtr;

    class Scoop : public ModelPlugin {
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr) {
			// Init the topic to publish measure msg
			this->pub_node = transport::NodePtr(new transport::Node());  
			this->pub_node->Init();          
			this->pub = this->pub_node->Advertise<measure_scoop_msgs::msgs::MeasureScoop>("~/measure/scoop");
			this->measure_scoop_msg.set_scooped(false);
			// Init the node to listen to grain collision msg
			this->sub_node = transport::NodePtr(new transport::Node());
			this->sub_node->Init();
			this->sub = this->sub_node->Subscribe("~/grain/collision", &Scoop::Scooping, this); 	
            // Store the pointer to the model, tool link and joint
            this->model = _parent;
			// get pointer to link of tool
            this->link_tool = this->model->GetLink("tool");
            if (this->link_tool == NULL){
				std::cout << "Could not get link of tool" << std::endl;
				std::cout << -2 << std::endl;
				system("pkill gzserver");
			}
            // get pointer to link of action part bottom
            this->link_action_bottom = this->model->GetWorld()->GetModel("action_bottom_model")->GetLink("action_bottom_link");
            if (this->link_action_bottom == NULL){
				std::cout << "Could not get link of action bottom" << std::endl;
				std::cout << -3 << std::endl;
				system("pkill gzserver");
			}
			this->joint_elbow = this->model->GetJoint("joint_elbow");
			stop_up_height = this->link_action_bottom->GetWorldPose().pos.z;   
			initial_joint_elbow_angle = fabs(joint_elbow->GetAngle(0).Degree());   
			ACTION_BOTTOM_INITIAL_X = this->link_action_bottom->GetWorldPose().pos.x;
			ACTION_BOTTOM_INITIAL_Y = this->link_action_bottom->GetWorldPose().pos.y;
			ACTION_BOTTOM_INITIAL_Z = this->link_action_bottom->GetWorldPose().pos.z;      
        }		
		// move tool down until action bottom is below a pre-defined height
		// there are two states: going_down and going_up
        public: void Scooping(GrainCollisionPtr &msg) {						
			if (waiting_for_measurement) {			
				this->pub->Publish(this->measure_scoop_msg); 
				if (VERBOSE) std::cout << "Scooping finished; published message to measure (wait vel up):\t\t" << WAIT_VEL_UP << std::endl; 
			}
			else {
				action_bottom_pos[0] = this->link_action_bottom->GetWorldPose().pos.x;
				action_bottom_pos[1] = this->link_action_bottom->GetWorldPose().pos.y;
				action_bottom_pos[2] = this->link_action_bottom->GetWorldPose().pos.z;
				double action_bottom_dist_diff = std::sqrt(pow(action_bottom_pos[0]-ACTION_BOTTOM_INITIAL_X,2)+pow(action_bottom_pos[1]-ACTION_BOTTOM_INITIAL_Y,2)+pow(action_bottom_pos[2]-ACTION_BOTTOM_INITIAL_Z,2));
				//std::cout << "Action bottom dist diff:\t\t" << action_bottom_dist_diff << std::endl;
				if (waiting_to_go_up){
					if (VERBOSE) std::cout << "Joint has reached target angle; waiting before going up (curr step; steps to wait)\t\t" << this->counter_steps << "\t\t" << STEPS_B4_GOING_UP << std::endl;
					this->counter_steps++;
					if (this->counter_steps >= STEPS_B4_GOING_UP) {
						waiting_to_go_up = false;
						going_up = true;
						vel_up = SCOOP_VEL_UP;
						if (VERBOSE) std::cout << "Finished waiting before going up; reversing to go upwards" << std::endl;						
					}
				}
				else {
					joint_elbow_angle = fabs(joint_elbow->GetAngle(0).Degree());
					if (going_down){			
						if (joint_elbow_angle-initial_joint_elbow_angle > 90) {
							going_down = false;
							waiting_to_go_up = true;								
							initial_joint_elbow_angle = joint_elbow_angle;
						}			
						else {
							counter_steps_downwards++;
							if (counter_steps_downwards > MAX_STEPS_DOWNWARDS) {
								if (VERBOSE) std::cout << "Tool got stuck while trying to scoop" << std::endl;
								std::cout << -4 << std::endl;
								system("pkill gzserver");
							}
							this->link_tool->SetAngularVel(math::Vector3(0, 1, 0));
							if (VERBOSE) std::cout << "Going down (joint angle abs diff; target diff; counter; max steps):\t\t" << joint_elbow_angle-initial_joint_elbow_angle << "\t\t" << 90 << "\t\t" << counter_steps_downwards << "\t\t" << MAX_STEPS_DOWNWARDS << std::endl;
						}
					}
					else if (going_up) {
						//if (action_bottom_dist_diff < 0.001) {
						if ((ACTION_BOTTOM_INITIAL_X - action_bottom_pos[0]) < 0.01 && action_bottom_pos[2] > ACTION_BOTTOM_INITIAL_Z) {
							this->measure_scoop_msg.set_scooped(true);
							this->measure_scoop_msg.set_cont_bottom_z(action_bottom_pos[2]);
							going_up = false;
							waiting_for_measurement = true;	
							if (VERBOSE) std::cout << "Joint has reached target angle (action bottom dist diff)" << action_bottom_dist_diff << "\t\t" << 0.01 << std::endl;
						}
						else {                        		
							this->link_tool->SetAngularVel(math::Vector3(0, -1, 0));
							if (VERBOSE) std::cout << "Going up (action bottom dist diff)" << action_bottom_dist_diff << "\t\t" << 0.01 << std::endl;
						}
					}
				}
			}
        }
        // verbose mode
        private: const bool VERBOSE = 0;              
        private: const double SCOOP_VEL_DOWN = -.025;    
        private: const double SCOOP_VEL_UP = 0.5; 
        private: const double SCOOP_VEL_UP_INCR = .01;
        private: const double WAIT_VEL_UP = .1; 
        private: double initial_joint_elbow_angle = 0;
        private: double joint_elbow_angle = 0;
        private: double counter_steps_downwards = 0;
        private: const double MAX_STEPS_DOWNWARDS = 3000;
        private: const double STOP_DOWN_HEIGHT = 0.85;
        private: int counter1 = 0;
        private: double ACTION_BOTTOM_INITIAL_X = 0;
        private: double ACTION_BOTTOM_INITIAL_Y = 0;
        private: double ACTION_BOTTOM_INITIAL_Z = 0;
        private: double action_bottom_pos[];
        private: double vel_up = SCOOP_VEL_UP;
        private: double fixed_vel_up = SCOOP_VEL_UP;
        private: int counter_steps = 0;
        private: const int STEPS_B4_GOING_UP = 100;
        private: double stop_up_height = 1;
        private: bool waiting_to_go_up = false;
        private: bool waiting_for_measurement = false;
        private: bool going_down = true;
        private: bool going_up = false;
        private: physics::ModelPtr model;
        private: physics::LinkPtr link_tool;
        private: physics::JointPtr joint_elbow;
        private: physics::LinkPtr link_action_bottom;
        private: transport::SubscriberPtr sub;
        private: transport::NodePtr sub_node;
        private: transport::NodePtr pub_node;
        private: transport::PublisherPtr pub;
        private: measure_scoop_msgs::msgs::MeasureScoop measure_scoop_msg;
    };
    // Register plugin
    GZ_REGISTER_MODEL_PLUGIN(Scoop)
}
