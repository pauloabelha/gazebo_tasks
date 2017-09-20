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
#include "pancake_sensor.pb.h"
#include "measure_pancake.pb.h"
#include "pancake_stuck.pb.h"

namespace gazebo {

    // Typedef to store a pointer to the defined msg type PancakeCollision
    typedef const boost::shared_ptr<
        const pancake_sensor_msgs::msgs::PancakeCollision>
            PancakeCollisionPtr;

    // typedef to  store a pointer to the second msg to be received
    typedef const boost::shared_ptr<
                const pancake_stuck_msgs::msgs::PancakeStuck>
                    PancakeStuckPtr;    

    class Lift : public ModelPlugin {

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {

            // Store the pointer to the model, tool link and joint
            this->model = _parent;
            this->tool = this->model->GetLink("tool");
            this->joint = this->model->GetJoint("joint_elbow");
            if (!this->joint)
				this->joint = this->model->GetJoint("joint");
            // Store the pointer to the node
            this->pub_node = transport::NodePtr(new transport::Node());
            this->sub_node1 = transport::NodePtr(new transport::Node());
            this->sub_node2 = transport::NodePtr(new transport::Node());
            // Init the node
            this->pub_node->Init();
            this->sub_node1->Init();
            this->sub_node2->Init();
            // Init subsriber on this topic and callback ApplyVelocity function
            this->sub1 = this->sub_node1->Subscribe("~/pancake/collision",
                &Lift::ApplyVelocity, this);


            this->sub2 = this->sub_node2->Subscribe("~/pancake/stuck",
                &Lift::Stop, this);
            // Init the topic that will be published
            this->pub = this->pub_node->Advertise<measure_pancake_msgs::msgs::MeasurePancake>("~/pancake/stop");
            this->tool_initial_height = this->tool->GetWorldPose().pos.z;
            
        }

        public: void Stop(PancakeStuckPtr &msg) {

            //send stuck message as true
            this->measure_pan.set_stopped(false);
            this->measure_pan.set_stuck(true);
            this->pub->Publish(this->measure_pan);
            this->threadCount ++;

        }

        public: void PublishMsg() {

            // sleep in microseconds
            //usleep(1000000); // 1 second
            
            //send stop message as true
            
            this->measure_pan.set_stopped(true);
            this->measure_pan.set_stuck(false);
            this->pub->Publish(this->measure_pan);
            if (VERBOSE) std::cout << "\tPublishing message of pancake status: " << std::endl;
            if (VERBOSE) std::cout << "\t\tStopped: " << this->measure_pan.stopped() << std::endl;
            if (VERBOSE) std::cout << "\t\tStuck: " << this->measure_pan.stuck() << std::endl;
            if (VERBOSE) std::cout << "\t\tLift Height: " << this->measure_pan.lift_height() << std::endl;

        }


        public: void ApplyVelocity(PancakeCollisionPtr &msg) {

			

            // if msg collison is false, there is no collision with the back box yet
            if(!msg->collision()) {
				//if (VERBOSE) std::cout << "\tNo collision with back box..." << std::endl;
                // apply linear velocity in X to make tool move to the pancake
                this->tool->SetLinearVel(math::Vector3(APPROACH_VELOCITY, 0, 0)); 
                // check if stopped msg has been sent and send it only once
                if (this->initCount < 1) {
                    this->measure_pan.set_stopped(false);
                    this->measure_pan.set_stuck(false);
                    this->pub->Publish(this->measure_pan);
                    this->initCount ++;
                }    
            }
            // there is collision with the box
            else {
				//if (VERBOSE) std::cout << "\tCollision with back box!" << std::endl;
                // stopped msg is false so tool stil should move
                if(!this->measure_pan.stopped()) {
                    // change axis in the joint so that the tool can lift the pancake                    
                    if (!has_changed_axis)
                    {
						this->joint->SetAxis(0, math::Vector3(0, 0, 1));
						if (VERBOSE) std::cout << "\t\t\tChanging tool axis to Z..." << std::endl;
						has_changed_axis = 1;
					}
			
                    // check if tool has not reached end point                    
                    
                    float tool_travelled_height = fabs(this->tool->GetWorldPose().pos.z - this->tool_initial_height);
                    
                    if(tool_travelled_height < LIFT_HEIGHT_TO_TRAVEL) {
						if (VERBOSE) std::cout << "\t\t\t\t\tLifting tool... (" << tool_travelled_height << " m)" << std::endl;
						/*
						math::Pose worldPose = this->tool->GetWorldPose();
						math::Quaternion quatRot = math::Quaternion(0,-0.05,0);
						worldPose = worldPose.RotatePositionAboutOrigin(quatRot);
						this->tool->SetWorldPose(worldPose);*/

                        // apply velocity in Z to lift the pancake
                        double tool_mass = tool->GetInertial()->GetMass();
                        double vel_prop = sqrt(0.2/tool_mass);
                        this->tool->SetLinearVel(math::Vector3(0, 0, LIFT_VELOCITY*vel_prop));
                    }
                    // the tool has reached end point so start the joint thread
                    else {						
                        if(this->threadCount < 1) {
							if (VERBOSE) std::cout << "\tReached end point" << std::endl;
							if (VERBOSE) std::cout << "\tt\tChanging tool axis to X..." << std::endl;
							this->joint->SetAxis(0, math::Vector3(1, 0, 0));							
							float curr_lift_height = BOX_BELOW_HEIGHT+tool_travelled_height+PANCAKE_EXTRA_Z_POSE;
							if (VERBOSE) std::cout << "\tSetting expected geight for each pancake sphere as: " << curr_lift_height<< std::endl;
							this->measure_pan.set_lift_height(curr_lift_height);
                            this->threadCount ++;
                            this->MsgThread = new boost::thread(&Lift::PublishMsg, this);
                        }
                    }  
                    
                }
                else
                {
					//if (VERBOSE) std::cout << "\tStopped msg is TRUE" << std::endl;
				}
            }
        }
        private: bool VERBOSE = 0;
        private: bool has_changed_axis = 0;
        private: float LIFT_VELOCITY = .25;
        private: float APPROACH_VELOCITY = .1;
        private: float tool_initial_height = -1;
		// amount traveled in Z by the tool after coliding with the back box and before stopping
		private: const float LIFT_HEIGHT_TO_TRAVEL = 0.15;
		// pointer to the box below (boxBelow), where the pancake lies
		private: const float BOX_BELOW_HEIGHT = 0.1;
		// safety margin on the initial Z psoe of the pancake (defines in the .world file)
		private: const float PANCAKE_EXTRA_Z_POSE = 0;
        // Pointer to the model
        private: physics::ModelPtr model;
        // Pointer to a link
        private: physics::LinkPtr tool;
        // Pointer to the joint
        private: physics::JointPtr joint;
        // Pointer to the node
        private: transport::NodePtr sub_node1, sub_node2, pub_node;
        // Pointer to the Publisher
        private: transport::PublisherPtr pub;
        // Pointer to the subscriber
        private: transport::SubscriberPtr sub1, sub2;
        // A thread to publish the stopped msg
        private: boost::thread* MsgThread;
        // Pointer to the msg that will be published
        private: measure_pancake_msgs::msgs::MeasurePancake measure_pan;
        // Integers to control publisher - it should publish msg only twice
        // Once after Init, once after lifting has stopped (in the thread)
        private: int threadCount = 0;
        private: int initCount = 0;

    };

    // Register plugin
    GZ_REGISTER_MODEL_PLUGIN(Lift)
}
