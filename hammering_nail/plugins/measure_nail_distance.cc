#include <iostream>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>


#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "nail_distance.pb.h"
#include "nail_sensor.pb.h"
#include "check_time.pb.h"


namespace gazebo
{

    // Typedef to store a pointer to the defined msg type NailDistance that
    // will be received in the callback function 

    typedef const boost::shared_ptr<const nail_sensor_msgs::msgs::NailCollision> NailCollisionPtr;
            
    typedef const boost::shared_ptr<const time_msgs::msgs::CheckTime> CheckTimePtr;

    class MeasureNailDistance : public WorldPlugin
    {

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
        {
	
           // Store the pointer to the model, links and joint
            this->world = _parent;
		
            // 0 - ground, 1 - tool, 2 - nail
            this->tool = this->world->GetModel(1);
            this->nail = this->world->GetModel(3);
            
            this->original_pose = this->nail->GetWorldPose();
            
            // Store the pointer to the node
            this->node = transport::NodePtr(new transport::Node());
            // Init the node
            this->node->Init();
            // Init subsriber on this topic and callback MeasureDistance function
            this->sub = this->node->Subscribe("~/nail/collision",
                &MeasureNailDistance::Measure, this);
            // Store the pointer to the node
            this->node2 = transport::NodePtr(new transport::Node());
            // Init the node
            this->node2->Init();
            // Init subsriber on this topic and callback MeasureDistance function
            this->sub2 = this->node2->Subscribe("~/nail/checktime",
                &MeasureNailDistance::GreatAttractor, this);


        }

        public: void Calculate() {
            
            this->final_distance = this->original_pose.pos.z - this->final_pose.pos.z;
            
			if(this->final_distance >= 0.01) {
				this->category = 4;
			} 
			else if (final_distance > 0.005) {
				this->category = 3;
			}
			else if (final_distance > 0.001) {
				this->category = 2;
			}
			else if(final_distance < 0.001) {
				this->category = 1;
			}

        }

        public: void Measure(NailCollisionPtr &msg) {
			nail_col = msg->collision();
            if(nail_col){
				if (nail_hasnt_been_hit_before)
					nail_col_time = this->world->GetSimTime().Double();
				else
					nail_hasnt_been_hit_before = false;
			}
        }
        
        /*
         * At every OnUpdate step of Gazebo, this function will be called to check the time.
         * If there is nail collision, it sleeps for a bit and then measures the final nail position (it lets the nail settle down)
         * If, after 2 seconds, there is no nail collision, the simulator moves towards the centre of the Great Attractor :( - i.e. stops
         */
        public: void GreatAttractor(CheckTimePtr &msg) {
			if (nail_col) {				
				// wait 1 simulation sec after hitting the nail
				counter_before_measuring_nail++;
				if (VERBOSE) std::cout << "\tWaiting to measure nail (Step)\t\t" << counter_before_measuring_nail << std::endl;
				if (counter_before_measuring_nail > N_STEPS_BEFORE_MEASURING_NAIL) {
					if (VERBOSE) std::cout << "\tMeasuring nail..." << std::endl;	
					this->final_pose = this->nail->GetWorldPose();
					MeasureNailDistance::Calculate();
					std::cout << this->final_distance << std::endl;	
					system("pkill gzserver");
				}
			}	  
			else
				if (this->world->GetSimTime().Double() > MAX_TIME)
				{
					if (VERBOSE) std::cout << "\tReached maximum time for simulation:\t\t" << MAX_TIME << std::endl;	
					std::cout << this->final_distance << std::endl;	
					system("pkill gzserver");
				}				  
		}
		
		
		// verbose mode
        private: const bool VERBOSE = false; 
        
        private: const float MAX_TIME = 2;
		
		private: int counter_before_measuring_nail= 0;
        private: const int N_STEPS_BEFORE_MEASURING_NAIL = 500;
		
		
		private: bool nail_hasnt_been_hit_before = true;
		
		private: bool nail_col = false;
		
		private: double final_distance = -1;
		private: double category = -1;
		private: double nail_col_time = -1;

        // Pointer to the world
        private: physics::WorldPtr world;
        // Pointer to the model
        private: physics::ModelPtr nail, tool;
        // Store the pose of the nail before and after collision
        private: gazebo::math::Pose original_pose, final_pose;
        // Pointer to the node
        private: transport::NodePtr node;
        // Pointer to the subscriber
        private: transport::SubscriberPtr sub;
        // Will hold the tool's suitability based on how much the nail has travelled
        private: std::string suitability;
        
        
        // Pointer to the node
        private: transport::NodePtr node2;
        // Pointer to the subscriber
        private: transport::SubscriberPtr sub2;


    };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(MeasureNailDistance)
}
