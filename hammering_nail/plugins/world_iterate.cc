#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>

#include <iostream>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "nail_distance.pb.h"

namespace gazebo
{
	
	
	typedef const boost::shared_ptr<
        const nail_distance_msgs::msgs::NailDistance>
            NailDistancePtr;
            
    class WorldIterate : public WorldPlugin
    {

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
        {
           this->world = _parent;
           this->world->InsertModelFile("model://hammer_training");
           this->world->InsertModelFile("model://nail");
           
           // Store the pointer to the node
            this->node = transport::NodePtr(new transport::Node());
            // Init the node
            this->node->Init();
            // Init subsriber on this topic and callback MeasureDistance function
            this->sub = this->node->Subscribe("~/nail/distance",
                &WorldIterate::Iterate, this);
                
		}    
		
		/*public: void (CheckTimePtr &msg) {   
			fprintf("%f",this->GetRealTime().Double());
			if(this->GetElapsedTime().Double() > 10)
				system("pkill gzserver");
		}*/
            
            public: void Iterate(NailDistancePtr &msg) {
				/*	if (this->index < 4) {
						std::cout << "Iterate" << std::endl;										
						//std::cout << this->world->GetModels()[1]->GetName() << std::endl;
						//this->world->InsertModelFile("model://hammer_training_" + std::to_string(this->index));
						//this->world->ClearModels();
						//this->world->RemoveModel(this->world->GetModels()[1]);
						this->world->RemoveModel("nail");
						std::cout << "aaaaa" << std::endl;
						//this->world->RemoveModel("hammer_training_" + std::to_string(this->index));
						//this->index++;
						//this->world->InsertModelFile("model://hammer_training_" + std::to_string(this->index));
						//this->world->InsertModelFile("model://nail");
					}
					else {
						std::cout << "It's the end of the world as we know it..." << std::endl;	
						this->world->Stop();
						}*/
					
				
           }


        // Pointer to the world
        private: physics::WorldPtr world;
        // Pointer to the node
        private: transport::NodePtr node;
        // Pointer to the subscriber
        private: transport::SubscriberPtr sub;
		private: int index = 1;

    };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldIterate)
}
