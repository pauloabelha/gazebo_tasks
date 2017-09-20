#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/thread.hpp>

#include "gazebo/transport/transport.hh"
#include <math.h>
#include <sdf/sdf.hh>
#include <vector>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "measure_scoop.pb.h"

namespace gazebo
{
	// Typedef to store a pointer to the defined msg type MeasureScoop
    typedef const boost::shared_ptr<const measure_scoop_msgs::msgs::MeasureScoop> MeasureScoopPtr;

    class Measurer : public WorldPlugin
    {

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
        {          

           // Store the pointer to the model, links and joint
            this->world = _parent;

            this->toolModel = this->world->GetModel("tool_scooping_grains");
            // Store the pointer to the nodes
            this->sub_node = transport::NodePtr(new transport::Node());
            // Init the nodes
            this->sub_node->Init();
      
            // Init subsriber on this topic and callback ApplyVelocity function
            this->sub = this->sub_node->Subscribe("~/measure/scoop", &Measurer::CountScoop, this);
            
            // Listen to the update event. It is broadcast every 
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Measurer::OnUpdate, this, _1));

        }

        // Called by the world update start event
        public: void OnUpdate(const common::UpdateInfo & /*_info*/) {
            if(this->world->GetSimTime().Double() > MAX_SIM_TIME) {
				if (VERBOSE) std::cout << "Reached max simulation time: " << this->world->GetSimTime().Double() << "/" << MAX_SIM_TIME << std::endl;
                std::cout << -1 <<  std::endl;
				system("pkill gzserver");
            }        

        } 

        public: void CountScoop(MeasureScoopPtr &msg) {		
			const float MAX_Z_DIFF = 0.015;		
			physics::ModelPtr tool_model = this->world->GetModel(3);
			physics::LinkPtr tool_link = tool_model->GetLink("tool");
			math::Box tool_bounding_box = tool_link->GetBoundingBox();
			if (VERBOSE) std::cout << "\tReceived message of scooped:\t" << msg->scooped() << std::endl;
			if (VERBOSE) std::cout << "\tAction tracker Z position:\t" << msg->cont_bottom_z() << std::endl;
			if (VERBOSE) std::cout << "\tMaximum Z difference:\t" << MAX_Z_DIFF << std::endl << std::endl << std::endl;
			if (msg->scooped()) {	
				std::vector<physics::ModelPtr> models = this->world->GetModels();
				int num_grains_scooped = 0;
				int n_models = models.size();
				for (int i=0;i<n_models;i++) {					
					std::string model_prefix = "";
					std::string model_name = models[i]->GetName();
					if (model_name.size() > 4)
						model_prefix = model_name.substr (0,5);
					// if the contact is with any of the grains, publish collision msg as true
					if(model_prefix.compare("grain") == 0) {
						float grain_abs_pose = models[i]->GetWorldPose().pos.z + models[i]->GetLink(model_name)->GetRelativePose().pos.z;
						float diff_Z_pos = grain_abs_pose - msg->cont_bottom_z();
						if (diff_Z_pos >= MAX_Z_DIFF) {
							num_grains_scooped++;	
							if (VERBOSE) std::cout << "\t----------------------------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
							if (VERBOSE) std::cout << "\tMeasuring the scooping (Grain Name; Grain Pose; Action Part Bottom Pose):\t\t" << model_name << "\t\t" << grain_abs_pose << "\t\t" << diff_Z_pos << std::endl;
							if (VERBOSE) std::cout << "\t----------------------------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
						}
						else
							if (VERBOSE) std::cout << "\tMeasuring the scooping (Grain Name; Grain Pose; Action Part Bottom Pose):\t\t" << model_name << "\t\t" << grain_abs_pose << "\t\t" << diff_Z_pos << std::endl;
					}					
				}
				if (VERBOSE) std::cout << "\tNum of grains scooped: ";
				std::cout << num_grains_scooped << std::endl;
				system("pkill gzserver");
			}			
		}   
        
        private: bool VERBOSE = 0;
        private : const float MAX_SIM_TIME = 30;
		// final tool height (after tool has lifted scoop)
		private: float expected_final_height = -1;
        // Pointer to the world
        private: physics::WorldPtr world;
        // Pointer to the models
        private: physics::ModelPtr scoop;
        private: physics::ModelPtr toolModel, scoop_left, scoop_right;
        // Pointer to a link
        private: physics::LinkPtr tool;
        /// Pointer to the node
        private: transport::NodePtr sub_node, pub_node;
        // Pointer to the Publisher
        private: transport::PublisherPtr pub;
        // Pointer to the subscriber
        private: transport::SubscriberPtr sub;
        // Pointer to the msg that will be published
        private: measure_scoop_msgs::msgs::MeasureScoop measure_scoop_msg;
        // suitbality based on tool and centre
        private: int current_suitability1, current_suitability2;
        // the tool's suitability based on the end position of the particles
        private: std::string suitability;
        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
        // A thread to wait before calculating the suitability
        private: boost::thread* SuitabilityThread;      


    };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Measurer)
}
