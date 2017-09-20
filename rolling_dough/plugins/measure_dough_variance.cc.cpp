#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "dough_variance.pb.h"
#include "dough_sensor.pb.h"
#include "end_sensor.pb.h"
#include "dough_stuck.pb.h"

#include "SVD.h"
#include "bestfit.h"

namespace gazebo {

	// typedef the message type that will be received
	typedef const boost::shared_ptr<const dough_variance_msgs::msgs::DoughVariance> DoughVariancePtr; 

	class MeasureDoughVariance : public WorldPlugin { 
		
		public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {      
			// pointer to the world
			this->world = _parent;  
			// 0 - ground, 1 - tool, 2 - platform, 3 - dough
			this->tool = this->world->GetModel(1);
			// Store the pointer to the nodes
			this->pub_node = transport::NodePtr(new transport::Node());
			this->sub_node = transport::NodePtr(new transport::Node());
			// Init the nodes
			this->pub_node->Init();
			this->sub_node->Init();
			  
			this->sub = this->sub_node->Subscribe("~/dough/variance",&MeasureDoughVariance::Measure, this);
			// Init the topic that will be published 
			this->pub = this->pub_node->Advertise<dough_stuck_msgs::msgs::DoughStuck>("~/dough/stuck");
			// Listen to the update event. It is broadcast every simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MeasureDoughVariance::OnUpdate, this, _1)); 
			   
		}

		// Called by the world update start event
		public: void OnUpdate(const common::UpdateInfo & /*_info*/) {
			if(this->world->GetSimTime().Double() > MAX_SIM_TIME) {
				std::cout << -1 << std::endl;
				system("pkill gzserver");
				if(!this->is_stuck) {
					this->dough_stuck.set_stuck(true);
					this->pub->Publish(this->dough_stuck);
					this->is_stuck = true;
				}
			}
		} 

		public: void Measure(DoughVariancePtr &msg) {
			
			this->dough = this->world->GetModel("dough");
			// variance msg has been received
			// the tool has finished performing the task
			// time to calculate suitability
			if(!msg->variance())
				return;
					  
			if(VERBOSE) std::cout << "Measuring variance..." << std::endl;
			
			std::vector<double> heights;
			// For each particle in the dough, store the final height
			if(VERBOSE) std::cout << "Dough base height: " << msg->dough_base_height() << std::endl;
			for (unsigned int i=0; i<this->dough->GetLinks().size() / 2; ++i) {
				std::string link_at = "dough_" + std::to_string(i);
				heights.push_back(fabs(this->dough->GetLink(link_at)->GetWorldPose().pos.z - msg->dough_base_height()));
			}
			// calculate sum of the distances of all particles after collision
			double sum = std::accumulate(heights.begin(), heights.end(), 0.0);
			double mean = sum / heights.size();
			double deviations = 0;
			double variance;					
			
			
			// get matrix of the position of each dough block
			float n_rows = heights.size();
            float n_cols = 3;
            int n_entries = n_rows*n_cols;
            float pancake_points[n_entries];
            for (int i=0;i<n_entries;i++)
				pancake_points[i] = -1;
            Matrix<float> pancake_matrix = Matrix<float>(n_rows,n_cols,pancake_points);
            // fill the matrix with the dough block positions
            if(VERBOSE) std::cout << "Dough blocks positions: " << std::endl;
            if (VERBOSE) std::cout << "\tX\t\t\t\tY\t\t\t\tZ\t\t\t\tRelative Height" << std::endl;
            for(unsigned int i=0; i < heights.size(); i++) {
				std::string link_at = "dough_" + std::to_string(i);
				pancake_matrix[i][0] = this->dough->GetLink(link_at)->GetWorldPose().pos.z;
                pancake_matrix[i][1] = this->dough->GetLink(link_at)->GetWorldPose().pos.y;
                pancake_matrix[i][2] = this->dough->GetLink(link_at)->GetWorldPose().pos.z;
                deviations += pow((heights.at(i) - mean),2);
                if (VERBOSE) std::cout << "\t";
                if (VERBOSE) std::cout << pancake_matrix[i][0] << "\t\t\t";
                if (VERBOSE) std::cout << pancake_matrix[i][1] << "\t\t\t";
                if (VERBOSE) std::cout << pancake_matrix[i][2] << "\t\t\t";
                if (VERBOSE) std::cout << heights.at(i) << std::endl;
			}			
			
			if(VERBOSE) std::cout << std::endl;
			if(VERBOSE) std::cout << "Mean: " << mean << std::endl;
			// calculate variance   
			if(VERBOSE) std::cout << "Deviations: " << deviations << std::endl;
			variance = deviations / (heights.size() - 1);
			if(VERBOSE) std::cout << "Variance: " << variance << std::endl;
			// calculate suitability
			double inv_variance = -1;
			float category_score = 0;
			/*
			if (variance > 0)
				inv_variance = 1/variance;
				if(inv_variance < 1/1e-5)
				{
					this->suitability = "Not Good";
					category_score = 1;
				}
				else if(inv_variance < 1/5e-5)
				{
					this->suitability = "Slightly Effective";
					category_score = 2;
				}
				else if(inv_variance < 1/5e-6)
				{
					this->suitability = "Good with Effort";
					category_score = 3;
				}
				else
				{
					this->suitability = "Very Good";
					category_score = 4;
				}
				* */
			double score = 10 - (((mean-0.003)+deviations)*1000);
			if (score < 0)
				score = 0;
			if(VERBOSE) std::cout << this->tool->GetName() << "'s score is " << this->suitability << std::endl;
			std::cout << score << std::endl;
			if (VERBOSE) std::cout << "Category: " << category_score << std::endl;
			system("pkill gzserver");						 
		}
		
		private: const bool VERBOSE = false;
		private: const float MAX_SIM_TIME = 10;
		// Pointer to the world
		private: physics::WorldPtr world;
		// Pointer to the models
		private: physics::ModelPtr dough, tool;
		// Pointer to the node
		private: transport::NodePtr sub_node, pub_node;
		// Pointer to the Publisher
		private: transport::PublisherPtr pub;
		// Pointer to the subscriber
		private: transport::SubscriberPtr sub;
		// Pointer to the msg that will be published
		private: dough_stuck_msgs::msgs::DoughStuck dough_stuck;
		// the tool's suitability based on the variance of the dough
		private: std::string suitability;
		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;
		// bool to send msg only once
		private: bool is_stuck = false;

	};
	// Register this plugin with the simulator
	GZ_REGISTER_WORLD_PLUGIN(MeasureDoughVariance)
}
