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
#include "measure_pancake.pb.h"
#include "pancake_stuck.pb.h"

#include "SVD.h"
#include "bestfit.h"

namespace gazebo
{

    typedef const boost::shared_ptr<
        const measure_pancake_msgs::msgs::MeasurePancake>
            MeasurePancakePtr;

    class MeasurePancakeParticles : public WorldPlugin
    {

        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
        {
           

           // Store the pointer to the model, links and joint
            this->world = _parent;

            // 0 - ground, 1 - boxBack, 2-tool, 3-boxBelow, 4-pancake
            this->toolModel = this->world->GetModel(2);
            this->tool = this->toolModel->GetLink("tool");
            // Store the pointer to the nodes
            this->pub_node = transport::NodePtr(new transport::Node());
            this->sub_node = transport::NodePtr(new transport::Node());
            // Init the nodes
            this->pub_node->Init();
            this->sub_node->Init();
      
            // Init subsriber on this topic and callback ApplyVelocity function
            this->sub = this->sub_node->Subscribe("~/pancake/stop",
                &MeasurePancakeParticles::StoreMeasurements, this);
            // Init the topic that will be published 
              this->pub = this->pub_node->Advertise<pancake_stuck_msgs::msgs::PancakeStuck
                >("~/pancake/stuck");

            // Listen to the update event. It is broadcast every 
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&MeasurePancakeParticles::OnUpdate, this, _1));

        }

        // Called by the world update start event
        public: void OnUpdate(const common::UpdateInfo & /*_info*/) {
            if(this->world->GetSimTime().Double() > MAX_SIM_TIME) {
				if (VERBOSE) std::cout << "\tReached max simulation time: " << this->world->GetSimTime().Double() << "/" << MAX_SIM_TIME << std::endl;
				/*
                if(!this->is_stuck) {
                  this->pancake_stuck.set_stuck(true);
                  this->pub->Publish(this->pancake_stuck);
                  this->is_stuck = true;
                }*/
                std::cout << -1 <<  std::endl;
				system("pkill gzserver");
            }          

        } 


        public: void StoreMeasurements(MeasurePancakePtr &msg) {
						
			if (VERBOSE) std::cout << "\tReceived message of pancake stopped and not stuck" << std::endl;
            if (VERBOSE) std::cout << "\t\tStopped: " << msg->stopped() << std::endl;
            if (VERBOSE) std::cout << "\t\tStuck: " << msg->stopped() << std::endl;
            if (VERBOSE) std::cout << "\t\tLift Height: " << msg->lift_height() << std::endl;
			
			// Model 0 - ground, 1 - boxBack, 2-tool, 3-boxBelow, 4-boxLeft, 5-boxRight, 6-pancake   
			
            this->pancake = this->world->GetModel(6);
            /*
            this->pancake_left = this->world->GetModel(7);
            this->pancake_right = this->world->GetModel(8);
            * */
            
            // tool has stopped and is not stuck in the pancake
            if (msg->stopped() && !msg->stuck()) {
				
				// wait 1 simulation second before measuring
				WaitForNSimSecs(1);
				
				// get the final height
				this->expected_final_height = msg->lift_height();
				if (VERBOSE) std::cout << "\tExpected final height: " << this->expected_final_height << std::endl;
				//if (VERBOSE) std::cout << "\tGetting tool pose" << std::endl;
                // Get tool world pose after lift has stopped
                //this->tool_pose = this->tool->GetWorldPose().pos;
                // Get pancake world pose for centre sphere after lift has stopped
                //this->pancake_centre_pose_after = this->pancake->GetLink("sphere_link_center")->GetWorldPose().pos;
                // Get pancake world pose for the other particles after lift has stopped
                //if (VERBOSE) std::cout << "\tPancake size: \t\t" << this->pancake->GetLinks().size() << std::endl;
                
                if (VERBOSE) std::cout << "\tGetting all the pancake's spheres' positions" << std::endl;
				for(unsigned int i=0; i < this->pancake->GetLinks().size()-1; i++) {
                    std::string link_at = "sphere_link_" + std::to_string(1) + "_" + std::to_string(i);
                    this->pancake_spheres_pose_after.push_back(this->pancake->GetLink(link_at)->GetWorldPose().pos);
                    
                }
                /*
                this->world->Stop();
                this->SuitabilityThread = new boost::thread(&MeasurePancakeParticles::EvaluateMeasurementsNewBlockPancake, this);
                */
                if (VERBOSE) std::cout << "\tCalling the function for evaluating the final measurements of the pancake" << std::endl;
                				
                this->SuitabilityThread = new boost::thread(&MeasurePancakeParticles::EvaluateMeasurements, this);               

            }
            // tool still moves 
            else if (!msg->stopped() && !msg->stuck()){
                // Get pancake world pose for centre sphere before lift
                this->pancake_centre_pose_before = this->pancake->GetLink("sphere_link_center")->GetWorldPose().pos;
                // Get pancake world pose for the other particles before lift
                for(unsigned int i=0; i < this->pancake->GetLinks().size()-1; i++) {
                    std::string link_at = "sphere_link_" + std::to_string(1) + "_" + std::to_string(i);
                    this->pancake_spheres_pose_before.push_back(this->pancake->GetLink(link_at)->GetWorldPose().pos);
                    
                }
            }
            // tool is stuck
            else if (!msg->stopped() && msg->stuck()) {
                if (VERBOSE) std::cout << "\tf" << std::endl;
                this->world->Stop();
                this->suitability = "not good";
                if (VERBOSE) std::cout  << this->toolModel->GetName() << " is " << this->suitability <<  std::endl;
            }

            
        }
        
        public: void WaitForNSimSecs(double n_secs){
			double prior_sim_time = this->world->GetSimTime().Double();
			while( (this->world->GetSimTime().Double() - prior_sim_time) < n_secs );
		}

        public: void EvaluateMeasurements() {
			
			if (VERBOSE) std::cout << "\tStarted function for evaluating final measurements of the pancake" << std::endl;
            
            // sleep in microseconds
            //usleep(1000000); // 1 seconds

            //unsigned int bellowTool = 0;
            //unsigned int deviated = 0;
     /*
            for(unsigned int i=0; i < this->pancake_spheres_pose_after.size(); i++) {

                // Check each particles Z position against the tool Z
                if(this->pancake_spheres_pose_after.at(i).z < this->tool_pose.z) bellowTool++;
                
            }
            
            // Check centre particle Z position against the tool Z
            if(pancake_centre_pose_after.z < this->tool_pose.z ) bellowTool++;
            */
            
            
            /*
             *	Calculate the distance between each sphere and the center sphere 
             * 	before and after the pancake has been moved from its initial position
             * 	up to its final one
             */
            float sum_dist = 0;
            float sum_X = 0;
            float sum_Y = 0;
            float sum_Z = 0;
            if (VERBOSE) std::cout << "\tExpected height:\t" << this->expected_final_height << std::endl;
            
            int n_spheres = this->pancake_spheres_pose_after.size();
            
            float n_rows = n_spheres;
            float n_cols = 3;
            int n_entries = n_rows*n_cols;
            float pancake_points[n_entries];
            for (int i=0;i<n_entries;i++)
				pancake_points[i] = -1;
            Matrix<float> pancake_matrix = Matrix<float>(n_rows,n_cols,pancake_points);
            for(unsigned int i=0; i < this->pancake_spheres_pose_after.size(); i++) {
                /*
                double bx, by, ax, ay, bz, az;
				bx = fabs(expected_final_centre_x- this->pancake_spheres_pose_before.at(i).x);
                ax = fabs(this->pancake_centre_pose_after.x - this->pancake_spheres_pose_after.at(i).x);  
                by = fabs(this->pancake_centre_pose_before.y - this->pancake_spheres_pose_before.at(i).y);
                ay = fabs(this->pancake_centre_pose_after.y - this->pancake_spheres_pose_after.at(i).y);
                bz = fabs(this->pancake_centre_pose_before.z - this->pancake_spheres_pose_before.at(i).z);
                az = fabs(this->pancake_centre_pose_after.z - this->pancake_spheres_pose_after.at(i).z);
                * 
                if(fabs(ax - bx) > 0.01 || fabs(ay - by) > 0.01) {

                    deviated++;

                }
                if (VERBOSE) std::cout << bx << "\t" << ax << "\t" << by << "\t" << ay << "\t" << bz << "\t" << az << std::endl;
                * */
                //float diff_x = this->pancake_spheres_pose_after.at(i).x - this->pancake_spheres_pose_before.at(i).x;
                //float diff_y = this->pancake_spheres_pose_after.at(i).y - this->pancake_spheres_pose_before.at(i).y;
                float dist = fabs(this->pancake_spheres_pose_after.at(i).z - this->expected_final_height);
                //float dist = sqrt(pow(diff_x,2) + pow(diff_y,2) + pow(diff_z,2));
                
                
                //if (VERBOSE) std::cout << "\tX: \t\t" << this->pancake_spheres_pose_after.at(i).x << std::endl;
                //if (VERBOSE) std::cout << "\tY: \t\t" << this->pancake_spheres_pose_after.at(i).y << std::endl;
                //if (VERBOSE) std::cout << "\tZ: \t\t" << this->pancake_spheres_pose_after.at(i).z << std::endl;
                //if (VERBOSE) std::cout << "\t\t\t\t\tDist: " << dist << std::endl;
                
                sum_dist += dist;
                
                pancake_matrix[i][0] = this->pancake_spheres_pose_after.at(i).x;
                pancake_matrix[i][1] = this->pancake_spheres_pose_after.at(i).y;
                pancake_matrix[i][2] = this->pancake_spheres_pose_after.at(i).z;
                //pancake_matrix[3][i] = 1;           
                
                sum_X += this->pancake_spheres_pose_after.at(i).x; 
                sum_Y += this->pancake_spheres_pose_after.at(i).y; 
                sum_Z += this->pancake_spheres_pose_after.at(i).z;                
            }
            float avg_dist = sum_dist/this->pancake_spheres_pose_after.size();
            if (VERBOSE) std::cout << "\tAverage dist: " << avg_dist << std::endl;  
            
            /*
            float mean_X = sum_Z/this->pancake_spheres_pose_after.size();
            float mean_Y = sum_Z/this->pancake_spheres_pose_after.size();
            float mean_Z = sum_Z/this->pancake_spheres_pose_after.size();
            * */
			
			if (VERBOSE) std::cout << "\tOriginal matrix: " << std::endl;
            for (int i=0;i<n_rows;i++)
            {
				for (int j=0;j<n_cols;j++)
				{
					if (VERBOSE) std::cout << "\t\t" << pancake_matrix[i][j] << " ";
					//pancake_matrix[i][j] = pancake_matrix[i][j] - means[i];
				}
				if (VERBOSE) std::cout << std::endl;
			}
			if (VERBOSE) std::cout << std::endl;

			if (VERBOSE) std::cout << "\tFitting a plane to the pancake..." << std::endl;
			float pancake_plane[4];
			getBestFitPlane(3,pancake_matrix.GetRawData(),sizeof(float)*3,0,0,pancake_plane);
			if (VERBOSE) std::cout << "\tPlane fitted: [";
			for (int i=0;i<4;i++)
				if (VERBOSE) std::cout << pancake_plane[i] << " ";
			if (VERBOSE) std::cout << "\t]" << std::endl;			
			// calculate how close the pancake plane is to the XY plane
			const float Z_vec[3] = {0,0,1};
			// get unit vector of the pancake plane normal vector
			float pancake_plane_norm_vec_length = 0;
			for (int i = 0; i < 3; i++)
				pancake_plane_norm_vec_length += pow(pancake_plane[i],2);
			pancake_plane_norm_vec_length = sqrt(pancake_plane_norm_vec_length);
			float pancake_plane_unit_vec[3];
			if (VERBOSE) std::cout << "\tPancake plane unit vector: [";
			for (int i = 0; i < 3; i++)
			{
				pancake_plane_unit_vec[i] = pancake_plane[i]/pancake_plane_norm_vec_length;
				if (VERBOSE) std::cout << pancake_plane_unit_vec[i] << " ";
			}
			if (VERBOSE) std::cout << "\t]" << std::endl;
			// get inner product (plane_dist) from pancake plane's normal to the normal to the XY plane (Z_vec)
			float dot_pancake_plane_XY = 0;
			for (int i = 0; i < 3; i++)
				dot_pancake_plane_XY += pancake_plane_unit_vec[i]*Z_vec[i];
			float plane_angle_dist = acos(dot_pancake_plane_XY);
			plane_angle_dist = fmod(plane_angle_dist,M_PI);
			if (plane_angle_dist < M_PI/2)
				plane_angle_dist = fmod(plane_angle_dist,M_PI);
			else
				plane_angle_dist = M_PI - fmod(plane_angle_dist,M_PI);
			if (VERBOSE) std::cout << "\tPancake plane distance to the XY plane (radians): " << plane_angle_dist << std::endl;
			plane_angle_dist = 1 + (plane_angle_dist/(M_PI/2));
			if (VERBOSE) std::cout << "\tPancake normalised plane distance to the XY plane (1 - 2): " << plane_angle_dist << std::endl;
			// calculate dist from each pancake sphere to the fitted pancake plane			
			if (VERBOSE) std::cout << "\tCalculating distance from each sphere to the fitted plane " << std::endl;
			float tot_dist_to_plane = 0;
			float MAX_ACCEPTABLE_DIST_PLANE = 0.01;
			int n_shperes_close_to_plane = 0;
			for (int i=0;i<n_rows;i++)
            {
				float dist_to_plane = 0;
				for (int j=0;j<n_cols;j++)
					dist_to_plane += pancake_plane[j]*pancake_matrix[i][j];
				dist_to_plane += pancake_plane[3];
				dist_to_plane = fabs(dist_to_plane);
				if (VERBOSE) std::cout << "\tDist to plane: " << dist_to_plane << std::endl;
				tot_dist_to_plane += dist_to_plane;
				if (dist_to_plane <= MAX_ACCEPTABLE_DIST_PLANE)
					n_shperes_close_to_plane++;
			}
			if (VERBOSE) std::cout << "\tTotal dist to plane: " << tot_dist_to_plane << std::endl;	
			if (VERBOSE) std::cout << "\t#Spheres close to plane: " << n_shperes_close_to_plane << std::endl;		
			float score_n_spheres = 2 - (n_shperes_close_to_plane/n_rows);
			if (VERBOSE) std::cout << "\tScore of #Spheres : " << score_n_spheres << std::endl;	
			
			float center_sphere_final_height = this->pancake_spheres_pose_after.at(0).z;
			if (VERBOSE) std::cout << "\tCentral sphere final height : " << center_sphere_final_height << std::endl;
			if (VERBOSE) std::cout << "\tExpected final height for central sphere: " << this->expected_final_height << std::endl;
			float diff_height_central_sphere = fabs(center_sphere_final_height - this->expected_final_height);
            if (VERBOSE) std::cout << "\tDifference in height for central sphere: " << diff_height_central_sphere << std::endl;            
            const float MAX_ACCEPTABLE_DIFF_HEIGHT_CENTRAL_SPHERE = 0.04;
            float penalty_diff_height_central_sphere = diff_height_central_sphere/MAX_ACCEPTABLE_DIFF_HEIGHT_CENTRAL_SPHERE;
            penalty_diff_height_central_sphere = penalty_diff_height_central_sphere < 1 ? 1 : penalty_diff_height_central_sphere;
            if (VERBOSE) std::cout << "\tPenalty for difference in height for central sphere: " << penalty_diff_height_central_sphere << std::endl; 
            
            /*
            float height_variance = 0;
            for(unsigned int i=0; i < this->pancake_spheres_pose_after.size(); i++) {
				 height_variance += std::pow(this->pancake_spheres_pose_after.at(i).z - mean_Z,2);
			}
			height_variance = height_variance/(this->pancake_spheres_pose_after.size()-1);
			if (VERBOSE) std::cout << "\tHeight variance: " << height_variance << std::endl;
			*/
            
            float score = 4 - (score_n_spheres * plane_angle_dist * penalty_diff_height_central_sphere);
            int category_score = 0;
            if (score < 0)
				score = 0;
            if(score <= 1 ) {
                category_score  = 1;                
            }
            else if(score <= 1.5 ) {
                category_score  = 2;   

            }
            else if(score <= 2.5 ){
                category_score  = 3;
                   
            }
            else {
                category_score  = 4;              
            }
            
			if (VERBOSE) std::cout << "\tCategory score: " << category_score << std::endl; 
            if (VERBOSE) std::cout << "\tFinal score: ";  
            std::cout << score <<  std::endl;
            if (VERBOSE) std::cout << "\tKilling simulation" << std::endl;
            system("pkill gzserver");
            
        }
        
    
        
        private: bool VERBOSE = 10;
        private : const float MAX_SIM_TIME = 30;
		// final tool height (after tool has lifted pancake)
		private: float expected_final_height = -1;
        // Pointer to the world
        private: physics::WorldPtr world;
        // Pointer to the models
        private: physics::ModelPtr pancake;
        private: physics::ModelPtr toolModel, pancake_left, pancake_right;
        // Pointer to a link
        private: physics::LinkPtr tool;
        /// Pointer to the node
        private: transport::NodePtr sub_node, pub_node;
        // Pointer to the Publisher
        private: transport::PublisherPtr pub;
        // Pointer to the subscriber
        private: transport::SubscriberPtr sub;
        // Pointer to the msg that will be published
        private: pancake_stuck_msgs::msgs::PancakeStuck pancake_stuck;
        // suitbality based on tool and centre
        private: int current_suitability1, current_suitability2;
        // the tool's suitability based on the end position of the particles
        private: std::string suitability;
        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
        // bool to send msg only once
        private: bool is_stuck = false;
        // Pose of tool
        //private: gazebo::math::Vector3 tool_pose;
        private: gazebo::math::Vector3 pancake_centre_pose_after, pancake_centre_pose_before;
        // Pose Vector for pancake
        private: std::vector<gazebo::math::Vector3> pancake_spheres_pose_after, pancake_spheres_pose_before;
        // A thread to wait before calculating the suitability
        private: boost::thread* SuitabilityThread;

        


    };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(MeasurePancakeParticles)
}
