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
#include "measure_cut.pb.h"

#include <vector>
#include "Matrix.h"

namespace gazebo
{
	// Typedef to store a pointer to the defined msg type MeasureCut
    typedef const boost::shared_ptr<const measure_cut_msgs::msgs::MeasureCut> MeasureCutPtr;

    class Measurer : public WorldPlugin
    {
        public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
        {          
			// Store the pointer to the model, links and joint
            this->world = _parent;
            // Store the pointer to the node
            this->transport_node1 = transport::NodePtr(new transport::Node());
            // Init transport nodes
            this->transport_node1->Init();
            // Init subscriber on this topic and callback ApplyVelocity function
            this->transport_subscriber = this->transport_node1->Subscribe("~/measure/cut", &Measurer::Measure, this);            
            // Listen to the update event. It is broadcast every simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Measurer::OnUpdate, this, _1));
            
            
        }

        // Called by the world update start event
        public: void OnUpdate(const common::UpdateInfo & /*_info*/) {
            if(this->world->GetSimTime().Double() > MAX_SIM_TIME) {
				if (VERBOSE) std::cout << "Reached max simulation time:\t\t" << this->world->GetSimTime().Double() << "\t\t" << MAX_SIM_TIME << std::endl;
                if (VERBOSE) std::cout << -1 <<  std::endl;
				system("pkill gzserver");
            }
        } 

        public: void Measure(MeasureCutPtr &msg) {	
			if (msg->initial_measure())	{
				if (VERBOSE) std::cout << "Measuring lasagne pieces positions" << std::endl;
				this->start_positions = MeasurePositions();
				int n_pieces = this->start_positions.GetRows();
				if (VERBOSE) std::cout << "Lasagne initial positions:\t\t" << n_pieces << std::endl;
				PrintPositions(this->start_positions, "initial");
			}
			else {		
				if (VERBOSE) std::cout << "Received message for measuring (msg; action bottom pos):\t\t" << msg->cut() << "\t\t" << msg->action_bottom_x() << std::endl;		
				if (!msg->lasagna_collision()) {
					if (VERBOSE) std::cout << "No lasagna collision!" << std::endl;
					std::cout << -1 << std::endl;
					system("pkill gzserver");
				}
				else {		
					this->end_positions = MeasurePositions();
					PrintPositions(this->start_positions, "end");
					int n_pieces = this->start_positions.GetRows();
					double median_diff = MedianDiffPos(this->start_positions,this->end_positions,n_pieces);
					if (VERBOSE) std::cout << "Median difference in positions is:\t\t" << median_diff << std::endl;
					double prop_balls_displaced = PropBallsDisplaced(this->start_positions,this->end_positions,n_pieces);					
					double score1 = 1 - median_diff;
					if (score1 < 0)
						score1 = 0;
					double score2 = 1 - prop_balls_displaced;
					double score = score1*score2;
					if (VERBOSE) std::cout << "Score1:\t" << score1 << "\t\t Score2:\t" << score2 << std::endl; 
					std::cout << score << std::endl;
					system("pkill gzserver");
				}
			}			
		}
		
		public: void PrintPositions(Matrix<double> positions, std::string name) {
			int n_pieces = positions.GetRows();
			if (VERBOSE) std::cout << "Lasagne " << name << " positions:\t\t" << n_pieces << std::endl;
			for (int i=0;i<n_pieces;i++) {	 
				for (int j=0;j<3;j++)
					if (VERBOSE) std::cout << positions[i][j] << "\t\t";
				if (VERBOSE) std::cout << std::endl;
			}
		}
		
		public: Matrix<double> MeasurePositions() {   
			// get World models
			std::vector<physics::ModelPtr> models = this->world->GetModels();
			int n_models = models.size();
			// count number of lasagne pieces
			std::vector<physics::LinkPtr> lasagne_pieces;
			int n_pieces = 0;
			Matrix<double> positions;
			for (int i=0;i<n_models;i++) {
				std::string model_prefix = "";
				std::string model_name = models[i]->GetName();				
				if (model_name.size() > 5)
					model_prefix = model_name.substr (0,6);
				if (VERBOSE) std::cout << model_name << "\t\t" << model_prefix << std::endl;
				if(model_prefix.compare("lasagn") == 0) {
					std::vector<physics::LinkPtr> lasagne_pieces = models[i]->GetLinks();
					// get position of every lasagne piece
					n_pieces = lasagne_pieces.size();
					positions = Matrix<double>(n_pieces,3);
					for (int i=0;i<n_pieces;i++) {
						positions[i][0] = lasagne_pieces[i]->GetRelativePose().pos.x;
						positions[i][1] = lasagne_pieces[i]->GetRelativePose().pos.y;
						positions[i][2] = lasagne_pieces[i]->GetRelativePose().pos.z;
					}				
					return positions;
					break;
				}
			}
			return positions;
		}
		
		public: double PropBallsDisplaced(Matrix<double> start, Matrix<double> end, int n_rows) {   
			//double tot_diff = 0;
			std::vector<double> diffs;
			const double MAX_DIST = 0.05;
			// get diffs				
			for (int i=0;i<n_rows;i++) {
				double diff = sqrt(pow(end[i][0]-start[i][0],2)+pow(end[i][1]-start[i][1],2)+pow(end[i][2]-start[i][2],2));
				if (diff >= MAX_DIST) {
					diffs.push_back(diff);
					if (VERBOSE) std::cout << "\t\tDisplaced ball:\t\t" << diff << std::endl;
				}			
			}	
			double tot_n_balls = n_rows;
			double prop_balls_displaced = diffs.size()/tot_n_balls;
			if (VERBOSE) std::cout << "Displaced balls (#; tot; prop):\t\t" << diffs.size() << "\t\t" << tot_n_balls << "\t\t" << prop_balls_displaced << std::endl;			
			return prop_balls_displaced;
		}
		
		public: double MedianDiffPos(Matrix<double> start, Matrix<double> end, int n_rows) {   
			//double tot_diff = 0;
			std::vector<double> diffs;
			// get diffs
			for (int i=0;i<n_rows;i++) {
				double diff = sqrt(pow(end[i][0]-start[i][0],2)+pow(end[i][1]-start[i][1],2)+pow(end[i][2]-start[i][2],2));
				if (diff >= 0.001) {
					diffs.push_back(diff);
					if (VERBOSE) std::cout << "\t\tDiff:\t\t" << diff << std::endl;
				}
				//tot_diff += diff;				
			}	
			// sort diffs
			std::vector<double> sorted_diffs = diffs;
			int n_diffs = diffs.size();
			if (n_diffs < 1)
				return 0;
			std::sort(sorted_diffs.begin(), sorted_diffs.begin()+n_diffs-1);
			// return median
			double median = -1;
			if (n_rows % 2 == 0)
				median = (sorted_diffs[n_diffs/2-1] + sorted_diffs[n_diffs/2])/2;
			else
				median = sorted_diffs[n_diffs/2];
			return median;
			// return average
			//return tot_diff/n_rows;			
		}
        
        private: bool VERBOSE = 0;
        private : const float MAX_SIM_TIME = 10;
        // holds the lasagne pieces start positions
        Matrix<double> start_positions;
        // holds the lasagne pieces end positions
        Matrix<double> end_positions;
        // Pointer to the world
        private: physics::WorldPtr world;
        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
		// Pointer to the node
        private: transport::NodePtr transport_node1;
        // Pointer to the subscriber
        private: transport::SubscriberPtr transport_subscriber;
        // Link of action bottom
        private: physics::LinkPtr link_action_bottom;
    };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Measurer)
}
