#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/transport/transport.hh"
#include <math.h>
#include <random>
#include <sdf/sdf.hh>


namespace gazebo {

  class FactoryGrain : public WorldPlugin {

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
		//temporary strings to read from sdf, later to convert to double
		std::string  N_GRAINS_X, N_GRAINS_Y, N_GRAINS_Z, GRAIN_RADIUS, GRAIN_MASS, X, Y, Z, MU, MU2;
        // the values from the sdf atributes after conversion from string
        float n_grains_x, n_grains_y, n_grains_z, grain_radius, grain_mass, x, y, z, mu, mu2;
        // Read all sdf atributes' values to string 
        N_GRAINS_X = _sdf->GetElement("n_grains_x")->GetValue()->GetAsString();
        N_GRAINS_Y = _sdf->GetElement("n_grains_y")->GetValue()->GetAsString();
        N_GRAINS_Z = _sdf->GetElement("n_grains_z")->GetValue()->GetAsString();
        GRAIN_RADIUS = _sdf->GetElement("grain_radius")->GetValue()->GetAsString();
        GRAIN_MASS = _sdf->GetElement("grain_mass")->GetValue()->GetAsString();
        MU = _sdf->GetElement("mu")->GetValue()->GetAsString();
        MU2 = _sdf->GetElement("mu2")->GetValue()->GetAsString();
        X = _sdf->GetElement("x")->GetValue()->GetAsString();
        Y = _sdf->GetElement("y")->GetValue()->GetAsString();
        Z = _sdf->GetElement("z")->GetValue()->GetAsString();         
        // Convert all read sdf atributes from string to double
        n_grains_x = atof(N_GRAINS_X.c_str());
        n_grains_y = atof(N_GRAINS_Y.c_str());
        n_grains_z = atof(N_GRAINS_Z.c_str());
        grain_radius = atof(GRAIN_RADIUS.c_str());
        grain_mass = atof(GRAIN_MASS.c_str());
        x = atof(X.c_str());
        y = atof(Y.c_str());
        z = atof(Z.c_str());
        mu = atof(MU.c_str());
        mu2 = atof(MU2.c_str());
        // start writing sdf for the grains model
        float x_offset = n_grains_x*grain_radius;
        float y_offset = n_grains_y*grain_radius;
        float spacing_x = grain_radius;
        float spacing_y = grain_radius;
        std::vector<std::string> colours = {"Red", "Green", "Blue", "Yellow", "Grey", "White"};
        //generate all the grains
        for(int i=0; i<n_grains_x; i++) {
			for(int j=0; j<n_grains_y; j++) {
				for(int k=0; k<n_grains_z; k++) {
					// get random spacing
					float spacing_x_rnd = get_random(-spacing_x, spacing_x);	
					float spacing_y_rnd = get_random(-spacing_y, spacing_y);				
					// will hold the SDF xml
					std::stringstream xml;
					xml << "<sdf version ='1.6'>\n";
					xml << "<model name ='grain_" << i << "_" << j << "_" << k << "'>\n";
					xml << "\t<pose>" << x-x_offset << " " << y-y_offset << " " << z << " 0 0 0</pose>\n";
					float pos[3] = {(i*2*grain_radius)+grain_radius+spacing_x_rnd, (j*2*grain_radius)+grain_radius+(i/100)+spacing_y_rnd, (k*2*grain_radius)+grain_radius};			
					xml << "\t\t<link name ='grain_" << i << "_" << j << "_" << k << "'>\n";
					xml << "\t\t\t<pose>" << pos[0] << " " << pos[1] << " " << pos[2] << " 0 0 0</pose>\n";
					xml << "\t\t\t<inertial>\n";
					xml << "\t\t\t\t<pose> 0 0 0 0 0 0 </pose>\n";
					xml << "\t\t\t\t<mass>" << grain_mass << "</mass>\n";
					xml << "\t\t\t</inertial>\n";
					xml << "\t\t\t<collision name ='collision_grain_2" << i << "_" << j << "_" << k << "'>\n";
					xml << "\t\t\t\t<geometry>\n";
					xml << "\t\t\t\t\t<sphere>\n";
					xml << "\t\t\t\t\t\t<radius>" << grain_radius << "</radius>\n";
					xml << "\t\t\t\t\t</sphere>\n";
					xml << "\t\t\t\t</geometry>\n";
					
					xml << "\t\t\t\t<surface>\n";
					xml << "\t\t\t\t\t<friction>\n";
					xml << "\t\t\t\t\t\t<ode>\n";
					xml << "\t\t\t\t\t\t\t<mu>" << mu << "</mu>\n";
					xml << "\t\t\t\t\t\t\t<mu2>" << mu2 << "</mu2>\n";
					xml << "\t\t\t\t\t\t</ode>\n";
					xml << "\t\t\t\t\t</friction>\n";
					xml << "\t\t\t\t</surface>\n";

					xml << "\t\t\t</collision>\n";
					xml << "\t\t\t<visual name ='visual_grain_" << i << "_" << j << "_" << k << "'>\n";
					xml << "\t\t\t\t<geometry>\n";
					xml << "\t\t\t\t\t<sphere>\n";
					xml << "\t\t\t\t\t\t<radius>" << grain_radius << "</radius>\n";
					xml << "\t\t\t\t\t</sphere>\n";
					xml << "\t\t\t\t</geometry>\n";
					xml << "\t\t\t\t<material>\n";
					xml << "\t\t\t\t\t<script>\n";
					xml << "\t\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n";
					xml << "\t\t\t\t\t\t<name>Gazebo/" << colours[2] << "</name>\n";
					xml << "\t\t\t\t\t</script>\n";
					xml << "\t\t\t\t</material>\n";
					xml << "\t\t\t</visual>\n";
					xml << "\t\t</link>\n";
					// close xml
					xml << "</model>\n";
					xml << "</sdf>\n"; 
					// create SDF from XML string
					sdf::SDF grainsSDF;
					grainsSDF.SetFromString(xml.str());
					// insert the SDF into the world in runtime
					_parent->InsertModelSDF(grainsSDF);  
				}
			}
		}    
		          
    }
    
    /* Returns a random double between min and max */
    public: float get_random(double min, double max) {		
		return (max - min) * ( (float)rand() / (float)RAND_MAX ) + min;
	}
	
	/* Returns a random int between min and max */
    public: int get_random_int(int min, int max) {		
		return (max - min) * ( (int)rand() / (int)RAND_MAX ) + min;
	}

  };
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(FactoryGrain)
}
