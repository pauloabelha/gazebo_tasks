#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/transport/transport.hh"
#include <math.h>
#include <random>
#include <sdf/sdf.hh>


namespace gazebo {

  class FactoryLasagna : public WorldPlugin {

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

        // will hold the sdf
        std::stringstream xml;
        
        // temporary strings to read from sdf, later to convert to double
        std::string PROB_COLOUR_RED, ROWS, COLS, X, Y, Z, RADIUS, MASS, MU, MU2, SOFT_CFM, SOFT_ERP, JOINT_DAMPING, JOINT_FRICTION;

        // will hold values to the sdf attributes
        double prob_colour_red, rows, cols, x, y, z, radius, mass, mu, mu2, soft_cfm, soft_erp, joint_damping, joint_friction;

        // Read all sdf atributes' values to string 
        PROB_COLOUR_RED = _sdf->GetElement("prob_colour_red")->GetValue()->GetAsString();
        ROWS = _sdf->GetElement("rows")->GetValue()->GetAsString();
        COLS = _sdf->GetElement("cols")->GetValue()->GetAsString();
        X = _sdf->GetElement("x")->GetValue()->GetAsString();
        Y = _sdf->GetElement("y")->GetValue()->GetAsString();
        Z = _sdf->GetElement("z")->GetValue()->GetAsString();
        RADIUS = _sdf->GetElement("radius")->GetValue()->GetAsString();
        MASS = _sdf->GetElement("mass")->GetValue()->GetAsString();
        MU = _sdf->GetElement("mu")->GetValue()->GetAsString();
        MU2 = _sdf->GetElement("mu2")->GetValue()->GetAsString();
        SOFT_CFM = _sdf->GetElement("soft_cfm")->GetValue()->GetAsString();
        SOFT_ERP = _sdf->GetElement("soft_erp")->GetValue()->GetAsString();
        JOINT_DAMPING = _sdf->GetElement("joint_damping")->GetValue()->GetAsString();
        JOINT_FRICTION = _sdf->GetElement("joint_friction")->GetValue()->GetAsString();        

        // Convert all read sdf atributes from string to double
        prob_colour_red = atof(PROB_COLOUR_RED.c_str()); 
        rows = atof(ROWS.c_str());
        cols = atof(COLS.c_str());
        x = atof(X.c_str());
        y = atof(Y.c_str());
        z = atof(Z.c_str());
        radius = atof(RADIUS.c_str());
        mass = atof(MASS.c_str());
        mu = atof(MU.c_str());
        mu2 = atof(MU2.c_str());
        soft_cfm = atof(SOFT_CFM.c_str());
        soft_erp = atof(SOFT_ERP.c_str());
        joint_damping = atof(JOINT_DAMPING.c_str());
        joint_friction = atof(JOINT_FRICTION.c_str());

        // Find the origin of the model based on the colums
        double origin_y = ( cols * radius ) / 2;

        // start writing sdf for the lasagna model
        xml << "<sdf version ='1.6'>\n";
        xml << "<model name ='lasagna'>\n";
        xml << "\t<pose>0 -" << origin_y << " 0 0 0 0</pose>\n";
        
        

        // Generate all links - lasagna spheres with friction
       for(int i=0; i<rows; i++) {

            // increment x unless it is the first row
            if (i != 0) {
                x = x + radius;
            }
            
            for (int j=0; j < cols; j++) {
                // increment y unless is the first or last row
                if (j > 0 && j < cols-1) {
                    y = y + radius;    
                }
                else {
                    y = 0.0;
                }
                
                // get random colours between yellow (.9) and red (.1)                
                std::string colour = "Yellow";
                if (rand() % 100 + 1 <= (prob_colour_red*100))
					colour = "Red";
                
                // Write to the sdf model the sdf tags of dough, box and joint
                xml << "\t\t<link name='lasagna_" << i << "_" << j << "'>\n";
                xml << "\t\t\t<pose>" << x << " " << y << " " << z << " 0 0 0</pose>\n";
                xml << "\t\t\t<inertial>\n";
                xml << "\t\t\t\t<mass>" << mass << "</mass>\n";
                xml << "\t\t\t</inertial>\n";
                xml << "\t\t\t<collision name ='lasagna_collision'>\n";
                xml << "\t\t\t\t<geometry>\n";
                xml << "\t\t\t\t\t<sphere>\n";
                xml << "\t\t\t\t\t\t<radius>" << radius << "</radius>\n";
                xml << "\t\t\t\t\t</sphere>\n";
                xml << "\t\t\t\t</geometry>\n";
                xml << "\t\t\t\t<surface>\n";
                xml << "\t\t\t\t\t<friction>\n";
                xml << "\t\t\t\t\t\t<ode>\n";
                xml << "\t\t\t\t\t\t\t<mu>" << mu << "</mu>\n";
                xml << "\t\t\t\t\t\t\t<mu2>" << mu2 << "</mu2>\n";
                xml << "\t\t\t\t\t\t</ode>\n";
                xml << "\t\t\t\t\t</friction>\n";
                xml << "\t\t\t\t\t<contact>\n";
                if (soft_cfm > 0 && soft_erp > 0) {
					xml << "\t\t\t\t\t\t<ode>\n";
					xml << "\t\t\t\t\t\t\t<soft_cfm>" << soft_cfm << "</soft_cfm>\n";
					xml << "\t\t\t\t\t\t\t<soft_erp>" << soft_erp << "</soft_erp>\n";
					xml << "\t\t\t\t\t\t</ode>\n";
				}
                xml << "\t\t\t\t\t</contact>\n";
                xml << "\t\t\t\t</surface>\n";
                xml << "\t\t\t</collision>\n";
                xml << "\t\t\t<visual name ='lasagna_visual'>\n";
                xml << "\t\t\t\t<geometry>\n";
                xml << "\t\t\t\t\t<sphere>\n";
                xml << "\t\t\t\t\t\t<radius>" << radius << "</radius>\n";
                xml << "\t\t\t\t\t</sphere>\n";
                xml << "\t\t\t\t</geometry>\n";
                xml << "\t\t\t\t<material>\n";
                xml << "\t\t\t\t\t<script>\n";
                xml << "\t\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n";
                xml << "\t\t\t\t\t\t<name>Gazebo/" << colour << "</name>\n";
                xml << "\t\t\t\t\t</script>\n";
                xml << "\t\t\t\t</material>\n";
                xml << "\t\t\t</visual>\n";
                xml << "\t\t</link>\n";

                xml << "\t\t<link name='lasagna2_" << i << "_" << j << "'>\n";
                xml << "\t\t\t<pose>" << x << " " << y << " " << z+radius << " 0 0 0</pose>\n";
                xml << "\t\t\t<inertial>\n";
                xml << "\t\t\t\t<mass>" << mass << "</mass>\n";
                xml << "\t\t\t</inertial>\n";
                xml << "\t\t\t<collision name ='lasagna_collision'>\n";
                xml << "\t\t\t\t<geometry>\n";
                xml << "\t\t\t\t\t<sphere>\n";
                xml << "\t\t\t\t\t\t<radius>" << radius << "</radius>\n";
                xml << "\t\t\t\t\t</sphere>\n";
                xml << "\t\t\t\t</geometry>\n";
                xml << "\t\t\t\t<surface>\n";
                xml << "\t\t\t\t\t<friction>\n";
                xml << "\t\t\t\t\t\t<ode>\n";
                xml << "\t\t\t\t\t\t\t<mu>" << mu << "</mu>\n";
                xml << "\t\t\t\t\t\t\t<mu2>" << mu2 << "</mu2>\n";
                xml << "\t\t\t\t\t\t</ode>\n";
                xml << "\t\t\t\t\t</friction>\n";
                xml << "\t\t\t\t\t<contact>\n";
                if (soft_cfm > 0 && soft_erp > 0) {
					xml << "\t\t\t\t\t\t<ode>\n";
					xml << "\t\t\t\t\t\t\t<soft_cfm>" << soft_cfm << "</soft_cfm>\n";
					xml << "\t\t\t\t\t\t\t<soft_erp>" << soft_erp << "</soft_erp>\n";
					xml << "\t\t\t\t\t\t</ode>\n";
				}
                xml << "\t\t\t\t\t</contact>\n";
                xml << "\t\t\t\t</surface>\n";
                xml << "\t\t\t</collision>\n";
                xml << "\t\t\t<visual name ='lasagna_visual'>\n";
                xml << "\t\t\t\t<geometry>\n";
                xml << "\t\t\t\t\t<sphere>\n";
                xml << "\t\t\t\t\t\t<radius>" << radius << "</radius>\n";
                xml << "\t\t\t\t\t</sphere>\n";
                xml << "\t\t\t\t</geometry>\n";
                xml << "\t\t\t\t<material>\n";
                xml << "\t\t\t\t\t<script>\n";
                xml << "\t\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n";
                xml << "\t\t\t\t\t\t<name>Gazebo/" << colour << "</name>\n";
                xml << "\t\t\t\t\t</script>\n";
                xml << "\t\t\t\t</material>\n";
                xml << "\t\t\t</visual>\n";
                xml << "\t\t</link>\n";        

                xml << "\t\t<joint name ='joint_" << i << "_" << j << "' type='prismatic'>\n";
                xml << "\t\t\t\t<pose>0 0 0.03 0 0 0</pose>\n";
                xml << "\t\t\t\t<parent>lasagna_" << i << "_" << j << "</parent>\n";
                xml << "\t\t\t\t<child>lasagna2_" << i << "_" << j << "</child>\n";
                xml << "\t\t\t\t<axis>\n";
                xml << "\t\t\t\t\t<dynamics>\n";
                xml << "\t\t\t\t\t\t<damping>" << joint_damping << "</damping>\n";
                xml << "\t\t\t\t\t\t<friction>" << joint_friction << "</friction>\n";
                xml << "\t\t\t\t\t</dynamics>\n";
                xml << "\t\t\t\t\t<xyz>0 0 1</xyz>\n";
                xml << "\t\t\t\t</axis>\n";
                
                xml << "\t\t</joint>\n";

            }
       }

       ///////////////////////////////////////////////////////////////////
        xml << "</model>\n";
        xml << "</sdf>\n";

        /////////////////////////////////
        //std::cout << xml.str() << "\n";

        sdf::SDF lasagnaSDF;
        lasagnaSDF.SetFromString(xml.str());

        _parent->InsertModelSDF(lasagnaSDF);
            
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(FactoryLasagna)
}
