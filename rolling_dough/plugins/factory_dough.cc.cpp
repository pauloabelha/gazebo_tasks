#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/transport/transport.hh"
#include <math.h>
#include <random>
#include <sdf/sdf.hh>


namespace gazebo {

  class FactoryDough : public WorldPlugin {

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {

        // will hold the sdf
        std::stringstream xml;

      //temporary strings to read from sdf, later to convert to double
        std::string ROWS, COLS, X, Y, Z, DOUGH_MASS, DOUGH_MU, DOUGH_MU2,
            BOX_MASS, BOX_MU, BOX_MU2, DAMPING, FRICTION,
            DOUGH_ELEM_THICKNESS, DOUGH_ELEM_WIDTH,
            DOUGH_MIN_HEIGHT, DOUGH_MAX_HEIGHT;
        // the values from the sdf atributes after conversion from string
        double rows, cols, x, y, z, dough_mass, dough_mu, dough_mu2,
            box_mass, box_mu, box_mu2, damping, friction,
            dough_elem_width, dough_elem_thickness,
            dough_min_height, dough_max_height;


        // Read all sdf atributes' values to string 
        ROWS = _sdf->GetElement("rows")->GetValue()->GetAsString();
        COLS = _sdf->GetElement("cols")->GetValue()->GetAsString();
        X = _sdf->GetElement("x")->GetValue()->GetAsString();
        Y = _sdf->GetElement("y")->GetValue()->GetAsString();
        Z = _sdf->GetElement("z")->GetValue()->GetAsString(); 

        DOUGH_MASS = _sdf->GetElement("dough_mass")->GetValue()->GetAsString();
        DOUGH_MU = _sdf->GetElement("dough_mu")->GetValue()->GetAsString();
        DOUGH_MU2 = _sdf->GetElement("dough_mu2")->GetValue()->GetAsString();
       
        BOX_MASS = _sdf->GetElement("box_mass")->GetValue()->GetAsString();
        BOX_MU = _sdf->GetElement("box_mu")->GetValue()->GetAsString();
        BOX_MU2 = _sdf->GetElement("box_mu2")->GetValue()->GetAsString();
        
        DAMPING = _sdf->GetElement("damping")->GetValue()->GetAsString();
        FRICTION = _sdf->GetElement("friction")->GetValue()->GetAsString();
        
        DOUGH_MIN_HEIGHT = _sdf->GetElement("dough_min_height")->GetValue()->GetAsString();
        DOUGH_MAX_HEIGHT = _sdf->GetElement("dough_max_height")->GetValue()->GetAsString();
        
        DOUGH_ELEM_WIDTH = _sdf->GetElement("dough_elem_width")->GetValue()->GetAsString();
        DOUGH_ELEM_THICKNESS = _sdf->GetElement("dough_elem_thickness")->GetValue()->GetAsString();
        
        // Convert all read sdf atributes from string to double
        rows = atof(ROWS.c_str());
        cols = atof(COLS.c_str());
        x = atof(X.c_str());
        y = atof(Y.c_str());
        z = atof(Z.c_str());

        dough_mass = atof(DOUGH_MASS.c_str());
        dough_mu = atof(DOUGH_MU.c_str());
        dough_mu2 = atof(DOUGH_MU2.c_str());
        
        box_mass = atof(BOX_MASS.c_str());
        box_mu = atof(BOX_MU.c_str());
        box_mu2 = atof(BOX_MU2.c_str());
       
        damping = atof(DAMPING.c_str());
        friction = atof(FRICTION.c_str());
        
        dough_min_height = atof(DOUGH_MIN_HEIGHT.c_str());
        dough_max_height = atof(DOUGH_MAX_HEIGHT.c_str());
        
        dough_elem_width = atof(DOUGH_ELEM_WIDTH.c_str());
        dough_elem_thickness = atof(DOUGH_ELEM_THICKNESS.c_str());

        // set up random generator for height
        double dough_height;
        double origin_y = ( cols * dough_elem_width ) / 2;
        int particle_counter = 0;

        // start writing sdf for the dough model
        xml << "<sdf version ='1.6'>\n";
        xml << "<model name ='dough'>\n";
        xml << "\t<pose>0 -" << origin_y << " " << z << " 0 0 0</pose>\n";

        // Generate all links - dough + box + joint between them in the range of rows - cols
       for(int i=0; i<rows; i++) {

            // increment x unless it is the first row
            if (i != 0) {
                x = x + dough_elem_width;
            }
            
            for (int j=0; j < cols; j++) {
                // increment y unless is the first or last row
                if (j > 0 && j < cols-1) {
                    y = y + dough_elem_width;    
                }
                else {
                    y = 0.0;
                }
                // Generate a random height for the dough link at this iteration
                dough_height = FactoryDough::get_random(dough_min_height, dough_max_height);
                // Write to the sdf model the sdf tags of dough, box and joint
                xml << "\t\t<link name='dough_" << particle_counter << "'>\n";
					xml << "\t\t\t<pose>" << x << " " << y << " " << dough_height << " 0 0 0</pose>\n";
					xml << "\t\t\t<inertial>\n";
						xml << "\t\t\t\t<mass>" << dough_mass << "</mass>\n";
					xml << "\t\t\t</inertial>\n";
					xml << "\t\t\t<collision name ='dough_collision'>\n";
						xml << "\t\t\t\t<geometry>\n";
							xml << "\t\t\t\t\t<box>\n";
								xml << "\t\t\t\t\t\t<size>" << dough_elem_width << " " << dough_elem_width << " " << dough_elem_thickness << "</size>\n";
							xml << "\t\t\t\t\t</box>\n";
						xml << "\t\t\t\t</geometry>\n";
						xml << "\t\t\t\t<surface>\n";
							xml << "\t\t\t\t\t<friction>\n";
								xml << "\t\t\t\t\t\t<ode>\n";
								xml << "\t\t\t\t\t\t\t<mu>" << dough_mu << "</mu>\n";
								xml << "\t\t\t\t\t\t\t<mu2>" << dough_mu2 << "</mu2>\n";
								xml << "\t\t\t\t\t\t</ode>\n";
							xml << "\t\t\t\t\t</friction>\n";
						xml << "\t\t\t\t</surface>\n";
					xml << "\t\t\t</collision>\n";
					xml << "\t\t\t<visual name ='dough_visual'>\n";
						xml << "\t\t\t\t<geometry>\n";
							xml << "\t\t\t\t\t<box>\n";
								xml << "\t\t\t\t\t\t<size>" << dough_elem_width << " " << dough_elem_width << " " << dough_elem_thickness << "</size>\n";
							xml << "\t\t\t\t\t</box>\n";
						xml << "\t\t\t\t</geometry>\n"; 
						xml << "\t\t\t\t<material>\n";
							xml << "\t\t\t\t\t<script>\n";
								xml << "\t\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n";
								xml << "\t\t\t\t\t\t<name>Gazebo/Yellow</name>\n";
							xml << "\t\t\t\t\t</script>\n";
						xml << "\t\t\t\t</material>\n";
					xml << "\t\t\t</visual>\n";
                xml << "\t\t</link>\n";
				
                xml << "\t\t<link name='box_" << particle_counter << "'>\n";
					xml << "\t\t\t<pose>" << x << " " << y << " 0 0 0 0</pose>\n";
					xml << "\t\t\t<inertial>\n";
						xml << "\t\t\t\t<mass>" << box_mass << "</mass>\n";
					xml << "\t\t\t</inertial>\n";
					xml << "\t\t\t<collision name ='box_collision'>\n";
						xml << "\t\t\t\t<geometry>\n";
							xml << "\t\t\t\t\t<box>\n";
								xml << "\t\t\t\t\t\t<size>" << dough_elem_width << " " << dough_elem_width << " " << ".001</size>\n";
							xml << "\t\t\t\t\t</box>\n";
						xml << "\t\t\t\t</geometry>\n";
						xml << "\t\t\t\t<surface>\n";
							xml << "\t\t\t\t\t<friction>\n";
								xml << "\t\t\t\t\t\t<ode>\n";
								xml << "\t\t\t\t\t\t\t<mu>" << box_mu << "</mu>\n";
								xml << "\t\t\t\t\t\t\t<mu2>" << box_mu2 << "</mu2>\n";
								xml << "\t\t\t\t\t\t</ode>\n";
							xml << "\t\t\t\t\t</friction>\n";
						xml << "\t\t\t\t</surface>\n";
					xml << "\t\t\t</collision>\n";
					xml << "\t\t\t<visual name ='box_visual'>\n";
						xml << "\t\t\t\t<geometry>\n";
						xml << "\t\t\t\t\t<box>\n";
						xml << "\t\t\t\t\t\t<size>" << dough_elem_width << " " << dough_elem_width << " " << ".001</size>\n";
						xml << "\t\t\t\t\t</box>\n";
						xml << "\t\t\t\t</geometry>\n";
						xml << "\t\t\t\t<material>\n";
						xml << "\t\t\t\t\t<script>\n";
						xml << "\t\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n";
						xml << "\t\t\t\t\t\t<name>Gazebo/Yellow</name>\n";
						xml << "\t\t\t\t\t</script>\n";
						xml << "\t\t\t\t</material>\n"; 
					xml << "\t\t\t</visual>\n";
                xml << "\t\t</link>\n";

                xml << "\t\t<joint name ='joint_" << particle_counter << "' type='prismatic'>\n";
					xml << "\t\t\t\t<pose>0 0 " << DOUGH_MIN_HEIGHT << " 0 0 0</pose>\n";
					xml << "\t\t\t\t<parent>box_" << particle_counter << "</parent>\n";
					xml << "\t\t\t\t<child>dough_" << particle_counter << "</child>\n";
					xml << "\t\t\t\t<axis>\n";
						xml << "\t\t\t\t\t<dynamics>\n";
							xml << "\t\t\t\t\t\t<damping>" << damping << "</damping>\n";
							xml << "\t\t\t\t\t\t<friction>" << friction << "</friction>\n";
						xml << "\t\t\t\t\t</dynamics>\n";
						xml << "\t\t\t\t\t<xyz>0 0 1</xyz>\n";
					xml << "\t\t\t\t</axis>\n";      
                xml << "\t\t</joint>\n";
       
                particle_counter++;

            }
       }

       ///////////////////////////////////////////////////////////////////
        xml << "</model>\n";
        xml << "</sdf>\n";

        /////////////////////////////////
        //std::cout << xml.str() << "\n";

        sdf::SDF doughSDF;
        doughSDF.SetFromString(xml.str());
        // insert the model into the world in runtime
        _parent->InsertModelSDF(doughSDF);
            
    }

    public: double get_random(double min, double max) {
        /* Returns a random double between min and max */
        return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
        }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(FactoryDough)
}
