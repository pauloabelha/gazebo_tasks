#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include "gazebo/transport/transport.hh"
#include <math.h>
#include <sdf/sdf.hh>

#define PI 3.14159265

namespace gazebo
{
  class FactoryPancake : public WorldPlugin
  {
    
    
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        std::stringstream xml;

        // will hold the strings read from the sdf
        std::string px, py, pz, Ssphere_radius, Spancake_radius, Sinertia, Smass, Smu, Smu2, Sc_cfm, Sc_erp, Skp, Skd,
                Sj_cfm, Slimit_cfm, Slimit_erp;

        // will hold the actuavl values converted from the sdf strings
        double posx, posy, posz, sphere_radius, pancake_radius,  inertia, mass, mu, mu2, c_cfm, c_erp, kp, kd,
                 j_cfm, limit_cfm, limit_erp;



        // READ SDF ELEMENTS TO STRING
        px = _sdf->GetElement("x")->GetValue()->GetAsString();
        py = _sdf->GetElement("y")->GetValue()->GetAsString();
        pz = _sdf->GetElement("z")->GetValue()->GetAsString();   

        Spancake_radius = _sdf->GetElement("pancake_radius")->GetValue()->GetAsString();
        Ssphere_radius = _sdf->GetElement("sphere_radius")->GetValue()->GetAsString();

        Sinertia = _sdf-> GetElement("inertia")->GetValue()->GetAsString();
        Smass = _sdf->GetElement("mass")->GetValue()->GetAsString();
        Smu = _sdf->GetElement("mu")->GetValue()->GetAsString();
        Smu2 = _sdf->GetElement("mu2")->GetValue()->GetAsString();
         
        Sc_cfm = _sdf->GetElement("c_cfm")->GetValue()->GetAsString();
        Sc_erp = _sdf->GetElement("c_erp")->GetValue()->GetAsString();
        Skp = _sdf->GetElement("kp")->GetValue()->GetAsString();
        Skd = _sdf->GetElement("kd")->GetValue()->GetAsString();
       
        Sj_cfm = _sdf->GetElement("j_cfm")->GetValue()->GetAsString();
        Slimit_cfm = _sdf->GetElement("limit_cfm")->GetValue()->GetAsString();
        Slimit_erp = _sdf->GetElement("limit_erp")->GetValue()->GetAsString();

        
        
        // CONVERT SDF ELEMENTS FROM STRING TO DOUBLE
        posx = atof(px.c_str());
        posy = atof(py.c_str());
        posz = atof(pz.c_str());
 
        pancake_radius = atof(Spancake_radius.c_str());
        sphere_radius = atof(Ssphere_radius.c_str());

        inertia = atof(Sinertia.c_str());
        mass = atof(Smass.c_str());
        mu = atof(Smu.c_str());
        mu2 = atof(Smu2.c_str());
           
        c_cfm = atof(Sc_cfm.c_str());
        c_erp = atof(Sc_erp.c_str());
        kp = atof(Skp.c_str());
        kd = atof(Skd.c_str());
 
        j_cfm = atof(Sj_cfm.c_str());
        limit_cfm = atof(Slimit_cfm.c_str());
        limit_erp = atof(Slimit_erp.c_str());
       
        
        // TAKEN FROM ANDREI HAIDU ....................................
        unsigned int n_min = 7; //smallest number of spheres to be made
        unsigned int circles = int(PI * pancake_radius/sphere_radius);
        unsigned int n_circles = 0;
        double x, y, x_axis, y_axis;
        // ............................................................


        // START WRITING PANCAKE MODEL

        xml << "<sdf version ='1.6'>\n";
        xml << "<model name ='pancake'>\n";
        xml << "\t<static>false</static>\n";
        xml << "\t<pose>" << posx << " " << posy << " " << posz << " 0 0 0</pose>\n";

        // Center Link
        xml << "\t\t<link name='sphere_link_center'>\n";
        xml << "\t\t\t<self_collide>true</self_collide>\n";
        xml << "\t\t\t<pose>0 0 0 0 0 0</pose>\n";
        xml << "\t\t\t<inertial>\n";
        xml << "\t\t\t\t<pose>0 0 0 0 0 0</pose>\n";
        xml << "\t\t\t\t<inertia>\n";
        xml << "\t\t\t\t\t<ixx>" << inertia << "</ixx>\n";
        xml << "\t\t\t\t\t<ixy>0</ixy>\n";
        xml << "\t\t\t\t\t<ixz>0</ixz>\n";
        xml << "\t\t\t\t\t<iyy>" << inertia << "</iyy>\n";
        xml << "\t\t\t\t\t<iyz>0</iyz>\n";
        xml << "\t\t\t\t\t<izz>" << inertia << "</izz>\n";
        xml << "\t\t\t\t</inertia>\n";
        xml << "\t\t\t\t<mass>" << mass << "</mass>\n";
        xml << "\t\t\t</inertial>\n";
        xml << "\t\t\t<collision name ='collision_center'>\n";
        xml << "\t\t\t\t<geometry>\n";
        xml << "\t\t\t\t\t<sphere>\n";
        xml << "\t\t\t\t\t\t<radius>" << sphere_radius << "</radius>\n";
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
        xml << "\t\t\t\t\t\t<ode>\n";
        xml << "\t\t\t\t\t\t\t<soft_cfm>" << c_cfm << "</soft_cfm>\n";
        xml << "\t\t\t\t\t\t\t<soft_erp>" << c_erp << "</soft_erp>\n";
        xml << "\t\t\t\t\t\t\t<kp>" << kp << "</kp>\n";
        xml << "\t\t\t\t\t\t\t<kd>" << kd << "</kd>\n";
        xml << "\t\t\t\t\t\t</ode>\n";
        xml << "\t\t\t\t\t</contact>\n";
        xml << "\t\t\t\t</surface>\n";
        xml << "\t\t\t</collision>\n";
        xml << "\t\t\t<visual name ='visual_center'>\n";
        xml << "\t\t\t\t<geometry>\n";
        xml << "\t\t\t\t\t<sphere>\n";
        xml << "\t\t\t\t\t\t<radius>" << sphere_radius << "</radius>\n";
        xml << "\t\t\t\t\t</sphere>\n";
        xml << "\t\t\t\t</geometry>\n";
        xml << "\t\t\t\t<material>\n";
        xml << "\t\t\t\t\t<script>\n";
        xml << "\t\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n";
        xml << "\t\t\t\t\t\t<name>Gazebo/Yellow</name>\n";
        xml << "\t\t\t\t\t</script>\n";
        xml << "\t\t\t\t</material>\n";
        xml << "\t\t\t</visual>\n";
        xml << "\t\t</link>\n";



        // TAKEN FROM ANDREI HAIDU..............................................................................................        
        // Calculate the number of Circle Links that can fit
        while (circles > n_min)
        {
            circles = circles/2;
            n_circles++;
            
        }

        // Write Circle Links
        for (int i=0; i < (int) PI*(pancake_radius*2/pow(2,n_circles))/sphere_radius; i++)
        {
            x = (pancake_radius*2/pow(2,n_circles)) * cos(i * 2 * PI / (PI * (pancake_radius*2/pow(2,n_circles))/sphere_radius));
            y = (pancake_radius*2/pow(2,n_circles)) * sin(i * 2 * PI / (PI * (pancake_radius*2/pow(2,n_circles))/sphere_radius));

            x_axis = (double) -y / sqrt((x*x)+(y*y));
            if (x_axis == -0){x_axis = 0;}
            y_axis = (double)  x / sqrt((x*x)+(y*y));
        // ,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,

            xml << "\t\t<link name ='sphere_link_" << n_circles << "_" << i << "'>\n";
            xml << "\t\t\t<self_collide>true</self_collide>\n";
            xml << "\t\t\t<pose>" << x << " " << y << " 0 0 0 0</pose>\n";
            xml << "\t\t\t<inertial>\n";
            xml << "\t\t\t\t<pose> 0 0 0 0 0 0 </pose>\n";
            xml << "\t\t\t\t<inertia>\n";
            xml << "\t\t\t\t\t<ixx>0.000001</ixx>\n";
            xml << "\t\t\t\t\t<ixy>0</ixy>\n";
            xml << "\t\t\t\t\t<ixz>0</ixz>\n";
            xml << "\t\t\t\t\t<iyy>0.000001</iyy>\n";
            xml << "\t\t\t\t\t<iyz>0</iyz>\n";
            xml << "\t\t\t\t\t<izz>0.000001</izz>\n";
            xml << "\t\t\t\t</inertia>\n";
            xml << "\t\t\t\t<mass>" << mass*5 << "</mass>\n";
            xml << "\t\t\t</inertial>\n";
            xml << "\t\t\t<collision name ='collision_" << n_circles << "_" << i << "'>\n";
            xml << "\t\t\t\t<geometry>\n";
            xml << "\t\t\t\t\t<sphere>\n";
            xml << "\t\t\t\t\t\t<radius>" << sphere_radius << "</radius>\n";
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
            xml << "\t\t\t\t\t\t<ode>\n";
            xml << "\t\t\t\t\t\t\t<soft_cfm>" << c_cfm << "</soft_cfm>\n";
            xml << "\t\t\t\t\t\t\t<soft_erp>" << c_erp << "</soft_erp>\n";
            xml << "\t\t\t\t\t\t\t<kp>" << kp << "</kp>\n";
            xml << "\t\t\t\t\t\t\t<kd>" << kd << "</kd>\n";
            xml << "\t\t\t\t\t\t</ode>\n";
            xml << "\t\t\t\t\t</contact>\n";
            xml << "\t\t\t\t</surface>\n";
            xml << "\t\t\t</collision>\n";
            xml << "\t\t\t<visual name ='visual_" << n_circles << "_" << i << "'>\n";
            xml << "\t\t\t\t<geometry>\n";
            xml << "\t\t\t\t\t<sphere>\n";
            xml << "\t\t\t\t\t\t<radius>" << sphere_radius << "</radius>\n";
            xml << "\t\t\t\t\t</sphere>\n";
            xml << "\t\t\t\t</geometry>\n";
            xml << "\t\t\t\t<material>\n";
            xml << "\t\t\t\t\t<script>\n";
            xml << "\t\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n";
            xml << "\t\t\t\t\t\t<name>Gazebo/Yellow</name>\n";
            xml << "\t\t\t\t\t</script>\n";
            xml << "\t\t\t\t</material>\n";
            xml << "\t\t\t</visual>\n";
            xml << "\t\t</link>\n";

            xml << "\t\t<joint name ='joint_" << n_circles << "_" << i << "' type='revolute'>\n";
            xml << "\t\t\t\t<pose>0 0 0 0 0 0</pose>\n";
            xml << "\t\t\t\t<parent>sphere_link_" << n_circles << "_" << i << "</parent>\n";
            xml << "\t\t\t\t<child>sphere_link_center</child>\n";
            xml << "\t\t\t\t<axis>\n";
            xml << "\t\t\t\t\t<dynamics>\n";
            xml << "\t\t\t\t\t\t<damping>0.0</damping>\n";
            xml << "\t\t\t\t\t</dynamics>\n"; 
            xml << "\t\t\t\t\t<xyz>" << x_axis << " " << y_axis << " 0</xyz>\n";
            xml << "\t\t\t\t</axis>\n";
            xml << "\t\t\t\t<physics>\n";
            xml << "\t\t\t\t\t<ode>\n";
            xml << "\t\t\t\t\t\t<cfm>" << j_cfm << "</cfm>\n";
            xml << "\t\t\t\t\t\t<limit>\n";
            xml << "\t\t\t\t\t\t\t<cfm>" << limit_cfm << "</cfm>\n";
            xml << "\t\t\t\t\t\t\t<erp>" << limit_erp << "</erp>\n";
            xml << "\t\t\t\t\t\t</limit>\n";
            xml << "\t\t\t\t\t</ode>\n";
            xml << "\t\t\t\t</physics>\n";
            xml << "\t\t</joint>\n";
        }

        xml << "</model>\n";
        xml << "</sdf>\n";

        sdf::SDF pancakeSDF;
        pancakeSDF.SetFromString(xml.str());

        _parent->InsertModelSDF(pancakeSDF);

    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(FactoryPancake)
}
