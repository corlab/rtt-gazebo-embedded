#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <Eigen/Dense>

using namespace RTT;
using namespace RTT::os;

class RTTGazebo : public RTT::TaskContext
{
public:
    RTTGazebo(const std::string& name):
    TaskContext(name),
    world_path("worlds/empty.world"),
    model_name(name)
    {
        RTT::log(RTT::Info) << "Creating " << name <<" with gazebo embedded !" << RTT::endlog();
        this->addProperty("world_path",world_path).doc("The path to the .world file.");
        this->addOperation("add_plugin",&RTTGazebo::addPlugin,this,RTT::OwnThread).doc("The path to a plugin file.");
        this->addProperty("argv",argv).doc("argv passed to the deployer's main.");
        this->addProperty("model_name",model_name).doc("The name of the robot.");
        
        this->ports()->addPort("JointPosition", port_joint_position_out).doc("");
        this->ports()->addPort("JointVelocity", port_joint_velocity_out).doc("");
        this->ports()->addPort("JointTorque", port_joint_torque_out).doc("");

        this->ports()->addPort("JointPositionCommand", port_joint_position_cmd_in).doc("");
        this->ports()->addPort("JointVelocityCommand", port_joint_velocity_cmd_in).doc("");
        this->ports()->addPort("JointTorqueCommand", port_joint_torque_cmd_in).doc("");
        
        gazebo::printVersion();
        gazebo::common::Console::SetQuiet(false);
    }
    void addPlugin(const std::string& filename)
    {
        gazebo::addPlugin(filename);
    }
    void setWorldFilePath(const std::string& file_path)
    {
        world_path = file_path;
    }
    bool configureHook()
    {
        RTT::log(RTT::Info) << "Creating world at "<< world_path << RTT::endlog();
        
        if(! gazebo::setupServer(argv))
        {
            RTT::log(RTT::Error) << "Could not setupServer " << RTT::endlog();
            return false;
        }

        world = gazebo::loadWorld(world_path);
        return world != 0;
    }
    void cleanupHook()
    {
        RTT::log(RTT::Info) << "Cleanup "<< gazebo::shutdown() << RTT::endlog();
    }
    void updateHook()
    {
        RTT::log(RTT::Debug) << "Running once at  "<< RTT::os::TimeService::Instance()->getNSecs() << RTT::endlog();
        gazebo::runWorld(world, 1);
        robot =  world->GetModel(model_name);
        
        gazeboConfigureHook(robot);
        readPorts();
        updateSim(robot);
        writetoPorts();
    }
    void readPorts()
    {
        if(!model_configured)
            return;
        jnt_pos_cmd_in_fs = port_joint_position_cmd_in.readNewest(jnt_pos_cmd_in);
        jnt_trq_cmd_in_fs = port_joint_torque_cmd_in.readNewest(jnt_trq_cmd_in);
    }
    void writetoPorts()
    {
        if(!model_configured)
            return;
        port_joint_position_out.write(jnt_pos);
        port_joint_velocity_out.write(jnt_vel);
        port_joint_torque_out.write(jnt_trq);
    }
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model)
    {
        if(!model) {
            RTT::log(RTT::Error)<<"No model could be loaded"<<RTT::endlog();
            return false;
        }
        
        if(model_configured)
            return true;
        
        // Get the joints
        gazebo_joints = model->GetJoints();
        gazebo_links = model->GetLinks();

        RTT::log(RTT::Info)<<"Model has "<<gazebo_joints.size()<<" joints"<<RTT::endlog();

        // NOTE: Get the joint names and store their indices
        // Because we have base_joint (fixed), j0...j6, ati_joint (fixed)
        int idx = 0;
        for(auto& jit : gazebo_joints)
        {

            const std::string name = jit->GetName();
            // NOTE: Remove fake fixed joints (revolute with upper==lower==0
            // NOTE: This is not used anymore thanks to <disableFixedJointLumping>
            // Gazebo option (ati_joint is fixed but gazebo can use it )

            if(jit->GetLowerLimit(0u) == jit->GetUpperLimit(0u))
            {
                RTT::log(RTT::Warning)<<"Not adding (fake) fixed joint ["<<name<<"] idx:"<<idx<<RTT::endlog();
                continue;
            }
            joint_idx.push_back(idx);
            joint_names.push_back(name);
            RTT::log(RTT::Info)<<"Adding joint ["<<name<<"] idx:"<<idx<<RTT::endlog();
            idx++;
        }

        if(joint_idx.size() == 0)
        {
            RTT::log(RTT::Error) << "No Joints could be added, exiting" << RTT::endlog();
            return false;
        }

        RTT::log(RTT::Info)<<"Gazebo model found "<<joint_idx.size()<<" joints "<<RTT::endlog();
        
        jnt_pos.resize(joint_idx.size());
        jnt_vel.resize(joint_idx.size());
        jnt_trq.resize(joint_idx.size());
        jnt_pos_cmd_in.resize(joint_idx.size());
        jnt_trq_cmd_in.resize(joint_idx.size());
        
        model_configured = true;
        return true;
    }
    
    void updateSim(gazebo::physics::ModelPtr model)
    {
        if(!model){
            return;
        }

        // Read From gazebo simulation
        for(unsigned j=0; j<joint_idx.size(); j++) {
            jnt_pos[j] = gazebo_joints[joint_idx[j]]->GetAngle(0).Radian();
            jnt_vel[j] = gazebo_joints[joint_idx[j]]->GetVelocity(0);
            jnt_trq[j] = gazebo_joints[joint_idx[j]]->GetForce(0u);
        }


        if(port_joint_torque_cmd_in.connected())
        {
            // Enable gravity for everyone
            for(auto& link : gazebo_links)
                link->SetGravityMode(true);

            // Set Gravity Mode or specified links
            for(auto& grav_mode : gravity_mode)
                grav_mode.first->SetGravityMode(grav_mode.second);

            for(unsigned j=0; j<joint_idx.size(); j++)
                gazebo_joints[joint_idx[j]]->SetForce(0,jnt_trq_cmd_in[j]);
        }
        else
        {
            // If no one is connected, stop gravity
            for(auto& link : gazebo_links)
                    link->SetGravityMode(false);
        }
        log(RTT::Debug) << getName() << " gazeboUpdateHook() END "<< TimeService::Instance()->getNSecs() << endlog();
    }
protected:
    gazebo::physics::WorldPtr world;
    std::string world_path;
    std::vector<std::string> argv;
    std::string model_name;
    gazebo::physics::Link_V gazebo_links;
    gazebo::physics::Joint_V gazebo_joints;
    std::vector<unsigned> joint_idx;
    std::vector<std::string> joint_names;
    gazebo::physics::ModelPtr robot;
    std::map<gazebo::physics::LinkPtr,bool> gravity_mode;
    bool model_configured;
    
    RTT::FlowStatus jnt_pos_cmd_in_fs,
                    jnt_trq_cmd_in_fs;
    RTT::OutputPort<Eigen::VectorXd> port_joint_position_out,
                                     port_joint_velocity_out,
                                     port_joint_torque_out;
    
    Eigen::VectorXd jnt_pos,
                    jnt_vel,
                    jnt_trq;
                    
    RTT::InputPort<Eigen::VectorXd> port_joint_position_cmd_in,
                                    port_joint_velocity_cmd_in,
                                    port_joint_torque_cmd_in;
    Eigen::VectorXd jnt_pos_cmd_in,
                    jnt_vel_cmd_in,
                    jnt_trq_cmd_in;                                 
};


ORO_CREATE_COMPONENT(RTTGazebo)