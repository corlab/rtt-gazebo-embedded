#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <Eigen/Dense>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity_manager.h>
#include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <rtt_ros_kdl_tools/tools.hpp>
#include <kdl/chaindynparam.hpp>
#include <std_srvs/Empty.h>
#include <rtt_roscomm/rosservice.h>

using namespace RTT;
using namespace RTT::os;

class RTTGazebo : public RTT::TaskContext
{
public:
    RTTGazebo(const std::string& name):
    TaskContext(name),
    world_path("worlds/empty.world"),
    model_name(name),
    iters(1),
    model_configured(false)
    {
        RTT::log(RTT::Info) << "Creating " << name <<" with gazebo embedded !" << RTT::endlog();
        this->addProperty("world_path",world_path).doc("The path to the .world file.");
        this->addOperation("add_plugin",&RTTGazebo::addPlugin,this,RTT::OwnThread).doc("The path to a plugin file.");
        //this->addOperation("gazeboConfigure",&RTTGazebo::gazeboConfigureHookThread,this,RTT::ClientThread).doc("Wait on the model and load it.");
        this->addProperty("argv",argv).doc("argv passed to the deployer's main.");
        this->addProperty("model_name",model_name).doc("The name of the robot.");
        this->addProperty("iters",iters).doc("The number of iterations to do at each step() of the world.");
        this->addOperation("isModelConfigured",&RTTGazebo::isModelConfigured,this,RTT::ClientThread).doc("True if the model has been loaded.");
        this->ports()->addPort("JointPosition", port_joint_position_out).doc("");
        this->ports()->addPort("JointVelocity", port_joint_velocity_out).doc("");
        this->ports()->addPort("JointTorque", port_joint_torque_out).doc("");

        this->ports()->addPort("JointPositionCommand", port_joint_position_cmd_in).doc("");
        this->ports()->addPort("JointVelocityCommand", port_joint_velocity_cmd_in).doc("");
        this->ports()->addPort("JointTorqueCommand", port_joint_torque_cmd_in).doc("");
        
        this->addOperation("readyROSService",&RTTGazebo::readyROSService,this,RTT::OwnThread).doc("Say everyone the component is ready.");

        gazebo::printVersion();
        gazebo::common::Console::SetQuiet(false);
    }
    bool isModelConfigured()
    {
        return model_configured;
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
        if(!world) return false;
                
        if(!gazeboConfigureHookThread())
            return false;

        rtt_rosclock::use_ros_clock_topic();
        rtt_rosclock::enable_sim();
        
        return true;
    }
    bool gazeboConfigureHookThread()
    {
        int n = 20;
        while(n-- > 0)
        {

            gazebo::runWorld(world, iters);
            robot = world->GetModel(model_name);
            if(gazeboConfigureHook(robot))
                break;
            //updateROSClock();
            usleep(1E6);
        }
        //configure_mutex.unlock();
        if(n>0)
            return true;
        return false;
    }
    void updateROSClock()
    {
        gazebo::common::Time gz_time = world->GetSimTime();

        rtt_rosclock::update_sim_clock(ros::Time(gz_time.sec, gz_time.nsec));
    }
    void cleanupHook()
    {
        gazebo::shutdown();
    }
    void updateHook()
    {
        /*RTT::log(RTT::Debug) << "Running once at  "<< RTT::os::TimeService::Instance()->getNSecs() << RTT::endlog();
        RTT::log(RTT::Debug) << "- Sim clock : "<< world->GetSimTime()
            <<"\n- Real Time : "<<world->GetRealTime()
            <<"\n- common::Time::GetWallTime() : "<<gazebo::common::Time::GetWallTime()
            <<"\n- rosclock : "<<rtt_rosclock::host_now()<<RTT::endlog();*/

        readPorts();

        writeToSim();

        // BUG : https://bitbucket.org/osrf/gazebo/issues/1216/getrealtime-resets-with-each-call-to
        // So I have to modify this runBlocking function manually waiting for this bug to be fixed
        
        gazebo::runWorld(world, iters); 
        
        if(!robot)
            robot =  world->GetModel(model_name);
        
        if(robot && !model_configured)
            gazeboConfigureHook(robot);

        readSim();
        writetoPorts();

        if(0 && model_configured)
            updateROSClock();
    }
    void readPorts()
    {
        if(!model_configured)
            return;
        jnt_pos_cmd_in_fs = port_joint_position_cmd_in.readNewest(jnt_pos_cmd_in);
        jnt_trq_cmd_in_fs = port_joint_torque_cmd_in.readNewest(jnt_trq_cmd_in);
    }
    void writeToSim()
    {
        if(!model_configured)
            return;
        if(port_joint_position_cmd_in.connected() 
            || port_joint_torque_cmd_in.connected())
        {
            dyn_param->JntToGravity(jnt_pos,jnt_trq_gravity);
            // Enable gravity for everyone
           for(auto& link : gazebo_links)
                 link->SetGravityMode(true);

            // Set Gravity Mode or specified links
            for(auto& it : gravity_mode)
                        it.first->SetGravityMode(it.second);

            for(unsigned j=0; j<joint_idx.size(); j++)
                gazebo_joints[joint_idx[j]]->SetForce(0,jnt_trq_cmd_in[j] + jnt_trq_gravity.data[j]);
        }
        else
        {
            // If no one is connected, stop gravity
           for(auto& link : gazebo_links)
                 link->SetGravityMode(false);
        }
    }
    void writetoPorts()
    {
        if(!model_configured)
            return;
        port_joint_position_out.write(jnt_pos.data);
        port_joint_velocity_out.write(jnt_vel);
        port_joint_torque_out.write(jnt_trq);
    }
    bool gazeboConfigureHook(gazebo::physics::ModelPtr model)
    {
        if(!model) {
            RTT::log(RTT::Warning)<<"Waiting for "<<model_name<<" to be loaded"<<RTT::endlog();
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
                RTT::log(RTT::Info)<<"Not adding (fake) fixed joint ["<<name<<"] idx:"<<idx<<RTT::endlog();
                idx++;
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
        jnt_trq_gravity.resize(joint_idx.size());

        jnt_pos.data.setZero();
        jnt_vel.setZero();
        jnt_trq.setZero();
        jnt_pos_cmd_in.setZero();
        jnt_vel_cmd_in.setZero();
        jnt_trq_cmd_in.setZero();
        jnt_trq_gravity.data.setZero();
        
        port_joint_position_out.setDataSample(jnt_pos.data);
        port_joint_velocity_out.setDataSample(jnt_vel);
        port_joint_torque_out.setDataSample(jnt_trq);

        RTT::log(RTT::Info)<<"Creating KDL Chain "<<RTT::endlog();
        if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(tree,chain))
            return false;
        dyn_param.reset(new KDL::ChainDynParam(chain,KDL::Vector(world->Gravity().X(),
                                                                 world->Gravity().Y(),
                                                                 world->Gravity().Z())));

//         boost::shared_ptr<rtt_rosservice::ROSService> rosservice = this->getProvider<rtt_rosservice::ROSService>("rosservice");
//         if(rosservice)
//             rosservice->connect("readyROSService",getName()+"/ready","std_srvs/Empty");
        //rtt_rosclock::use_manual_clock();
        //rtt_rosclock::enable_sim();
        model_configured = true;
        RTT::log(RTT::Info)<<"Done configuring model "<<RTT::endlog();
        return true;
    }

    void readSim()
    {
        if(!model_configured)
            return;

        // Read From gazebo simulation
        for(unsigned j=0; j<joint_idx.size(); j++) {
            jnt_pos.data[j] = gazebo_joints[joint_idx[j]]->GetAngle(0).Radian();
            jnt_vel[j] = gazebo_joints[joint_idx[j]]->GetVelocity(0);
            jnt_trq[j] = gazebo_joints[joint_idx[j]]->GetForce(0u);
        }
    }
    virtual ~RTTGazebo(){}
    bool readyROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& res)
    {
        return true;
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
    KDL::JntArray jnt_trq_gravity,jnt_pos;
    std::atomic<bool> model_configured;
    unsigned int iters;
    std::string root_link,tip_link,robot_description;
    boost::thread gz_conf_th;

    RTT::FlowStatus jnt_pos_cmd_in_fs,
                    jnt_trq_cmd_in_fs;

    RTT::OutputPort<Eigen::VectorXd> port_joint_position_out,
                                     port_joint_velocity_out,
                                     port_joint_torque_out;

    Eigen::VectorXd jnt_vel,
                    jnt_trq;

    RTT::InputPort<Eigen::VectorXd> port_joint_position_cmd_in,
                                    port_joint_velocity_cmd_in,
                                    port_joint_torque_cmd_in;
    Eigen::VectorXd jnt_pos_cmd_in,
                    jnt_vel_cmd_in,
                    jnt_trq_cmd_in;
    boost::scoped_ptr<KDL::ChainDynParam> dyn_param;
    RTT::os::MutexRecursive configure_mutex;
    KDL::Chain chain;
    KDL::Tree tree;
};


ORO_CREATE_COMPONENT(RTTGazebo)
