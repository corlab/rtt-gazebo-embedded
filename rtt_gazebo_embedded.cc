#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/os/Semaphore.hpp>
#include <Eigen/Dense>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity_manager.h>
#include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <rtt_ros_kdl_tools/tools.hpp>
#include <rtt_ros_kdl_tools/chaincogsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <std_srvs/Empty.h>
#include <memory>
#include <thread>
#include <visualization_msgs/Marker.h>
#include <rtt/Port.hpp>

using namespace RTT;
using namespace RTT::os;

class RTTGazebo : public RTT::TaskContext
{
public:
    RTTGazebo(const std::string& name):
    TaskContext(name),
    world_path("worlds/empty.world"),
    model_name(name),
    go_sem(0),
    model_timeout_s(20.0),
    use_rtt_sync(false),
    model_configured(false)
    {
        RTT::log(RTT::Info) << "Creating " << name <<" with gazebo embedded !" << RTT::endlog();
        this->ports()->addPort("sync",port_sync).doc("Migth be used to trigger your component's updateHook().");
        this->addProperty("use_rtt_sync",use_rtt_sync).doc("Gazebo ties to run at the component's rate (or slower).");
        this->addProperty("world_path",world_path).doc("The path to the .world file.");
        this->addOperation("add_plugin",&RTTGazebo::addPlugin,this,RTT::OwnThread).doc("The path to a plugin file.");
        this->addProperty("argv",argv).doc("argv passed to the deployer's main.");
        this->addProperty("model_name",model_name).doc("The name of the robot.");
        this->addProperty("model_timeout_s",model_timeout_s).doc("Time during which we wait for the model to be spawned.");
        this->addProperty("run_world_elapsed",run_world_elapsed).doc("Duration of run world");
        this->addOperation("isModelConfigured",&RTTGazebo::isModelConfigured,this,RTT::ClientThread).doc("True if the model has been loaded.");
        
        this->ports()->addPort("CenterOfGravityMarker", port_cog_marker).doc("");
        
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
        if(std::ifstream(file_path))  
            world_path = file_path;
        else
            RTT::log(RTT::Error) << "File "<<file_path<<"does not exists."<<RTT::endlog();
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
                
        /*if(!gazeboConfigureHookThread())
        {
            RTT::log(RTT::Fatal) << "Gazebo configure Thread failed" << RTT::endlog();
            return false;
        }*/
        gz_conf_th = std::thread(std::bind(&RTTGazebo::gazeboConfigureHookThread,this));
        gz_conf_th.join();
        
        //rtt_rosclock::use_ros_clock_topic();
        //rtt_rosclock::enable_sim();
        
        RTT::log(RTT::Info) << "Binding world events" << RTT::endlog();
        world_begin =  gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RTTGazebo::writeToSim,this));
        world_end = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&RTTGazebo::readSim,this));
        
        return true;
    }
    bool gazeboConfigureHookThread()
    {
        auto tstart = RTT::os::TimeService::Instance()->getTicks();
        while(true)
        {
            auto elapsed = RTT::os::TimeService::Instance()->getSeconds(tstart);
            if(elapsed > model_timeout_s)
            {
                RTT::log(RTT::Error) << "Model timeout" << RTT::endlog();
                return false;
            }
            gazebo::runWorld(world, 1);
            robot = world->GetModel(model_name);
            if(gazeboConfigureHook(robot))
            {
                RTT::log(RTT::Info)<<"Done configuring model "<<RTT::endlog();
                return true;
            }
            usleep(1E6);
        }
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
    bool startHook()
    {
        if(!run_th.joinable())
            run_th = std::thread(std::bind(&RTTGazebo::runWorldForever,this));
        else
            std::cout <<"\x1B[32m[[--- Gazebo already running ---]]\033[0m"<<std::endl;
        return true;
    }
    void runWorldForever()
    {
        std::cout <<"\x1B[32m[[--- Gazebo running ---]]\033[0m"<<std::endl;
        gazebo::runWorld(world, 0); // runs forever 
    }
    void updateHook()
    {
        go_sem.signal();
        return;
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
        ticks_start = RTT::os::TimeService::Instance()->getTicks();

        if(!model_configured) 
            return;
        
        readPorts();
        
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
            
            jnt_trq_cmd_out = jnt_trq_cmd_in + jnt_trq_gravity.data;
                        
            //std::cout << "T "<<jnt_trq_cmd_out.transpose()<<std::endl;
           
            for(unsigned j=0; j<joint_idx.size(); j++)
                gazebo_joints[joint_idx[j]]->SetForce(0,jnt_trq_cmd_out[j]);
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
        
        port_sync.write(true);
        
        static visualization_msgs::Marker marker;
        static KDL::Vector cog;
        cog_solver->JntToCoG(jnt_pos,cog);
        createCoGMarker("",gazebo_links[0]->GetName(),0.1,cog,marker);
        port_cog_marker.write(marker);
    }
    bool gazeboConfigureHook(const gazebo::physics::ModelPtr& model)
    {
        if(!model) {
            RTT::log(RTT::Warning)<<"Waiting for "<<model_name<<" to be loaded"<<RTT::endlog();
            return false;
        }

        if(model_configured)
            return true;

        // Get the joints
        joint_idx.clear();
        joint_names.clear();
        
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

        RTT::log(RTT::Info)<<"Creating KDL Chain "<<RTT::endlog();
        
        static KDL::Chain chain;
        static KDL::Tree tree;
    
        if(!rtt_ros_kdl_tools::initChainFromROSParamURDF(tree,chain))
        {
            RTT::log(RTT::Fatal)<<"Could not create the KDL chain"<<RTT::endlog();
            return false;
        }
        const unsigned int dof = chain.getNrOfJoints();
        
        if(dof != joint_idx.size())
        {
            RTT::log(RTT::Fatal)<<"Sizes dont match : gazebo("<<joint_idx.size()<<") , kdl("<<dof<<")"<<RTT::endlog();
            return false;
        }
        
        dyn_param.reset(new KDL::ChainDynParam(chain,KDL::Vector(world->Gravity().X(),
                                                                 world->Gravity().Y(),
                                                                 world->Gravity().Z())));
        cog_solver.reset(new KDL::ChainCoGSolver(chain));
        
        jnt_pos.resize(dof);
        jnt_vel.resize(dof);
        jnt_trq.resize(dof);
        jnt_pos_cmd_in.resize(dof);
        jnt_trq_cmd_in.resize(dof);
        jnt_trq_gravity.resize(dof);
        jnt_trq_cmd_out.resize(dof);
        
        jnt_pos.data.setZero();
        jnt_vel.setZero();
        jnt_trq.setZero();
        jnt_pos_cmd_in.setZero();
        jnt_vel_cmd_in.setZero();
        jnt_trq_cmd_in.setZero();
        jnt_trq_gravity.data.setZero();
        jnt_trq_cmd_out.setZero();
        
        port_joint_position_out.setDataSample(jnt_pos.data);
        port_joint_velocity_out.setDataSample(jnt_vel);
        port_joint_torque_out.setDataSample(jnt_trq);
        
        model_configured = true;
        
        return true;
    }
    
    void createCoGMarker(const std::string& ns, const std::string& frame_id, double radius, const KDL::Vector& cog, visualization_msgs::Marker& marker) const{
        marker.header.frame_id = frame_id;
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cog.x();
        marker.pose.position.y = cog.y();
        marker.pose.position.z = cog.z();
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = radius;
        marker.color.r = 1.0;
        marker.color.a = 0.8;
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
        
        writetoPorts();
        ticks_stop = RTT::os::TimeService::Instance()->getTicks();
        run_world_elapsed = RTT::os::TimeService::Instance()->getSeconds(ticks_start);
        
        if(use_rtt_sync)
            go_sem.wait();
    }
    virtual ~RTTGazebo(){}
    bool readyROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& res)
    {
        return true;
    }
protected:
    gazebo::event::ConnectionPtr world_begin;
    gazebo::event::ConnectionPtr world_end;
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
    std::string root_link,tip_link,robot_description;
    std::thread gz_conf_th,run_th;

    RTT::FlowStatus jnt_pos_cmd_in_fs,
                    jnt_trq_cmd_in_fs;
    RTT::OutputPort<bool> port_sync;
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
                    jnt_trq_cmd_in,
                    jnt_trq_cmd_out;
                    
    std::unique_ptr<KDL::ChainDynParam> dyn_param;
    std::unique_ptr<KDL::ChainCoGSolver> cog_solver;
    RTT::OutputPort<visualization_msgs::Marker> port_cog_marker;
    RTT::os::MutexRecursive configure_mutex;
    RTT::os::TimeService::ticks ticks_start,ticks_stop;
    double run_world_elapsed,model_timeout_s;
    RTT::os::Semaphore go_sem;
    bool use_rtt_sync;
};


ORO_CREATE_COMPONENT(RTTGazebo)
