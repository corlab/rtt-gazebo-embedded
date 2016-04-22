#ifndef __GZ_EMBEDDED__
#define __GZ_EMBEDDED__

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/os/Semaphore.hpp>
#include <rtt/OperationCaller.hpp>
#include <Eigen/Dense>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity.h>
#include <rtt_rosclock/rtt_rosclock_sim_clock_activity_manager.h>

#include <kdl/chaindynparam.hpp>
#include <std_srvs/Empty.h>
#include <memory>
#include <functional>
#include <thread>
#include <rtt/Port.hpp>

class RTTGazeboEmbedded : public RTT::TaskContext
{
public:
    RTTGazeboEmbedded(const std::string& name);
    void addPlugin(const std::string& filename);
    void setWorldFilePath(const std::string& file_path);
    bool configureHook();
//     void updateROSClock();

    virtual ~RTTGazeboEmbedded(){};
    bool readyROSService(std_srvs::EmptyRequest& req,std_srvs::EmptyResponse& res);
    // Not thread safe !
    gazebo::physics::WorldPtr getWorldPtr();
    gazebo::physics::ModelPtr getModelPtr(const std::string& model_name,double timeout_s);
    
protected:
    bool startHook();
    void runWorldForever();
    void updateHook();
    void stopHook();
    void writeToSim();
    void cleanupHook();
    void readSim();
    void pause();
    void unPause();
    void checkClientConnections();
    std::string world_path;
    gazebo::physics::WorldPtr world;
    gazebo::event::ConnectionPtr world_begin;
    gazebo::event::ConnectionPtr world_end;
    
    std::thread get_model_thread;
    
    std::vector<std::string> argv;
    std::string model_name;
    gazebo::physics::Link_V gazebo_links;
    gazebo::physics::Joint_V gazebo_joints;
    std::vector<unsigned> joint_idx;
    std::vector<std::string> joint_names;
    
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
    RTT::os::MutexRecursive configure_mutex;
    RTT::os::TimeService::ticks ticks_start,ticks_stop;
    double run_world_elapsed,model_timeout_s;
    RTT::os::Semaphore go_sem;
    bool use_rtt_sync;
    
    struct ClientConnection
    {
        ClientConnection(){}
        ClientConnection(RTT::OperationCaller<void(void)> read,
                         RTT::OperationCaller<void(void)> write):
                         read_callback(read),
                         write_callback(write){}
        RTT::OperationCaller<void(void)> read_callback;
        RTT::OperationCaller<void(void)> write_callback;
        RTT::SendHandle<void(void)> write_handle;
        RTT::SendHandle<void(void)> read_handle;
    };
    std::map<std::string,ClientConnection> client_map;
};

#endif
