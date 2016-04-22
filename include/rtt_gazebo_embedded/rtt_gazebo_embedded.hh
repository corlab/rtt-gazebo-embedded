#ifndef __GZ_EMBEDDED__
#define __GZ_EMBEDDED__

// Gazebo headers
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// RTT headers
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/Semaphore.hpp>

#include <thread>


class RTTGazeboEmbedded : public RTT::TaskContext
{
public:
    RTTGazeboEmbedded(const std::string& name);
    void addPlugin(const std::string& filename);
    void setWorldFilePath(const std::string& file_path);
    bool configureHook();

    virtual ~RTTGazeboEmbedded(){};
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
  
    std::vector<std::string> argv;
    bool use_rtt_sync;
    RTT::os::Semaphore go_sem;
    
    std::thread run_th;
    
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
