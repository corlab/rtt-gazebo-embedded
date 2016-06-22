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
//    bool spawnModel(const std::string& instanceName, const std::string& modelName);
    bool spawnModel(const std::string& instanceName,
    		const std::string& modelName, const int timeoutSec);
    bool toggleDynamicsSimulation(const bool activate);

    ~RTTGazeboEmbedded();

protected:
    void WorldUpdateBegin();
    void WorldUpdateEnd();
    void OnPause(const bool _pause);

    bool resetModelPoses();
    bool resetWorld();

    bool startHook();
    void runWorldForever();
    void updateHook();
    void stopHook();

    void cleanupHook();

    void pauseSimulation();
    void unPauseSimulation();

    void checkClientConnections();

    std::string world_path;
    gazebo::physics::WorldPtr world;
    gazebo::event::ConnectionPtr world_begin;
    gazebo::event::ConnectionPtr world_end;
    gazebo::event::ConnectionPtr _pause;

    std::vector<double> gravity_vector;
    std::vector<std::string> argv;
    bool use_rtt_sync;
    RTT::os::Semaphore go_sem;

    std::thread run_th;

    boost::atomic<bool> _is_paused;

    bool isWorldConfigured;

    // Useful for threaded updates
    struct ClientConnection
    {
        ClientConnection(){}
        ClientConnection(RTT::OperationCaller<void(void)> world_update_begin,
                         RTT::OperationCaller<void(void)> world_update_end):
                         world_begin(world_update_begin),
                         world_end(world_update_end){}
        RTT::OperationCaller<void(void)> world_begin;
        RTT::OperationCaller<void(void)> world_end;
        RTT::SendHandle<void(void)> world_begin_handle;
        RTT::SendHandle<void(void)> world_end_handle;
    };

    std::map<std::string,ClientConnection> client_map;
};

#endif
