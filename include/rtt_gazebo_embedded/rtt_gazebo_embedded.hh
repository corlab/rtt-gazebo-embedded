#ifndef __GZ_EMBEDDED__
#define __GZ_EMBEDDED__

// Gazebo headers
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/util/LogRecord.hh>

// RTT headers
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/Semaphore.hpp>

// TinyXML header
#include "tinyxml.h"

// RST-RT headers
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/geometry/Translation.hpp>
#include <rst-rt/geometry/Rotation.hpp>

#include <thread>

class RTTGazeboEmbedded : public RTT::TaskContext
{
  public:
	RTTGazeboEmbedded(const std::string &name);
	void addPlugin(const std::string &filename);
	void setWorldFilePath(const std::string &file_path);
	bool configureHook();
	//    bool spawnModel(const std::string& instanceName, const std::string& modelName);
	bool spawnModel(const std::string &instanceName,
					const std::string &modelName, const int timeoutSec);

	bool spawnModelAtPos(const std::string &instanceName,
						 const std::string &modelName, double x, double y, double z);
	bool spawnModelAtPosition(const std::string &instanceName,
							  const std::string &modelName, rstrt::geometry::Translation t);
	bool spawnModelAtPositionAndOrientation(const std::string &instanceName,
											const std::string &modelName, rstrt::geometry::Translation t, rstrt::geometry::Rotation r);
	bool spawnModelAtPositionAndOrientationEuler(const std::string &instanceName,
												 const std::string &modelName, rstrt::geometry::Translation t, const Eigen::VectorXf &rpy);

	/**
	 * Sets the initial configuration for a specific model that was spawned before.
	 *
	 * instanceName chosen model
	 * jointConfig  joint configuration
	 *
	 * returns		success or not
	 */
	bool setInitialConfigurationForModel(const std::string &instanceName,
										 const rstrt::kinematics::JointAngles &jointConfig);

	bool toggleDynamicsSimulation(const bool activate);

	bool joinLinksFixed(const std::string &modelName1, const std::string &linkName1,
						const std::string &modelName2, const std::string &linkName2);
	bool detachLinks(const std::string &modelName1, const std::string &linkName1,
					 const std::string &modelName2, const std::string &linkName2);

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

	/// \brief Internal representation of a fixed joint from https://github.com/pal-robotics/gazebo_ros_link_attacher
	struct fixedJoint
	{
		std::string model1;
		gazebo::physics::ModelPtr m1;
		std::string link1;
		gazebo::physics::LinkPtr l1;
		std::string model2;
		gazebo::physics::ModelPtr m2;
		std::string link2;
		gazebo::physics::LinkPtr l2;
		gazebo::physics::JointPtr joint;
	};
	std::vector<fixedJoint> joints;

	// Useful for threaded updates
	struct ClientConnection
	{
		ClientConnection()
		{
		}
		ClientConnection(RTT::OperationCaller<void(void)> world_update_begin,
						 RTT::OperationCaller<void(void)> world_update_end) : world_begin(world_update_begin), world_end(world_update_end)
		{
		}
		RTT::OperationCaller<void(void)> world_begin;
		RTT::OperationCaller<void(void)> world_end;
		RTT::SendHandle<void(void)> world_begin_handle;
		RTT::SendHandle<void(void)> world_end_handle;
	};

	std::map<std::string, ClientConnection> client_map;

  private:
	bool spawnModelInternal(const std::string &instanceName,
							const std::string &modelName, const int timeoutSec, double x,
							double y, double z, double roll, double pitch, double yaw);
	void handleURDF(TiXmlElement *robotElement,
					gazebo::math::Vector3 initial_xyz,
					gazebo::math::Quaternion initial_q);
	void handleSDF(sdf::ElementPtr modelElement,
				   gazebo::math::Vector3 initial_xyz,
				   gazebo::math::Quaternion initial_q);
	gazebo::math::Vector3 parseVector3(const std::string &str);
	gazebo::math::Pose parsePose(const std::string &str);

	bool getJoint(std::string model1, std::string link1,
				  std::string model2, std::string link2,
				  fixedJoint &joint);

	gazebo::util::LogRecord *recorder;
};

#endif
