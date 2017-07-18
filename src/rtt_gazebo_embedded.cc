#include <rtt_gazebo_embedded/rtt_gazebo_embedded.hh>
#include <boost/algorithm/string.hpp>

using namespace RTT;
using namespace RTT::os;
using namespace std;

#ifndef GAZEBO_GREATER_6
struct g_vectorStringDup {
	char *operator()(const std::string &_s) {
		return strdup(_s.c_str());
	}
};

namespace gazebo {
bool setupServer(const std::vector<std::string> &_args) {
	std::vector<char *> pointers(_args.size());
	std::transform(_args.begin(), _args.end(), pointers.begin(),
			g_vectorStringDup());
	pointers.push_back(0);
	bool result = gazebo::setupServer(_args.size(), &pointers[0]);

	// Deallocate memory for the command line arguments allocated with strdup.
	for (size_t i = 0; i < pointers.size(); ++i)
		free(pointers.at(i));

	return result;
}
}
#endif

RTTGazeboEmbedded::RTTGazeboEmbedded(const std::string& name) :
		TaskContext(name), world_path("worlds/empty.world"), use_rtt_sync(
				false), go_sem(0), gravity_vector(3), isWorldConfigured(false), _is_paused(
				true) {

	log(Info) << "Creating " << name << " with gazebo embedded !" << endlog();
	this->addProperty("use_rtt_sync", use_rtt_sync).doc(
			"At world end, Gazebo waits on rtt's updatehook to finish (setPeriod(1) will make gazebo runs at 1Hz)");
	this->addProperty("world_path", world_path).doc(
			"The path to the .world file.");
	this->addOperation("add_plugin", &RTTGazeboEmbedded::addPlugin, this,
			RTT::OwnThread).doc("The path to a plugin file.");
	this->addProperty("argv", argv).doc("argv passed to the deployer's main.");
	this->addConstant("gravity_vector", gravity_vector); //.doc("The gravity vector from gazebo, available after configure().");

	this->addOperation("spawn_model", &RTTGazeboEmbedded::spawnModel, this,
			RTT::OwnThread).doc(
			"The instance name of the model to be spawned and then the model name.");

	this->addOperation("spawn_model_at_pos",
			&RTTGazeboEmbedded::spawnModelAtPos, this, RTT::OwnThread).doc(
			"Spawning an URDF/SRDF model at a specific position in the world.").arg(
			"instanceName", "instance name").arg("modelName",
			"model to spawn (i.e. model://iit-coman)").arg("x",
			"spawning coordinate X").arg("y", "spawning coordinate Y").arg("z",
			"spawning coordinate Z");

	this->addOperation("spawn_model_at_position",
			&RTTGazeboEmbedded::spawnModelAtPosition, this, RTT::OwnThread).doc(
			"Spawning an URDF/SRDF model at a specific position in the world.").arg(
			"instanceName", "instance name").arg("modelName",
			"model to spawn (i.e. model://iit-coman)").arg("t",
			"spawning position");

	this->addOperation("spawn_model_at_position_and_orientation",
			&RTTGazeboEmbedded::spawnModelAtPositionAndOrientation, this, RTT::OwnThread).doc(
			"Spawning an URDF/SRDF model at a specific position and orientation in the world.").arg(
			"instanceName", "instance name").arg("modelName",
			"model to spawn (i.e. model://iit-coman)").arg("t",
			"spawning position").arg("r", "spawning orientation");

	this->addOperation("reset_model_poses", &RTTGazeboEmbedded::resetModelPoses,
			this, RTT::ClientThread).doc("Resets the model poses.");

	this->addOperation("reset_world", &RTTGazeboEmbedded::resetWorld, this,
			RTT::ClientThread).doc("Resets the entire world and time.");

	this->addOperation("toggleDynamicsSimulation",
			&RTTGazeboEmbedded::toggleDynamicsSimulation, this,
			RTT::ClientThread).doc(
			"Activate or Deactivate the physics engine of Gazebo.");

	this->addOperation("setInitialConfigurationForModel",
				&RTTGazeboEmbedded::setInitialConfigurationForModel, this,
				RTT::ClientThread).doc(
				"Set the initial joint configuration for a specific model.")
                                                    .arg("instanceName", "model to be initialized with the joint configuration")
                                                    .arg("jointConfig", "joint configuration inlucding all joints in the order of gazebo.");

	gazebo::printVersion();
#ifdef GAZEBO_GREATER_6
	gazebo::common::Console::SetQuiet(false);
#endif
}

void RTTGazeboEmbedded::addPlugin(const std::string& filename) {
	gazebo::addPlugin(filename);
}

bool RTTGazeboEmbedded::resetModelPoses() {
	if (world) {
		this->world->ResetEntities(gazebo::physics::Base::MODEL);
		return true;
	} else {
		RTT::log(RTT::Warning)
				<< "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
				<< RTT::endlog();
		return false;
	}
}

bool RTTGazeboEmbedded::resetWorld() {
	if (world) {
		this->world->Reset();
		return true;
	} else {
		RTT::log(RTT::Warning)
				<< "The world pointer was not yet retrieved. This needs to be done first, in order to be able to call this operation."
				<< RTT::endlog();
		return false;
	}
}

void RTTGazeboEmbedded::setWorldFilePath(const std::string& file_path) {
	if (std::ifstream(file_path))
		world_path = file_path;
	else
		RTT::log(RTT::Error) << "File " << file_path << "does not exists."
				<< RTT::endlog();
}

bool RTTGazeboEmbedded::toggleDynamicsSimulation(const bool activate) {
	if (!isWorldConfigured) {
		std::cout
				<< "\x1B[33m[[--- You have to configure this component first! ---]]\033[0m"
				<< std::endl;
		return false;
	}
	world->EnablePhysicsEngine(activate);
	return true;
}

bool RTTGazeboEmbedded::configureHook() {
	RTT::log(RTT::Info) << "Creating world at " << world_path << RTT::endlog();

	try {
		if (!gazebo::setupServer(argv)) {
			RTT::log(RTT::Error) << "Could not setupServer " << RTT::endlog();
			return false;
		}
	} catch (...) {
	}

	world = gazebo::loadWorld(world_path);

	gravity_vector[0] = world->GetPhysicsEngine()->GetGravity()[0];
	gravity_vector[1] = world->GetPhysicsEngine()->GetGravity()[1];
	gravity_vector[2] = world->GetPhysicsEngine()->GetGravity()[2];

	isWorldConfigured = true;

	if (!world)
		return false;

	//RTT::log(RTT::Info) << "Binding world events" << RTT::endlog();
	//world_begin =  gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RTTGazeboEmbedded::WorldUpdateBegin,this));
	world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
			std::bind(&RTTGazeboEmbedded::WorldUpdateEnd, this));

	_pause = gazebo::event::Events::ConnectPause(
			boost::bind(&RTTGazeboEmbedded::OnPause, this, _1));

	return true;
}

void RTTGazeboEmbedded::OnPause(const bool _pause) {
	if (_pause) {
		if (this->isRunning()) {
			if (!_is_paused)
				this->stop();
		}
	} else {
		if (!this->isRunning()) {
			if (_is_paused)
				this->start();
		}
	}
}

bool RTTGazeboEmbedded::spawnModel(const std::string& instanceName,
		const std::string& modelName, const int timeoutSec) {
	gazebo::math::Quaternion initial_q(1, 0, 0, 0);
	gazebo::math::Vector3 rpy = initial_q.GetAsEuler();
	return spawnModelInternal(instanceName, modelName, timeoutSec, 0.0, 0.0, 0.0, rpy.x, rpy.y, rpy.z);
}

bool RTTGazeboEmbedded::spawnModelAtPos(const std::string& instanceName,
		const std::string& modelName, double x, double y, double z) {
	gazebo::math::Quaternion initial_q(1, 0, 0, 0);
	gazebo::math::Vector3 rpy = initial_q.GetAsEuler();
	return spawnModelInternal(instanceName, modelName, 10, x, y, z, rpy.x, rpy.y, rpy.z);
}

bool RTTGazeboEmbedded::spawnModelAtPosition(const std::string& instanceName,
		const std::string& modelName, rstrt::geometry::Translation t) {
	gazebo::math::Quaternion initial_q(1, 0, 0, 0);
	gazebo::math::Vector3 rpy = initial_q.GetAsEuler();
	return spawnModelInternal(instanceName, modelName, 10, t.translation(0), t.translation(1), t.translation(2), rpy.x, rpy.y, rpy.z);
}

bool RTTGazeboEmbedded::spawnModelAtPositionAndOrientation(const std::string& instanceName,
		const std::string& modelName, rstrt::geometry::Translation t, rstrt::geometry::Rotation r) {
	gazebo::math::Quaternion initial_q(r.rotation(0), r.rotation(1), r.rotation(2), r.rotation(3));
	gazebo::math::Vector3 rpy = initial_q.GetAsEuler();
	return spawnModelInternal(instanceName, modelName, 10, t.translation(0), t.translation(1), t.translation(2), rpy.x, rpy.y, rpy.z);
}

bool RTTGazeboEmbedded::setInitialConfigurationForModel(
		const std::string& instanceName,
		const rstrt::kinematics::JointAngles& jointConfig) {
	// check if model was spawned
	gazebo::physics::ModelPtr model;
	if (!(model = world->GetModel(instanceName))) {
		RTT::log(RTT::Warning)
				<< "Model could not be found. Perhaps it was not loaded at all before. Try using the spawn_model call."
				<< RTT::endlog();
		return false;
	}
	if (!model) {
		RTT::log(RTT::Warning)
				<< "Model could not be found. Perhaps it was not loaded at all before. Try using the spawn_model call."
				<< RTT::endlog();
		return false;
	}
	// pause gazebo
            bool turnSimulationOnAfterwards = this->isRunning();
	if (turnSimulationOnAfterwards) {
		this->stop();
	}
	// disable physics
	bool turnPhysicsOnAfterwards = world->GetEnablePhysicsEngine();
	if (turnPhysicsOnAfterwards) {
		toggleDynamicsSimulation(false);
	}
	// disable collisions
	std::vector<gazebo::physics::JointPtr> gazebo_joints_ =
					model->GetJoints();

             // TODO make this better without twice of the calculations as it is now...
            // count the joints to make a proper sanety check for the dimensions.
             int count = 0;
             for (gazebo::physics::Joint_V::iterator jit = gazebo_joints_.begin();
                jit != gazebo_joints_.end(); ++jit) {
                          const std::string name = (*jit)->GetName();
                          if ((*jit)->GetLowerLimit(0u) == (*jit)->GetUpperLimit(0u)) {
                                continue;
                          }
                          count++;
             }


	if (count == jointConfig.angles.rows()) {
		int j = -1;
		for (gazebo::physics::Joint_V::iterator jit = gazebo_joints_.begin();
				jit != gazebo_joints_.end(); ++jit, ++j) {
			const std::string name = (*jit)->GetName();


                          if ((*jit)->GetLowerLimit(0u) == (*jit)->GetUpperLimit(0u)) {
                                RTT::log(RTT::Info) << "Not adding (fake) fixed joint ["
                                << name << "] j:" << j << RTT::endlog();
                                continue;
                          }

#ifdef GAZEBO_GREATER_6
                          std::cout << "set " << name << " to " << jointConfig.angles(j) << std::endl;
			(*jit)->SetPosition(0, (double) jointConfig.angles(j));
#else
			(*jit)->SetAngle(0, (double) jointConfig.angles(j));
#endif
		}
		usleep(1000);
	} else {
		RTT::log(RTT::Warning) << "Config size doesn't match: Received: "
				<< jointConfig.angles.rows() << ", but expected: "
				<< count << RTT::endlog();

                for (gazebo::physics::Joint_V::iterator jit = gazebo_joints_.begin();
                jit != gazebo_joints_.end(); ++jit) {
                          const std::string name = (*jit)->GetName();
                          if ((*jit)->GetLowerLimit(0u) == (*jit)->GetUpperLimit(0u)) {
                                continue;
                          }
                          RTT::log(RTT::Warning) << "Set Joint: " << name << RTT::endlog();
                }
	}
	// enable collisions

	// enable physics again if needed
	if (turnPhysicsOnAfterwards) {
		toggleDynamicsSimulation(true);
	}
	// unpause gazebo
            if (turnSimulationOnAfterwards) {
                this->start();
            }
}

bool RTTGazeboEmbedded::spawnModelInternal(const std::string& instanceName,
		const std::string& modelName, const int timeoutSec, double x, double y,
		double z, double roll, double pitch, double yaw) {
	if (!isWorldConfigured) {
		std::cout
				<< "\x1B[33m[[--- You have to configure this component first! ---]]\033[0m"
				<< std::endl;
		return false;
	}

	gazebo::math::Vector3 initial_xyz(x, y, z);
	gazebo::math::Quaternion initial_q(roll, pitch, yaw);

	//
	//	gazebo::common::ModelDatabase* modelDatabaseInst =
	//			gazebo::common::ModelDatabase::Instance();

	//check if file exists
	const string path = gazebo::common::SystemPaths::Instance()->FindFileURI(
			modelName);
	if (path.empty()) {
		std::cout << "\x1B[32m[[--- Model " << modelName
				<< " couldn't be found ---]]\033[0m" << std::endl;
	}

	std::string model_xml;
	std::ifstream ifsURDF((path + "/model.urdf").c_str());
	if (!ifsURDF) {
		std::ifstream ifsSDF((path + "/model.sdf").c_str());
		if (!ifsSDF) {
			std::cout
					<< "\x1B[31m[[--- Can't be parsed: No model.urdf or model.sdf found! ---]]\033[0m"
					<< std::endl;
			return false;
		} else {
			model_xml.assign((std::istreambuf_iterator<char>(ifsSDF)),
					(std::istreambuf_iterator<char>()));
		}
	} else {
		model_xml.assign((std::istreambuf_iterator<char>(ifsURDF)),
				(std::istreambuf_iterator<char>()));
	}

	TiXmlDocument gazebo_model_xml;
	gazebo_model_xml.Parse(model_xml.c_str());

	TiXmlElement* nameElement = gazebo_model_xml.FirstChildElement("robot");
	if (!nameElement) {
		cout << "it's not an urdf check for sdf" << endl;
		nameElement = gazebo_model_xml.FirstChildElement("model");
		if (!nameElement) {
			std::cout
					<< "\x1B[31m[[--- Can't be parsed: No <model> or <robot> tag found! ---]]\033[0m"
					<< std::endl;
			return false;
		} else {
			// handle sdf
			sdf::SDF root;
			root.SetFromString(model_xml);
			sdf::ElementPtr nameElementSDF = root.Root()->GetElement("model");
			nameElementSDF->GetAttribute("name")->SetFromString(instanceName);

			handleSDF(nameElementSDF, initial_xyz, initial_q);
		}
	} else {
		// handle urdf

		if (nameElement->Attribute("name") != NULL) {
			// removing old model name
			nameElement->RemoveAttribute("name");
		}
		// replace with user specified name
		nameElement->SetAttribute("name", instanceName);

		// change initital pos and rot
		handleURDF(nameElement, initial_xyz, initial_q);
	}

	//	world->InsertModelFile(modelName);
	TiXmlPrinter printer;
	printer.SetIndent("    ");
	gazebo_model_xml.Accept(&printer);

	world->InsertModelString(printer.CStr());

	gazebo::common::Time timeout((double) timeoutSec);

	boost::shared_ptr<gazebo::common::Timer> modelDeployTimer(
			new gazebo::common::Timer());

	modelDeployTimer->Start();
	while (modelDeployTimer->GetRunning()) {
		if (modelDeployTimer->GetElapsed() > timeout) {
			gzerr
					<< "SpawnModel: Model pushed to spawn queue, but spawn service timed out waiting for model to appear in simulation under the name "
					<< instanceName << endl;
			modelDeployTimer->Stop();
			return false;
		}

		{
			//boost::recursive_mutex::scoped_lock lock(*world->GetMRMutex());
			if (world->GetModel(instanceName)) {
				modelDeployTimer->Stop();
				break;
			}
		}
		usleep(2000);
	}

	return true;

}

void RTTGazeboEmbedded::handleSDF(sdf::ElementPtr modelElement,
		gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q) {
	sdf::ElementPtr pose_element;

	// Check for the pose element
	pose_element = modelElement->GetElement("pose");
	gazebo::math::Pose model_pose;

	// Create the pose element if it doesn't exist
	// Remove it if it exists, since we are inserting a new one
	if (pose_element) {
		// save pose_element in math::Pose and remove child
		model_pose = this->parsePose(pose_element->Get<std::string>());
		modelElement->RemoveChild(pose_element);
	}

	// add pose_element Pose to initial pose
	gazebo::math::Pose new_model_pose = model_pose
			+ gazebo::math::Pose(initial_xyz, initial_q);

	// Create the string of 6 numbers
	std::ostringstream pose_stream;
	gazebo::math::Vector3 model_rpy = model_pose.rot.GetAsEuler(); // convert to Euler angles for Gazebo XML
	pose_stream << model_pose.pos.x << " " << model_pose.pos.y << " "
			<< model_pose.pos.z << " " << model_rpy.x << " " << model_rpy.y
			<< " " << model_rpy.z;

	// Add value to pose element
//	TiXmlText* text = new TiXmlText(pose_stream.str());
	sdf::ElementPtr new_pose_element = modelElement->AddElement("pose");
//	sdf::ElementPtr new_pose_element = new TiXmlElement("pose");
	new_pose_element->Set < std::string > (pose_stream.str());
//	new_pose_element->LinkEndChild(text);
//	modelElement->LinkEndChild(new_pose_element);
//	modelElement->InsertElement(new_pose_element);
}

gazebo::math::Pose RTTGazeboEmbedded::parsePose(const string &str) {
	std::vector<std::string> pieces;
	std::vector<double> vals;

	boost::split(pieces, str, boost::is_any_of(" "));
	for (unsigned int i = 0; i < pieces.size(); ++i) {
		if (pieces[i] != "") {
			try {
				vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
			} catch (boost::bad_lexical_cast &e) {
				log(Error) << "xml key [" << str << "][" << i << "] value ["
						<< pieces[i] << "] is not a valid double from a 3-tuple"
						<< endlog();
				return gazebo::math::Pose();
			}
		}
	}

	if (vals.size() == 6)
		return gazebo::math::Pose(vals[0], vals[1], vals[2], vals[3], vals[4],
				vals[5]);
	else {
		log(Error) << "Beware: failed to parse string " << str.c_str()
				<< " as gazebo::math::Pose, returning zeros." << endlog();
		return gazebo::math::Pose();
	}
}

void RTTGazeboEmbedded::handleURDF(TiXmlElement* robotElement,
		gazebo::math::Vector3 initial_xyz, gazebo::math::Quaternion initial_q) {
	// find first instance of xyz and rpy, replace with initial pose
	TiXmlElement* origin_key = robotElement->FirstChildElement("origin");

	// if there is no origin tag then create one
	if (!origin_key) {
		origin_key = new TiXmlElement("origin");
		robotElement->LinkEndChild(origin_key);
	}

	gazebo::math::Vector3 xyz;
	gazebo::math::Vector3 rpy;
	if (origin_key->Attribute("xyz")) {
		xyz = this->parseVector3(origin_key->Attribute("xyz"));
		origin_key->RemoveAttribute("xyz");
	}
	if (origin_key->Attribute("rpy")) {
		rpy = this->parseVector3(origin_key->Attribute("rpy"));
		origin_key->RemoveAttribute("rpy");
	}

	// add xyz, rpy to initial pose
	gazebo::math::Pose model_pose = gazebo::math::Pose(xyz, rpy)
			+ gazebo::math::Pose(initial_xyz, initial_q);

	std::ostringstream xyz_stream;
	xyz_stream << model_pose.pos.x << " " << model_pose.pos.y << " "
			<< model_pose.pos.z;

	std::ostringstream rpy_stream;
	gazebo::math::Vector3 model_rpy = model_pose.rot.GetAsEuler(); // convert to Euler angles for URDF/SDF XML
	rpy_stream << model_rpy.x << " " << model_rpy.y << " " << model_rpy.z;

	origin_key->SetAttribute("xyz", xyz_stream.str());
	origin_key->SetAttribute("rpy", rpy_stream.str());
}

gazebo::math::Vector3 RTTGazeboEmbedded::parseVector3(const string &str) {
	std::vector<std::string> pieces;
	std::vector<double> vals;

	boost::split(pieces, str, boost::is_any_of(" "));
	for (unsigned int i = 0; i < pieces.size(); ++i) {
		if (pieces[i] != "") {
			try {
				vals.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
			} catch (boost::bad_lexical_cast &e) {
				log(Error) << "xml key [" << str << "][" << i << "] value ["
						<< pieces[i] << "] is not a valid double from a 3-tuple"
						<< endlog();
				return gazebo::math::Vector3();
			}
		}
	}

	if (vals.size() == 3)
		return gazebo::math::Vector3(vals[0], vals[1], vals[2]);
	else {
		log(Warning) << "Beware: failed to parse string " << str.c_str()
				<< " as gazebo::math::Vector3, returning zeros." << endlog();
		return gazebo::math::Vector3();
	}
}

bool RTTGazeboEmbedded::startHook() {
	if (!run_th.joinable()) {
		run_th = std::thread(
				std::bind(&RTTGazeboEmbedded::runWorldForever, this));
		gazebo::sensors::run_threads();
	} else {
		_is_paused = false;
		unPauseSimulation();
	}
	return true;
}
void RTTGazeboEmbedded::runWorldForever() {
	std::cout << "\x1B[32m[[--- Gazebo running ---]]\033[0m" << std::endl;
	gazebo::runWorld(world, 0); // runs forever
	std::cout << "\x1B[32m[[--- Gazebo exiting runWorld() ---]]\033[0m"
			<< std::endl;
}
void RTTGazeboEmbedded::updateHook() {
	if (use_rtt_sync)
		go_sem.signal();
	return;
}
void RTTGazeboEmbedded::pauseSimulation() {
	std::cout << "\x1B[32m[[--- Pausing Simulation ---]]\033[0m" << std::endl;
//	world->SetPaused(true);
	gazebo::event::Events::pause.Signal(true);
}
void RTTGazeboEmbedded::unPauseSimulation() {
	std::cout << "\x1B[32m[[--- Unpausing Simulation ---]]\033[0m" << std::endl;
//	world->SetPaused(false);
	gazebo::event::Events::pause.Signal(false);
}

void RTTGazeboEmbedded::stopHook() {
	if (!use_rtt_sync) {
		_is_paused = true;
		pauseSimulation();
	}
}

void RTTGazeboEmbedded::checkClientConnections() {
	if (getPeerList().size() && getPeerList().size() != client_map.size()) {
		for (auto p : getPeerList()) {
			if (client_map.find(p) == client_map.end()) {
				if (getPeer(p)->provides("gazebo")
						&& getPeer(p)->provides("gazebo")->hasOperation(
								"WorldUpdateBegin")
						&& getPeer(p)->provides("gazebo")->hasOperation(
								"WorldUpdateEnd")) {
					log(Info) << "Adding new client " << p << endlog();
					client_map[p] = ClientConnection(
							getPeer(p)->provides("gazebo")->getOperation(
									"WorldUpdateBegin"),
							getPeer(p)->provides("gazebo")->getOperation(
									"WorldUpdateEnd"));
				}
			}
		}
	}

	auto it = std::begin(client_map);
	while (it != std::end(client_map)) {
		if (!it->second.world_end.ready() || !it->second.world_begin.ready()) {
			log(Warning) << "Removing broken connection with client "
					<< it->first << endlog();
			it = client_map.erase(it);
		} else
			++it;
	}
	return;
}

void RTTGazeboEmbedded::WorldUpdateBegin() {
	checkClientConnections();

	for (auto c : client_map)
		if (getPeer(c.first)->isConfigured() && getPeer(c.first)->isRunning())
			c.second.world_begin_handle = c.second.world_begin.send();

	for (auto c : client_map)
		if (getPeer(c.first)->isConfigured() && getPeer(c.first)->isRunning())
			c.second.world_begin_handle.collect();
}

void RTTGazeboEmbedded::WorldUpdateEnd() {
// checkClientConnections();
//
// for(auto c : client_map)
//     if(getPeer(c.first)->isConfigured()
//         && getPeer(c.first)->isRunning())
//         c.second.world_end_handle = c.second.world_end.send();
//
// for(auto c : client_map)
//     if(getPeer(c.first)->isConfigured()
//         && getPeer(c.first)->isRunning())
//         c.second.world_end_handle.collect();
	gazebo::sensors::run_once();
	if (use_rtt_sync)
		go_sem.wait();
}
void RTTGazeboEmbedded::cleanupHook() {
	if (isWorldConfigured) {
		if (world)
			world->Fini();
	}
	isWorldConfigured = false;
	std::cout << "\x1B[32m[[--- Stoping Simulation ---]]\033[0m" << std::endl;
	gazebo::event::Events::sigInt.Signal();

}
RTTGazeboEmbedded::~RTTGazeboEmbedded() {
	std::cout << "\x1B[32m[[--- Gazebo Shutdown... ---]]\033[0m" << std::endl;

	gazebo::shutdown();
	if (run_th.joinable())
		run_th.join();

	std::cout << "\x1B[32m[[--- Exiting Gazebo ---]]\033[0m" << std::endl;
}

ORO_CREATE_COMPONENT(RTTGazeboEmbedded)
