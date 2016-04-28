#include <rtt_gazebo_embedded/rtt_gazebo_embedded.hh>

using namespace RTT;
using namespace RTT::os;
using namespace std;

#ifndef GAZEBO_GREATER_6
struct g_vectorStringDup
{
  char *operator()(const std::string &_s)
  {
    return strdup(_s.c_str());
  }
};

namespace gazebo{
    bool setupServer(const std::vector<std::string> &_args)
    {
      std::vector<char *> pointers(_args.size());
      std::transform(_args.begin(), _args.end(), pointers.begin(),
                     g_vectorStringDup());
      pointers.push_back(0);
      bool result = gazebo::setupServer(_args.size(), &pointers[0]);

      // Deallocate memory for the command line arguments alloocated with strdup.
      for (size_t i = 0; i < pointers.size(); ++i)
        free(pointers.at(i));

      return result;
    }
}
#endif

RTTGazeboEmbedded::RTTGazeboEmbedded(const std::string& name):
TaskContext(name),
world_path("worlds/empty.world"),
use_rtt_sync(false),
go_sem(0),
gravity_vector(3)
{
    log(Info) << "Creating " << name <<" with gazebo embedded !" << endlog();
    this->addProperty("use_rtt_sync",use_rtt_sync).doc("At world end, Gazebo waits on rtt's updatehook to finish (setPeriod(1) will make gazebo runs at 1Hz)");
    this->addProperty("world_path",world_path).doc("The path to the .world file.");
    this->addOperation("add_plugin",&RTTGazeboEmbedded::addPlugin,this,RTT::OwnThread).doc("The path to a plugin file.");
    this->addOperation("getModelPtr",&RTTGazeboEmbedded::getModelPtr,this,RTT::ClientThread).doc("Get a pointer to a loaded model. Has a timeout param");
    this->addProperty("argv",argv).doc("argv passed to the deployer's main.");
    this->addConstant("gravity_vector",gravity_vector);//.doc("The gravity vector from gazebo, available after configure().");

    gazebo::printVersion();
#ifdef GAZEBO_GREATER_6
    gazebo::common::Console::SetQuiet(false);
#endif
}
gazebo::physics::ModelPtr RTTGazeboEmbedded::getModelPtr(const std::string& model_name,double timeout_s)
{
    pauseSimulation();
    RTT::log(RTT::Info) <<"["<<getName()<<"] Trying to get "<<model_name<<" in less then "<<timeout_s<<"s" << RTT::endlog();
    gazebo::physics::ModelPtr model = nullptr;
    auto tstart = TimeService::Instance()->getTicks();
    while(true)
    {
        auto elapsed = TimeService::Instance()->getSeconds(tstart);
        if(elapsed > timeout_s)
        {
            RTT::log(RTT::Error) << "["<<getName()<<"] Model ["<<model_name<<"] timed out" << RTT::endlog();
            break;
        }

        model = world->GetModel(model_name);

        if(model){
            std::cout << "["<<getName()<<"] Model ["<<model_name<<"] acquired !" << std::endl;
            break;
        }

        usleep(1E6);
        std::cout << "["<<getName()<<"] waiting for model ["<<model_name<<"] to come up" << std::endl;
    }
    unPauseSimulation();
    return std::move(model);
}

void RTTGazeboEmbedded::addPlugin(const std::string& filename)
{
    gazebo::addPlugin(filename);
}
void RTTGazeboEmbedded::setWorldFilePath(const std::string& file_path)
{
    if(std::ifstream(file_path))
        world_path = file_path;
    else
        RTT::log(RTT::Error) << "File "<<file_path<<"does not exists."<<RTT::endlog();
}
bool RTTGazeboEmbedded::configureHook()
{
    RTT::log(RTT::Info) << "Creating world at "<< world_path << RTT::endlog();

    try{
        if(! gazebo::setupServer(argv))
        {
            RTT::log(RTT::Error) << "Could not setupServer " << RTT::endlog();
            return false;
        }
    }catch(...){}

    world = gazebo::loadWorld(world_path);

    gravity_vector[0] = world->GetPhysicsEngine()->GetGravity()[0];
    gravity_vector[1] = world->GetPhysicsEngine()->GetGravity()[1];
    gravity_vector[2] = world->GetPhysicsEngine()->GetGravity()[2];

    if(!world) return false;

    return true;
}


bool RTTGazeboEmbedded::startHook()
{
    if(!run_th.joinable())
        run_th = std::thread(std::bind(&RTTGazeboEmbedded::runWorldForever,this));
    else{
        unPauseSimulation();
    }
    return true;
}
void RTTGazeboEmbedded::runWorldForever()
{
    std::cout <<"\x1B[32m[[--- Gazebo running ---]]\033[0m"<<std::endl;
    gazebo::runWorld(world, 0); // runs forever
    std::cout <<"\x1B[32m[[--- Gazebo exiting runWorld() ---]]\033[0m"<<std::endl;
}
void RTTGazeboEmbedded::updateHook()
{
    if(use_rtt_sync)
    {
        gazebo::event::Events::pause.Signal(false);
        go_sem.signal();
    }
    return;
}
void RTTGazeboEmbedded::pauseSimulation()
{
    std::cout <<"\x1B[32m[[--- Pausing Simulation ---]]\033[0m"<<std::endl;
    gazebo::event::Events::pause.Signal(true);
}
void RTTGazeboEmbedded::unPauseSimulation()
{
    std::cout <<"\x1B[32m[[--- Unpausing Simulation ---]]\033[0m"<<std::endl;
    gazebo::event::Events::pause.Signal(false);
}

void RTTGazeboEmbedded::stopHook()
{
    pauseSimulation();
}

void RTTGazeboEmbedded::checkClientConnections()
{
    if(getPeerList().size() &&
        getPeerList().size() != client_map.size())
    {
        for(auto p : getPeerList())
        {
            if(client_map.find(p) == client_map.end())
            {
                if(getPeer(p)->provides("gazebo") &&
                    getPeer(p)->provides("gazebo")->hasOperation("WorldUpdateBegin") &&
                    getPeer(p)->provides("gazebo")->hasOperation("WorldUpdateEnd")
                )
                {
                    log(Info) << "Adding new client "<<p<<endlog();
                    client_map[p] = ClientConnection(getPeer(p)->provides("gazebo")->getOperation("WorldUpdateBegin"),
                                                    getPeer(p)->provides("gazebo")->getOperation("WorldUpdateEnd"));
                }
            }
        }
    }

    auto it = std::begin(client_map);
    while(it != std::end(client_map))
    {
        if(!it->second.world_end.ready() ||
            !it->second.world_begin.ready())
        {
            log(Warning) << "Removing broken connection with client "<<it->first<<endlog();
            it = client_map.erase(it);
        }else
            ++it;
    }
    return;
}

void RTTGazeboEmbedded::WorldUpdateBegin()
{
    checkClientConnections();

    for(auto c : client_map)
        if(getPeer(c.first)->isConfigured()
            && getPeer(c.first)->isRunning())
            c.second.world_begin_handle = c.second.world_begin.send();

    for(auto c : client_map)
        if(getPeer(c.first)->isConfigured()
            && getPeer(c.first)->isRunning())
            c.second.world_begin_handle.collect();
}

void RTTGazeboEmbedded::WorldUpdateEnd()
{
    checkClientConnections();

    for(auto c : client_map)
        if(getPeer(c.first)->isConfigured()
            && getPeer(c.first)->isRunning())
            c.second.world_end_handle = c.second.world_end.send();

    for(auto c : client_map)
        if(getPeer(c.first)->isConfigured()
            && getPeer(c.first)->isRunning())
            c.second.world_end_handle.collect();

    if(use_rtt_sync)
        go_sem.wait();
}
void RTTGazeboEmbedded::cleanupHook()
{
    std::cout <<"\x1B[32m[[--- Stoping Simulation ---]]\033[0m"<<std::endl;
    gazebo::event::Events::sigInt.Signal();
    std::cout <<"\x1B[32m[[--- Gazebo Shutdown... ---]]\033[0m"<<std::endl;
    //NOTE: This crashes as gazebo is running is a thread
    gazebo::shutdown();
    run_th.join();

    std::cout <<"\x1B[32m[[--- Exiting Gazebo ---]]\033[0m"<<std::endl;
}

ORO_CREATE_COMPONENT(RTTGazeboEmbedded)
