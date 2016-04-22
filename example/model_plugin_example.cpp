#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <gazebo/gazebo.hh>
#include <rtt/scripting/Scripting.hpp>

class ModelPluginExample : public RTT::TaskContext
{
public:
    ModelPluginExample(std::string const& name):
    RTT::TaskContext(name)
    {
        gazebo::printVersion();
        // Connecting Events
        world_begin =  gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPluginExample::WorldUpdateBegin,this));
        world_end   = gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&ModelPluginExample::WorldUpdateEnd,this));
    }

    bool configureHook()
    {
        // Verify that rtt_gazebo_embedded is loaded
        if(!hasPeer("gazebo"))
        {
            RTT::log(RTT::Error) << "Gazebo component is not loaded ! \n"
                "Please run loadComponent(\"gazebo\",\"RttGazeboEmbedded\")\n"
                "And connectPeers(\"gazebo\",\""<<getName()<<"\")"
                ""<< RTT::endlog();
            return false;
        }
        
        // Get the function from gazebo_embedded
        RTT::OperationCaller<gazebo::physics::ModelPtr(const std::string&,double)> get_model 
                = getPeer("gazebo")->getOperation("getModelPtr");
        
        // Verify if function exists
        assert(get_model.ready());
        
        // Get the model 
        model = get_model.call(getName(),20.0);
        
        return bool(model);
    }

    void updateHook()
    {
        RTT::log(RTT::Debug)  << "UpdateHook() should do nothing ! " << RTT::endlog();
    }
    
    void WorldUpdateBegin()
    {
       RTT::log(RTT::Debug)  << getName() 
       <<" Reading cmd from RTT ports => Writing to sim model" 
       << RTT::endlog();
    }
    
    void WorldUpdateEnd()
    {
        RTT::log(RTT::Debug) << getName() 
        <<"Reading sim model => Writing to RTT ports" 
        << RTT::endlog();
    }
protected:
    gazebo::event::ConnectionPtr world_begin;
    gazebo::event::ConnectionPtr world_end;
    
    gazebo::physics::ModelPtr model;
};

ORO_CREATE_COMPONENT(ModelPluginExample)