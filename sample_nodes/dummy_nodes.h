#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace DummyNodes
{
BT::NodeStatus SayHello();

BT::NodeStatus CheckBattery();

BT::NodeStatus CheckTemperature();

class GripperInterface
{
  public:
    GripperInterface() : _opened(true)
    {
    }

    BT::NodeStatus open();

    BT::NodeStatus close();

  private:
    bool _opened;
};

//--------------------------------------

// Example of custom SyncActionNode (synchronous action)
// without NodeParameters.
class ApproachObject : public BT::SyncActionNode
{
  public:
    ApproachObject(const std::string& name) : BT::SyncActionNode(name)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
};

// Example of custom SyncActionNode (synchronous action)
// with NodeParameters.
class SaySomething : public BT::SyncActionNode
{
  public:
    SaySomething(const std::string& name, const BT::NodeParameters& params)
      : BT::SyncActionNode(name, params)
    {
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // It is mandatory to define this static method.
    // If you don't, BehaviorTreeFactory::registerNodeType will not compile.
    static const BT::NodeParameters& requiredNodeParameters()
    {
        static BT::NodeParameters params = {{"message", ""}};
        return params;
    }
};

inline void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    static GripperInterface gi;
    factory.registerSimpleAction("SayHello", std::bind(SayHello));
    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
    factory.registerSimpleCondition("CheckTemperature", std::bind(CheckTemperature));
    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gi));
    factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gi));
    factory.registerNodeType<ApproachObject>("ApproachObject");
    factory.registerNodeType<SaySomething>("SaySomething");
}
}

#endif   // SIMPLE_BT_NODES_H
