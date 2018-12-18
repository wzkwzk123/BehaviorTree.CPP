#ifndef DECORATOR_SUBTREE_NODE_H
#define DECORATOR_SUBTREE_NODE_H

#include "behaviortree_cpp/decorator_node.h"

namespace BT
{
class DecoratorSubtreeNode : public DecoratorNode
{
  public:
    DecoratorSubtreeNode(const std::string& name, const NodeParameters& params);

    virtual ~DecoratorSubtreeNode() override = default;

    void onInit()
    {
        // store params in local blackboard TODO
    }

    static const NodeParameters& requiredNodeParameters()
    {
        static NodeParameters params;
        return params;
    }

  private:
    virtual BT::NodeStatus tick() override;

    virtual NodeType type() const override final
    {
        return NodeType::SUBTREE;
    }

};


}

#endif   // DECORATOR_SUBTREE_NODE_H
