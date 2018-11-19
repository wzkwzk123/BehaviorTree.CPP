/* Copyright (C) 2018 Davide Faconti -  All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "behaviortree_cpp/controls/priority_fallback.h"

namespace BT
{

constexpr const char* PriorityFallbackNode::CHILDREN_PRIORITY;

PriorityFallbackNode::PriorityFallbackNode(const std::string &name,
                                           const NodeParameters &params)
  :  ControlNode(name, params)
  , refresh_parameter_(false)
{
    if( !getParam(CHILDREN_PRIORITY, children_order_) )
    {
        throw std::runtime_error("Missing parameter [children_priority]"
                                 " in PrioritySequenceNode");
    }
    refresh_parameter_ = isBlackboardPattern( params.at(CHILDREN_PRIORITY) );
}

NodeStatus PriorityFallbackNode::tick()
{
    const unsigned children_count = children_nodes_.size();

    if( refresh_parameter_ )
    {
        if( !getParam(CHILDREN_PRIORITY, children_order_) )
        {
            throw std::runtime_error("Problem parsing vector [children_priority] in PrioritySequenceNode");
        }
    }
    if( children_order_.size() != children_count)
    {
        throw std::runtime_error("Size mismatch in [children_priority], at PrioritySequenceNode");
    }

    setStatus(NodeStatus::RUNNING);

    for (unsigned i = 0; i < children_count; i++)
    {
        int index = children_order_[i];
        if( index < 0)
        {
            continue;
        }
        TreeNode* child_node = children_nodes_[index];
        const NodeStatus child_status = child_node->executeTick();

        switch (child_status)
        {
            case NodeStatus::RUNNING:
            {
                return child_status;
            }
            case NodeStatus::SUCCESS:
            {
                for (unsigned t = 0; t <= i; t++)
                {
                    int idx = children_order_[t];
                    children_nodes_[ idx ]->setStatus(NodeStatus::IDLE);
                }
                for (unsigned t = i+1; t < children_count; t++)
                {
                    int idx = children_order_[t];
                    if (children_nodes_[idx]->status() == NodeStatus::RUNNING)
                    {
                        children_nodes_[idx]->halt();
                    }
                }
                return child_status;
            }
            case NodeStatus::FAILURE:
            {
                // continue;
            }
            break;

            case NodeStatus::IDLE:
            {
                throw std::runtime_error("This is not supposed to happen");
            }
        }   // end switch
    }       // end for loop

    for (auto& ch : children_nodes_)
    {
        ch->setStatus(NodeStatus::IDLE);
    }
    return NodeStatus::FAILURE;
}

}
