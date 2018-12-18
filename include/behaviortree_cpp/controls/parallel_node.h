/* Copyright (C) 2015-2018 Michele Colledanchise -  All Rights Reserved
 * Copyright (C) 2018 Davide Faconti -  All Rights Reserved
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

#ifndef PARALLEL_NODE_H
#define PARALLEL_NODE_H

#include "behaviortree_cpp/control_node.h"

namespace BT
{
class ParallelNode : public ControlNode
{
  public:

    ParallelNode(const std::string& name, unsigned threshold);

    ParallelNode(const std::string& name, const NodePorts& ports);

    static const NodePortsSet& nodePortsModel()
    {
        static NodePortsSet ports_set = { {THRESHOLD_KEY},{} };
        return ports_set;
    }

    ~ParallelNode() = default;

    virtual void halt() override;

    unsigned int thresholdM();
    void setThresholdM(unsigned int threshold_M);

  private:
    unsigned int threshold_;
    unsigned int success_childred_num_;
    unsigned int failure_childred_num_;

    bool read_parameter_from_input_port_;
    static constexpr const char* THRESHOLD_KEY = "threshold";

    virtual BT::NodeStatus tick() override;
};
}
#endif   // PARALLEL_NODE_H
