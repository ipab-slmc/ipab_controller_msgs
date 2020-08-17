//
// Copyright (c) 2020, University of Oxford
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef IPAB_CONTROLLER_MSGS_CONVERSIONS_H_
#define IPAB_CONTROLLER_MSGS_CONVERSIONS_H_

#include <ipab_controller_msgs/FeedbackPolicy.h>
#include <Eigen/Dense>

namespace ipab_controller_msgs
{
/// Converts an Eigen matrix into a Float64MultiArray message
template <class Derived>
void matrixEigenToMsg(const Eigen::MatrixBase<Derived> &e, std_msgs::Float64MultiArray &m)
{
    if (m.layout.dim.size() != 2) m.layout.dim.resize(2);
    m.layout.dim[0].stride = e.rows() * e.cols();
    m.layout.dim[0].size = e.rows();
    m.layout.dim[1].stride = e.cols();
    m.layout.dim[1].size = e.cols();

    if (static_cast<int>(m.data.size()) != e.size()) m.data.resize(e.size());
    int ii = 0;
    for (int i = 0; i < e.rows(); ++i)
        for (int j = 0; j < e.cols(); ++j)
            m.data[ii++] = e.coeff(i, j);
}

/// Converts a Float64MultiArray message into an Eigen matrix
template <class Derived>
void msgToEigenMatrix(const std_msgs::Float64MultiArray &m, Eigen::MatrixBase<Derived> &e)
{
    if (m.layout.dim.size() != 2)
        throw std::runtime_error("Only 2D matrices supported. Given: " + std::to_string(m.layout.dim.size()));

    if (e.rows() != static_cast<int>(m.layout.dim[0].size) || e.cols() != static_cast<int>(m.layout.dim[1].size)) e.resize(m.layout.dim[0].size, m.layout.dim[1].size);
    int ii = 0;
    for (int i = 0; i < e.rows(); ++i)
        for (int j = 0; j < e.cols(); ++j)
            e(i, j) = m.data[ii++];
}
}  // namespace ipab_controller_msgs

#endif  // IPAB_CONTROLLER_MSGS_CONVERSIONS_H_
