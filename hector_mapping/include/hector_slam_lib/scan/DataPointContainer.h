//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef __DataPointContainer_h_
#define __DataPointContainer_h_

#include <vector>

namespace hectorslam {

/**
 * 算法中统一使用DataPointContainer表示激光数据
 * @tparam DataPointType 激光点坐标数据类型
 */

template<typename DataPointType>
class DataPointContainer
{
public:

  DataPointContainer(int size = 1000)
  {
    dataPoints.reserve(size);
  }

  /**
   * 容器中数据坐标的缩放
   * @param other  原始数据
   * @param factor  缩放系数
   */
  void setFrom(const DataPointContainer& other, float factor)
  {
    origo = other.getOrigo()*factor;

    dataPoints = other.dataPoints;

    unsigned int size = dataPoints.size();

    for (unsigned int i = 0; i < size; ++i){
      dataPoints[i] *= factor;
    }

  }

  /** 容器中添加激光点 **/
  void add(const DataPointType& dataPoint)
  {
    dataPoints.push_back(dataPoint);
  }

  void clear()
  {
    dataPoints.clear();
  }

  int getSize() const
  {
    return dataPoints.size();
  }

  const DataPointType& getVecEntry(int index) const
  {
    return dataPoints[index];
  }

  /** 获取激光系原点坐标 --- laser的中心 **/
  DataPointType getOrigo() const
  {
    return origo;
  }

  void setOrigo(const DataPointType& origoIn)
  {
    origo = origoIn;
  }

protected:

  std::vector<DataPointType> dataPoints;// 激光点坐标
  DataPointType origo; // 这是激光body系原点startPoint
};

typedef DataPointContainer<Eigen::Vector2f> DataContainer; //激光点默认使用 Eigen::Vector2f 数据类型

}

#endif
