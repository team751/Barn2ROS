/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Otniel Rinaldo, Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_LINEOBSTACLE_H
#define EDGE_LINEOBSTACLE_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/teb_config.h>

#include "g2o/core/base_unary_edge.h"

#include <Eigen/Core>

#include <chrono>


namespace teb_local_planner
{
  
/**
 * @class EdgeLineObstacle
 * @brief Edge defining the cost function for keeping a minimum distance from obstacles (line shape).
 * 
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2polygon ) \cdot weight \f$. \n
 * \e dist2polygon denotes the minimum distance to the polygon obstacle. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow()
 * @see TebOptimalPlanner::AddEdgesObstacles
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 */   
class EdgeLineObstacle : public g2o::BaseUnaryEdge<1, const LineObstacle*, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */  
  EdgeLineObstacle() 
  {
    _vertices[0] = NULL;
  }
  
  /**
   * @brief Destruct edge.
   * 
   * We need to erase vertices manually, since we want to keep them even if TebOptimalPlanner::clearGraph() is called.
   * This is necessary since the vertices are managed by the Timed_Elastic_Band class.
   */    
  virtual ~EdgeLineObstacle() 
  {
    if(_vertices[0]) 
      _vertices[0]->edges().erase(this);
  }

  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeLineObstacle()");
    
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    double distance_to_line = fabs(_measurement->getMinimumDistance(bandpt->position()));

    _error[0] = penaltyBoundFromBelow(distance_to_line, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon, cfg_->optim.penalty_scale);

    ROS_ASSERT_MSG(!std::isnan(_error[0]) && !std::isinf(_error[0]), "EdgeLineObstacle::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);	  
  }
  
  /**
   * @brief Compute and return error / cost value.
   * 
   * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
   * @return 1D Cost / error vector
   */ 
  ErrorVector& getError()
  {
    computeError();
    return _error;
  }
  
  /**
   * @brief Read values from input stream
   */  
  virtual bool read(std::istream& is)
  {
//  is >> _measurement[0] >> _measurement[1];
    return true;
  }

  /**
   * @brief Write values to an output stream
   */    
  virtual bool write(std::ostream& os) const
  {
//  os << information()(0,0) << " Error: " << _error[0] << ", Measurement X: " << _measurement[0] << ", Measurement Y: " << _measurement[1];
    return os.good();
  }
  
  /**
   * @brief Set LineObstacle for the underlying cost function
   * @param obstacle Const pointer to the LineObstacle
   */     
  void setObstacle(const LineObstacle* obstacle) //Finished
  {
    _measurement = obstacle;
  }
  
  /**
   * @brief Assign the TebConfig class for parameters.
   * @param cfg TebConfig class
   */     
  void setTebConfig(const TebConfig& cfg) //Finished
  {
    cfg_ = &cfg;
  }

protected:
  const TebConfig* cfg_; //!< Store TebConfig class for parameters
  
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
  
  

  
  
  
  
}//end namespace teb_local_planner

#endif