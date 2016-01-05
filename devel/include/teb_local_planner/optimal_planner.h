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
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef OPTIMAL_PLANNER_H_
#define OPTIMAL_PLANNER_H_

#include <math.h>


// teb stuff
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/misc.h>
#include <teb_local_planner/timed_elastic_band.h>
#include <teb_local_planner/planner_interface.h>
#include <teb_local_planner/visualization.h>

// g2o lib stuff
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/g2o_types/edge_acceleration.h>
#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/g2o_types/edge_time_optimal.h>
#include <teb_local_planner/g2o_types/edge_point_obstacle.h>
#include <teb_local_planner/g2o_types/edge_line_obstacle.h>
#include <teb_local_planner/g2o_types/edge_polygon_obstacle.h>
#include <teb_local_planner/g2o_types/edge_dynamic_obstacle.h>

// messages
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <limits.h>

namespace teb_local_planner
{

//! Typedef for the block solver utilized for optimization
typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  TEBBlockSolver;

//! Typedef for the linear solver utilized for optimization
typedef g2o::LinearSolverCSparse<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;
//typedef g2o::LinearSolverCholmod<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;

/**
 * @class TebOptimalPlanner
 * @brief This class optimizes an internal Timed Elastic Band trajectory using the g2o-framework.
 * 
 * For an introduction and further details about the TEB optimization problem refer to:
 * 	- C. Rösmann et al.: Trajectory modification considering dynamic constraints of autonomous robots, ROBOTIK, 2012.
 * 	- C. Rösmann et al.: Efficient trajectory optimization using a sparse model, ECMR, 2013.
 * 	- R. Kümmerle et al.: G2o: A general framework for graph optimization, ICRA, 2011. 
 * 
 * @todo: Call buildGraph() only if the teb structure has been modified to speed up hot-starting from previous solutions.
 */
class TebOptimalPlanner : public PlannerInterface
{
public:
  
  /**
   * @brief Abbrev. for a vector in which each element corresponds to the cost value of a specific cost function (edge type).
   * @see computeCurrentCost()
   * @see optimizeTEB()
   */
  typedef Eigen::Matrix<double, 10, 1> TebCostVec;
  
  /**
   * @brief Default constructor
   */
  TebOptimalPlanner();
  
  /**
   * @brief Construct and initialize the TEB optimal planner.
   * @param cfg Const reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param visual Shared pointer to the TebVisualization class (optional)
   */
  TebOptimalPlanner(const TebConfig& cfg, ObstContainer* obstacles = NULL, TebVisualizationPtr visual = TebVisualizationPtr());
  
  /**
   * @brief Destruct the optimal planner.
   */
  virtual ~TebOptimalPlanner();
  
  /**
    * @brief Initializes the optimal planner
    * @param cfg Const reference to the TebConfig class for internal parameters
    * @param obstacles Container storing all relevant obstacles (see Obstacle)
    * @param visual Shared pointer to the TebVisualization class (optional)
    */
  void initialize(const TebConfig& cfg, ObstContainer* obstacles = NULL, TebVisualizationPtr visual = TebVisualizationPtr());
  
  
  /** @name Plan a trajectory  */
  //@{
  
  /**
   * @brief Plan a trajectory based on an initial reference plan.
   * 
   * Call this method to create and optimize a trajectory that is initialized
   * according to an initial reference plan (given as a container of poses). \n
   * The method supports hot-starting from previous solutions, if avaiable: \n
   * 	- If no trajectory exist yet, a new trajectory is initialized based on the initial plan,
   *	  see TimedElasticBand::initTEBtoGoal
   * 	- If a previous solution is avaiable, update the trajectory based on the initial plan,
   * 	  see bool TimedElasticBand::updateAndPruneTEB
   * 	- Afterwards optimize the recently initialized or updated trajectory by calling optimizeTEB() and invoking g2o
   * @param initial_plan vector of geometry_msgs::PoseStamped
   * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x and angular.z are used)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  virtual bool plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel = NULL, bool free_goal_vel=false);
  
  /**
   * @brief Plan a trajectory between a given start and goal pose (tf::Pose version)
   * 
   * Call this method to create and optimize a trajectory that is initialized between a given start and goal pose. \n
   * The method supports hot-starting from previous solutions, if avaiable: \n
   * 	- If no trajectory exist yet, a new trajectory is initialized between start and goal poses,
   *	  see TimedElasticBand::initTEBtoGoal
   * 	- If a previous solution is avaiable, update the trajectory @see bool TimedElasticBand::updateAndPruneTEB
   * 	- Afterwards optimize the recently initialized or updated trajectory by calling optimizeTEB() and invoking g2o
   * @param start tf::Pose containing the start pose of the trajectory
   * @param goal tf::Pose containing the goal pose of the trajectory
   * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x and angular.z are used)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  virtual bool plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel = NULL, bool free_goal_vel=false);
  
  /**
   * @brief Plan a trajectory between a given start and goal pose
   * 
   * Call this method to create and optimize a trajectory that is initialized between a given start and goal pose. \n
   * The method supports hot-starting from previous solutions, if avaiable: \n
   * 	- If no trajectory exist yet, a new trajectory is initialized between start and goal poses
   *	  @see TimedElasticBand::initTEBtoGoal
   * 	- If a previous solution is avaiable, update the trajectory @see bool TimedElasticBand::updateAndPruneTEB
   * 	- Afterwards optimize the recently initialized or updated trajectory by calling optimizeTEB() and invoking g2o
   * @param start PoseSE2 containing the start pose of the trajectory
   * @param goal PoseSE2 containing the goal pose of the trajectory
   * @param start_vel Initial velocity at the start pose (2D vector containing the translational and angular velocity).
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  virtual bool plan(const PoseSE2& start, const PoseSE2& goal, const Eigen::Vector2d& start_vel, bool free_goal_vel=false);
  
  
  /**
   * @brief Get the velocity command from a previously optimized plan to control the robot at the current sampling interval.
   * @warning Call plan() first and check if the generated plan is feasible.
   * @return 2D vector containing the translational and angular velocity 
   */
  virtual Eigen::Vector2d getVelocityCommand() const;
  
  
  /**
   * @brief Optimize a previously initialized trajectory (actual TEB optimization loop).
   * 
   * optimizeTEB implements the main optimization loop. \n
   * It consist of two nested loops:
   * 	- The outer loop resizes the trajectory according to the temporal resolution by invoking TimedElasticBand::autoResize().
   * 	  Afterwards the internal method optimizeGraph() is called that constitutes the innerloop.
   * 	- The inner loop calls the solver (g2o framework, resp. sparse Levenberg-Marquardt) and iterates a specified
   * 	  number of optimization calls (\c iterations_innerloop).
   * 
   * The outer loop is repeated \c iterations_outerloop times. \n
   * The ratio of inner and outer loop iterations significantly defines the contraction behavior 
   * and convergence rate of the trajectory optimization. Based on our experiences, 2-6 innerloop iterations are sufficient. \n
   * The number of outer loop iterations should be determined by considering the maximum CPU time required to match the control rate. \n
   * Optionally, the cost vector can be calculated by specifying \c compute_cost_afterwards, see computeCurrentCost().
   * @remarks This method is usually called from a plan() method
   * @param iterations_innerloop Number of iterations for the actual solver loop
   * @param iterations_outerloop Specifies how often the trajectory should be resized followed by the inner solver loop.
   * @param compute_cost_afterwards if \c true Calculate the cost vector according to computeCurrentCost(),
   * 				    the vector can be accessed afterwards using getCurrentCost().
   * @return \c true if the optimization terminates successfully, \c false otherwise
   */	  
  bool optimizeTEB(unsigned int iterations_innerloop, unsigned int iterations_outerloop, bool compute_cost_afterwards = false);
  
  //@}
  
  
  /** @name Desired initial and final velocity */
  //@{
  
  /**
   * @brief Set the initial velocity at the trajectory's start pose (e.g. the robot's velocity).
   * @remarks Calling this function is not neccessary if the initial velocity is passed via the plan() method
   * @param vel_start 2D vector containing the translational and angular velocity 
   */
  void setVelocityStart(const Eigen::Ref<const Eigen::Vector2d>& vel_start);
  
  /**
   * @brief Set the initial velocity at the trajectory's start pose (e.g. the robot's velocity) [twist overload].
   * @remarks Calling this function is not neccessary if the initial velocity is passed via the plan() method
   * @param vel_start Current start velocity (e.g. the velocity of the robot, only linear.x and angular.z are used)
   */
  void setVelocityStart(const geometry_msgs::Twist& vel_start);
  
  /**
   * @brief Set the desired final velocity at the trajectory's goal pose.
   * @remarks Call this function only if a non-zero velocity is desired and if \c free_goal_vel is set to \c false in plan()
   * @param vel_goal 2D vector containing the translational and angular final velocity 
   */
  void setVelocityGoal(const Eigen::Ref<const Eigen::Vector2d>& vel_goal);
  
  /**
   * @brief Set the desired final velocity at the trajectory's goal pose to be the maximum velocity limit
   * @remarks Calling this function is not neccessary if \c free_goal_vel is set to \c false in plan()
   */
  void setVelocityGoalFree() {vel_goal_.first = false;}
  
  //@}
  
  
  /** @name Take obstacles into account */
  //@{
  
  
  /**
   * @brief Assign a new set of obstacles
   * @remarks This method overrids the obstacle container optinally assigned in the constructor.
   */
  void setObstVector(ObstContainer* obst_vector) {obstacles_ = obst_vector;}
  
  /**
   * @brief Access the internal obstacle container.
   * @return Const reference to the obstacle container
   */
  const ObstContainer& getObstVector() const {return *obstacles_;};

  //@}
	  
  
  /** @name Visualization */
  //@{
  
  /**
   * @brief Register a TebVisualization class to enable visiualization routines (e.g. publish the local plan and pose sequence)
   * @param visualization shared pointer to a TebVisualization instance
   * @see visualize
   */
  void setVisualization(TebVisualizationPtr visualization);
  
  /**
   * @brief Publish the local plan and pose sequence via ros topics (e.g. subscribe with rviz).
   * 
   * Make sure to register a TebVisualization instance before using setVisualization() or an overlaoded constructor.
   * @see setVisualization
   */
  virtual void visualize();
  
  //@}
  
  
  /** @name Utility methods and more */
  //@{
        
  /**
   * @brief Reset the planner by clearing the internal graph and trajectory.
   */
  virtual void clearPlanner() 
  {
    clearGraph();
    teb_.clearTimedElasticBand();
  }
  
  /**
   * @brief Register the vertices and edges defined for the TEB to the g2o::Factory.
   * 
   * This allows the user to export the internal graph to a text file for instance.
   * Access the optimizer() for more details.
   */
  static void registerG2OTypes();
  
  /**
   * @brief Access the internal TimedElasticBand trajectory.
   * @warning In general, the underlying teb must not be modified directly. Use with care...
   * @return reference to the teb
   */
  TimedElasticBand& teb() {return teb_;};
  
  /**
   * @brief Access the internal TimedElasticBand trajectory (read-only).
   * @return const reference to the teb
   */
  const TimedElasticBand& teb() const {return teb_;};
  
  /**
   * @brief Access the internal g2o optimizer.
   * @warning In general, the underlying optimizer must not be modified directly. Use with care...
   * @return const shared pointer to the g2o sparse optimizer
   */
  boost::shared_ptr<g2o::SparseOptimizer> optimizer() {return optimizer_;};
  
  /**
   * @brief Access the internal g2o optimizer (read-only).
   * @return const shared pointer to the g2o sparse optimizer
   */
  boost::shared_ptr<const g2o::SparseOptimizer> optimizer() const {return optimizer_;};
  
  /**
   * @brief Check if last optimization was successful
   * @return \c true if the last optimization returned without errors, 
   *         otherwise \c false (also if no optimization has been called before).
   */
  bool isOptimized() const {return optimized_;};
	
  /**
   * @brief Compute the cost vector of a given optimization problen (hyper-graph must exist).
   * 
   * Use this method to obtain information about the current edge errors / costs (local cost functions). \n
   * The vector of cost values is composed according to the different edge types (time_optimal, obstacles, ...). \n
   * Refer to the method declaration for the detailed composition. \n
   * The cost for the edges that minimize time differences (EdgeTimeOptimal) corresponds to the sum of all single
   * squared time differneces: \f$ \sum_i \Delta T_i^2 \f$. Sometimes, the user may want to get a value that is proportional
   * or identical to the actual trajectory transition time \f$ \sum_i \Delta T_i \f$. \n
   * Set \c alternative_time_cost to true in order to get the cost calculated using the latter equation, but check the 
   * implemented definition, if the value is scaled to match the magnitude of other cost values.
   * 
   * @todo Remove the scaling term for the alternative time cost.
   * 
   * @see getCurrentCost
   * @see optimizeTEB
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time.
   * @return TebCostVec containing the cost values
   */
  void computeCurrentCost(bool alternative_time_cost=false);
  
  /**
   * @brief Access the cost vector.
   *
   * The cost vector must previously calculated using computeCurrentCost 
   * or by calling optimizeTEB with enabled cost flag.
   * @return const reference to the TebCostVec.
   */
  const TebCostVec& getCurrentCost() const {return cost_;}
  
  /**
   * @brief Check whether the planned trajectory is feasible or not.
   * 
   * This method currently checks only that the trajectory, or a part of the trajectory is collision free.
   * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
   * @param costmap_model Pointer to the costmap model
   * @param footprint The specification of the footprint of the robot in world coordinates
   * @param inscribed_radius The radius of the inscribed circle of the robot
   * @param circumscribed_radius The radius of the circumscribed circle of the robot
   * @param look_ahead_idx Number of poses along the trajectory that should be verified, if -1, the complete trajectory will be checked.
   * @return \c true, if the robot footprint along the first part of the trajectory intersects with 
   * 		      any obstacle in the costmap, \c false otherwise.
   */
  virtual bool isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec, double inscribed_radius = 0.0,
				    double circumscribed_radius=0.0, int look_ahead_idx=-1);
  
  
  //@}
  
protected:
  
  /** @name Hyper-Graph creation and optimization */
  //@{
  
  /**
   * @brief Build the hyper-graph representing the TEB optimization problem.
   * 
   * This method creates the optimization problem according to the hyper-graph formulation. \n
   * For more details refer to the literature cited in the TebOptimalPlanner class description.
   * @see optimizeGraph
   * @see clearGraph
   * @return \c true, if the graph was created successfully, \c false otherwise.
   */
  bool buildGraph();
  
  /**
   * @brief Optimize the previously constructed hyper-graph to deform / optimize the TEB.
   * 
   * This method invokes the g2o framework to solve the optimization problem considering dedicated sparsity patterns. \n
   * The current implementation calls a non-constrained sparse Levenberg-Marquardt algorithm. Constraints are considered
   * by utilizing penalty approximations. Refer to the literature cited in the TebOptimalPlanner class description.
   * @see buildGraph
   * @see clearGraph
   * @param no_iterations Number of solver iterations
   * @param clear_after Clear the graph after optimization.
   * @return \c true, if optimization terminates successfully, \c false otherwise.
   */
  bool optimizeGraph(int no_iterations,bool clear_after=true);
  
  /**
   * @brief Clear an existing internal hyper-graph.
   * @see buildGraph
   * @see optimizeGraph
   */
  void clearGraph();
  
  /**
   * @brief Add all relevant vertices to the hyper-graph as optimizable variables.
   * 
   * Vertices (if unfixed) represent the variables that will be optimized. \n
   * In case of the Timed-Elastic-Band poses and time differences form the vertices of the hyper-graph. \n
   * The order of insertion of vertices (to the graph) is important for efficiency,
   * since it affect the sparsity pattern of the underlying hessian computed for optimization.
   * @see VertexPose
   * @see VertexTimeDiff
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddTEBVertices();
  
  /**
   * @brief Add all edges (local cost functions) for limiting the translational and angular velocity.
   * @see EdgeVelocity
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesVelocity();
  
  /**
   * @brief Add all edges (local cost functions) for limiting the translational and angular acceleration.
   * @see EdgeAcceleration
   * @see EdgeAccelerationStart
   * @see EdgeAccelerationGoal
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesAcceleration();
  
  /**
   * @brief Add all edges (local cost functions) for minimizing the transition time (resp. minimize time differences)
   * @see EdgeTimeOptimal
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesTimeOptimal();
  
  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from static obstacles
   * @see EdgePointObstacle
   * @see EdgePolygonObstacle
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesObstacles();
  
  /**
   * @brief Add all edges (local cost functions) related to keeping a distance from dynamic (moving) obstacles.
   * @warning experimental 
   * @see EdgeDynamicObstacle
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesDynamicObstacles();  

  /**
   * @brief Add all edges (local cost functions) for satisfying kinematic constraints
   * @see EdgeKinematics
   * @see buildGraph
   * @see optimizeGraph
   */
  void AddEdgesKinematics();
  
  //@}
  
  
  /**
   * @brief Initialize and configure the g2o sparse optimizer.
   * @return shared pointer to the g2o::SparseOptimizer instance
   */
  boost::shared_ptr<g2o::SparseOptimizer> initOptimizer();
    

  // external objects (store weak pointers)
  ObstContainer* obstacles_; //!< Store obstacles that are relevant for planning
  const TebConfig* cfg_; //!< Config class that stores and manages all related parameters
  
  TebCostVec cost_; //!< Store composed cost vector of the current hyper-graph
  
  // internal objects (memory management owned)
  TebVisualizationPtr visualization_; //!< Instance of the visualization class
  TimedElasticBand teb_; //!< Actual trajectory object
  boost::shared_ptr<g2o::SparseOptimizer> optimizer_; //!< g2o optimizer for trajectory optimization
  std::pair<bool, Eigen::Vector2d> vel_start_; //!< Store the initial velocity at the start pose
  std::pair<bool, Eigen::Vector2d> vel_goal_; //!< Store the final velocity at the goal pose

  bool initialized_; //!< Keeps track about the correct initialization of this class
  bool optimized_; //!< This variable is \c true as long as the last optimization has been completed successful
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
};

//! Abbrev. for shared instances of the TebOptimalPlanner
typedef boost::shared_ptr<TebOptimalPlanner> TebOptimalPlannerPtr;
//! Abbrev. for shared const TebOptimalPlanner pointers
typedef boost::shared_ptr<const TebOptimalPlanner> TebOptimalPlannerConstPtr;
//! Abbrev. for containers storing multiple teb optimal planners
typedef std::vector< TebOptimalPlannerPtr > TebOptPlannerContainer;

} // namespace teb_local_planner

#endif /* OPTIMAL_PLANNER_H_ */
