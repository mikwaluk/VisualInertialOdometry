#include <math.h>
#include <ros/ros.h>

#include "ConstraintZ.h"
#include "GraphSolver.h"
#include "JPLNavStatePrior.h"


void GraphSolver::addmeasurement_imu(double timestamp, Eigen::Vector3d linacc, Eigen::Vector3d angvel, Eigen::Vector4d orientation) {
  // Request access to the imu measurements
  std::unique_lock<std::mutex> lock(imu_mutex);

  // Append this new measurement to the array
  imu_times.push_back(timestamp);
  imu_linaccs.push_back(linacc);
  imu_angvel.push_back(angvel);
  imu_orientation.push_back(orientation);

}

void GraphSolver::addmeasurement_uv(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv) {

  // Return if the node already exists in the graph
  if (ct_state_lookup.find(timestamp) != ct_state_lookup.end())
    return;

  // Return if we don't actually have any plane measurements
  if(leftids.empty())
      return;

  // Return if we don't actually have any IMU measurements
  if(imu_times.size() < 2)
      return;

  // We should try to initialize now
  // Or add the current a new IMU measurement and state!
  if(!systeminitalized) {

      initialize(timestamp);

      // Return if we have not initialized the system yet
      if(!systeminitalized)
          return;

  } else {

      ImuFactorCPIv2 imuFactor = createimufactor_cpi_v2(timestamp, values_initial);
      graph_new->add(imuFactor);
      graph->add(imuFactor);

      // Original models
      JPLNavState newstate = getpredictedstate_v2(imuFactor, values_initial);

      // Move node count forward in time
      ct_state++;

      // Append to our node vectors
      values_new.insert(    X(ct_state), newstate);
      values_initial.insert(X(ct_state), newstate);

      // Add ct state to map
      ct_state_lookup[timestamp] = ct_state;
      timestamp_lookup[ct_state] = timestamp;
      newTimestamps[X(ct_state)] = timestamp;
  }

  // Assert our vectors are equal (note will need to remove top one eventually)
  assert(leftids.size() == leftuv.size());

  // Request access
  std::unique_lock<std::mutex> features_lock(features_mutex);

  // If we are using inverse depth, then lets call on it
  process_feat_smart(timestamp, leftids, leftuv);
}

void GraphSolver::optimize() {

  // Return if not initialized
  if(!systeminitalized && ct_state < 2)
      return;

  // Perform smoothing update
  try {
    ROS_INFO_STREAM("Printing the new timestamps");
    for(auto it = newTimestamps.cbegin(); it != newTimestamps.cend(); ++it)
    {
       ROS_INFO_STREAM("Key: " << it->first << " Time: " << it->second);
    }
    isam2->update(*graph_new, values_new, newTimestamps);
    ROS_INFO_STREAM("Calculating a new estimate");
    values_initial = isam2->calculateEstimate();
    ROS_INFO_STREAM("Timestamps within the graph: ");
    for(const gtsam::FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: isam2->timestamps())
    {
      ROS_INFO_STREAM("Key: " << key_timestamp.first << " Time: " << key_timestamp.second);
    }
    const gtsam::NonlinearFactorGraph & factorGraph = isam2->getFactors();
  } catch(gtsam::IndeterminantLinearSystemException &e) {
      ROS_ERROR("FORSTER2 gtsam indeterminate linear system exception!");
      std::cerr << e.what() << std::endl;
      exit(EXIT_FAILURE);
  }

  // Remove the used up nodes
  values_new.clear();

  // Remove the used up factors
  graph_new->resize(0);

  // Clear the new timestamps
  newTimestamps.clear();

}

void GraphSolver::initialize(double timestamp) {
  ROS_INFO_STREAM("initialize");
  // If we have already initialized, then just return
  if (systeminitalized)
    return;
    
  // Wait for enough IMU readings if we initialize from rest
  if (imu_times.size() < config->imuWait)
    return;
  
  // Reading from launch file
  gtsam::Vector4 q_GtoI = config->prior_qGtoI;
  gtsam::Vector3 p_IinG = config->prior_pIinG;
  gtsam::Vector3 v_IinG = config->prior_vIinG;
  gtsam::Vector3 ba = config->prior_ba;
  gtsam::Vector3 bg = config->prior_bg;

  Eigen::Matrix<double,15,15> cov = Eigen::Matrix<double,15,15>::Zero();
  cov.block(0, 0, 3, 3) = config->sigma_prior_rotation * Eigen::Matrix3d::Identity();
  cov.block(3, 3, 3, 3) = config->sigma_prior_translation * Eigen::Matrix3d::Identity();
  cov.block(6, 6, 3, 3) = config->sigma_velocity * Eigen::Matrix3d::Identity();
  cov.block(9, 9, 6, 6) = config->sigma_bias * Eigen::Matrix<double,6,6>::Identity();

  JPLNavStatePrior priorfactor(X(ct_state), cov, q_GtoI, bg, v_IinG, ba, p_IinG);
  graph_new->add(priorfactor);
  graph->add(priorfactor);

  // Add initial state to the graph
  values_new.insert(X(ct_state), JPLNavState(q_GtoI, bg, v_IinG, ba, p_IinG));
  values_initial.insert(X(ct_state), JPLNavState(q_GtoI, bg, v_IinG, ba, p_IinG));

  // Add ct state to map
  ct_state_lookup[timestamp] = ct_state;
  timestamp_lookup[ct_state] = timestamp;
  newTimestamps[X(ct_state)] = timestamp;

  // Clear all old imu messages (keep the last two)
  imu_times.erase(imu_times.begin(), imu_times.end() - 1);
  imu_linaccs.erase(imu_linaccs.begin(), imu_linaccs.end() - 1);
  imu_angvel.erase(imu_angvel.begin(), imu_angvel.end() - 1);
  imu_orientation.erase(imu_orientation.begin(), imu_orientation.end() - 1);

  systeminitalized = true;

  // Debug info
  ROS_INFO("\033[0;32m[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\033[0m",q_GtoI(0),q_GtoI(1),q_GtoI(2),q_GtoI(3));
  ROS_INFO("\033[0;32m[INIT]: velocity = %.4f, %.4f, %.4f\033[0m",v_IinG(0),v_IinG(1),v_IinG(2));
  ROS_INFO("\033[0;32m[INIT]: position = %.4f, %.4f, %.4f\033[0m",p_IinG(0),p_IinG(1),p_IinG(2));
  ROS_INFO("\033[0;32m[INIT]: bias accel = %.4f, %.4f, %.4f\033[0m",ba(0),ba(1),ba(2));
  ROS_INFO("\033[0;32m[INIT]: bias gyro = %.4f, %.4f, %.4f\033[0m",bg(0),bg(1),bg(2));
}
