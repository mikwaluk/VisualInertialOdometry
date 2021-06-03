#include "GraphSolver.h"

bool GraphSolver::set_imu_preintegration(const gtsam::State& prior_state) {

  // Create GTSAM preintegration parameters for use with Foster's version
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params;
  params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(config->gravity(2));  // Z-up navigation frame: gravity points along negative Z-axis !!!
  
  params->setAccelerometerCovariance(gtsam::I_3x3 * config->sigma_a_sq);  // acc white noise in continuous
  params->setGyroscopeCovariance(gtsam::I_3x3 * config->sigma_g_sq);  // gyro white noise in continuous
  params->biasAccCovariance = config->sigma_wa_sq * gtsam::Matrix33::Identity(3,3);  // acc bias in continuous
  params->biasOmegaCovariance = config->sigma_wg_sq * gtsam::Matrix33::Identity(3,3);  // gyro bias in continuous
  params->setIntegrationCovariance(gtsam::I_3x3 * 0.1);  // error committed in integrating position from velocities
  params->biasAccOmegaInt = 1e-5*gtsam::Matrix66::Identity(6,6); // error in the bias used for preintegration
  
  // Actually create the GTSAM preintegration
  preint_gtsam = new gtsam::PreintegratedCombinedMeasurements(params, prior_state.b());
  return true;
}

/**
 * This function will create a discrete IMU factor using the GTSAM preintegrator class
 * This will integrate from the current state time up to the new update time
 */
gtsam::CombinedImuFactor GraphSolver::create_imu_factor(double updatetime, gtsam::Values& values_initial) {

    int imucompound = 0;

    // TODO: Clean this code, and use the mutex
    while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
        double dt = imu_times.at(1) - imu_times.at(0);
        if (dt >= 0) {
            // Our IMU measurement
            Eigen::Vector3d meas_angvel;
            Eigen::Vector3d meas_linaccs;
            meas_angvel = imu_angvel.at(0);
            meas_linaccs = imu_linaccs.at(0);
            // Preintegrate this measurement!
            preint_gtsam->integrateMeasurement(meas_linaccs, meas_angvel, dt);
        }
        //std::cout << "state time = " << updatetime << " | imu0 = " << imu_times.at(0) << " | imu1 = " << imu_times.at(1) << " | dt = " << dt << std::endl;
        //cout << "imu dt = " << dt << " | am = " << imu_linaccs.at(0).transpose() << " | wm = " << imu_angvel.at(0).transpose() << endl;
        imu_angvel.erase(imu_angvel.begin());
        imu_linaccs.erase(imu_linaccs.begin());
        imu_times.erase(imu_times.begin());
        imucompound++;
    }

    // TODO: Clean this code, and use the mutex
    double dt_f = updatetime - imu_times.at(0);
    if (dt_f > 0) {
        // Our IMU measurement
        Eigen::Vector3d meas_angvel;
        Eigen::Vector3d meas_linaccs;
        meas_angvel = imu_angvel.at(0);
        meas_linaccs = imu_linaccs.at(0);
        // Preintegrate this measurement!
        preint_gtsam->integrateMeasurement(meas_linaccs, meas_angvel, dt_f);
        imu_times.at(0) = updatetime;
        imucompound++;
    }
 
    return gtsam::CombinedImuFactor(X(ct_state  ), V(ct_state),
                             X(ct_state+1), V(ct_state+1),  
                             B(ct_state  ), B(ct_state+1),
                             *preint_gtsam);
}


/**
 * This function will get the predicted state based on the IMU measurement
 */

gtsam::State GraphSolver::get_predicted_state(gtsam::Values& values_initial) {

  // Get the current state (t=k)
  gtsam::State stateK = gtsam::State(values_initial.at<gtsam::Pose3>(X(ct_state)),
                                     values_initial.at<gtsam::Vector3>(V(ct_state)),
                                     values_initial.at<gtsam::Bias>(B(ct_state)));
  
  // From this we should predict where we will be at the next time (t=K+1)
  gtsam::NavState stateK1 = preint_gtsam->predict(gtsam::NavState(stateK.pose(), stateK.v()), stateK.b());
  return gtsam::State(stateK1.pose(), stateK1.v(), stateK.b());
}

void GraphSolver::reset_imu_integration() {
  
  // Use the optimized bias to reset integration
  if (values_initial.exists(B(ct_state)))
    preint_gtsam->resetIntegrationAndSetBias(values_initial.at<gtsam::Bias>(B(ct_state)));
    //preint_gtsam_->resetIntegrationAndSetBias(Bias());
  
  return;
}

/**
 * This function will create a CPI Model 2 IMU factor.
 * This will integrate from the current state time up to the new update time
 */
gtsam::ImuFactorCPIv2 GraphSolver::createimufactor_cpi_v2(double updatetime, gtsam::Values& values_initial) {


    // Get the current state and its bias estimate
    JPLNavState stateK = values_initial.at<JPLNavState>(X(ct_state));
    Bias3 bg_K = stateK.bg();
    Bias3 ba_K = stateK.ba();
    JPLQuaternion q_K = stateK.q();

    // Create our preintegrator, default to analytical flags
    CpiV2 cpi(config->sigma_g,config->sigma_wg,config->sigma_a,config->sigma_wa);
    cpi.setLinearizationPoints(bg_K,ba_K,q_K,config->gravity);
    cpi.imu_avg = false;
    cpi.state_transition_jacobians = true;

    int imucompound = 0;

    // TODO: Clean this code, and use the mutex
    while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
        double dt = imu_times.at(1) - imu_times.at(0);
        if (dt >= 0) {
            cpi.feed_IMU(imu_times.at(0), imu_times.at(1), imu_angvel.at(0), imu_linaccs.at(0), imu_angvel.at(1), imu_linaccs.at(1));
        }
        //cout << "state time = " << updatetime << " | imu0 = " << imu_times.at(0) << " | imu1 = " << imu_times.at(1) << " | dt = " << dt << endl;
        //cout << "imu dt = " << dt << " | am = " << imu_linaccs.at(0).transpose() << " | wm = " << imu_angvel.at(0).transpose() << endl;
        imu_angvel.erase(imu_angvel.begin());
        imu_linaccs.erase(imu_linaccs.begin());
        imu_times.erase(imu_times.begin());
        imucompound++;
    }

    // TODO: Clean this code, and use the mutex
    double dt_f = updatetime - imu_times.at(0);
    if (dt_f > 0) {
        cpi.feed_IMU(imu_times.at(0), updatetime, imu_angvel.at(0), imu_linaccs.at(0), imu_angvel.at(0), imu_linaccs.at(0));
        imu_times.at(0) = updatetime;
        imucompound++;
    }
    // Debug info
    //cout << "imu dt total = " << cpi.DT << " from " << imucompound << " readings" << endl;

    // Create our factor between current and next state!
    return ImuFactorCPIv2(X(ct_state),X(ct_state+1),cpi.P_meas,cpi.DT,cpi.grav,cpi.alpha_tau,cpi.beta_tau,cpi.q_k2tau,
                          cpi.q_k_lin,cpi.b_a_lin,cpi.b_w_lin,cpi.J_q,cpi.J_b,cpi.J_a,cpi.H_b,cpi.H_a,cpi.O_b,cpi.O_a);



}

/**
 * This function will get the predicted state based on the CPI measurement
 * These are based off of equation 117-118 in the tech report
 * http://udel.edu/~ghuang/papers/tr_cpi.pdf
 */
JPLNavState GraphSolver::getpredictedstate_v2(ImuFactorCPIv2& imuFactor, gtsam::Values& values_initial) {

    // Get the current state (t=k)
    JPLNavState stateK = values_initial.at<JPLNavState>(X(ct_state));
    JPLQuaternion q_GtoK = stateK.q();
    Bias3 bg_K = stateK.bg();
    Velocity3 v_KinG = stateK.v();
    Bias3 ba_K = stateK.ba();
    Vector3 p_KinG = stateK.p();

    // From this we should predict where we will be at the next time (t=K+1)
    JPLQuaternion q_GtoK1 = quat_multiply(imuFactor.m_q(),q_GtoK);
    Vector3 v_K1inG = v_KinG + quat_2_Rot(Inv(q_GtoK))*imuFactor.m_beta();
    Vector3 p_K1inG = p_KinG + v_KinG*imuFactor.dt() + quat_2_Rot(Inv(q_GtoK))*imuFactor.m_alpha();

    // Return our new state!
    return JPLNavState(q_GtoK1, bg_K, v_K1inG, ba_K, p_K1inG);

}
