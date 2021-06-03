
#include "GraphSolver.h"
#include "JPLImageUVFactor.h"

#include <ros/ros.h>

void GraphSolver::process_feat_smart(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv) {
        //==============================================================================
    // Loop through LEFT features
    for(size_t i=0; i<leftids.size(); i++) {
        // Check to see if it is already in the graph
        if(measurement_lookup.find(leftids.at(i)) != measurement_lookup.end() && (ct_state-measurement_state_lookup[leftids.at(i)]) < (size_t)config->uvWindowSize) {
            Eigen::Matrix<double,2,2> sqrtQ = config->sigma_camera_sq*Eigen::Matrix<double,2,2>::Identity();
            JPLImageUVFactor factor(X(ct_state),F(measurement_lookup[leftids.at(i)]),sqrtQ,leftuv.at(i),config->R_C0toI,config->p_IinC0);
            graph->add(factor);
            graph_new->add(factor);
            continue;
        }
        // Next check to see if this feature is in our queue
        // We know this is in our queue, so lets add this new measurement to it
        if(measurement_queue.find(leftids.at(i)) != measurement_queue.end()) {
            measurement_queue[leftids.at(i)].leftids.push_back(leftids.at(i));
            measurement_queue[leftids.at(i)].leftuv.push_back(leftuv.at(i));
            measurement_queue[leftids.at(i)].leftstateids.push_back(ct_state);
            continue;
        }
        // Else this is a new feature, so lets just add it as a new feature
        // Create our new feature object
        feature featnew;
        featnew.leftids.push_back(leftids.at(i));
        featnew.leftuv.push_back(leftuv.at(i));
        featnew.leftstateids.push_back(ct_state);
        // Push into our queue
        measurement_queue[leftids.at(i)] = featnew;
    }



    //==============================================================================
    // Track which features should be removed from the queue
    std::vector<int> toremove;
    // Create our initializer class
    FeatureInitializer initializer(config, values_initial, ct_state);
    int ct_successes = 0;
    int ct_failures = 0;
    // Lastly lets add all the features that have reached the
    for(auto& measurement: measurement_queue) {

        // Boolean logic statements, a correct queue feature does not have the min pose requirement
        // A queue feature should also have a non-empty state ID that is equal to the current state (i.e. is being actively tracked)
        bool leftminposes = measurement.second.leftuv.size() < (size_t)config->minPoseFeatureInit;
        bool rightminposes = measurement.second.rightuv.size() < (size_t)config->minPoseFeatureInit;
        bool leftlost = measurement.second.leftstateids.empty() || measurement.second.leftstateids.at(measurement.second.leftstateids.size()-1) != ct_state;
        bool rightlost = measurement.second.rightstateids.empty() || measurement.second.rightstateids.at(measurement.second.rightstateids.size()-1) != ct_state;

        // If the feature has lost track, then we should remove it if doesn't have the needed UV measurement size
        if(leftlost && rightlost && leftminposes && rightminposes) {
            toremove.push_back(measurement.first);
            //ROS_ERROR("Removing feature #%d as it has lost tracking (%d leftuv, %d rightuv)",measurement.first,(int)measurement.second.leftuv.size(),(int)measurement.second.rightuv.size());
            continue;
        }

        // Remove if less then half right features as there are left ones
        if(measurement.second.rightuv.size() < 0.5*measurement.second.leftuv.size()) {
            toremove.push_back(measurement.first);
            //ROS_ERROR("Removing feature #%d as not enough right features (%d leftuv, %d rightuv)",measurement.first,(int)measurement.second.leftuv.size(),(int)measurement.second.rightuv.size());
            continue;
        }

        // Skip if it has not reached max size, but is still being actively tracked
        if(leftminposes && rightminposes)
            continue;

        // We are good, either the left or right has enough poses to initialize
        // Store our key so that it will be removed from the queue
        toremove.push_back(measurement.first);

        // Check that all IDs are the same
        if(!measurement.second.leftids.empty()) assert(std::equal(measurement.second.leftids.begin()+1, measurement.second.leftids.end(), measurement.second.leftids.begin()));
        if(!measurement.second.rightids.empty()) assert(std::equal(measurement.second.rightids.begin()+1, measurement.second.rightids.end(), measurement.second.rightids.begin()));

        // Initialize the 3D position of the feature
        bool successMODEL2 = initializer.initialize_feature(measurement.second);
        std::pair<int, feature> measurementMODEL2 = measurement;

        // If not successful skip this feature
        if(!successMODEL2) {
            ct_failures++;
            continue;
        }

        // Ensure we have at least one left feature (we pick this to be our anchor)
        if(measurementMODEL2.second.leftuv.empty() || measurementMODEL2.second.rightuv.empty())
            continue;

        // Move feature ID forward in time, and add to our lookup data structure
        // Note: we set our anchor to be the first left camera state
        ct_features++;
        measurement_lookup[measurement.first] = ct_features;
        measurement_state_lookup[measurement.first] = ct_state;

        // Lets add the new feature to graph
        // NOTE: normal 3d feature since we are NOT using inverse depth
        values_new.insert(F(ct_features), gtsam::Point3(measurementMODEL2.second.pos_FinG));
        values_initial.insert(F(ct_features), gtsam::Point3(measurementMODEL2.second.pos_FinG));

        // Append to our fix lag smoother timestamps
        newTimestamps[F(ct_features)] = timestamp;

        // Next lets add all LEFT factors (all graphs have the same measurements!!!)
        for(size_t j=0; j<measurementMODEL2.second.leftuv.size(); j++) {
            Eigen::Matrix<double,2,2> sqrtQ = config->sigma_camera_sq*Eigen::Matrix<double,2,2>::Identity();
            JPLImageUVFactor factor(X(measurementMODEL2.second.leftstateids.at(j)),F(ct_features),sqrtQ,measurementMODEL2.second.leftuv.at(j),config->R_C0toI,config->p_IinC0);
            graph->add(factor);
            graph_new->add(factor);
        }
        // Record our success
        ct_successes++;

    }

    // Debug info
    if(ct_successes+ct_failures > 0) {
        ROS_WARN("[FEAT]: %d of %d features successfully initialized (non-inverse depth)", ct_successes, ct_successes + ct_failures);
    }


    //==============================================================================
    // Remove the "used up" features from the queue
    for (auto&& key : toremove) {
        measurement_queue.erase(key);
    }
}

