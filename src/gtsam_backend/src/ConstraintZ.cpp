#include "ConstraintZ.h"

#include <ros/ros.h>


gtsam::Vector ConstraintZ::evaluateError(const gtsam::Pose3& q,
                       boost::optional<gtsam::Matrix&> J) const
{
     if (J)
     {
         (*J) = (gtsam::Matrix16() << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
     }
    const double zError = q.z() - measuredZ;
    //ROS_INFO_STREAM("Z error: " << zError << " estimated z: " << q.z() << " measured z: " << measuredZ);
    return (gtsam::Vector1() << zError).finished();

}
