#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <Eigen/Core>

class ConstraintZ: public gtsam::NoiseModelFactor1<gtsam::Pose3> {
    double measuredZ;
public:
  ConstraintZ(gtsam::Key j, const double measurement, gtsam::SharedDiagonal& model):
    gtsam::NoiseModelFactor1<gtsam::Pose3>(model, j), measuredZ {measurement}
    {}
  gtsam::Vector evaluateError(const gtsam::Pose3& q, boost::optional<gtsam::Matrix&> J = boost::none) const;
};
