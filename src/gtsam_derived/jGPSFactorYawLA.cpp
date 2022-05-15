/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   jGPSFactorYawLA.h
 *  @author Jin
 *  @brief  Header file for GPS factor with Yaw(Rnw_z), Lever Arm(LA)
 *  @date   July 17, 2021
 **/

#include "jGPSFactorYawLA.h"

using namespace std;

namespace gtsam {

//***************************************************************************
void jGPSFactorYawLA::print(const string& s, const KeyFormatter& keyFormatter) const {
  // cout << s << "jGPSFactorYawLA on " << keyFormatter(key()) << "\n";
  cout << "  GPS measurement: " << nT_ << "\n";
  noiseModel_->print("  noise model: ");
  cout << " lever arm: " << body_t_ << "\n";
  // if (this->body_t_) this->body_t_->print(" lever arm: ");
}

//***************************************************************************
bool jGPSFactorYawLA::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != nullptr && Base::equals(*e, tol) &&
         traits<Point3>::Equals(nT_, e->nT_, tol) &&
         body_t_.equals(e->body_t_);
}

//***************************************************************************
Vector jGPSFactorYawLA::evaluateError(const double& yaw, const Pose3& p,
                                     boost::optional<Matrix&> H1,
                                     boost::optional<Matrix&> H2) const {
  Rot3 Ryaw = Rot3::Rz(yaw);
  double sy = sin(yaw), cy = cos(yaw);
  Rot3 dRyaw_dyaw = Rot3(-sy, -cy, 0,  //
                         cy, -sy, 0,    //
                         0, 0, 0);
  Point3 twg = p.transformFrom(body_t_); // Rwi*tig+twi

  if (H1) {
    H1->resize(3, 1);

    *H1 = dRyaw_dyaw.matrix() * twg.vector();
  }
  if (H2) {
    H2->resize(3, 6);
    *H2 << -Ryaw.matrix() * p.rotation().matrix() * skewSymmetric(body_t_),
        Ryaw.matrix() * p.rotation().matrix();
  }

  return Ryaw * twg - nT_;
}

//***************************************************************************
pair<Pose3, Vector3> jGPSFactorYawLA::EstimateState(double t1, const Point3& NED1,
    double t2, const Point3& NED2, double timestamp) {
  // Estimate initial velocity as difference in NED frame
  double dt = t2 - t1;
  Point3 nV = (NED2 - NED1) / dt;

  // Estimate initial position as linear interpolation
  Point3 nT = NED1 + nV * (timestamp - t1);

  // Estimate Rotation
  double yaw = atan2(nV.y(), nV.x());
  Rot3 nRy = Rot3::Yaw(yaw); // yaw frame
  Point3 yV = nRy.inverse() * nV; // velocity in yaw frame
  double pitch = -atan2(yV.z(), yV.x()), roll = 0;
  Rot3 nRb = Rot3::Ypr(yaw, pitch, roll);

  // Construct initial pose
  Pose3 nTb(nRb, nT); // nTb

  return make_pair(nTb, nV);
}

//***************************************************************************

}/// namespace gtsam
