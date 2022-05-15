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
#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

/**
 * Prior on position in a Cartesian frame.
 * Possibilities include:
 *   ENU: East-North-Up navigation frame at some local origin
 *   NED: North-East-Down navigation frame at some local origin
 *   ECEF: Earth-centered Earth-fixed, origin at Earth's center
 * See Farrell08book or e.g. http://www.dirsig.org/docs/new/coordinates.html
 * @addtogroup Navigation
 */
class GTSAM_EXPORT jGPSFactorYawLA: public NoiseModelFactor2<double, Pose3> {

private:

  typedef NoiseModelFactor2<double, Pose3> Base;

  Point3 nT_;  ///< Position measurement in cartesian coordinates
  Point3 body_t_;  ///< Lever arm, i.e., antenna position in body frame

 public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<jGPSFactorYawLA> shared_ptr;

  /// Typedef to this class
  typedef jGPSFactorYawLA This;

  /** default constructor - only use for serialization */
  jGPSFactorYawLA() : nT_(0, 0, 0), body_t_(0, 0, 0) {}

  virtual ~jGPSFactorYawLA() {}

  /**
   * @brief Constructor from a measurement in a Cartesian frame.
   * Use GeographicLib to convert from geographic (latitude and longitude) coordinates
   * @param yawKey of the yaw between world frame and navigation frame
   * @param poseKey of the Pose3 variable that will be constrained
   * @param gpsIn measurement already in correct coordinates
   * @param model Gaussian noise model
   */
  jGPSFactorYawLA(Key yawKey, Key poseKey, const Point3& gpsIn,
                 const SharedNoiseModel& model, Point3 body_t)
      : Base(model, yawKey, poseKey), nT_(gpsIn), body_t_(body_t) {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const;

  /// equals
  virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const;

  /// vector of errors
  Vector evaluateError(const double& yaw, const Pose3& p,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none) const;

  inline const Point3 & measurementIn() const {
    return nT_;
  }

  /**
   *  Convenience function to estimate state at time t, given two GPS
   *  readings (in local NED Cartesian frame) bracketing t
   *  Assumes roll is zero, calculates yaw and pitch from NED1->NED2 vector.
   */
  static std::pair<Pose3, Vector3> EstimateState(double t1, const Point3& NED1,
      double t2, const Point3& NED2, double timestamp);

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(nT_);
  }
};

} /// namespace gtsam
