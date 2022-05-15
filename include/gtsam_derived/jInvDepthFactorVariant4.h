
/**
 * @file jInvDepthFactorVariant4.h
 * @brief Inverse Depth Factor based on VINS-Mono.
 * Landmarks are parameterized as (rho), it is of course only one double-typed
 * parameter just as you see, because theta and phi can be accurately determined
 * by Camera Projection. The factor involves two poses and a landmark. The first
 * pose is the reference frame from which the first landmark is measured.
 * @author Ronghe Jin
 */

#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

// /**
//  * Binary factor representing the first visual measurement using an
//  * inverse-depth parameterization
//  * Note(Jin): this factor has actually little effection on InvDepth,
//  *            you cans simplely ignore it when optimize such an issue1
//  */
// class InvDepthFactorVariant4a : public NoiseModelFactor2<Pose3, double> {
//  protected:
//   // Keep a copy of measurement and calibration for I/O
//   Point2 measured_;        ///< 2D measurement
//   Cal3_S2::shared_ptr K_;  ///< shared pointer to calibration object
//   boost::optional<Pose3>
//       body_P_sensor_;  ///< The pose of the sensor in the body frame

//  public:
//   /// shorthand for base class type
//   typedef NoiseModelFactor2<Pose3, double> Base;

//   /// shorthand for this class
//   typedef InvDepthFactorVariant4a This;

//   /// shorthand for a smart pointer to a factor
//   typedef boost::shared_ptr<This> shared_ptr;

//   /// Default constructor
//   InvDepthFactorVariant4a()
//       : measured_(0.0, 0.0), K_(new Cal3_S2(444, 555, 666, 777, 888)) {}

//   /**
//    * Constructor
//    * TODO: Mark argument order standard (keys, measurement, parameters)
//    * @param measured is the 2 dimensional location of point in image (the
//    * measurement)
//    * @param model is the standard deviation
//    * @param poseKey is the index of the camera pose
//    * @param pointKey is the index of the landmark
//    * @param invDepthKey is the index of inverse depth
//    * @param K shared pointer to the constant calibration
//    * @param body_P_sensor is the transform from body to sensor frame (default
//    * identity)
//    */
//   InvDepthFactorVariant4a(const Key poseKey, const Key landmarkKey,
//                           const Point2 &measured, const Cal3_S2::shared_ptr
//                           &K, const SharedNoiseModel &model,
//                           boost::optional<Pose3> body_P_sensor = boost::none)
//       : Base(model, poseKey, landmarkKey),
//         measured_(measured),
//         K_(K),
//         body_P_sensor_(body_P_sensor) {}

//   /** Virtual destructor */
//   virtual ~InvDepthFactorVariant4a() {}

//   /**
//    * print
//    * @param s optional string naming the factor
//    * @param keyFormatter optional formatter useful for printing Symbols
//    */
//   void print(const std::string &s = "InvDepthFactorVariant4a",
//              const KeyFormatter &keyFormatter = DefaultKeyFormatter) const {
//     Base::print(s, keyFormatter);
//     traits<Point2>::Print(measured_, s + ".z");
//     if (this->body_P_sensor_)
//       this->body_P_sensor_->print("  sensor pose in body frame: ");
//   }

//   /// equals
//   virtual bool equals(const NonlinearFactor &p, double tol = 1e-9) const {
//     const This *e = dynamic_cast<const This *>(&p);
//     return e && Base::equals(p, tol) &&
//            traits<Point2>::Equals(this->measured_, e->measured_, tol) &&
//            this->K_->equals(*e->K_, tol) &&
//            ((!body_P_sensor_ && !e->body_P_sensor_) ||
//             (body_P_sensor_ && e->body_P_sensor_ &&
//              body_P_sensor_->equals(*e->body_P_sensor_)));
//   }

//   Vector inverseDepthError(const Pose3 &pose, const double &rho) const {
//     try {
//       if (body_P_sensor_) {
//         // Calculate the 3D coordinates of the landmark in the Pose frame
//         Point2 p1n = K_->calibrate(measured_);
//         Point3 pose_P_landmark(p1n(0) / rho, p1n(1) / rho, 1. / rho);
//         // Convert the landmark to world coordinates
//         Pose3 world_pose_camera = pose.compose(*body_P_sensor_);
//         Point3 world_P_landmark =
//             world_pose_camera.transformFrom(pose_P_landmark);
//         // Project landmark into Pose2
//         PinholeCamera<Cal3_S2> camera(world_pose_camera, *K_);

//         // Vector prj = camera.project(world_P_landmark);
//         // std::cout << prj << std::endl;

//         return camera.project(world_P_landmark) - measured_;
//       } else {
//         // Calculate the 3D coordinates of the landmark in the Pose frame
//         Point2 p1n = K_->calibrate(measured_);
//         Point3 pose_P_landmark(p1n(0) / rho, p1n(1) / rho, 1. / rho);
//         // Convert the landmark to world coordinates
//         Point3 world_P_landmark = pose.transformFrom(pose_P_landmark);
//         // Project landmark into Pose2
//         PinholeCamera<Cal3_S2> camera(pose, *K_);

//         // Vector prj = camera.project(world_P_landmark);
//         // std::cout << prj << std::endl;

//         return camera.project(world_P_landmark) - measured_;
//       }
//     } catch (CheiralityException &e) {
//       std::cout << e.what() << ": Inverse Depth Landmark ["
//                 << DefaultKeyFormatter(this->key1()) << ","
//                 << DefaultKeyFormatter(this->key2()) << "]"
//                 << " moved behind camera [" <<
//                 DefaultKeyFormatter(this->key1())
//                 << "]" << std::endl;
//       return Vector::Ones(2) * 2.0 * K_->fx();
//     }
//     return (Vector(1) << 0.0).finished();
//   }

//   /// Evaluate error h(x)-z and optionally derivatives
//   Vector evaluateError(const Pose3 &pose, const double &rho,
//                        boost::optional<Matrix &> H1 = boost::none,
//                        boost::optional<Matrix &> H2 = boost::none) const {
//     if (H1) {
//       (*H1) = numericalDerivative11<Vector, Pose3>(
//           boost::bind(&InvDepthFactorVariant4a::inverseDepthError, this, _1,
//                       rho),
//           pose);
//       // std::cout << "4a-de_dT1:" << std::endl << *H1 << std::endl;
//     }
//     if (H2) {
//       (*H2) = numericalDerivative11<Vector, double>(
//           boost::bind(&InvDepthFactorVariant4a::inverseDepthError, this,
//           pose,
//                       _1),
//           rho);
//       // std::cout << "4a-de_drho:" << std::endl << *H2 << std::endl;
//     }

//     return inverseDepthError(pose, rho);
//   }

//   /** return the measurement */
//   const Point2 &imagePoint() const { return measured_; }

//   /** return the calibration object */
//   const Cal3_S2::shared_ptr calibration() const { return K_; }

//  private:
//   /// Serialization function
//   friend class boost::serialization::access;
//   template <class ARCHIVE>
//   void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
//     ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
//     ar &BOOST_SERIALIZATION_NVP(measured_);
//     ar &BOOST_SERIALIZATION_NVP(K_);
//     ar &BOOST_SERIALIZATION_NVP(body_P_sensor_);
//   }
// };

/**
 * Ternary factor representing a visual measurement using an inverse-depth
 * parameterization
 */
class InvDepthFactorVariant4b : public NoiseModelFactor3<Pose3, Pose3, double> {
 protected:
  // Keep a copy of measurement and calibration for I/O
  Point2 measured_ref_;    ///< 2D measurement of first frame
  Point2 measured_;        ///< 2D measurement
  Cal3_S2::shared_ptr K_;  ///< shared pointer to calibration object
  boost::optional<Pose3>
      body_P_sensor_;  ///< The pose of the sensor in the body frame

 public:
  /// shorthand for base class type
  typedef NoiseModelFactor3<Pose3, Pose3, double> Base;

  /// shorthand for this class
  typedef InvDepthFactorVariant4b This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  InvDepthFactorVariant4b()
      : measured_ref_(0.0, 0.0),
        measured_(0.0, 0.0),
        K_(new Cal3_S2(444, 555, 666, 777, 888)) {}

  /**
   * Constructor
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the
   * measurement)
   * @param model is the standard deviation
   * @param poseKey is the index of the camera pose
   * @param pointKey is the index of the landmark
   * @param invDepthKey is the index of inverse depth
   * @param K shared pointer to the constant calibration
   * @param body_P_sensor is the transform from body to sensor frame (default
   * identity
   */
  InvDepthFactorVariant4b(const Key poseKey1, const Key poseKey2,
                          const Key landmarkKey, const Point2 &measured_ref,
                          const Point2 &measured, const Cal3_S2::shared_ptr &K,
                          const SharedNoiseModel &model,
                          boost::optional<Pose3> body_P_sensor = boost::none)
      : Base(model, poseKey1, poseKey2, landmarkKey),
        measured_ref_(measured_ref),
        measured_(measured),
        K_(K),
        body_P_sensor_(body_P_sensor) {}

  /** Virtual destructor */
  virtual ~InvDepthFactorVariant4b() {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string &s = "InvDepthFactorVariant4b",
             const KeyFormatter &keyFormatter = DefaultKeyFormatter) const {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_ref_, s + ".z");
    traits<Point2>::Print(measured_, s + ".z");
    if (this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
  }

  /// equals
  virtual bool equals(const NonlinearFactor &p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This *>(&p);
    return e && Base::equals(p, tol) &&
           traits<Point2>::Equals(this->measured_ref_, e->measured_ref_, tol) &&
           traits<Point2>::Equals(this->measured_, e->measured_, tol) &&
           this->K_->equals(*e->K_, tol) &&
           ((!body_P_sensor_ && !e->body_P_sensor_) ||
            (body_P_sensor_ && e->body_P_sensor_ &&
             body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  Vector inverseDepthError(const Pose3 &pose1, const Pose3 &pose2,
                           const double &rho) const {
    try {
      if (body_P_sensor_) {
        // Calculate the 3D coordinates of the landmark in the Pose1 frame
        Point2 p1n = K_->calibrate(measured_ref_);
        Point3 pose1_P_landmark(p1n(0) / rho, p1n(1) / rho, 1. / rho);
        // Convert the landmark to world coordinates
        Pose3 world_pose1_camera = pose1.compose(*body_P_sensor_);
        Point3 world_P_landmark =
            world_pose1_camera.transformFrom(pose1_P_landmark);
        // Project landmark into Pose2
        Pose3 world_pose2_camera = pose2.compose(*body_P_sensor_);
        PinholeCamera<Cal3_S2> camera(world_pose2_camera, *K_);

        // Vector prj = camera.project(world_P_landmark);
        // std::cout << prj << std::endl;

        return camera.project(world_P_landmark) - measured_;
      } else {
        // Calculate the 3D coordinates of the landmark in the Pose1 frame
        Point2 p1n = K_->calibrate(measured_ref_);
        Point3 pose1_P_landmark(p1n(0) / rho, p1n(1) / rho, 1. / rho);
        // Convert the landmark to world coordinates
        Point3 world_P_landmark = pose1.transformFrom(pose1_P_landmark);
        // Project landmark into Pose2
        PinholeCamera<Cal3_S2> camera(pose2, *K_);

        // Vector prj = camera.project(world_P_landmark);
        // std::cout << prj << std::endl;

        return camera.project(world_P_landmark) - measured_;
      }
    } catch (CheiralityException &e) {
      std::cout << e.what() << ": Inverse Depth Landmark ["
                << DefaultKeyFormatter(this->key1()) << ","
                << DefaultKeyFormatter(this->key3()) << "]"
                << " moved behind camera " << DefaultKeyFormatter(this->key2())
                << std::endl;
      return Vector::Ones(2) * 2.0 * K_->fx();
    }
    return (Vector(1) << 0.0).finished();
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3 &pose1, const Pose3 &pose2,
                       const double &rho,
                       boost::optional<Matrix &> H1 = boost::none,
                       boost::optional<Matrix &> H2 = boost::none,
                       boost::optional<Matrix &> H3 = boost::none) const {
    if (H1)
      (*H1) = numericalDerivative11<Vector, Pose3>(
          boost::bind(&InvDepthFactorVariant4b::inverseDepthError, this, _1,
                      pose2, rho),
          pose1);
    // if (H1) std::cout << "4b-de_dT1:" << std::endl << *H1 << std::endl;

    if (H2)
      (*H2) = numericalDerivative11<Vector, Pose3>(
          boost::bind(&InvDepthFactorVariant4b::inverseDepthError, this, pose1,
                      _1, rho),
          pose2);
    // if (H2) std::cout << "4b-de_dT2:" << std::endl << *H2 << std::endl;

    if (H3)
      (*H3) = numericalDerivative11<Vector, double>(
          boost::bind(&InvDepthFactorVariant4b::inverseDepthError, this, pose1,
                      pose2, _1),
          rho);
    // if (H3) std::cout << "4b-de_drho:" << std::endl << *H3 << std::endl;

    return inverseDepthError(pose1, pose2, rho);
  }

  /** return the measurement */
  const Point2 &imagePoint() const { return measured_; }

  /** return the calibration object */
  const Cal3_S2::shared_ptr calibration() const { return K_; }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(measured_);
    ar &BOOST_SERIALIZATION_NVP(K_);
    ar &BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
};

/**
 * Ternary factor representing a visual measurement using an inverse-depth
 * parameterization
 */
class InvDepthFactorVariant4c : public NoiseModelFactor3<Pose3, Pose3, double> {
 protected:
  // Keep a copy of measurement and calibration for I/O
  Point2 measured_ref_;  ///< 2D measurement of first normalized plane
  Point2 measured_;      ///< 2D measurement normalized plane
  boost::optional<Pose3>
      body_P_sensor_;  ///< The pose of the sensor in the body frame

 public:
  /// shorthand for base class type
  typedef NoiseModelFactor3<Pose3, Pose3, double> Base;

  /// shorthand for this class
  typedef InvDepthFactorVariant4c This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  InvDepthFactorVariant4c() : measured_ref_(0.0, 0.0), measured_(0.0, 0.0) {}

  /**
   * Constructor
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the
   * measurement)
   * @param model is the standard deviation
   * @param pose1Key is the index of the reference camera pose
   * @param pose2Key is the index of the current camera pose
   * @param invDepthKey is the index of inverse depth
   * @param body_P_sensor is the transform from body to sensor frame (default
   * identity
   */
  InvDepthFactorVariant4c(const Key poseKey1, const Key poseKey2,
                          const Key landmarkKey, const Point2 &measured_ref,
                          const Point2 &measured, const SharedNoiseModel &model,
                          boost::optional<Pose3> body_P_sensor = boost::none)
      : Base(model, poseKey1, poseKey2, landmarkKey),
        measured_ref_(measured_ref),
        measured_(measured),
        body_P_sensor_(body_P_sensor) {}

  /** Virtual destructor */
  virtual ~InvDepthFactorVariant4c() {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string &s = "InvDepthFactorVariant4c",
             const KeyFormatter &keyFormatter = DefaultKeyFormatter) const {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_ref_, s + ".z");
    traits<Point2>::Print(measured_, s + ".z");
    if (this->body_P_sensor_)
      this->body_P_sensor_->print("  sensor pose in body frame: ");
  }

  /// equals
  virtual bool equals(const NonlinearFactor &p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This *>(&p);
    return e && Base::equals(p, tol) &&
           traits<Point2>::Equals(this->measured_ref_, e->measured_ref_, tol) &&
           traits<Point2>::Equals(this->measured_, e->measured_, tol) &&
           ((!body_P_sensor_ && !e->body_P_sensor_) ||
            (body_P_sensor_ && e->body_P_sensor_ &&
             body_P_sensor_->equals(*e->body_P_sensor_)));
  }

  Vector inverseDepthError(const Pose3 &pose1, const Pose3 &pose2,
                           const double &rho) const {
    try {
      if (body_P_sensor_) {
        // Calculate the 3D coordinates of the landmark in the Pose1 frame
        Point3 pose1_P_landmark(measured_ref_(0) / rho, measured_ref_(1) / rho,
                                1. / rho);
        // Convert the landmark to world coordinates
        Pose3 world_pose1_camera = pose1.compose(*body_P_sensor_);
        Point3 world_P_landmark =
            world_pose1_camera.transformFrom(pose1_P_landmark);
        // Project landmark into Pose2
        Pose3 world_pose2_camera = pose2.compose(*body_P_sensor_);
        PinholeBase normalized_plane(world_pose2_camera);

        // Vector prj = normalized_plane.project2(world_P_landmark);
        // std::cout << prj << std::endl;

        return normalized_plane.project2(world_P_landmark) - measured_;
      } else {
        // Calculate the 3D coordinates of the landmark in the Pose1 frame
        Point3 pose1_P_landmark(measured_ref_(0) / rho, measured_ref_(1) / rho,
                                1. / rho);
        // Convert the landmark to world coordinates
        Point3 world_P_landmark = pose1.transformFrom(pose1_P_landmark);
        // Project landmark into Pose2
        PinholeBase camera(pose2);

        // Vector prj = camera.project2(world_P_landmark);
        // std::cout << prj << std::endl;

        return camera.project2(world_P_landmark) - measured_;
      }
    } catch (CheiralityException &e) {
      std::cout << e.what() << ": Inverse Depth Landmark ["
                << DefaultKeyFormatter(this->key1()) << ","
                << DefaultKeyFormatter(this->key3()) << "]"
                << " moved behind camera " << DefaultKeyFormatter(this->key2())
                << std::endl;
      return Vector::Ones(2) * 2.0;
    }
    return (Vector(1) << 0.0).finished();
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3 &pose1, const Pose3 &pose2,
                       const double &rho,
                       boost::optional<Matrix &> H1 = boost::none,
                       boost::optional<Matrix &> H2 = boost::none,
                       boost::optional<Matrix &> H3 = boost::none) const {
    if (H1)
      (*H1) = numericalDerivative11<Vector, Pose3>(
          boost::bind(&InvDepthFactorVariant4c::inverseDepthError, this, _1,
                      pose2, rho),
          pose1);
    // if (H1) std::cout << "4c-de_dT1:" << std::endl << *H1 << std::endl;

    if (H2)
      (*H2) = numericalDerivative11<Vector, Pose3>(
          boost::bind(&InvDepthFactorVariant4c::inverseDepthError, this, pose1,
                      _1, rho),
          pose2);
    // if (H2) std::cout << "4c-de_dT2:" << std::endl << *H2 << std::endl;

    if (H3)
      (*H3) = numericalDerivative11<Vector, double>(
          boost::bind(&InvDepthFactorVariant4c::inverseDepthError, this, pose1,
                      pose2, _1),
          rho);
    // if (H3) std::cout << "4c-de_drho:" << std::endl << *H3 << std::endl;

    return inverseDepthError(pose1, pose2, rho);
  }

  /** return the measurement */
  const Point2 &imagePoint() const { return measured_; }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(measured_);
    ar &BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
};

// 四元函数
class InvDepthFactorVariant4PPP
    : public NoiseModelFactor4<Pose3, Pose3, Pose3, double> {
 protected:
  // Keep a copy of measurement and calibration for I/O
  Point2 measured_ref_;  ///< 2D measurement of first normalized plane
  Point2 measured_;      ///< 2D measurement normalized plane

  public:
  /// shorthand for base class type
  typedef NoiseModelFactor4<Pose3, Pose3, Pose3, double> Base;

  /// shorthand for this class
  typedef InvDepthFactorVariant4PPP This;

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  InvDepthFactorVariant4PPP() : measured_ref_(0.0, 0.0), measured_(0.0, 0.0) {}
  
  /**
   * Constructor
   * TODO: Mark argument order standard (keys, measurement, parameters)
   * @param measured is the 2 dimensional location of point in image (the
   * measurement)
   * @param model is the standard deviation
   * @param pose1Key is the index of the reference camera pose
   * @param pose2Key is the index of the current camera pose
   * @param transformKey is the index of the body-camera transform
   * @param invDepthKey is the index of inverse depth
   * @param body_P_sensor is the transform from body to sensor frame (default
   * identity
   */
  InvDepthFactorVariant4PPP(const Key poseKey1, const Key poseKey2, Key transformKey,
                          const Key invDepthKey, const Point2 &measured_ref,
                          const Point2 &measured, const SharedNoiseModel &model)
      : Base(model, poseKey1, poseKey2, transformKey, invDepthKey),
        measured_ref_(measured_ref),
        measured_(measured) {}

  /** Virtual destructor */
  virtual ~InvDepthFactorVariant4PPP() {}

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string &s = "InvDepthFactorVariant4PPP",
             const KeyFormatter &keyFormatter = DefaultKeyFormatter) const {
    Base::print(s, keyFormatter);
    traits<Point2>::Print(measured_ref_, s + ".z");
    traits<Point2>::Print(measured_, s + ".z");
  }

  /// equals
  virtual bool equals(const NonlinearFactor &p, double tol = 1e-9) const {
    const This *e = dynamic_cast<const This *>(&p);
    return e && Base::equals(p, tol) &&
           traits<Point2>::Equals(this->measured_ref_, e->measured_ref_, tol) &&
           traits<Point2>::Equals(this->measured_, e->measured_, tol);
  }

  Vector inverseDepthError(const Pose3 &pose1, const Pose3 &pose2,
                           const Pose3 &transform, const double &rho) const {
    try {
      // Calculate the 3D coordinates of the landmark in the Pose1 frame
      Point3 pose1_P_landmark(measured_ref_(0) / rho, measured_ref_(1) / rho,
                              1. / rho);
      // Convert the landmark to world coordinates
      Pose3 world_pose1_camera = pose1.compose(transform);
      Point3 world_P_landmark =
          world_pose1_camera.transformFrom(pose1_P_landmark);
      // Project landmark into Pose2
      Pose3 world_pose2_camera = pose2.compose(transform);
      PinholeBase normalized_plane(world_pose2_camera);

      // Vector prj = normalized_plane.project2(world_P_landmark);
      // std::cout << prj << std::endl;

      return normalized_plane.project2(world_P_landmark) - measured_;
    } catch (CheiralityException &e) {
      std::cout << e.what() << ": Inverse Depth Landmark ["
                << DefaultKeyFormatter(this->key1()) << ","
                << DefaultKeyFormatter(this->key3()) << "]"
                << " moved behind camera " << DefaultKeyFormatter(this->key2())
                << std::endl;
      return Vector::Ones(2) * 2.0;
    }
    return (Vector(1) << 0.0).finished();
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Pose3 &pose1, const Pose3 &pose2,
                       const Pose3 &transform, const double &rho,
                       boost::optional<Matrix &> H1 = boost::none,
                       boost::optional<Matrix &> H2 = boost::none,
                       boost::optional<Matrix &> H3 = boost::none,
                       boost::optional<Matrix &> H4 = boost::none) const {
    if (H1)
      (*H1) = numericalDerivative11<Vector, Pose3>(
          boost::bind(&InvDepthFactorVariant4PPP::inverseDepthError, this, _1,
                      pose2, transform, rho),
          pose1);
    // if (H1) std::cout << "4PPP-de_dT1:" << std::endl << *H1 << std::endl;

    if (H2)
      (*H2) = numericalDerivative11<Vector, Pose3>(
          boost::bind(&InvDepthFactorVariant4PPP::inverseDepthError, this, pose1,
                      _1, transform, rho),
          pose2);
    // if (H2) std::cout << "4PPP-de_dT2:" << std::endl << *H2 << std::endl;

    if (H3)
      (*H3) = numericalDerivative11<Vector, Pose3>(
          boost::bind(&InvDepthFactorVariant4PPP::inverseDepthError, this, pose1,
                      pose2, _1, rho),
          transform);
    // if (H3) std::cout << "4PPP-de_dtran:" << std::endl << *H3 << std::endl;

    if (H4)
      (*H4) = numericalDerivative11<Vector, double>(
          boost::bind(&InvDepthFactorVariant4PPP::inverseDepthError, this, pose1,
                      pose2, transform, _1),
          rho);
    // if (H4) std::cout << "4PPP-de_drho:" << std::endl << *H4 << std::endl;

    return inverseDepthError(pose1, pose2, transform, rho);
  }

  /** return the measurement */
  const Point2 &imagePoint() const { return measured_; }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(measured_);
  }

};
}  // namespace gtsam
