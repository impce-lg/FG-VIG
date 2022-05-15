#pragma once

#include "Configuration.h"
#include "Mapper.h"
#include "Tracker.h"
#include "VIGInitializer.h"
#include "Optimizer.h"
#include "SimSystem.h"
#include "VIGCommon.h"
#include "Viewer.h"

namespace VIG {
class Configuration;
class Tracker;
class Mapper;
class SimSystem;
class Viewer;

class TestSimVIO {
 public:
  TestSimVIO(const Configuration *pConfig, const string &ConfigFile);
  void ReadNav(const string &strNavFile);
  void InitializeVIO(SimSystem *pSimSystem);
  bool CalcPixelNoise_InvDepth(const gtsam::Pose3 &RefKFTnc,
                                        const gtsam::Pose3 &CurKFTnc,
                                        const double &Rho,
                                        const gtsam::Point2 &Measured_ref,
                                        const gtsam::Point2 &Measured,
                                        const int CurFeatId,
                                        const double &CurKFEpoch);
  void RunVIO(const double CurIMUEpoch);
  void ConfigVision(const pair<Frame, map<int, Point2D>> &prsCurFrame);

  void AddIMUFactor(const int NavInd, PIIMU *pPreIntgIMUs);
  void AddVisualFactor();
  void ConstructGraph();
  void Optimize();
  void UpdateResults_FLSmoother();
  void UpdateVision();

 private:
  string mifImg_;
  int mNTrue;

  map<double, gtsam::Pose3> mTrueTni;
  map<double, gtsam::NavState> mTrueNav;

  // IMU
  IMU *mpIMU_;
  deque<IMU> mdIMUs;
  boost::shared_ptr<PIIMUParams> mIMU_params_;
  PIIMU *mpPIIMUMeas_;

  // Camera
  IsotropicNoise_ptr mNoiseModel_Vision_;
  pair<Frame, map<int, Point2D>> mCurFrame_;
  map<double, pair<Frame, map<int, Point2D>>> mFrames_, mCachedFrames_;
  map<double, KeyFrame> mKeyFrames_;

  Tracker *mpTracker_;
  Mapper *mpMapper_;
  VIGInitializer *mpVIGInitializer_;
  Optimizer *mpOptimizer_;
  Viewer *mpViewer_;

  // Solver
  DiagonalNoise_ptr mInitPoseSigma_;
  IsotropicNoise_ptr mInitVelSigma_;
  DiagonalNoise_ptr mInitBiasSigma_;
  DiagonalNoise_ptr mNoiseModel_BetwBias_;

  double mWinTs_, mWinTe_;
  static int mNavIndex_;
  static int mLMIndex_;
  gtsam::imuBias::ConstantBias mOptCurBias_;
  gtsam::NavState mPredNextNavState_, mOptCurNavState_;
  gtsam::Pose3 mPredTni_, mOptCurTni_;

  double mCurNavEpoch_, mSmootherLag_, mStartEpoch_;
  gtsam::NonlinearFactorGraph *mpGraph_ = new gtsam::NonlinearFactorGraph();
  gtsam::Values mInitValues_, mFLResultValues_, mResultValues_;

  // Test
  double mTestPauseEpoch_;
  gtsam::Rot3 mRl_, mRr_;
};

}  // namespace VIG