#pragma once

#include "Configuration.h"
#include "Mapper.h"
#include "Tracker.h"
#include "Optimizer.h"
#include "VIGCommon.h"
#include "Viewer.h"

namespace VIG {
class Configuration;
class Tracker;
class Mapper;
class Viewer;

class TestVSlam {
 public:
  TestVSlam(const Configuration *pConfig,
                  const double FirstNavStateIMUTime,
                  const string &ConfigFile);
  void ReadNav(const string &strNavFile);
  void SetEstimator(const Configuration *pConfig);
  bool CalcPixelNoise_InvDepth(const gtsam::Pose3 &RefKFTnc,
                                        const gtsam::Pose3 &CurKFTnc,
                                        const double &Rho,
                                        const gtsam::Point2 &Measured_ref,
                                        const gtsam::Point2 &Measured,
                                        const int CurFeatId,
                                        const double &CurKFEpoch);
  void UpdateXVBTimeStamps(double CurPoseTime);
  void RunVSlam(const Configuration *pConfig, const IMU *pIMU,
                Optimizer *pOptimizer);
  void ConfigVision(const gtsam::Pose3 &CurTnc,
                    const pair<Frame, map<int, Point2D>> &prsCurFrame);
  bool InitializeVSlam();
  void AddVisualFactor_Init();
  void AddSynVisFactor();
  void AddBetwFactorDepend();
  void Optimize();
  void UpdateResults_FLSmoother();
  void UpdateSynVision();

 private:
  int mNTrue;

  map<double, gtsam::Pose3> mTrueTni;
  map<double, gtsam::NavState> mTrueNav;

  DiagonalNoise_ptr mInitPoseSigma_;

  // Camera
  int mInitWinSize_;
  bool mbInitPub_ = 0, mbInitFeature_ = 0;
  double mInitSigma_Vision_;
  IsotropicNoise_ptr mNoiseModel_Vision_;
  pair<Frame, map<int, Point2D>> mCurFrame_;
  map<double, pair<Frame, map<int, Point2D>>> mFrames_;
  map<double, KeyFrame> mKeyFrames_;

  Tracker *mpTracker_;
  Mapper *mpMapper_;
  Viewer *mpViewer_;

  // Solver
  eVIGState geVIGState_;
  vector<double> mvPosesTimeStamps_;
  static int mPoseIndex_;
  static int mLMIndex_;
  gtsam::Pose3 mPredTni_, mOptCurTni_;

  gtsam::ISAM2Params mIsamParams_;
  double mSmootherLag_;
  gtsam::FixedLagSmoother::KeyTimestampMap mtdmpNewKey_TimeStamps_;
  gtsam::IncrementalFixedLagSmoother mFLSmootherISAM2_;
  gtsam::Values mNewInitialValues_;
  gtsam::NonlinearFactorGraph *mpNewGraph_ = new gtsam::NonlinearFactorGraph();
  gtsam::Values mFLResultValues_, mResultValues_;
};

}  // namespace VIG