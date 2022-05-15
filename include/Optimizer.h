#pragma once

#include <gtsam/navigation/GPSFactorLA.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "Mapper.h"
#include "jInvDepthFactorVariant4.h"
#include "jBatchFLSmoother.h"

namespace VIG {
class Configuration;
class Mapper;
class jPreintegratedIMUs;
// class jBatchFLSmoother;

typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                       gtsam::Cal3_S2>
    ProjectionFactor;
typedef gtsam::InvDepthFactorVariant4c InvDepthFactor;
typedef gtsam::noiseModel::Diagonal::shared_ptr DiagonalNoise_ptr;
typedef gtsam::noiseModel::Isotropic::shared_ptr IsotropicNoise_ptr;

class Optimizer {
 public:
  Optimizer(const Configuration *pConfig);
  void SetInitParas(const Configuration *pConfig);
  void SetIMUParas(const Configuration *pConfig);
  void SetCamParas(const Configuration *pConfig);
  void SetPrior();
  void PushIMU(const IMU &sIMU);

  // Mapper *GetMapper() const { return mpMapper; }
  map<double, pair<Frame, map<int, Point2D>>> GetFrames() const {
    return mFrames;
  }
  map<int, Point3D> GetMapPoints() const {
    map<int, Point3D> AllMPs = mpWinMapper->mIdleMapPoints;
    AllMPs.insert(mpWinMapper->mNewMapPoints.begin(),
                  mpWinMapper->mNewMapPoints.end());
    return AllMPs;
  }
  map<double, GNSSSol> const GetGNSSs() { return mGNSSs; }
  map<double, pair<Frame, map<int, Point2D>>> GetKeyFrames() const {
    auto AllKeyFrames = mMargFrames;
    AllKeyFrames.insert(mKeyFrames.begin(), mKeyFrames.end());
    return AllKeyFrames;
  }
  gtsam::imuBias::ConstantBias GetCurBias() const { return mCurPre_State.OptBias; }
  double CalNoiseScale(const double error);
  void PreIntegrate(const double PreEpoch, const double CurEpoch,
                    PIIMU *pPIIMU);
  void Predict(double CurEpoch);
  void ConfigGNSS(const int NavIndex, GNSSSol &sGNSSSol);
  void ConfigCurFrame(const int NavIndex,
                      pair<Frame, map<int, Point2D>> &CurFrame);
  bool CheckNewKeyFrame(const double CurEpoch);
  void InterplKFOnGNSS(const double GNSSEpoch);
  bool LMOptimizePoint(Point3D &SubPoint);
  bool LMOptimizePointAndCheck(Point3D &CurPoint);
  void SelectNewPoints();

  void AddIMUFactor(const uint64 PreNavInd, const uint64 CurNavInd,
                    PIIMU *pPreIntgIMUs);
  void AddGNSSFactor(const GNSSSol &InGNSSSol);
  // Used for Syn type
  void AddCurKFFactor(const pair<Frame, map<int, Point2D>> &CurFrame);
  void AddMapperFactor();

  void UpdateGNSS4Solver();
  void UpdateVision4Solver();
  void Optimize();
  void MarginalizeIG();
  gtsam::KeyVector GetMargKeys();
  void UpdateNavInd_Results();
  void ProcessMarg();

  void UpdateGNSS();
  void UpdateVision();
  void SaveNavState(const double &NavEpoch, const uint64 &NavInd);
  void SaveTail();
  void SaveTail(const map<int, Eigen::Vector3d> &mpTrueMap);
  // // Test
  // void PrintCurFactorGraph(gtsam::Values &CurLinearizationPoint);

 public:
  gtsam::Pose3 mInitTni;
  gtsam::Vector3 mInitVel_i2n;
  DiagonalNoise_ptr mInitPoseSigma;
  IsotropicNoise_ptr mInitVelSigma;
  DiagonalNoise_ptr mInitBiasSigma;

  // IMU Settings
  boost::shared_ptr<PIIMUParams> mIMU_params;
  DiagonalNoise_ptr mNoiseModel_BetwBias;
  // boost::shared_ptr<gtsam::PreintegrationParams> mIMU_params;
  PIIMU *mpPIIMUMeas;
  gtsam::Vector6 mInitSigma6_BetwBias;

  // GNSS Settings
  gtsam::Vector3 mInitSigma3_GNSSpos;
  DiagonalNoise_ptr mNoiseModel_GNSSpos;
  gtsam::Point3 mt_ig;

  // Camera params
  double mInitVSigma;
  gtsam::Pose3 mTic;

  IsotropicNoise_ptr mNoiseModel_KF, mNoiseModel_KF4G;

  // Solver
  double mSmootherLag, mMargEpoch;
  uint64 mMargNavInd; 
  ofstream mofNavState, mofNavError, mofIMUBias, mofMapPoints;
  bool mbWriteMapPoints;

 protected:
  uint64 mPreNavInd, mCurNavInd;
  map<double, IMU> mIMUs;
  map<double, GNSSSol> mGNSSs;
  // bool mbRightNearGNSS = false;
  // pair<Frame, map<int, Point2D>> mPreFrame, mCurFrame;
  map<double, pair<Frame, map<int, Point2D>>> mFrames, mKeyFrames, mMargFrames;
  // 分别表示所有点、窗口内的点
  Mapper *mpMargMapper, *mpWinMapper;

  // Solver
  SingleEpochState mCurPre_State;

  vector<InvDepthFactor::shared_ptr> mvInvDepthFactors;
  gtsam::KeyVector mMargKeys;
  gtsam::NonlinearFactorGraph mGraph, mMargFactors;
  gtsam::Values mInitValues, mFLResultValues, mResultValues;
  jBatchFLSmoother mBFLSmoother;
  map<int, gtsam::Matrix> mResultCovs;

  friend class TestSimVIO;
  friend class TestSynVIO;

  // Test
  map<double, uint64> mpNavInds, mMargNavInds;
  map<double, Eigen::Matrix<double, 9, 1>> mpNavTrue;
};

}  // namespace VIG
