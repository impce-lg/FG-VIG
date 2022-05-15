#pragma once

#include "Optimizer.h"

namespace VIG {
class jPreintegratedIMUs;

class VIGInitializer : public Optimizer {
 public:
  VIGInitializer(const Configuration *pConfig);
  void Clear();
  void drPreIntegrate(const double PreEpoch, const double CurEpoch,
                      PIIMU &drPIIMU);

  // optimize initial VIG loosely, including bias
  void TriangulateSFMFt_TwoView(const double LEpoch,
                                const double REpoch,
                                Point3D &CurSFMPoint);
  void TriangulateSFMs_TwoFrm(const double LEpoch,
                              const double REpoch);
  bool SolvePnP(const pair<Frame, map<int, Point2D>> &CurFrame,
                gtsam::Pose3 &Tnc);

  void SolveBg();
  void SolveStar();
  void SolveRefine();
  bool SolveBiasesAndVel();
  
  bool CheckConverge_db(const double CurEpoch);
  bool CheckConverge_std();

  void OptimizeOverAll();
  void OptimizeMonoSFM();
  void OptimizeCurPose(pair<Frame, map<int, Point2D>> &CurFrame);
  void TriangulateMono(const pair<Frame, map<int, Point2D>> CurFrame);
  void TrackMonoInit();
  bool IsKFsEnough() {
    return mKeyFrames.rbegin()->first - mKeyFrames.begin()->first >=
           mSmootherLag;
  }
  bool Initialize_Loosely();

 private:
  gtsam::Cal3_S2::shared_ptr mK;
  int mInitWinSize;
  pair<Frame, map<int, Point2D>> mPreFrame, mCurFrame;
  Mapper *mpInitMapper;

  gtsam::Pose3 mTrl;
  map<double, gtsam::Pose3> mCVTnc;  // , mCVTnc_unOpt;
  int mFrameId;
  map<int, jPreintegratedIMUs> mPreInt4Frame;
  gtsam::Values mSFMResultValues;

  deque<pair<Frame, map<int, Point2D>>> mdFrames;
  int mNTrackedFrms;
  double mS_star, mS;
  Eigen::Vector3d mG_w_star, mG_ref;
  Eigen::Vector3d mBg, mdBg, mBa, mdBa;
  Eigen::Matrix3d mRbc, mRcb, mRni;
  Eigen::Vector3d mPcb;

  vector<double> mvEpoch;
  vector<Eigen::Vector3d> mvG_ref;
  vector<double> mvScale;
  // mfCond;
  vector<Eigen::Vector3d> mvBa, mvBg;

  int mInitLMIndex = -1;
  int mNIt = 0;
  ofstream mfGw, mfScale, mfCond, mfBa, mfBg, mfKFPose, mfGNSSvIGPose;
};

}  // namespace VIG
