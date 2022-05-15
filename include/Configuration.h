#pragma once

#include "VIGCommon.h"

namespace VIG {

class Configuration {
 public:
  Configuration(const string &ConfigFile);
  void SetGlobalParas();

 public:
  // input files
  string mNavFile, mIMUFile, mGNSSFile, mImgPath, mImageFile, mTrueMapfile;
  // output files
  string mNavResult, mNavError, mIMUBias, mKMLFile, mMapPoints, mVideo;
  string mNavFrame;
  bool mbWriteMapPoints;

  // IMU Configuration
  bool mbUseIMU;
  double mStartTime, mEndTime;
  gtsam::Rot3 mRic;
  Vector3d mtic;
  gtsam::Pose3 mTic;

  double mInitAttSigma, mInitPosSigma, mInitVelSigma;
  double mInitAccBiasSigma, mInitGyroBiasSigma;
  double mInitAccBetwBiasSigma, mInitGyroBetwBiasSigma;
  double mAccSigma, mGyroSigma, mAccBiasRW, mGyroBiasRW;
  double mBiasAccOmegaInt, mIntgSigma;

  double mNormG;
  Vector3d mw_coriolis;

  // GNSS Configuration
  bool mbUseGNSS;
  Vector3d mt_ig;

  // Camera Configuration
  bool mbUseCam;
  double mFps;
  int mInitWinSize;
  bool mbEqualize;
  string mMask;
  int mPubFrequency;
  int mRow, mCol;
  Matrix3d mCamK;
  // [ Cam.fc(1)   Cam.alpha*Cam.fc(1)   Cam.cc(1)
  //   0           Cam.fc(2)             Cam.cc(2)
  //   0           0                     1   ];
  Eigen::Matrix<double, 5, 1> mDistCoef;
  double mSigma_Pixel;

  // Feature Tracking Configuration
  int mMaxFtNumPerFrame;  // maximum points in the table, giving the table
                               // col number
  // before creating a new key frame of features by triangulation from the
  // feature table
  double mMinDist4Feature;
  double mFMatThreshPix;  // CV Find Fundamental Matrix Threshold
  int mNumUnDisIter;

  // Mapper Configuration
  double mFocalLength;
  double mMinKeyFrameParallax;
  int mMinCoFrames, mMinCoFeatures;
  double mDepthThreshold;

  // Optimization Configuration
  double mSmootherLag;
  int mNumFLSmootherUpdIters;

  // Viewer Configuration
  bool mbUseViewer;

  // Test params
  bool mbVSlam;
  bool mbVIO;
  double mVIOLag;
  double mTestPauseEpoch;
  int mTestFramesThreshold;
  double mLframeEpoch;
  gtsam::Rot3 mRl, mRr;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace VIG
