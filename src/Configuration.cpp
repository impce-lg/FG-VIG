#include "Configuration.h"

namespace VIG {

Configuration::Configuration(const string &ConfigFile) {
  string config_file = ConfigFile;
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  
  if (!fsSettings.isOpened()) {
    cerr << "Error open yaml config file!" << endl;
    exit(-1);
  }

  // input file names
  fsSettings["NavFile"] >> mNavFile;
  fsSettings["IMUfile"] >> mIMUFile;
  fsSettings["GNSSfile"] >> mGNSSFile;
  fsSettings["ImgPath"] >> mImgPath;
  fsSettings["ImgTimeFile"] >> mImageFile;
  fsSettings["TrueMapfile"] >> mTrueMapfile;
  // output file names
  fsSettings["NavResult"] >> mNavResult;
  fsSettings["NavError"] >> mNavError;
  fsSettings["IMUBias"] >> mIMUBias;
  fsSettings["KMLFile"] >> mKMLFile;
  fsSettings["MapPoints"] >> mMapPoints;
  fsSettings["bWriteMapPoints"] >> mbWriteMapPoints;
  fsSettings["Video"] >> mVideo;
  fsSettings["NavFrame"] >> mNavFrame;
  assert(mNavFrame == "NED" || mNavFrame == "ENU");

  // IMU Configuration
  fsSettings["UseIMU"] >> mbUseIMU;
  mStartTime = fsSettings["IMUStartTime"];
  mEndTime = fsSettings["IMUEndTime"];

  // Extrinsic parameter between IMU and body.
  cv::Mat i_R_c, i_t_c;
  Matrix3d egRic;
  fsSettings["i_R_c"] >> i_R_c;
  cv::cv2eigen(i_R_c, egRic);
  mRic = gtsam::Rot3(egRic);
  fsSettings["i_t_c"] >> i_t_c;
  cv::cv2eigen(i_t_c, mtic);
  mTic = gtsam::Pose3(mRic, gtsam::Point3(mtic));

  // initial navigation info
  mInitAttSigma = fsSettings["InitAttSigma"];
  mInitAttSigma *= kDeg2Rad;
  mInitPosSigma = fsSettings["InitPosSigma"];
  mInitVelSigma = fsSettings["InitVelSigma"];

  mInitAccBiasSigma = fsSettings["InitAccBiasSigma"];
  mInitGyroBiasSigma = fsSettings["InitGyroBiasSigma"];
  mInitAccBetwBiasSigma = fsSettings["InitAccBetwBiasSigma"];
  mInitGyroBetwBiasSigma = fsSettings["InitGyroBetwBiasSigma"];

  mAccSigma = fsSettings["AccSigma"];
  mGyroSigma = fsSettings["GyroSigma"];
  mAccBiasRW = fsSettings["AccBiasRW"];
  mGyroBiasRW = fsSettings["GyroBiasRW"];
  mIntgSigma = fsSettings["IntgSigma"];
  mBiasAccOmegaInt = fsSettings["BiasAccOmegaInt"];

  mNormG = fsSettings["Norm_g"];

  // GNSS Configuration
  fsSettings["UseGNSS"] >> mbUseGNSS;
  cv::Mat t_ig;
  fsSettings["t_ig"] >> t_ig;
  cv::cv2eigen(t_ig, mt_ig);

  // Camera Configuration
  fsSettings["UseCam"] >> mbUseCam;
  mFps = fsSettings["Cam.fps"];
  mInitWinSize = fsSettings["InitWindowSize"];  // (int)mFps + 1;
  fsSettings["Equalize"] >> mbEqualize;
  fsSettings["Mask"] >> mMask;
  fsSettings["PubFrequency"] >> mPubFrequency;
  mRow = fsSettings["Cam.nRows"];
  mCol = fsSettings["Cam.nCols"];

  double fx = fsSettings["Cam.fcx"];
  double fy = fsSettings["Cam.fcy"];
  double cx = fsSettings["Cam.ccx"];
  double cy = fsSettings["Cam.ccy"];
  double alpha = fsSettings["Cam.alpha"];
  mCamK.setIdentity();
  mCamK(0, 0) = fx;
  mCamK(0, 1) = alpha * fx;
  mCamK(0, 2) = cx;
  mCamK(1, 1) = fy;
  mCamK(1, 2) = cy;

  cv::Mat DistCoef;
  fsSettings["Cam.DistCoef"] >> DistCoef;
  cv::cv2eigen(DistCoef, mDistCoef);
  mSigma_Pixel = fsSettings["Sigma_Pixel"];

  // Feature Tracker Configuration
  mMaxFtNumPerFrame = fsSettings["MaxFeatureNumPerFrame"];
  mMinDist4Feature = fsSettings["MinDist4Feature"];
  mFMatThreshPix = fsSettings["FMatThreshPix"];
  mNumUnDisIter = fsSettings["NumUnDisIter"];

  // Feature Manager Configuration
  mFocalLength = fsSettings["FocalLength"];
  mMinKeyFrameParallax = fsSettings["MinKeyFrameParallax"];
  mMinCoFrames = fsSettings["MinCoFrames"];
  mMinCoFeatures = fsSettings["MinCoFeatures"];
  mDepthThreshold = fsSettings["DepthThreshold"];

  // Optimization Configuration
  mSmootherLag = fsSettings["SmootherLag"];
  mNumFLSmootherUpdIters = fsSettings["NumFLSmootherUpdIters"];

  // Viewer Configuration
  fsSettings["UseViewer"] >> mbUseViewer;

  // Test params
  fsSettings["VSlam"] >> mbVSlam;
  fsSettings["VIO"] >> mbVIO;
  mVIOLag = fsSettings["VIOLag"];
  mTestPauseEpoch = fsSettings["TestPauseEpoch"];
  mTestFramesThreshold = fsSettings["TestFramesThreshold"];

  cv::Mat cRl, cRr;
  Eigen::Vector3d eRl, eRr;
  fsSettings["RPYl"] >> cRl;
  fsSettings["RPYr"] >> cRr;
  cv::cv2eigen(cRl* kDeg2Rad, eRl);
  cv::cv2eigen(cRr* kDeg2Rad, eRr);
  mRl = gtsam::Rot3::RzRyRx(eRl)*mRic;
  mRr = gtsam::Rot3::RzRyRx(eRr)*mRic;
}

void Configuration::SetGlobalParas() {
  gStartTime = mStartTime;
  gEndTime = mEndTime;

  // GNSS
  gbUseGNSS = mbUseGNSS;

  // Camera
  gbUseCam = mbUseCam;
  gCamK = mCamK;
  gDistCoef = mDistCoef;
  gFps = mFps;
  gMinCoFrames = mMinCoFrames;
  gMinCoFeatures = mMinCoFeatures;

  gFocalLength = mFocalLength;
  gMinKeyFrameParallax = mMinKeyFrameParallax / gFocalLength;
  gInvDepthThreshold = 1. / mDepthThreshold;
}

}  // namespace VIG
