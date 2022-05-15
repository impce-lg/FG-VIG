#pragma once

#include <float.h>
#include <pangolin/pangolin.h>

#include <mutex>

#include "Mapper.h"

namespace VIG {
class Mapper;

class Viewer {
 public:
  Viewer(const string &ConfigFile);
  void InitiViewer(const gtsam::Pose3 &InitFrameTnc);
  void RunViewer(const map<int, Point3D> &MapPoints,
                 const map<double, pair<Frame, map<int, Point2D>>> &mpKeyFrames,
                 const cv::Mat &CurImg);
  void GetCurrentOpenGLCameraMatrix(const gtsam::Pose3 &sCurCamTnc);
  void DrawCurrentCamera();
  void DrawKeyFrames(
      const map<double, pair<Frame, map<int, Point2D>>> &mpKeyFrames);
  void DrawMapPoints(const gtsam::Pose3 &CurTnc,
                     const map<int, Point3D> &MapPoints);
  cv::Mat DrawFrame(const map<int, Point3D> &MapPoints,
                    const map<int, Point2D> &CurFrameFeatures,
                    const cv::Mat &CurImg);
  void DrawTextInfo(const cv::Mat &im, const int nState, cv::Mat &imText);
  int NumOptimized() { return mNumOptimized; }

 private:
  // 1/fps in ms
  double mT;
  float mImageWidth, mImageHeight;
  float mCameraSize, mCameraLineWidth, mKeyFrameSize, mKeyFrameLineWidth,
      mPointSize;
  float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
  bool mbWriteVideo;
  cv::VideoWriter mVideoWriter;

  pangolin::OpenGlRenderState mS_cam;
  pangolin::View mD_cam;
  pangolin::OpenGlMatrix mTnc;

  int mNumMPs, mNumKFs, mNumOptimized, mNumTracked, mNumRejected;
  int mCurFrameId;

  bool mbFollow = false, mbShowKeyFrames = false, mbShowMapPoints = false,
       mbSaveCurMap = false;
  // std::mutex mMutexStop;
};
}  // namespace VIG
