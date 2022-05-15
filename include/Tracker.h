#pragma once

#include <float.h>

#include "Configuration.h"

namespace VIG {
class Configuration;

class Tracker {
 public:
  struct FrameFeatures {
    vector<int> vPointIds;
    vector<Vector2d> vDuvs;
    vector<Vector2d> vUuvs;
    vector<Vector3d> vUxy1s;
    vector<bool> vbOptimized;
    vector<int> vTrackNums;
  };

  Tracker(const Configuration *pConfig);
  cv::Mat GetCurImage() const { return mCurImage.clone(); }
  pair<Frame, FrameFeatures> GetCurFrameFeatures() const {
    return msCurFeatures;
  }

  bool IsInBorder(const cv::Point2f &pt);
  template <typename T>
  void ReduceVector(vector<T> &v, vector<uchar> status);
  void ReduceByStatus(const vector<uchar> &Status);
  void CVFeat2V2dFeat(const vector<cv::Point2f> &vCV, vector<Vector2d> &vV2d);
  void RejectWithFundamentalMat();
  void SetMask();
  void SetMask2(const int MaskWidth);
  void AddFeatures();

  void Distortion(const Vector2d &p_u, Vector2d &d_u);
  void UndistorteFeatures();
  bool UpdateIds(const unsigned int i);
  pair<Frame, map<int, Point2D>> Track(const double CurEpoch, const int ImgID);

 public:
  int mCol, mRow;
  bool mbEqualize;
  int mMaskWidth;
  cv::Mat mRawMask, mMask;
  double mMinDist4Feature = 30.0;
  double mFMatThreshPix;
  int mNumUnDisIter;

  // file
  string mImgPath;

  pair<Frame, FrameFeatures> msCurFeatures;

  cv::Mat mPreImage, mCurImage;
  vector<cv::Point2f> mvPreCV2fs, mvCurCV2fs, mvNewCV2fs;
  static int mCurPointID;

 private:
  int mMaxFtNumPerFrame;  // maximum points in the table, giving the table
                               // col number
};

}  // namespace VIG
