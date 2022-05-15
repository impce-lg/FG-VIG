#include "Tracker.h"

namespace VIG {

int Tracker::mCurPointID = 0;  // start from 0

Tracker::Tracker(const Configuration *pConfig) {
  mRow = pConfig->mRow;
  mCol = pConfig->mCol;
  mbEqualize = pConfig->mbEqualize;
  mRawMask = cv::imread(pConfig->mMask, 0);

  mMinDist4Feature = pConfig->mMinDist4Feature;
  mFMatThreshPix = pConfig->mFMatThreshPix;
  mNumUnDisIter = pConfig->mNumUnDisIter;

  mMaxFtNumPerFrame = pConfig->mMaxFtNumPerFrame;

  mImgPath = pConfig->mImgPath;
}

bool Tracker::IsInBorder(const cv::Point2f &pt) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < mCol - BORDER_SIZE &&
         BORDER_SIZE <= img_y && img_y < mRow - BORDER_SIZE;
}

template <typename T>
void Tracker::ReduceVector(vector<T> &v, vector<uchar> Status) {
  int j = 0;
  for (size_t i = 0; i < v.size(); i++)
    if (Status[i]) v[j++] = v[i];
  v.resize(j);
}

void Tracker::ReduceByStatus(const vector<uchar> &Status) {
  ReduceVector(mvPreCV2fs, Status);
  ReduceVector(mvCurCV2fs, Status);
  ReduceVector(msCurFeatures.second.vPointIds, Status);
  ReduceVector(msCurFeatures.second.vTrackNums, Status);
}

void Tracker::CVFeat2V2dFeat(const vector<cv::Point2f> &vCV,
                             vector<Vector2d> &vV2d) {
  vV2d.clear();
  for (size_t i = 0; i < vCV.size(); i++) {
    vV2d.push_back(Vector2d(vCV[i].x, vCV[i].y));
  }
  vV2d.resize(vCV.size());
}

void Tracker::RejectWithFundamentalMat() {
  if (mvCurCV2fs.size() >= 8) {
    vector<uchar> Status;
    unsigned int Num = mvPreCV2fs.size();
    vector<cv::Point2f> UnPreFts(Num), UnCurFts(Num);
    vector<cv::Point2f> PreNewUVs(Num), CurNewUVs(Num);
    for (unsigned int i = 0; i < Num; i++) {
      Eigen::Vector3d tmp_p;
      LiftProjective(Vector2d(mvPreCV2fs[i].x, mvPreCV2fs[i].y), mNumUnDisIter,
                     tmp_p);
      UnPreFts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
      tmp_p.x() = gFocalLength * tmp_p.x() / tmp_p.z() + mCol / 2.0;
      tmp_p.y() = gFocalLength * tmp_p.y() / tmp_p.z() + mRow / 2.0;
      PreNewUVs[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

      LiftProjective(Vector2d(mvCurCV2fs[i].x, mvCurCV2fs[i].y), mNumUnDisIter,
                     tmp_p);
      UnCurFts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
      tmp_p.x() = gFocalLength * tmp_p.x() / tmp_p.z() + mCol / 2.0;
      tmp_p.y() = gFocalLength * tmp_p.y() / tmp_p.z() + mRow / 2.0;
      CurNewUVs[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
    }
    cv::findFundamentalMat(PreNewUVs, CurNewUVs, cv::FM_RANSAC, mFMatThreshPix,
                           0.99, Status);

    ReduceByStatus(Status);
  }
}

void Tracker::SetMask() {
  if (!mRawMask.empty())
    mMask = mRawMask.clone();
  else
    mMask = cv::Mat(mRow, mCol, CV_8UC1, cv::Scalar(255));
  if (mMask.empty()) cout << "Mask is empty " << endl;
  if (mMask.type() != CV_8UC1) cout << "Mask type wrong " << endl;

  // prefer to keep features that are tracked for long time
  vector<pair<int, pair<cv::Point2f, int>>> vprTrNs_prPtsIds;

  for (unsigned int i = 0; i < mvCurCV2fs.size(); i++)
    vprTrNs_prPtsIds.push_back(
        make_pair(msCurFeatures.second.vTrackNums[i],
                  make_pair(mvCurCV2fs[i], msCurFeatures.second.vPointIds[i])));

  //! 因为刚开始的时候mTrackNums的大小和Max_Features是相等的
  sort(vprTrNs_prPtsIds.begin(), vprTrNs_prPtsIds.end(),
       [](const pair<int, pair<cv::Point2f, int>> &a,
          const pair<int, pair<cv::Point2f, int>> &b) {
         return a.first > b.first;
       });

  //! 清除当前帧的所有Features，id及跟踪次数
  mvCurCV2fs.clear();
  msCurFeatures.second.vPointIds.clear();
  msCurFeatures.second.vTrackNums.clear();

  //! 将mask中以Features为圆心的，MIN_DIST为半径的区域全部置0，
  // 在后面就不在这些区域中选取强角点了。
  for (auto &itprTrN_prPtId : vprTrNs_prPtsIds) {
    if (mMask.at<uchar>(itprTrN_prPtId.second.first) == 255) {
      mvCurCV2fs.push_back(itprTrN_prPtId.second.first);
      msCurFeatures.second.vPointIds.push_back(itprTrN_prPtId.second.second);
      msCurFeatures.second.vTrackNums.push_back(itprTrN_prPtId.first);
      cv::circle(mMask, itprTrN_prPtId.second.first, mMinDist4Feature, 0, -1);
    }
  }
}

void Tracker::SetMask2(const int MaskWidth) {
  cv::Rect Roi;
  GetRoi(mCol, mRow, MaskWidth, Roi);

  mMask = cv::Mat::zeros(mRow, mCol, CV_8UC1);
  mMask(Roi).setTo(255);
}

void Tracker::AddFeatures() {
  for (auto &uv : mvNewCV2fs) {
    mvCurCV2fs.push_back(uv);
    msCurFeatures.second.vPointIds.push_back(-1);
    msCurFeatures.second.vTrackNums.push_back(1);
  }
}

void Tracker::UndistorteFeatures() {
  msCurFeatures.second.vDuvs.clear();
  msCurFeatures.second.vUxy1s.clear();
  msCurFeatures.second.vUuvs.clear();
  // cv::undistortPoints(mvCurCV2fs, un_pts, K, cv::Mat());
  for (unsigned int i = 0; i < mvCurCV2fs.size(); i++) {
    Vector2d Duv(mvCurCV2fs[i].x, mvCurCV2fs[i].y);
    msCurFeatures.second.vDuvs.push_back(Duv);

    Vector3d Uxy1;
    LiftProjective(Duv, mNumUnDisIter, Uxy1);
    msCurFeatures.second.vUxy1s.push_back(Uxy1);

    Vector3d Uuv1 = gCamK * Uxy1;
    msCurFeatures.second.vUuvs.push_back(Uuv1.head(2));
  }
  msCurFeatures.second.vDuvs.resize(mvCurCV2fs.size());
  msCurFeatures.second.vUxy1s.resize(mvCurCV2fs.size());
  msCurFeatures.second.vUuvs.resize(mvCurCV2fs.size());
}

bool Tracker::UpdateIds(const unsigned int i) {
  if (i < msCurFeatures.second.vPointIds.size()) {
    if (msCurFeatures.second.vPointIds[i] == -1)
      msCurFeatures.second.vPointIds[i] = mCurPointID++;
    return true;
  } else
    return false;
}

pair<Frame, map<int, Point2D>> Tracker::Track(const double CurEpoch,
                                              const int ImgID) {
  msCurFeatures.first.FrameId = ImgID;
  msCurFeatures.first.Epoch = CurEpoch;

  // Read Current Image
  cv::Mat InitCurIamge = cv::imread(mImgPath + to_string(ImgID) + ".png", 0);
  if (InitCurIamge.empty()) {
    cerr << "Current Image is empty!";
    exit(1);
  }

  cv::Mat CurImg;
  if (mbEqualize) {
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(InitCurIamge, CurImg);
  } else
    CurImg = InitCurIamge;

  if (mCurImage.empty()) {
    mPreImage = mCurImage = CurImg;
  } else {
    mCurImage = CurImg;
  }

  // tracking all the points given their previous image coordinates and
  // predicted image coordinates
  if (mvPreCV2fs.size() > 0) {
    mvCurCV2fs.clear();

    vector<uchar> Status;
    vector<float> Error;
    // // uni-directional optical flow
    // cv::calcOpticalFlowPyrLK(mPreImage, mCurImage, mvPreCV2fs,
    // mvCurCV2fs, Status, Error, cv::Size(21, 21), 3);
    /***** Start: bi-directional optical flow ****/
    // 先用一层金字塔跟踪
    cv::calcOpticalFlowPyrLK(mPreImage, mCurImage, mvPreCV2fs,
                             mvCurCV2fs, Status, Error, cv::Size(21, 21),
                             1);

    // 一层跟踪的点数太少，换成3层
    int succ_num = 0;
    for (size_t i = 0; i < Status.size(); i++) {
      if (Status[i]) succ_num++;
    }
    if (succ_num < 10)
      cv::calcOpticalFlowPyrLK(mPreImage, mCurImage, mvPreCV2fs,
                               mvCurCV2fs, Status, Error, cv::Size(21, 21),
                               3);

    // 用1层金字塔反向跟踪
    vector<uchar> ReverseStatus;
    vector<cv::Point2f> ReverseCVFeatures = mvPreCV2fs;
    cv::calcOpticalFlowPyrLK(
        mCurImage, mPreImage, mvCurCV2fs, ReverseCVFeatures, ReverseStatus,
        Error, cv::Size(21, 21), 1,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    // 只有当正反向跟踪都成功时才算成功
    for (size_t i = 0; i < Status.size(); i++) {
      if (Status[i] && ReverseStatus[i] &&
          CalCVDistance(mvPreCV2fs[i], ReverseCVFeatures[i]) <= 0.5) {
        Status[i] = 1;
      } else
        Status[i] = 0;
    }
    /***** End: bi-directional optical flow ****/

    // 看看跟踪的新点有没有超出边界
    for (size_t i = 0; i < mvCurCV2fs.size(); i++)
      if (Status[i] && !IsInBorder(mvCurCV2fs[i])) Status[i] = 0;

    ReduceByStatus(Status);
    // 用对极几何剔除粗差
    RejectWithFundamentalMat();
    for (auto &TrackNum : msCurFeatures.second.vTrackNums) TrackNum++;
  }

  // 去掉靠近图片边缘的, 并根据已有点设置其周围的提取禁区
  // TODO: 这一步不能合并到上上行的括号里，原因尚不清楚
  SetMask();
  // mMaskWidth = 13;
  // SetMask2(mMaskWidth);
  int MaxCnt = mMaxFtNumPerFrame - (int)mvCurCV2fs.size();
  if (MaxCnt > 0) {
    cv::goodFeaturesToTrack(mCurImage, mvNewCV2fs, MaxCnt, 0.01,
                            mMinDist4Feature, mMask);

    // cv::Size WinSize = cv::Size(5, 5);
    // cv::Size ZeroZone = cv::Size(-1, -1);
    // cv::TermCriteria Criteria = cv::TermCriteria(CV_TERMCRIT_EPS +
    // CV_TERMCRIT_ITER, 40, 0.001); cv::cornerSubPix(mCurImage,
    // mvNewCV2fs, WinSize, ZeroZone, Criteria);
  } else
    mvNewCV2fs.clear();
  AddFeatures();

  mPreImage = mCurImage;
  mvPreCV2fs = mvCurCV2fs;
  UndistorteFeatures();

  // Update Feature Ids
  for (unsigned int i = 0;; i++) {
    bool completed = false;
    completed |= UpdateIds(i);
    if (!completed) break;
  }

  // Get frames
  pair<Frame, map<int, Point2D>> prCurFeatures;
  prCurFeatures.first = msCurFeatures.first;
  // Gather nice features( trackednum > 1 )
  for (unsigned i = 0; i < msCurFeatures.second.vPointIds.size(); i++) {
    if (msCurFeatures.second.vTrackNums[i] > 1) {
      int FeatId = msCurFeatures.second.vPointIds[i];
      Point2D sFeature;

      sFeature.Duv = msCurFeatures.second.vDuvs[i];
      sFeature.Uuv = msCurFeatures.second.vUuvs[i];
      sFeature.Uxy1 = msCurFeatures.second.vUxy1s[i];
      sFeature.TrackNum = msCurFeatures.second.vTrackNums[i];
      prCurFeatures.second[FeatId] = sFeature;
    }
  }

  return prCurFeatures;
}

}  // namespace VIG
