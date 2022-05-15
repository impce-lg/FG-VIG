#include "Mapper.h"

namespace VIG {
Mapper::Mapper(const Configuration *pConfig) {
  ClearState();
}

void Mapper::ClearState() {
  for (auto mpitNewMP : mNewMapPoints) {
    if (mIdleMapPoints.find(mpitNewMP.first) == mIdleMapPoints.end()) {
      mpitNewMP.second.SolvedFlag = SOLVED;
      mIdleMapPoints.insert(mpitNewMP);
    } else
      break;
  }

  mNewMapPoints.clear();
  // mIdleMapPoints.clear();
}

void Mapper::AddMapPoints(const pair<Frame, map<int, Point2D>> &CurFrame) {
  double Epoch = CurFrame.first.Epoch;
  int FrameId = CurFrame.first.FrameId;

  for (auto mpitFt : CurFrame.second) {
    Pt2DInfo InfoCurKF;
    InfoCurKF.FrameId = FrameId;
    InfoCurKF.Uuv = mpitFt.second.Uuv;
    InfoCurKF.Uxy1 = mpitFt.second.Uxy1;

    int PointId = mpitFt.first;

    if (mNewMapPoints.find(PointId) == mNewMapPoints.end()) {
      Point3D CurMapPoint = Point3D(PointId, InfoCurKF.Uxy1);
      CurMapPoint.mpPt2DKFs[Epoch] = InfoCurKF;

      mNewMapPoints.insert(make_pair(PointId, CurMapPoint));

    } else {
      mNewMapPoints.at(PointId).mpPt2DKFs[Epoch] = InfoCurKF;
      // mpit->second.mpPt2DKFs[Epoch] = InfoCurKF;
    }
  }
}

void Mapper::FormMapPoints(
    const map<double, pair<Frame, map<int, Point2D>>> &mpFrames) {
  mNewMapPoints.clear();

  for (auto mpit : mpFrames) {
    double Epoch = mpit.first;
    int FrameId = mpit.second.first.FrameId;

    for (auto mpitFt : mpit.second.second) {
      Pt2DInfo InfoCurKF;
      InfoCurKF.FrameId = FrameId;
      InfoCurKF.Uuv = mpitFt.second.Uuv;
      InfoCurKF.Uxy1 = mpitFt.second.Uxy1;

      int PointId = mpitFt.first;

      if (mNewMapPoints.find(PointId) == mNewMapPoints.end()) {
        Point3D CurMapPoint = Point3D(PointId, InfoCurKF.Uxy1);
        CurMapPoint.mpPt2DKFs.at(Epoch) = InfoCurKF;

        mNewMapPoints.insert(make_pair(PointId, CurMapPoint));

      } else {
        mNewMapPoints.at(PointId).mpPt2DKFs[Epoch] = InfoCurKF;
      }
    }
  }
}

int Mapper::CountRefCurFrame(const double Epoch) {
  int Nref = 0;

  for (auto &mpit : mNewMapPoints) {
    if (mpit.second.mpPt2DKFs.begin()->first == Epoch) {
      Nref++;
    }
  }

  return Nref;
}

int Mapper::CountCurFrame(const double Epoch) {
  int N = 0;

  for (auto &mpit : mNewMapPoints) {
    if (mpit.second.mpPt2DKFs.find(Epoch) != mpit.second.mpPt2DKFs.end()) {
      N++;
    }
  }

  return N;
}

int Mapper::CountMatch(const double LEpoch, const double REpoch) {
  int NCorres = 0;

  for (auto &mpit : mNewMapPoints) {
    if (mpit.second.mpPt2DKFs.find(LEpoch) != mpit.second.mpPt2DKFs.end() &&
        mpit.second.mpPt2DKFs.find(REpoch) != mpit.second.mpPt2DKFs.end()) {
      NCorres++;
    }
  }

  return NCorres;
}

vector<pair<Vector3d, Vector3d>> Mapper::GetMatch(
    const double LEpoch, const double REpoch) {
  vector<pair<Vector3d, Vector3d>> vprCorresUxy1Pair;

  for (auto &mpit : mNewMapPoints) {
    if (mpit.second.mpPt2DKFs.begin()->first <= LEpoch &&
        mpit.second.mpPt2DKFs.rbegin()->first >= REpoch) {
      Vector3d lp = Vector3d::Zero(), rp = Vector3d::Zero();

      lp = mpit.second.mpPt2DKFs.at(LEpoch).Uxy1;
      rp = mpit.second.mpPt2DKFs.at(REpoch).Uxy1;

      vprCorresUxy1Pair.push_back(make_pair(lp, rp));
    }
  }

  return vprCorresUxy1Pair;
}

// Trl:  Point from l frame to r frame
bool Mapper::CalRelativeTrl(const double LEpoch,
                                    const double REpoch,
                                    gtsam::Pose3 &Trl) {
  //！在滑窗内寻找与最新的关键帧共视点超过20(像素点)的关键帧
  // find previous frame which contians enough correspondance and parallex with
  // newest frame

  vector<pair<Vector3d, Vector3d>> vprCorresUxy1Pair;
  vprCorresUxy1Pair = GetMatch(LEpoch, REpoch);

  //! 共视的Features应该大于20
  if (vprCorresUxy1Pair.size() > 20) {
    double sum_parallax = 0;
    // double average_parallax;
    //! 求取匹配的特征点在图像上的视差和(归一化平面上)

    for (size_t j = 0; j < vprCorresUxy1Pair.size(); j++) {
      Vector2d pts_0(vprCorresUxy1Pair[j].first(0),
                     vprCorresUxy1Pair[j].first(1));
      Vector2d pts_1(vprCorresUxy1Pair[j].second(0),
                     vprCorresUxy1Pair[j].second(1));
      double parallax = (pts_0 - pts_1).norm();
      sum_parallax = sum_parallax + parallax;
    }

    // //! 求取所有匹配的特征点的平均视差
    // average_parallax = 1.0 * sum_parallax / (int)vprCorresUxy1Pair.size();
    //! 视差大于一定阈值，并且能够有效地求解出变换矩阵
    if (SolveTrlByFMat(vprCorresUxy1Pair, Trl)) {
      // l = i;
      return true;
    }
  }

  return false;
}

// 详解见：https://blog.csdn.net/OORRANNGGE/article/details/90054028
//        4.5.1 检测相机运动
bool Mapper::CheckEnableTriangulate(
    const Point3D &CurPoint,
    const map<double, pair<Frame, map<int, Point2D>>> &mpFrames) {
  if (CurPoint.SolvedFlag == SOLVING) return true;

  auto StartKF = CurPoint.mpPt2DKFs.begin();
  auto &Tnc_s = mpFrames.at(StartKF->first).first.Tnc;

  //* 获取第一个相机坐标系中特征点的向量方向在世界坐标系的投影
  Eigen::Vector3d feature_direction = StartKF->second.Uxy1;
  feature_direction = feature_direction / feature_direction.norm();
  feature_direction = Tnc_s.rotation() * feature_direction;

  //* 参考msckf-vio
  //* https://github.com/KumarRobotics/msckf_vio
  auto EndKF = CurPoint.mpPt2DKFs.rbegin();
  auto &Tnc_e = mpFrames.at(EndKF->first).first.Tnc;

  Eigen::Vector3d translation = Tnc_e.translation() - Tnc_s.translation();
  double parallel_translation = translation.transpose() * feature_direction;
  Eigen::Vector3d orthogonal_translation =
      translation - parallel_translation * feature_direction;
      
  if (orthogonal_translation.norm() >
      0.2)  // TODO 三角化指标 初步设置为0.2,后续需要测试调整
    return true;
  else
    return false;
}

bool Mapper::CheckMotionStatus(const Point3D &CurPoint) {
  const auto &InfoKFs = CurPoint.mpPt2DKFs;

  std::vector<double> vVx, vVy;
  for (auto LF = InfoKFs.begin(); LF != (InfoKFs.end()); LF++) {
    auto RF = LF;
    RF++;
    if (RF == InfoKFs.end()) break;

    double dt = RF->first - LF->first;
    vVx.emplace_back((LF->second.Uxy1.x() - RF->second.Uxy1.x()) / dt);
    vVy.emplace_back((LF->second.Uxy1.y() - RF->second.Uxy1.y()) / dt);
  }

  int nV = vVx.size();
  if (nV < gMinCoFrames - 1) {
    return false;
  }

  double x_mean = std::accumulate(vVx.begin(), vVx.end(), 0.0) / nV;
  double y_mean = std::accumulate(vVy.begin(), vVy.end(), 0.0) / nV;
  double x_std = 0.0, y_std = 0.0;
  for (int i = 0; i < nV; i++) {
    x_std += pow(vVx[i] - x_mean, 2.0) / nV;
    y_std += pow(vVy[i] - y_mean, 2.0) / nV;
  }
  x_std = sqrt(x_std);
  y_std = sqrt(y_std);

  // 相机坐标系xyz-右下前，一般y轴变化比x小
  // 80,60分别是x，y轴的像素变化率
  if (x_std > (80 / gFocalLength) || y_std > (60 / gFocalLength)) {
    return false;
  }

  for (int i = 0; i < nV; i++) {
    if (vVx[i] > x_mean + (x_std * 3) || vVx[i] < x_mean - (x_std * 3)) {
      return false;
    }
    if (vVy[i] > y_mean + (y_std * 3) || vVy[i] < y_mean - (y_std * 3)) {
      return false;
    }
  }
  return true;
}

void Mapper::TriangulateMP_MultView(
    Point3D &CurPoint,
    const map<double, pair<Frame, map<int, Point2D>>> &mpFrames) {

  if ((int)CurPoint.mpPt2DKFs.size() < gMinCoFrames) return;

  double StartKFEpoch = CurPoint.mpPt2DKFs.begin()->first;

  MatrixXd svd_A(2 * (int)CurPoint.mpPt2DKFs.size(), 4);
  int svd_idx = 0;
  for (auto &mpitInfoPerKF : CurPoint.mpPt2DKFs) {
    double CurKFEpoch = mpitInfoPerKF.first;

    gtsam::Pose3 Tcx_c0 = mpFrames.at(CurKFEpoch).first.Tnc.inverse() *
                          mpFrames.at(StartKFEpoch).first.Tnc;
    Eigen::Matrix<double, 4, 4> Tcx_c0_mt = Tcx_c0.matrix();

    Vector3d XthUxy1 = mpitInfoPerKF.second.Uxy1.normalized();
    svd_A.row(svd_idx++) =
        XthUxy1[0] * Tcx_c0_mt.row(2) - XthUxy1[2] * Tcx_c0_mt.row(0);
    svd_A.row(svd_idx++) =
        XthUxy1[1] * Tcx_c0_mt.row(2) - XthUxy1[2] * Tcx_c0_mt.row(1);
  }

  assert(svd_idx == svd_A.rows());
  Vector4d svd_V =
      JacobiSVD<MatrixXd>(svd_A, ComputeThinV).matrixV().rightCols<1>();

  double InvDepth = svd_V[3] / svd_V[2];

  // if(CurPoint.SolvedFlag == RAW)
  // {
  if (InvDepth < 0) {
    CurPoint.SolvedFlag = FAILED;
    return;
  } else if (InvDepth < gInvDepthThreshold)
    CurPoint.SolvedFlag = OVERSIZE;
  else if (CurPoint.SolvedFlag == RAW)
    CurPoint.SolvedFlag = SOLVING;

  CurPoint.InvDepth = InvDepth;
  // }
  // else if(CurPoint.SolvedFlag == SOLVING){
  //   if (InvDepth >= gInvDepthThreshold) CurPoint.InvDepth = InvDepth;
  // }
  CurPoint.Pos3w = mpFrames.at(StartKFEpoch).first.Tnc.transformFrom(
          1. / CurPoint.InvDepth * CurPoint.Uxy1_ref);
}

void Mapper::TriangulateOptMPs_MultView(
    const map<double, pair<Frame, map<int, Point2D>>> &mpFrames) {
  for (auto &mpit : mNewMapPoints) {

    if ((int)mpit.second.mpPt2DKFs.size() < gMinCoFrames) continue;

    double StartKFEpoch = mpit.second.mpPt2DKFs.begin()->first;

    MatrixXd svd_A(2 * (int)mpit.second.mpPt2DKFs.size(), 4);
    int svd_idx = 0;
    for (auto &mpitInfoPerKF : mpit.second.mpPt2DKFs) {
      double CurKFEpoch = mpitInfoPerKF.first;

      gtsam::Pose3 Tcx_c0 = mpFrames.at(CurKFEpoch).first.Tnc.inverse() *
                            mpFrames.at(StartKFEpoch).first.Tnc;
      Eigen::Matrix<double, 4, 4> Tcx_c0_mt = Tcx_c0.matrix();

      Vector3d XthUxy1 = mpitInfoPerKF.second.Uxy1.normalized();
      svd_A.row(svd_idx++) =
          XthUxy1[0] * Tcx_c0_mt.row(2) - XthUxy1[2] * Tcx_c0_mt.row(0);
      svd_A.row(svd_idx++) =
          XthUxy1[1] * Tcx_c0_mt.row(2) - XthUxy1[2] * Tcx_c0_mt.row(1);
    }

    assert(svd_idx == svd_A.rows());
    Vector4d svd_V =
        JacobiSVD<MatrixXd>(svd_A, ComputeThinV).matrixV().rightCols<1>();

    double InvDepth = svd_V[3] / svd_V[2];

    // if(mpit.second.SolvedFlag == RAW)
    // {
      if (InvDepth < 0) {
        mpit.second.SolvedFlag = FAILED;
        continue;
      } else if (InvDepth < gInvDepthThreshold)
        mpit.second.SolvedFlag = OVERSIZE;
      else if (mpit.second.SolvedFlag == RAW)
        mpit.second.SolvedFlag = SOLVING;

      mpit.second.InvDepth = InvDepth;
      // }
      // else if(mpit.second.SolvedFlag == SOLVING){
      //   if (InvDepth >= gInvDepthThreshold) mpit.second.InvDepth = InvDepth;
      // }

      // // Test
      // Vector3d P1(mpit.second.Pos3w), P2, dP;
      // cout << mpit.second.PointId << " " << P1.transpose() << " | ";

      mpit.second.Pos3w = mpFrames.at(StartKFEpoch).first.Tnc.transformFrom(
          1. / mpit.second.InvDepth * mpit.second.Uxy1_ref);

      // P2 = mpit.second.Pos3w, dP = P2 - P1;
      // cout << P2.transpose() << " | " << dP.transpose();
      // cout << endl;
  }
}

bool Mapper::SolvePnP4AllKeyFrames(
    deque<pair<Frame, map<int, Point2D>>> &dprTrackedFrames,
    const map<double, pair<Frame, map<int, Point2D>>> &mpFrames,
    const gtsam::Pose3 *arCVTw_fx, map<int, Vector3d> &mpSFMFeatIds_Pos3ws) {
  auto ditprsTrackedFrame = dprTrackedFrames.begin();
  auto mpitKeyFrame = mpFrames.begin();
  map<int, Vector3d>::iterator mpitSFMFeatIds_Pos3w;
  for (int i = 0; ditprsTrackedFrame != dprTrackedFrames.end();
       ditprsTrackedFrame++) {
    //！如果帧头和滑窗内关键帧的帧头相同，则直接读取位姿即可
    // provide initial guess
    cv::Mat r_w2cx, rvec_w2cx, t_w2cx, D, tmp_r;
    if ((ditprsTrackedFrame->first.Epoch) == mpitKeyFrame->first) {
      //！一次性转换到IMU坐标系下
      // ditprsTrackedFrame->second.bIsKeyFrame = true;
      ditprsTrackedFrame->first.Tnc = arCVTw_fx[i];
      i++;
      mpitKeyFrame++;
      continue;
    }
    if ((ditprsTrackedFrame->first.Epoch) > mpitKeyFrame->first) {
      i++;
      mpitKeyFrame++;
    }

    //! 将滑窗内第i帧的变换矩阵当做初始值
    gtsam::Pose3 Tfx_w = arCVTw_fx[i].inverse();
    cv::eigen2cv(Tfx_w.rotation().matrix(), tmp_r);
    cv::Rodrigues(tmp_r, rvec_w2cx);
    cv::eigen2cv(Vector3d(Tfx_w.translation().matrix()), t_w2cx);

    //! 如果这部分位姿不作为关键帧的化，求解出来就没有意义啊
    vector<cv::Point3f> vCVPos3ws;
    vector<cv::Point2f> vCVUxys;

    //! 遍历该帧的所有Features
    for (auto mpitFt : ditprsTrackedFrame->second) {
      int PointId = mpitFt.first;

      mpitSFMFeatIds_Pos3w = mpSFMFeatIds_Pos3ws.find(PointId);
      if (mpitSFMFeatIds_Pos3w != mpSFMFeatIds_Pos3ws.end()) {
        Vector3d world_pts = mpitSFMFeatIds_Pos3w->second;
        cv::Point3f CVPos3w(world_pts(0), world_pts(1), world_pts(2));
        vCVPos3ws.push_back(CVPos3w);

        //！
        //注意这里的2D点img_pts是去畸变归一化平面上的点,而不是常规的像素平面的点
        // 因为下面的内参K已设置为单位阵，畸变矩阵D已设置为空
        Vector2d Uxy = mpitFt.second.Uxy1.head(2);
        cv::Point2f CVUxy(Uxy(0), Uxy(1));
        vCVUxys.push_back(CVUxy);
      }
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (vCVPos3ws.size() < 6) {
      cout << "vCVPos3ws size " << vCVPos3ws.size() << endl;
      cout << "Not enough points for solve pnp !" << endl;
      return false;
    }
    if (!cv::solvePnP(vCVPos3ws, vCVUxys, K, D, rvec_w2cx, t_w2cx, 1)) {
      cout << "solve pnp fail!" << endl;
      return false;
    }
    //! PnP求解出的位姿要取逆
    cv::Rodrigues(rvec_w2cx, r_w2cx);
    MatrixXd Rpnp_w2cx;
    cv::cv2eigen(r_w2cx, Rpnp_w2cx);
    MatrixXd Tpnp_w2cx;
    cv::cv2eigen(t_w2cx, Tpnp_w2cx);

    gtsam::Rot3 Rot_w2cx = gtsam::Rot3(Rpnp_w2cx);
    gtsam::Point3 Trans_w2cx = gtsam::Point3(Tpnp_w2cx);
    gtsam::Pose3 Tcx_w = gtsam::Pose3(Rot_w2cx, Trans_w2cx);

    ditprsTrackedFrame->first.Tnc = Tcx_w.inverse();
  }
  return true;
}

bool Mapper::SolvePnP4CurFrameInKFQue(
    const pair<Frame, map<int, Point2D>> &prCurTrackedFrame,
    gtsam::Pose3 &CurTnc) {
  vector<cv::Point2f> vCVUxys;
  vector<cv::Point3f> vCVPos3ws;

  for (auto mpitFt : prCurTrackedFrame.second) {
    int PointId = mpitFt.first;
    auto mpit = mNewMapPoints.find(PointId);

    if (mpit->second.InvDepth < gInvDepthThreshold) continue;

    if (mpit != mNewMapPoints.end()) {
      //！
      //注意这里的2D点img_pts是去畸变归一化平面上的点,而不是常规的像素平面的点
      // 因为下面的内参K已设置为单位阵，畸变矩阵D已设置为空
      Vector2d Uxy = mpitFt.second.Uxy1.head(2);
      cv::Point2f CVUxy(Uxy(0), Uxy(1));
      vCVUxys.push_back(CVUxy);
      cv::Point3f CVPos3w(mpit->second.Pos3w[0], mpit->second.Pos3w[1],
                            mpit->second.Pos3w[2]);
      vCVPos3ws.push_back(CVPos3w);
    }
  }

  //! 若第i帧中包含的sfm_f中的特征点不足10，则直接返回
  if (vCVUxys.size() < 15) {
    cout << "unstable features tracking, please slowly move you device!"
         << endl;
    if (vCVUxys.size() < 10) return false;
  }

  if (!SolveTncByPnP(vCVPos3ws, vCVUxys, CurTnc)) return false;

  return true;
}

void Mapper::SetPointsPos(
    gtsam::Values &ResultValues,
    map<double, pair<Frame, map<int, Point2D>>> &mpKeyFrames) {
  // Update 3D Points in World frame
  for (auto &mpit : mNewMapPoints) {
    if (mpit.second.SolvedFlag != SOLVING) continue;

    // marginalized
    if (!ResultValues.exists(L(mpit.second.PointId))) {
      // mpit.second.SolvedFlag = OK_SOLVED;
      continue;
    }

    // cout << mpit.second.PointId << "  " << mpit.second.Pos3w.transpose();

    // set the point3 in world frame
    mpit.second.InvDepth = ResultValues.at<double>(L(mpit.second.PointId));
    if ((mpit.second.InvDepth < 0.)) mpit.second.InvDepth = 1. / gInitDepth;

    mpit.second.Pos3w =
        mpKeyFrames[mpit.second.mpPt2DKFs.begin()->first]
            .first.Tnc.transformFrom(1. / mpit.second.InvDepth *
                                     mpit.second.Uxy1_ref);

    // cout << "  " << mpit.second.Pos3w.transpose();
    // cout << endl;
  }
}

void Mapper::MoveWin(const double newStart) {
  for (auto mpit = mNewMapPoints.begin(), mpitNext = mNewMapPoints.begin();
       mpit != mNewMapPoints.end(); mpit = mpitNext) {
    mpitNext++;

    bool bMarg = false;
    while ((newStart - mpit->second.mpPt2DKFs.begin()->first) > 1e-7) {
      mpit->second.mpPt2DKFs.erase(mpit->second.mpPt2DKFs.begin());
      bMarg = true;
    }

    if (mpit->second.mpPt2DKFs.size() < 2) {
      if (mpit->second.SolvedFlag == SOLVING)
        mIdleMapPoints.insert(*mpit);
      mNewMapPoints.erase(mpit);
      continue;
    } else if (bMarg)
      mpit->second.Uxy1_ref =
          mpit->second.mpPt2DKFs.begin()->second.Uxy1;
  }
}

void Mapper::RemoveFailures() {
  for (auto mpit = mNewMapPoints.begin(), mpitNext = mNewMapPoints.begin();
       mpit != mNewMapPoints.end(); mpit = mpitNext) {
    mpitNext++;
    if (mpit->second.SolvedFlag == FAILED) mNewMapPoints.erase(mpit);
  }
}

double Mapper::compensatedParallax2(const Point3D &lit) {
  // check the second last frame is keyframe or not
  // Parallax betwwen seconde last frame and third last frame
  double ans = 0;
  auto mit = lit.mpPt2DKFs.rbegin();
  mit++;
  Vector3d p_2th = mit->second.Uxy1;
  mit++;
  Vector3d p_3th = mit->second.Uxy1;

  double u_2th = p_2th(0);
  double v_2th = p_2th(1);

  Vector3d p_3th_comp = p_3th;
  double dep_3th = p_3th(2);
  double u_3th = p_3th(0) / dep_3th;
  double v_3th = p_3th(1) / dep_3th;
  double du = u_3th - u_2th, dv = v_3th - v_2th;

  double dep_3th_comp = p_3th_comp(2);
  double u_3th_comp = p_3th_comp(0) / dep_3th_comp;
  double v_3th_comp = p_3th_comp(1) / dep_3th_comp;
  double du_comp = u_3th_comp - u_2th, dv_comp = v_3th_comp - v_2th;

  ans = max(
      ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

  return ans;
}

}  // namespace VIG
