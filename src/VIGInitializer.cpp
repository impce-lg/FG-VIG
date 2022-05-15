#include "VIGInitializer.h"

namespace VIG {

VIGInitializer::VIGInitializer(const Configuration *pConfig)
    : Optimizer(pConfig) {
  // mdInitialGNSSs.push_back(*pGNSS);
  mpInitMapper = new Mapper(pConfig);
  mInitWinSize = pConfig->mInitWinSize;
  mK = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2());
}

void VIGInitializer::Clear() {
  // // update navstate linearized point
  // for (int NavIndex = 1; NavIndex < mNavIndex; NavIndex++) {
  //   mFLSmootherISAM2.updateLinearizationPoint(
  //       X(NavIndex), mResultValues.at<gtsam::Pose3>(X(NavIndex)));
  //   mFLSmootherISAM2.updateLinearizationPoint(
  //       V(NavIndex), mResultValues.at<gtsam::Vector3>(V(NavIndex)));
  //   mFLSmootherISAM2.updateLinearizationPoint(
  //       B(NavIndex),
  //       mResultValues.at<gtsam::imuBias::ConstantBias>(B(NavIndex)));
  // }

  mpInitMapper->mNewMapPoints.clear();
  mpInitMapper->mIdleMapPoints.clear();
}

void VIGInitializer::drPreIntegrate(const double PreEpoch,
                                    const double CurEpoch, PIIMU &drPIIMU) {
  auto LIt = mIMUs.find(PreEpoch), RIt = mIMUs.find(PreEpoch);
  for (RIt++; RIt != mIMUs.end(); LIt++, RIt++) {
    double dt = RIt->first - LIt->first;
    Vector3d MeanAcc = (LIt->second.Acc + RIt->second.Acc) * 0.5;
    Vector3d MeanGyro = (LIt->second.Gyro + RIt->second.Gyro) * 0.5;

    drPIIMU.integrateMeasurement(MeanAcc, MeanGyro, dt);
    if (RIt->first >= CurEpoch) break;
  }
}

void VIGInitializer::TriangulateSFMFt_TwoView(const double LEpoch,
                                              const double REpoch,
                                              Point3D &CurSFMPoint) {
  //! 选择两帧共有的Features
  Vector3d LUxy1 = CurSFMPoint.mpPt2DKFs.at(LEpoch).Uxy1;
  Vector3d RUxy1 = CurSFMPoint.mpPt2DKFs.at(REpoch).Uxy1;

  double InvDepth_LFrame;
  TriangulateOneFt_DualView(LUxy1.head(2), mCVTnc.at(LEpoch), RUxy1.head(2),
                            mCVTnc.at(REpoch), InvDepth_LFrame,
                            CurSFMPoint.Pos3w);

  // First Frame in Key Frames queue
  if (CurSFMPoint.InvDepth < 0 &&
      LEpoch == CurSFMPoint.mpPt2DKFs.begin()->first) {
    CurSFMPoint.InvDepth = InvDepth_LFrame;
    // CurSFMPoint.Uxy1_ref = LUxy1;

    if (CurSFMPoint.InvDepth < 0)
      CurSFMPoint.SolvedFlag = FAILED;
    else if (CurSFMPoint.InvDepth < gInvDepthThreshold)
      CurSFMPoint.SolvedFlag = OVERSIZE;
    else
      CurSFMPoint.SolvedFlag = SOLVING;
  }
}

void VIGInitializer::TriangulateSFMs_TwoFrm(const double LEpoch,
                                            const double REpoch) {
  if (LEpoch == REpoch) return;
  for (auto &mpitMP : mpInitMapper->mNewMapPoints) {
    if (mpitMP.second.InvDepth >= 0) continue;

    if (mpitMP.second.mpPt2DKFs.find(LEpoch) != mpitMP.second.mpPt2DKFs.end() &&
        mpitMP.second.mpPt2DKFs.find(REpoch) != mpitMP.second.mpPt2DKFs.end())
      TriangulateSFMFt_TwoView(LEpoch, REpoch, mpitMP.second);
  }
}

bool VIGInitializer::SolvePnP(const pair<Frame, map<int, Point2D>> &CurFrame,
                              gtsam::Pose3 &Tnc) {
  vector<cv::Point2f> vCVUxys;
  vector<cv::Point3f> vCVPos3ws;
  for (auto &mpitFt : CurFrame.second) {
    if (mpInitMapper->mNewMapPoints.find(mpitFt.first) ==
        mpInitMapper->mNewMapPoints.end())
      continue;

    auto &MP = mpInitMapper->mNewMapPoints.at(mpitFt.first);
    if (MP.InvDepth < 0) continue;

    //！
    //注意这里的2D点img_pts是去畸变归一化平面上的点, 而不是常规的像素平面的点
    // 因为下面的内参K已设置为单位阵，畸变矩阵D已设置为空
    // 2D
    Vector3d Uxy1 = mpitFt.second.Uxy1;
    vCVUxys.push_back(cv::Point2f(Uxy1(0), Uxy1(1)));
    // 3D
    vCVPos3ws.push_back(cv::Point3f(MP.Pos3w[0], MP.Pos3w[1], MP.Pos3w[2]));
  }
  //! 若第i帧中包含的sfm_f中的特征点不足10，则直接返回
  if (int(vCVUxys.size()) < 15) {
    cout << "unstable features tracking, please slowly move your device!"
         << endl;
    if (int(vCVUxys.size()) < 10) return false;
  }

  if (!SolveTncByPnP(vCVPos3ws, vCVUxys, Tnc)) return false;

  return true;
}

void VIGInitializer::SolveBg() {
  // if (mNIt == 0)
  mRbc = Eigen::Matrix3d(mTic.rotation().matrix());
  mRcb = mRbc.transpose();

  Eigen::Matrix<double, Dynamic, 3> Abg;
  Eigen::Matrix<double, Dynamic, 1> Bbg;
  Abg.setZero(3 * (mNTrackedFrms - 1), 3);
  Bbg.setZero(3 * (mNTrackedFrms - 1), 1);

  for (mFrameId = 1; mFrameId < mNTrackedFrms; mFrameId++) {
    // Residual Error
    gtsam::Rot3 FrmDeltRij =
        mdFrames[mFrameId - 1].first.Tnc.rotation().inverse() *
        mdFrames[mFrameId].first.Tnc.rotation();

    gtsam::Rot3 IMUPreDeltRij = mPreInt4Frame[mFrameId].deltaRij();
    gtsam::Rot3 errR(IMUPreDeltRij.inverse() * gtsam::Rot3(mRbc) * FrmDeltRij *
                     gtsam::Rot3(mRcb));
    gtsam::Vector3 err = gtsam::Rot3::Logmap(errR);
    Bbg.middleRows<3>(3 * (mFrameId - 1)) = err;

    // Jacobian
    // // // w.r.t phi_bc
    // gtsam::Vector3 phi2_ = gtsam::Rot3::Logmap(FrmDeltRij);
    // Eigen::Matrix3d JrInv_err = gtsam::SO3::LogmapDerivative(err);
    // Eigen::Matrix3d Jr_RbcMULTphi = gtsam::SO3::ExpmapDerivative(Rbc *
    // phi2_); Abg.block<3, 3>(3 * (mFrameId - 1), 0) =
    //     -JrInv_err * Jr_RbcMULTphi * Rbc * Skew(phi2_);

    // // w.r.t bg
    Eigen::Matrix3d JlInv_err =
        gtsam::SO3::LogmapDerivative(-err);  // Jl^-1(phi) = Jr^-1(-phi)
    Abg.block<3, 3>(3 * (mFrameId - 1), 0) =
        JlInv_err * mPreInt4Frame[mFrameId].delRdelBiasOmega();

    Matrix3d hi = Abg.middleRows<3>(3 * (mFrameId - 1)).transpose() *
                  Abg.middleRows<3>(3 * (mFrameId - 1));
    Vector3d bi = Abg.middleRows<3>(3 * (mFrameId - 1)).transpose() * err;
    Vector3d delta_bgi = hi.ldlt().solve(bi);

    // gtsam::imuBias::ConstantBias NewBias =
    //     gtsam::imuBias::ConstantBias(mBa, delta_bgi);
    // mPreInt4Frame[mFrameId].BiasCorrectedDelta(NewBias);

    cout << delta_bgi.transpose() << endl;
  }
  // 1.2 solve equation (already updated in the above loop)
  Eigen::Matrix<double, 3, 3> ATA_bg = Abg.transpose() * Abg;
  Eigen::Matrix<double, 3, 1> ATB_bg = Abg.transpose() * Bbg;
  mdBg = ATA_bg.ldlt().solve(ATB_bg);
  cout << "dBg: " << mdBg.transpose() << endl;
  mBg = mBg + mdBg;
}

void VIGInitializer::SolveStar() {
  Eigen::Matrix3d Rbc = Eigen::Matrix3d(mTic.rotation().matrix());
  Eigen::Matrix3d Rcb = Rbc.transpose();
  if (mNIt == 0) mPcb = mTic.inverse().translation().matrix();

  gtsam::imuBias::ConstantBias NewBias;
  Eigen::Matrix<double, Dynamic, 9> Astar;
  Eigen::Matrix<double, Dynamic, 1> Bstar;
  Astar.setZero(3 * (mNTrackedFrms - 2), 9);
  Bstar.setZero(3 * (mNTrackedFrms - 2), 1);
  Vector3d ba_i;
  vector<Vector3d> vBa;
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  // 2.1 Approx Scale and Gravity vector in world frame (nav frame)
  for (mFrameId = 1; mFrameId < mNTrackedFrms - 1; mFrameId++) {
    pair<Frame, map<int, Point2D>> TrackedFrm1 = mdFrames[mFrameId - 1];
    pair<Frame, map<int, Point2D>> TrackedFrm2 = mdFrames[mFrameId];
    pair<Frame, map<int, Point2D>> TrackedFrm3 = mdFrames[mFrameId + 1];
    // Delta time between frames
    double dt12 = TrackedFrm2.first.Epoch - TrackedFrm1.first.Epoch;
    double dt23 = TrackedFrm3.first.Epoch - TrackedFrm2.first.Epoch;
    // Pre-integrated measurements
    Vector3d dp12 = mPreInt4Frame[mFrameId].deltaPij();
    Vector3d dv12 = mPreInt4Frame[mFrameId].deltaVij();
    Vector3d dp23 = mPreInt4Frame[mFrameId + 1].deltaPij();
    // Matrix3d Jpba12 = mPreInt4Frame[mFrameId].delPdelBiasAcc();
    // Matrix3d Jvba12 = mPreInt4Frame[mFrameId].delVdelBiasAcc();
    // Matrix3d Jpba23 = mPreInt4Frame[mFrameId + 1].delPdelBiasAcc();
    // Pose of camera in world frame
    Vector3d pc1 = TrackedFrm1.first.Tnc.translation().matrix();
    Vector3d pc2 = TrackedFrm2.first.Tnc.translation().matrix();
    Vector3d pc3 = TrackedFrm3.first.Tnc.translation().matrix();
    Matrix3d Rc1 = TrackedFrm1.first.Tnc.rotation().matrix();
    Matrix3d Rc2 = TrackedFrm2.first.Tnc.rotation().matrix();
    Matrix3d Rc3 = TrackedFrm3.first.Tnc.rotation().matrix();

    // Vector3d L2 = pc2;
    // Vector3d R20 = pc1 + /*TrackedFrm1.first.Vnb * dt12 +*/
    //                0.5 * gG * dt12 * dt12 + Rc1 * Rcb * dp12 +
    //                (Rc1 - Rc2) * mPcb;
    // Vector3d L3 = pc3;
    // Vector3d R30 = pc2 + (/*TrackedFrm1.first.Vnb + */ gG*dt12 +
    // Rc1*Rcb*dv12 ) * dt23 +
    //                0.5 * gG * dt23 * dt23 + Rc2 * Rcb * dp23 +
    //                (Rc2 - Rc3) * mPcb;
    // Vector3d delta20 = (L2 - R20) * dt23, delta30 = (L3 - R30) * dt12,
    //          Delta0 = delta20 - delta30;
    // cout << "Delta0: " << Delta0.transpose() << endl;

    Astar.block<3, 1>(3 * (mFrameId - 1), 0) =
        (pc2 - pc1) * dt23 - (pc3 - pc2) * dt12;  // s
    // // Astar.block<3, 3>(3 * (mFrameId - 1), 3) =
    // //     Rc2 * Rcb * Jpba23 * dt12 - Rc1 * Rcb * Jpba12 * dt23 +
    // //     Rc1 * Rcb * Jvba12 * dt12 * dt23;  // ba

    Astar.block<3, 3>(3 * (mFrameId - 1), 3) =
        0.5 * I3 * dt12 * dt23 * (dt12 + dt23);  // g
    // Astar.block<3, 3>(3 * (mFrameId - 1), 6) =
    //     -(Rc1 - Rc2) * dt23 + (Rc2 - Rc3) * dt12;  // pcb

    Bstar.middleRows<3>(3 * (mFrameId - 1)) =
        // (pc1 - pc2) * dt23 - (pc2 - pc3) * dt12 +                 // s
        (Rc1 - Rc2) * mPcb * dt23 -
        (Rc2 - Rc3) * mPcb * dt12  // pbc
        // - 0.5 * gG * dt12 * dt23 * (dt12 + dt23)                 // g
        + Rc1 * Rcb * dp12 * dt23 - Rc1 * Rcb * dv12 * dt12 * dt23 -
        Rc2 * Rcb * dp23 * dt12;

    // Matrix3d A3_i = Astar.block<3, 3>(3 * (mFrameId - 1), 3);
    // Vector3d B3_i = Bstar.middleRows<3>(3 * (mFrameId - 1));
    // ba_i = A3_i.ldlt().solve(B3_i);

    // // cout << mFrameId << endl;
    // // cout << A3_i << endl;
    // // cout << B3_i.transpose() << endl;
    // cout << "ba_i" << ba_i.transpose() << endl;

    // NewBias = gtsam::imuBias::ConstantBias(ba_i, mBg);
    // mPreInt4Frame[mFrameId].BiasCorrectedDelta(NewBias);
    // mPreInt4Frame[mFrameId + 1].BiasCorrectedDelta(NewBias);
  }
  // mPreInt4Frame.rbegin()->second.BiasCorrectedDelta(NewBias);

  // 2.2 solve equation
  // cout << Astar << endl << endl;
  // cout << Bstar << endl << endl;
  Eigen::Matrix<double, 9, 9> ATA = Astar.transpose() * Astar;
  Eigen::Matrix<double, 9, 1> ATB = Astar.transpose() * Bstar;
  Eigen::Matrix<double, 9, 1> dStar = ATA.ldlt().solve(ATB);
  cout << dStar.transpose() << endl;

  mS_star = dStar(0);
  mG_w_star = dStar.segment<3>(3);
  cout << "Gstar: " << mG_w_star.transpose() << endl;
}

void VIGInitializer::SolveRefine() {
  // mRcb = mRbc.transpose();
  Eigen::Matrix3d Rbc = Eigen::Matrix3d(mTic.rotation().matrix());
  Eigen::Matrix3d Rcb = Rbc.transpose();

  Vector3d g_I = Vector3d(0., 0., 1.), g_wn = mG_w_star / mG_w_star.norm();
  Vector3d g_Ixg_wn = g_I.cross(g_wn);
  double norm_g_Ixg_wn = g_Ixg_wn.norm();
  Vector3d v_hat = g_Ixg_wn / norm_g_Ixg_wn;
  double theta = atan2(norm_g_Ixg_wn, g_I.dot(g_wn));
  Matrix3d Rwi = gtsam::Rot3::Expmap(gtsam::Vector3(v_hat * theta)).matrix();

  Eigen::Matrix<double, Dynamic, 12> A_Rf;
  Eigen::Matrix<double, Dynamic, 1> B_Rf;
  A_Rf.setZero(3 * (mNTrackedFrms - 2), 12);
  B_Rf.setZero(3 * (mNTrackedFrms - 2), 1);
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  // 2.1 Approx Scale and Gravity vector in world frame (nav frame)
  for (mFrameId = 1; mFrameId < mNTrackedFrms - 1; mFrameId++) {
    pair<Frame, map<int, Point2D>> TrackedFrm1 = mdFrames[mFrameId - 1];
    pair<Frame, map<int, Point2D>> TrackedFrm2 = mdFrames[mFrameId];
    pair<Frame, map<int, Point2D>> TrackedFrm3 = mdFrames[mFrameId + 1];
    // Delta time between frames
    double dt12 = TrackedFrm2.first.Epoch - TrackedFrm1.first.Epoch;
    double dt23 = TrackedFrm3.first.Epoch - TrackedFrm2.first.Epoch;
    // Pre-integrated measurements
    Vector3d dp12 = mPreInt4Frame[mFrameId].deltaPij();
    Vector3d dv12 = mPreInt4Frame[mFrameId].deltaVij();
    Vector3d dp23 = mPreInt4Frame[mFrameId + 1].deltaPij();
    Matrix3d Jpba12 = mPreInt4Frame[mFrameId].delPdelBiasAcc();
    Matrix3d Jvba12 = mPreInt4Frame[mFrameId].delVdelBiasAcc();
    Matrix3d Jpba23 = mPreInt4Frame[mFrameId + 1].delPdelBiasAcc();
    // Pose of camera in world frame
    Vector3d pc1 = TrackedFrm1.first.Tnc.translation().matrix();
    Vector3d pc2 = TrackedFrm2.first.Tnc.translation().matrix();
    Vector3d pc3 = TrackedFrm3.first.Tnc.translation().matrix();
    Matrix3d Rc1 = TrackedFrm1.first.Tnc.rotation().matrix();
    Matrix3d Rc2 = TrackedFrm2.first.Tnc.rotation().matrix();
    Matrix3d Rc3 = TrackedFrm3.first.Tnc.rotation().matrix();

    A_Rf.block<3, 1>(3 * (mFrameId - 1), 0) =
        (pc2 - pc1) * dt23 - (pc3 - pc2) * dt12;  // s
    A_Rf.block<3, 3>(3 * (mFrameId - 1), 3) =
        -0.5 * I3 * dt12 * dt23 * (dt12 + dt23) * Rwi * Skew(gG);  // theta
    A_Rf.block<3, 3>(3 * (mFrameId - 1), 6) =
        Rc2 * mRcb * Jpba23 * dt12 - Rc1 * mRcb * Jpba12 * dt23 +
        Rc1 * mRcb * Jvba12 * dt12 * dt23;  // ba
    // A_Rf.block<3, 3>(3 * (mFrameId - 1), 9) =
    //     -(Rc1 - Rc2) * dt23 + (Rc2 - Rc3) * dt12;  // pcb

    B_Rf.middleRows<3>(3 * (mFrameId - 1)) =
        // (pc1 - pc2) * dt23 - (pc2 - pc3) * dt12 +                 // s
        (Rc1 - Rc2) * mPcb * dt23 - (Rc2 - Rc3) * mPcb * dt12  // pbc
        - 0.5 * Rwi * gG * dt12 * dt23 * (dt12 + dt23) +       // g
        Rc1 * Rcb * dp12 * dt23 - Rc1 * Rcb * dv12 * dt12 * dt23 -
        Rc2 * Rcb * dp23 * dt12;

    // Matrix3d hi = A_Rf.middleRows<3>(3 * (mFrameId - 1)).transpose() *
    //               A_Rf.middleRows<3>(3 * (mFrameId - 1));
    // Vector3d bi = A_Rf.middleRows<3>(3 * (mFrameId - 1)).transpose() *
    //               B_Rf.middleRows<3>(3 * (mFrameId - 1));
    // Vector3d pcb_i = hi.ldlt().solve(bi);

    // cout << mFrameId << endl;
    // cout << hi << endl;
    // cout << bi.transpose() << endl;
    // cout << TrackedFrm2.first.Epoch << " | " << pcb_i.transpose() << endl;

    // Matrix3d A1 = A_Rf.block<3, 3>(3 * (mFrameId - 1), 0)/1000,
    //          A2 = A_Rf.block<3, 3>(3 * (mFrameId - 1), 3);
    // cout << "A1" << A1 << endl << "A2" << A2 << endl;
    // Vector3d B3_i = B_Rf.middleRows<3>(3 * (mFrameId - 1));
    // ba_i = A3_i.ldlt().solve(B3_i);
    // ba_i =
    // Vector3d(-0.0028723274415282116,-0.0056657829349640197,-0.0028559606030115719);

    // gtsam::imuBias::ConstantBias NewBias =
    //     gtsam::imuBias::ConstantBias(ba_i, mBg);
    // mPreInt4Frame[mFrameId].BiasCorrectedDelta(NewBias);
    // mPreInt4Frame[mFrameId + 1].BiasCorrectedDelta(NewBias);
    // cout << "ba_i" << ba_i.transpose() << endl;
  }

  // 2.2 solve equation
  // cout << endl;
  // cout << A_Rf << endl << endl;
  // cout << B_Rf << endl << endl;
  Eigen::MatrixXd A;
  A.setZero(3 * (mNTrackedFrms - 2), 6);
  A.leftCols(1) = A_Rf.leftCols(1);  // s
  A.middleCols<2>(1) = A_Rf.middleCols<2>(3);
  A.rightCols(3) = A_Rf.middleCols<3>(6);
  // cout << A << endl << endl;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
  double cond = svd.singularValues()(0) /
                svd.singularValues()(svd.singularValues().size() - 1);

  Eigen::Matrix<double, 12, 12> ATA_Rf = A_Rf.transpose() * A_Rf;
  Eigen::Matrix<double, 12, 1> ATB_Rf = A_Rf.transpose() * B_Rf;
  Eigen::Matrix<double, 12, 1> refine = ATA_Rf.ldlt().solve(ATB_Rf);

  // Eigen::Vector3d Pcb = refine.tail(3);  //
  cout << "refine: " << refine.transpose() << endl;

  mS = refine(0);

  Eigen::Vector3d dphi = refine.segment<3>(3);
  auto dRwi = gtsam::Rot3::Expmap(dphi).matrix();

  mRni = Rwi * dRwi;
  mG_ref = mRni * gG;
  cout << "mG_ref: " << mG_ref.transpose() << endl;

  mdBa = refine.segment<3>(6);
  cout << "dBa: " << mdBa.transpose() << endl;
  mBa = mBa + mdBa;

  mfGw << mCurFrame.first.Epoch << " "                                       //
       << mG_ref(0) << " " << mG_ref(1) << " " << mG_ref(2) << " "           //
       << mG_w_star(0) << " " << mG_w_star(1) << " " << mG_w_star(2) << " "  //
       << endl;

  mfScale << mCurFrame.first.Epoch << " "  //
          << mS << " "                     //
          << mS_star << " "                //
          << endl;

  mfBa << mCurFrame.first.Epoch << " "                     //
       << mBa(0) << " " << mBa(1) << " " << mBa(2) << " "  //
       << endl;

  mfCond << mCurFrame.first.Epoch << " "  //
         << cond << " "                   //
         << endl;
  // Vector3d Pt = mPcb / mPcb.norm(), Pe = Pcb / Pcb.norm();
  // Vector3d Ptxe = Pt.cross(Pe);
  // double norm_Ptxe = Ptxe.norm();
  // Vector3d v_hat_ = Ptxe / norm_Ptxe;
  // double theta_ = atan2(norm_Ptxe, Pt.dot(Pe));
  // Matrix3d Ret_ = gtsam::Rot3::Expmap(gtsam::Vector3(v_hat_ *
  // theta_)).matrix();
}

bool VIGInitializer::CheckConverge_std() {
  // calculate std
  mvEpoch.push_back(mCurFrame.first.Epoch);
  mvG_ref.push_back(mG_ref);
  mvScale.push_back(mS);
  mvBa.push_back(mBa);
  mvBg.push_back(mBg);

  // 统计最新的15帧
  if (mvEpoch.size() > 10 && mCurFrame.first.Epoch - mvEpoch[0] > 5) {
    bool bGConverge = false, bSConverge = false, bBgConverge = false,
         bBaConverge = false;

    // 依次统计std，若前一个不达标则自动结束
    // gravity
    vector<Eigen::Vector3d> vNewG_ref(mvG_ref.end() - 10, mvG_ref.end());
    auto Gstd = Get3dVectorStd(vNewG_ref);
    if (abs(Gstd(0)) < 0.02 && abs(Gstd(1)) < 0.02 && abs(Gstd(2)) < 0.02) {
      bGConverge = true;

      // ba
      vector<Eigen::Vector3d> vNewBa(mvBa.end() - 10, mvBa.end());
      auto Ba_std = Get3dVectorStd(vNewBa);
      if (abs(Ba_std(0)) < 0.015 && abs(Ba_std(1)) < 0.015 &&
          abs(Ba_std(2)) < 0.015) {
        bBaConverge = true;

        // scale
        vector<double> vNewScale(mvScale.end() - 10, mvScale.end());
        auto Sstd = Get1dVectorStd(vNewScale);
        if (abs(Sstd) < 0.05) {
          bSConverge = true;

          // bg
          vector<Eigen::Vector3d> vNewBg(mvBg.end() - 10, mvBg.end());
          auto Bg_std = Get3dVectorStd(vNewBg);
          if (abs(Bg_std(0)) < 0.001 && abs(Bg_std(1)) < 0.001 &&
              abs(Bg_std(2)) < 0.001)
            bBgConverge = true;
        }
      }
    }

    //
    return bGConverge && bSConverge && bBaConverge && bBgConverge;

  } else
    return false;
}

bool VIGInitializer::CheckConverge_db(const double CurEpoch) {
  if (CurEpoch - gStartTime >= 8) {
    bool bBgConverge = false, bBaConverge = false;

    if (abs(mdBg(0)) < 0.001 && abs(mdBg(1)) < 0.001 && abs(mdBg(2)) < 0.001)
      bBgConverge = true;

    if (abs(mdBa(0)) < 0.01 && abs(mdBa(1)) < 0.01 && abs(mdBa(2)) < 0.01)
      bBaConverge = true;

    return bBgConverge && bBaConverge;
  }
  return false;
}

bool VIGInitializer::SolveBiasesAndVel() {
  gtsam::Pose3 Tci = mTic.inverse();

  for (auto mpit : mKeyFrames) mdFrames.push_back(mpit.second);
  mNTrackedFrms = (int)mdFrames.size();

  // 1 Ascociate IMU Pre-Int to Tracked Frames
  assert(mIMUs.begin()->first <= mdFrames.front().first.Epoch &&
         mIMUs.rbegin()->first >= mdFrames.back().first.Epoch);

  gtsam::imuBias::ConstantBias CurBias;
  if (mNIt == 0) {
    CurBias = mCurPre_State.OptBias;
    mBa = CurBias.accelerometer();
    mBg = CurBias.gyroscope();
  } else
    CurBias = gtsam::imuBias::ConstantBias(mBa, mBg);

  PIIMU drPIIMU(mIMU_params, CurBias);
  for (mFrameId = 1; mFrameId <= mNTrackedFrms; mFrameId++) {
    double PreEpoch = mdFrames[mFrameId - 1].first.Epoch,
           CurEpoch = mdFrames[mFrameId].first.Epoch;

    drPreIntegrate(PreEpoch, CurEpoch, drPIIMU);

    mPreInt4Frame[mFrameId] = drPIIMU;
    drPIIMU.resetIntegration();
  }

  // 2 solve Rbc, gyro bias, and correct gyro bias for PIM
  SolveBg();
  // SolveRbc_Q();
  // SolveRbc_J();
  // SolveBg_Rbc();

  // // 4.Sovle pcb and accelerometer bias
  // SolveStar();
  // SolveRefine();
  // SolvePcb2();

  // cout
  //     << Vector3d(gtsam::Rot3(mRbc).xyz() -
  //     mTic.rotation().xyz()).transpose() *
  //            kRad2Deg
  //     << " | " << (mPcb - Tci.translation().matrix()).transpose() << endl;

  // // Acc bias is affected by gyro bias, it is not correct yet
  // CurBias = gtsam::imuBias::ConstantBias(CurBias.accelerometer(), mBg);

  // // 3 Modify the biases to Solve vel
  // gtsam::NavState NavState_Pre, NavState_Init = gtsam::NavState(
  //                                   mdFrames.front().first.Tnc *
  //                                   mTic.inverse(),
  //                                   mdFrames.front().first.Vnb);
  // for (mFrameId = 1; mFrameId <= mNTrackedFrms - 1; mFrameId++) {
  //   NavState_Pre = mPreInt4Frame[mFrameId].predict(NavState_Init, CurBias);

  //   Vector3d v = mdFrames[mFrameId].first.Vnb, v_cal = NavState_Pre.v();
  //   cout << mdFrames[mFrameId].first.Epoch << endl;
  //   cout << v.transpose() << endl;
  //   cout << v_cal.transpose() << endl << endl;

  //   NavState_Init = NavState_Pre;
  // }

  mCurPre_State.OptBias =
      gtsam::imuBias::ConstantBias(mCurPre_State.OptBias.accelerometer(), mBg);
  // mKeyFrames.clear();
  mdFrames.clear();
  mCVTnc.clear();
  mPreInt4Frame.clear();
  // mpInitMapper->ClearState();
  mNIt++;

  // return CheckConverge_std();
  // return CheckConverge_db(mCurFrame.first.Epoch);
  return true;
}

void VIGInitializer::OptimizeOverAll() {
  double PreEpoch = gStartTime, CurEpoch;
  uint64 PreNavInd = 0, CurNavInd;

  // 先内插GNSS（第一个除外）
  {
    auto mpitGNSS = mGNSSs.begin();
    for (mpitGNSS++; mpitGNSS != mGNSSs.end(); mpitGNSS++) {
      if (mpitGNSS->first > mKeyFrames.begin()->first &&
          mpitGNSS->first < mKeyFrames.rbegin()->first) {
        // 内插
        InterplKFOnGNSS(mpitGNSS->first);
      }
    }
  }
  // 内插完在设置，是比较折中的办法
  geVIGState = VIG_OK;

  /** 先把首帧（也是首个关键帧）之前的先验、GNSS优化一遍，并边缘化到首帧 **/ 
  // 先验-第一个GNSS
  {
    // 先验
    SetPrior();

    auto &FstGNSS = mGNSSs.begin()->second;
    CurEpoch = FstGNSS.Epoch;
    CurNavInd = FstGNSS.NavIndex;
    mpNavInds[CurEpoch] = CurNavInd;
    // 预积分
    PreIntegrate(PreEpoch, CurEpoch, mpPIIMUMeas);
    FstGNSS.pGNSSPIIMU = mpPIIMUMeas;
    mpPIIMUMeas = new PIIMU(mIMU_params, mCurPre_State.OptBias);
    // 插入因子
    AddGNSSFactor(FstGNSS);
    AddIMUFactor(PreNavInd, CurNavInd, FstGNSS.pGNSSPIIMU);
    // 预测并插入状态
    Predict(CurEpoch);
    FstGNSS.SetState(mCurPre_State);
    mInitValues.insert(X(CurNavInd), FstGNSS.Tnb);
    mInitValues.insert(V(CurNavInd), FstGNSS.Vnb);
    mInitValues.insert(B(CurNavInd), FstGNSS.IMUBias);
    // 善后
    PreEpoch = CurEpoch;
    PreNavInd = CurNavInd;

    Optimize();
    UpdateGNSS();
  }

  // 第一个GNSS-第一帧
  {
    auto &FstGNSS = mGNSSs.begin()->second;
    AddGNSSFactor(FstGNSS);
    mInitValues.insert(X(PreNavInd), FstGNSS.Tnb);
    mInitValues.insert(V(PreNavInd), FstGNSS.Vnb);
    mInitValues.insert(B(PreNavInd), FstGNSS.IMUBias);

    auto &FstKF = mKeyFrames.begin()->second;
    CurEpoch = FstKF.first.Epoch;
    CurNavInd = FstKF.first.NavIndex;
    mpNavInds[CurEpoch] = CurNavInd;
    // 预积分
    PreIntegrate(PreEpoch, CurEpoch, mpPIIMUMeas);
    FstKF.first.pFramePIIMU = mpPIIMUMeas;
    mpPIIMUMeas = new PIIMU(mIMU_params, mCurPre_State.OptBias);
    // 插入因子
    AddIMUFactor(PreNavInd, CurNavInd, FstKF.first.pFramePIIMU);
    // 预测并插入状态
    Predict(CurEpoch);
    FstKF.first.SetState(mCurPre_State, mTic);
    mInitValues.insert(X(CurNavInd), FstKF.first.Tnc * mTic.inverse());
    mInitValues.insert(V(CurNavInd), FstKF.first.Vnb);
    mInitValues.insert(B(CurNavInd), FstKF.first.IMUBias);

    Optimize();
    UpdateGNSS();
  }

  /** 从这里起，一切从第一帧重新开始 **/ 
  mpNavInds.clear();
  // 前面给第一帧计算了一个边缘化因子，相当于其先验
  for (auto &mpitKF : mKeyFrames) {
    auto &CurKF = mpitKF.second;
    CurEpoch = mpitKF.first;
    CurNavInd = CurKF.first.NavIndex;
    mpNavInds[CurEpoch] = CurNavInd;

    // 如果不是首帧，添加预积分因子
    if (CurEpoch > mKeyFrames.begin()->first) {
      // IMU预积分
      PreIntegrate(PreEpoch, CurEpoch, mpPIIMUMeas);
      CurKF.first.pFramePIIMU = mpPIIMUMeas;
      mpPIIMUMeas = new PIIMU(mIMU_params, mCurPre_State.OptBias);
      AddIMUFactor(PreNavInd, CurNavInd, CurKF.first.pFramePIIMU);
    }

    // 如果不是GNSS时刻内插frame，预测state
    if (!CurKF.first.bOnGNSS) {
      Predict(CurEpoch);
      CurKF.first.SetState(mCurPre_State, mTic);
    }
    // 如果是GNSS时刻，要用内插的Frame state（来自于GNSS）更新mCurState.PredNav
    else {
      AddGNSSFactor(mGNSSs.at(CurEpoch));
      mCurPre_State.PredNav =
          gtsam::NavState(CurKF.first.Tnc * mTic.inverse(), CurKF.first.Vnb);
    }
    // insert initial values
    mInitValues.insert(X(CurNavInd), CurKF.first.Tnc * mTic.inverse());
    mInitValues.insert(V(CurNavInd), CurKF.first.Vnb);
    mInitValues.insert(B(CurNavInd), mCurPre_State.OptBias);

    PreEpoch = CurEpoch;
    PreNavInd = CurNavInd;
  }

  // 上面只添加了IMU因子，这里再添加视觉观测
  AddMapperFactor();

  // 4.10 Optimize
  Optimize();

  // Update Vision
  UpdateVision();

  // 4.11 Clear Initialization
  Clear();
}

void VIGInitializer::OptimizeMonoSFM() {
  gtsam::NonlinearFactorGraph NewGraph;
  vector<ProjectionFactor::shared_ptr> vProjectionFactors;
  gtsam::Values NewInitValues;

  // 2. Add intial values
  // cout << "Before: " << endl;
  for (auto &mpitF : mKeyFrames) {
    NewInitValues.insert(X(mpitF.second.first.NavIndex),
                         mpitF.second.first.Tnc);

    // // Test
    // PrintTns2PA(mpitF.first, mpitF.second.first.Tnc * mTic.inverse());
  }

  // 3. Add factors
  // 3.1 Add prior factors
  auto Noise_FirstPose = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.001 * Vector3d(1, 1, 1), 0.001 * Vector3d(1, 1, 1))
          .finished());
  NewGraph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(mKeyFrames.begin()->second.first.NavIndex),
      mKeyFrames.begin()->second.first.Tnc, Noise_FirstPose));

  // 3.2 Add SFMPoint factors
  for (auto mpitMP = mpInitMapper->mNewMapPoints.begin();
       mpitMP != mpInitMapper->mNewMapPoints.end(); mpitMP++) {
    auto &MP = mpitMP->second;

    if (MP.SolvedFlag == SOLVING) {
      // if (MP.LMIndex == -1) 
      if(mSFMResultValues.exists(L(MP.PointId))){
        auto PointNoise =
            gtsam::noiseModel::Isotropic::Sigma(3, 100);  // 是否设为先验？
        NewGraph.add(gtsam::PriorFactor<gtsam::Point3>(L(MP.PointId), MP.Pos3w,
                                                       PointNoise));
      }

      NewInitValues.insert(L(MP.PointId), MP.Pos3w);

      for (auto mpitInfoOnCurFrame : MP.mpPt2DKFs) {
        double CurKFEpoch = mpitInfoOnCurFrame.first;

        gtsam::Point2 Uxy_measure = mpitInfoOnCurFrame.second.Uxy1.head(2);

        auto NoiseModel = gtsam::noiseModel::Isotropic::Sigma(2, mInitVSigma);
        auto factor = boost::allocate_shared<ProjectionFactor>(
            Eigen::aligned_allocator<ProjectionFactor>(), Uxy_measure,
            NoiseModel, X(mKeyFrames.at(CurKFEpoch).first.NavIndex),
            L(MP.PointId), mK);

        NewGraph.add(factor);
        vProjectionFactors.emplace_back(factor);
      }
    }
  }

  // 4. Optimize
  gtsam::LevenbergMarquardtOptimizer LMOptimizer(NewGraph, NewInitValues);
  mSFMResultValues = LMOptimizer.optimize();
  // mSFMResultValues.print();

  if (true) {
    for (auto &factor : vProjectionFactors) {
      double scale =
          CalNoiseScale(factor->unwhitenedError(mSFMResultValues).norm());
      auto Noise = gtsam::noiseModel::Isotropic::Sigma(2, scale * mInitVSigma);
      factor->updateNoiseModel(Noise);
    }

    gtsam::LevenbergMarquardtOptimizer LMOptimizer2(NewGraph, NewInitValues);
    mSFMResultValues = LMOptimizer2.optimize();
  }

  // 5. Update
  // 5.1 Update Frame Poses
  // cout << "After: " << endl;
  for (auto &mpitF : mKeyFrames) {
    int NavIndex = mpitF.second.first.NavIndex;
    // 用优化的位姿给帧序列重新赋值
    mpitF.second.first.Tnc = mSFMResultValues.at<gtsam::Pose3>(X(NavIndex));

    PrintTns2PA(mpitF.first, mpitF.second.first.Tnc * mTic.inverse());
  }

  // 5.2 Update SFM Features
  for (auto mpitMP = mpInitMapper->mNewMapPoints.begin();
       mpitMP != mpInitMapper->mNewMapPoints.end(); mpitMP++) {
    if (mpitMP->second.SolvedFlag != SOLVING) continue;

    mpitMP->second.Pos3w =
        mSFMResultValues.at<gtsam::Point3>(L(mpitMP->second.PointId));

    double StartKFEpoch = mpitMP->second.mpPt2DKFs.begin()->first;
    gtsam::Point3 Point3_StartFrame =
        mKeyFrames.at(StartKFEpoch).first.Tnc.transformTo(mpitMP->second.Pos3w);
    mpitMP->second.InvDepth = 1. / Point3_StartFrame.z();

    // cout << mpitMP->first << ": " ;
    // mpitMP->second.Pos3w.print();
  }
}

void VIGInitializer::OptimizeCurPose(pair<Frame, map<int, Point2D>> &CurFrame) {
  gtsam::NonlinearFactorGraph NewGraph;
  vector<ProjectionFactor::shared_ptr> vProjectionFactors;
  gtsam::Values NewInitValues;
  vector<pair<Frame, map<int, Point2D>>> vLRFrames;

  // 2. Add intial values
  auto PreKF = mKeyFrames.lower_bound(CurFrame.first.Epoch);
  PreKF--;
  vLRFrames.push_back(PreKF->second);
  vLRFrames.push_back(CurFrame);

  // cout << "Before: " << endl;
  for (size_t i = 0; i < vLRFrames.size(); i++) {
    NewInitValues.insert(X(vLRFrames[i].first.NavIndex),
                         vLRFrames[i].first.Tnc);

    // // Test
    // PrintTns2PA(vLRFrames[i].first.Epoch, vLRFrames[i].first.Tnc *
    // mTic.inverse());
  }

  // 3. Add factors
  // 3.1 Add prior factors
  auto Noise_FirstPose = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.001 * Vector3d(1, 1, 1), 0.001 * Vector3d(1, 1, 1))
          .finished());
  NewGraph.add(gtsam::PriorFactor<gtsam::Pose3>(
      X(vLRFrames[0].first.NavIndex), vLRFrames[0].first.Tnc, Noise_FirstPose));

  // 3.2 Add SFMPoint factors
  for (auto mpitFt : vLRFrames[1].second) {
    if (vLRFrames[0].second.find(mpitFt.first) == vLRFrames[0].second.end())
      continue;

    assert(mpInitMapper->mNewMapPoints.find(mpitFt.first) !=
           mpInitMapper->mNewMapPoints.end());

    auto &MP = mpInitMapper->mNewMapPoints.at(mpitFt.first);
    if (MP.SolvedFlag == SOLVING) {
      for (int i = 0; i < 2; i++) {
        double KFEpoch = vLRFrames[i].first.Epoch;

        gtsam::Point2 Uxy_measure =
            vLRFrames[i].second.at(mpitFt.first).Uxy1.head(2);

        auto NoiseModel = gtsam::noiseModel::Isotropic::Sigma(2, mInitVSigma);
        auto factor = boost::allocate_shared<ProjectionFactor>(
            Eigen::aligned_allocator<ProjectionFactor>(), Uxy_measure,
            NoiseModel, X(vLRFrames[i].first.NavIndex), L(MP.PointId), mK);

        NewGraph.add(factor);
        vProjectionFactors.emplace_back(factor);
      }

      NewInitValues.insert(L(MP.PointId), MP.Pos3w);

      auto PointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
      NewGraph.add(gtsam::PriorFactor<gtsam::Point3>(L(MP.PointId), MP.Pos3w,
                                                     PointNoise));
      // mCurFrame_.second.vbOptimized[itFt] = 1;
    }
  }

  // 4. Optimize
  for (int i = 0; i < 4; i++) {
    gtsam::LevenbergMarquardtOptimizer LMOptimizer(NewGraph, NewInitValues);
    mSFMResultValues = LMOptimizer.optimize();

    // mSFMResultValues.print();
    for (auto &factor : vProjectionFactors) {
      double scale =
          CalNoiseScale(factor->unwhitenedError(mSFMResultValues).norm());
      auto Noise = gtsam::noiseModel::Isotropic::Sigma(2, scale * mInitVSigma);
      factor->updateNoiseModel(Noise);
    }
  }

  // 5. Update
  // 5.1 Update Frame Poses
  // cout << "After: " << endl;
  auto PreTnc = mSFMResultValues.at<gtsam::Pose3>(X(PreKF->second.first.NavIndex));
  CurFrame.first.Tnc =
      mSFMResultValues.at<gtsam::Pose3>(X(CurFrame.first.NavIndex));

  // // Test
  // PrintTns2PA(PreKF->first, PreTnc * mTic.inverse());
  // PrintTns2PA(CurFrame.first.Epoch, CurFrame.first.Tnc * mTic.inverse());

  // NewInitValues.print("Inits:");
  // mSFMResultValues.print("Result");
}

void VIGInitializer::TriangulateMono(
    const pair<Frame, map<int, Point2D>> CurFrame) {
  double REpoch = CurFrame.first.Epoch;
  auto mpit = mKeyFrames.find(REpoch);
  mpit--;
  double LEpoch = mpit->first;

  for (auto mpitFt : CurFrame.second) {
    auto &MP = mpInitMapper->mNewMapPoints.at(mpitFt.first);
    // 如果不是初始值-1000，或者虽然是初值但可能是上一步算出的错误逆深度
    if (MP.InvDepth != -1000.0 || MP.SolvedFlag != RAW) continue;

    if (MP.mpPt2DKFs.find(LEpoch) == MP.mpPt2DKFs.end() ||
        MP.mpPt2DKFs.find(REpoch) == MP.mpPt2DKFs.end())
      continue;

    //! 选择两帧共有的Features
    Vector3d LUxy1 = MP.mpPt2DKFs.at(LEpoch).Uxy1;
    Vector3d RUxy1 = MP.mpPt2DKFs.at(REpoch).Uxy1;

    double InvDepth_LFrame;
    TriangulateOneFt_DualView(LUxy1.head(2), mKeyFrames.at(LEpoch).first.Tnc,
                              RUxy1.head(2), mKeyFrames.at(REpoch).first.Tnc,
                              InvDepth_LFrame, MP.Pos3w);

    // First Frame in Key Frames queue
    if (MP.InvDepth < 0 && LEpoch == MP.mpPt2DKFs.begin()->first) {
      MP.InvDepth = InvDepth_LFrame;
      // MP.Uxy1_ref = LUxy1;

      if (MP.InvDepth < 0)
        MP.SolvedFlag = FAILED;
      else if (MP.InvDepth < gInvDepthThreshold)
        MP.SolvedFlag = OVERSIZE;
      else
        MP.SolvedFlag = SOLVING;
    }
  }
}

void VIGInitializer::TrackMonoInit() {
  if (mKeyFrames.size() == 1) {
    return;

  } else {
    auto KFit = mKeyFrames.rbegin();
    mCurFrame = KFit->second;
    KFit++;
    mPreFrame = KFit->second;

    if (mKeyFrames.size() == 2) {
      vector<pair<Vector3d, Vector3d>> vprCorresUxy1Pair;
      if (GetMatch(mPreFrame, mCurFrame, vprCorresUxy1Pair)) {

        SolveTrlByFMat(vprCorresUxy1Pair, mTrl);

        gtsam::Pose3 Trl_imu =
            mKeyFrames.at(mCurFrame.first.Epoch).first.Tnc.inverse() *
            mKeyFrames.at(mPreFrame.first.Epoch).first.Tnc;

        double s0 = Trl_imu.z() / mTrl.z();
        mTrl = gtsam::Pose3(
            mTrl.rotation(),
            gtsam::Point3(mTrl.x() * s0, mTrl.y() * s0, mTrl.z() * s0));

        // mKeyFrames.at(mPreFrame.first.Epoch).first.Tnc = gtsam::Pose3();
        mKeyFrames.at(mCurFrame.first.Epoch).first.Tnc =
            mKeyFrames.at(mPreFrame.first.Epoch).first.Tnc * mTrl.inverse();
        
        // Test
        PrintTns2PA(mPreFrame.first.Epoch, mPreFrame.first.Tnc * mTic.inverse());
        PrintTns2PA(mCurFrame.first.Epoch, mCurFrame.first.Tnc * mTic.inverse());

        //
        for (auto mpit : mKeyFrames) mpInitMapper->AddMapPoints(mpit.second);
        TriangulateMono(mCurFrame);
        OptimizeMonoSFM();
      }

    } else {
      //
      gtsam::Pose3 Tnc = mPreFrame.first.Tnc * mTrl.inverse();
      if (!SolvePnP(mCurFrame, Tnc))
        cout << "Issue with PnP when initialization!";

      mCurFrame.first.Tnc = Tnc;
      OptimizeCurPose(mCurFrame);
      mKeyFrames.at(mCurFrame.first.Epoch).first.Tnc = mCurFrame.first.Tnc;

      //
      mpInitMapper->AddMapPoints(mCurFrame);
      TriangulateMono(mCurFrame);
      OptimizeMonoSFM();

      //
      mTrl =
          mKeyFrames.rbegin()->second.first.Tnc.inverse() * mPreFrame.first.Tnc;
    }
  }
}

bool VIGInitializer::Initialize_Loosely() {
  // 1. Constuct SFM
  // if (!PrepareSFM()) return false;

  // // 2. Optimize SFM
  // // if (!OptimizeSFM_InvDepth()) return false;
  // if (!OptimizeSFM_Pos3w()) return false;

  // 3. Solvc Gyro and Acc Bias
  if (SolveBiasesAndVel()) {
    OptimizeOverAll();

    return true;

  } else
    return false;
}

}  // namespace VIG
