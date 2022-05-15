#include "TestSimVIO.h"
#include <gtsam/navigation/GPSFactor.h>

namespace VIG {

int TestSimVIO::mNavIndex_ = -1;    // start from 0
int TestSimVIO::mLMIndex_ = -1;  // start from 0

TestSimVIO::TestSimVIO(const Configuration *pConfig,
                       const string &ConfigFile) {

  // Nav true
  mNTrue = 0;
  ReadNav(pConfig->mNavFile);

  // IMU
  ifstream ifIMU_;
  ifIMU_.open(pConfig->mIMUFile, ios_base::in);
  IMU *pIMU_ = new IMU(ifIMU_);
  mdIMUs.push_back(*pIMU_);
  while (!ifIMU_.eof()) {
    pIMU_->MoveOn(ifIMU_);
    mdIMUs.push_back(*pIMU_);

    if (pIMU_->CurData.T <= pIMU_->PreData.T) {
      cerr << "error imu epoch at " << pIMU_->PreData.T << endl;
      exit(-1);
    }
  }

  // Vision
  mifImg_ = pConfig->mImgPath;
  mpTracker_ =
      new Tracker(pConfig, gStartTime);
  mpMapper_ = new Mapper(pConfig);

  // mpViewer_ = new Viewer(ConfigFile);
  // mpViewer_->InitiViewer(gInitTni);
  
  // Test
  mTestPauseEpoch_ = pConfig->mTestPauseEpoch;
  mRl_ = pConfig->mRl;
  mRr_ = pConfig->mRr;
}

void TestSimVIO::ReadNav(const string &strNavFile) {
  ifstream ifNav;
  ifNav.open(strNavFile, ios_base::in);

  string strLine;
  double t, E, N, U, ve, vn, vu, roll, pitch, yaw;

  while (!ifNav.eof()) {
    getline(ifNav, strLine);

    if (!strLine.empty()) {
      stringstream ss(strLine);
      ss >> t;
      ss >> E;
      ss >> N;
      ss >> U;
      ss >> ve;
      ss >> vn;
      ss >> vu;
      ss >> roll;
      ss >> pitch;
      ss >> yaw;

      Vector3d rpy_rad = Vector3d(roll, pitch, yaw) * kDeg2Rad;
      mTrueTni[t] =
          gtsam::Pose3(gtsam::Rot3::RzRyRx(rpy_rad), gtsam::Point3(E, N, U));

      mTrueNav[t] = gtsam::NavState(mTrueTni[t], gtsam::Vector3(ve, vn, vu));
    }
  }
}

void TestSimVIO::InitializeVIO(SimSystem *pSimSystem) {
  // Optimizer
  mpVIGInitializer_ = pSimSystem->mpVIGInitializer;
  mpOptimizer_ = pSimSystem->mpOptimizer;
  mSmootherLag_ = pSimSystem->mpOptimizer->mSmootherLag;

  mInitPoseSigma_ = mpVIGInitializer_->mInitPoseSigma;
  mInitVelSigma_ = mpVIGInitializer_->mInitVelSigma;
  mInitBiasSigma_ = mpVIGInitializer_->mInitBiasSigma;
  mNoiseModel_BetwBias_ = mpVIGInitializer_->mNoiseModel_BetwBias;

  // IMU
  mOptCurBias_= mpVIGInitializer_->GetCurBias();
  mpPIIMUMeas_ = new PIIMU(gIMU_params, mOptCurBias_);
  double StartFrameEpoch = mpOptimizer_->GetFrames().begin()->first;
  while (mdIMUs.front().CurData.T <= StartFrameEpoch) mdIMUs.pop_front();

  // Vision
  mNoiseModel_Vision_ =
      gtsam::noiseModel::Isotropic::Sigma(2, mInitVSigma);
  mLMIndex_ = mpVIGInitializer_->mLMIndex;

  while (mpTracker_->mImageEpoch <=
         pSimSystem->mpTracker->mImageEpoch)
    mpTracker_->GrabNextImage();
  mpMapper_ = mpVIGInitializer_->GetMapper();

  for (auto &mpitF : mpOptimizer_->GetFrames()) {
    int NavInd = mpitF.second.first.NavIndex;
    while (mdIMUs.front().CurData.T <= mpitF.first) {
      mpPIIMUMeas_->integrateMeasurement(mdIMUs.front().MeanAcc(),
                                         mdIMUs.front().MeanGyro(),
                                         mdIMUs.front().DeltaTime());
      mdIMUs.pop_front();
    }
    mpitF.second.first.pFramePIIMU = mpPIIMUMeas_;
    mpPIIMUMeas_ = new PIIMU(gIMU_params, mOptCurBias_);

    mFrames_[mpitF.first] = mpitF.second;
    mFrames_[mpitF.first].first.NavIndex = ++mNavIndex_;
    mFrames_[mpitF.first].first.GNSSPredNav_nb =
        gtsam::NavState(mFrames_[mpitF.first].first.Tnc * gTic.inverse(),
                        mFrames_[mpitF.first].first.Vnb);
  }

  mCurNavEpoch_ = mFrames_.rbegin()->first;
  mStartEpoch_ = mCurNavEpoch_ - mSmootherLag_;
  for (auto mpit : mFrames_) {
    if ((mStartEpoch_ - mpit.first) > 1e-7) {
      mCachedFrames_[mpit.first] = mFrames_[mpit.first];
      mFrames_.erase(mFrames_.find(mpit.first));
    }
  }
  mpMapper_->MoveSW(mStartEpoch_);
}

bool TestSimVIO::CalcPixelNoise_InvDepth(
    const gtsam::Pose3 &RefKFTnc, const gtsam::Pose3 &CurKFTnc,
    const double &Rho, const gtsam::Point2 &Measured_ref,
    const gtsam::Point2 &Measured, const int CurFeatId,
    const double &CurKFEpoch) {
  gtsam::Point3 pose1_P_landmark(Measured_ref(0) / Rho, Measured_ref(1) / Rho,
                                 1. / Rho);
  // Convert the landmark to world coordinates
  gtsam::Point3 Pos3w = RefKFTnc.transformFrom(pose1_P_landmark);
  // Project landmark into Pose2
  gtsam::PinholeBase normalized_plane(CurKFTnc);
  VectorXd ResReprj = normalized_plane.project2(Pos3w) - Measured;

  double Scale;
  bool bScaleAvaliable = GetNoiseScale(ResReprj, mInitVSigma, Scale);

  cout << CurFeatId << "," << setprecision(12) << CurFeatId << "  |  ";
  cout << ResReprj(0) << "  " << ResReprj(1) << "  |   " << Scale << endl;

  if (bScaleAvaliable) {
    mNoiseModel_Vision_ =
        gtsam::noiseModel::Isotropic::Sigma(2, Scale * mInitVSigma);
  }

  return bScaleAvaliable;
}

void TestSimVIO::RunVIO(const double CurIMUEpoch) {
  if (!gbUseCam && CurIMUEpoch >= mpTracker_->mImageEpoch) {
    // 1. get the image of index frmId 0 based
    ifstream mifIamge;
    mifIamge.open(mifImg_ + to_string(mpTracker_->mImageId) + ".txt",
                  ios_base::in);
    pair<Frame, map<int, Point2D>> prCurFeatures;
    prCurFeatures.first.FrameId = mpTracker_->mImageId;
    prCurFeatures.first.Epoch = mpTracker_->mImageEpoch;
    while (!mifIamge.eof()) {
      string strLine;
      int FeatId;
      double x, y;
      getline(mifIamge, strLine);
      if (!strLine.empty()) {
        stringstream ss(strLine);
        ss >> FeatId;
        ss >> x;
        ss >> y;
      }

      Point2D sFeature;
      sFeature.Uxy1 << x, y, 1.;
      prCurFeatures.second[FeatId] = sFeature;
    }

    // 4. Add TrackedFrames, FramesId_PosesInKFQue
    ConfigVision(prCurFeatures);

    // 7. Optimize
    ConstructGraph();
    Optimize();

    // 8. Update Poses in KeyFrames Queue of Estimator
    UpdateVision();

    // // 9. Viewer: TODO
    // mpViewer_->RunViewer(mpMapper_, mFrames_,
    //                      mpTracker_->mCurImage);

    // 10. grab the next image time
    mpTracker_->GrabNextImage();
  }
  // // Test
  // cout << mpViewer->NumOptimized() << endl;
}

void TestSimVIO::ConfigVision(
    const pair<Frame, map<int, Point2D>> &prsCurFrame) {
  mCurNavEpoch_ = prsCurFrame.first.Epoch;
  mCurFrame_ = prsCurFrame;

  assert(!mdIMUs.empty() &&
         mdIMUs.front().PreData.T - mFrames_.rbegin()->first < 1e-7);
  mCurFrame_.first.NavIndex = ++mNavIndex_;
  while (mdIMUs.front().CurData.T <= mCurNavEpoch_) {
    mpPIIMUMeas_->integrateMeasurement(mdIMUs.front().MeanAcc(),
                                       mdIMUs.front().MeanGyro(),
                                       mdIMUs.front().DeltaTime());
    mdIMUs.pop_front();
  }
  mCurFrame_.first.pFramePIIMU = mpPIIMUMeas_;
  mCurFrame_.first.pFramePIIMU->print();
  mpPIIMUMeas_ = new PIIMU(gIMU_params, mOptCurBias_);

  // gtsam::Pose3 mPredTni_0 = mTrueTni[mpTracker_->mImageEpoch];
  gtsam::NavState GPredNavState = mpOptimizer_->GetCurPreNavState();
  mCurFrame_.first.GNSSPredNav_nb = GPredNavState;

  gtsam::NavState VPredNavState = mCurFrame_.first.pFramePIIMU->predict(
      gtsam::NavState(mFrames_.rbegin()->second.first.Tnc * gTic.inverse(),
                      mFrames_.rbegin()->second.first.Vnb),
      mOptCurBias_);

  PrintNavs2PVA(mCurNavEpoch_, GPredNavState);
  PrintNavs2PVA(mCurNavEpoch_, VPredNavState);

  mCurFrame_.first.Tnc = VPredNavState.pose() * gTic;
  mCurFrame_.first.Vnb = VPredNavState.v();
  mCurFrame_.first.IMUBias = mOptCurBias_;

  // // Test
  mCurFrame_.first.bRightNearGNSS = mpOptimizer_->GetRightNearGNSS();
  mpOptimizer_->SetRightNearGNSS(false);

  mFrames_[mCurFrame_.first.Epoch] = mCurFrame_;

  // Add MapPoints
  mpMapper_->AddMapPoints(mCurFrame_);
}

void TestSimVIO::AddIMUFactor(const int NavInd, PIIMU *pPreIntgIMUs) {
  PIIMU *Preint_imu = dynamic_cast<PIIMU *>(pPreIntgIMUs);
  gtsam::ImuFactor IMU_Factor(X(NavInd - 1), V(NavInd - 1), X(NavInd),
                              V(NavInd), B(NavInd - 1), *Preint_imu);
  mpGraph_->add(IMU_Factor);
  gtsam::imuBias::ConstantBias Zero_bias(Vector3d(0, 0, 0), Vector3d(0, 0, 0));
  // mNoiseModel_BetwBias =
  // gtsam::noiseModel::Diagonal::Sigmas(pow(mIMUCounterBetwDualObss, 0.5) *
  //                                                               mInitSigma6_BetwBias);
  // Add Bias(Between) Factor
  mpGraph_->add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(NavInd - 1), B(NavInd), Zero_bias, mNoiseModel_BetwBias_));
}

void TestSimVIO::AddVisualFactor() {
  for (auto &lit : mpMapper_->mMapPoints) {
    if (lit.SolvedFlag == FRESH || lit.SolvedFlag == SOLVING) {
      if (lit.SolvedFlag == FRESH) lit.LMIndex = ++mLMIndex_;

      double StartKFEpoch = lit.mpInfoOnAllFrames.begin()->first;
      gtsam::Point2 Measured_ref = lit.Uxy1_ref.head(2);
      map<double, InvDepthFactor> mpInvDepthFactors;

      for (auto &mpitInfoPerFrame : lit.mpInfoOnAllFrames) {
        double CurKFEpoch = mpitInfoPerFrame.first;

        if (CurKFEpoch == StartKFEpoch) {
          continue;
        } else {
          gtsam::Point2 Measured = mpitInfoPerFrame.second.Uxy1.head(2);

          if (!CalcPixelNoise_InvDepth(
                  mFrames_[StartKFEpoch].first.Tnc,
                  mFrames_[CurKFEpoch].first.Tnc, lit.InvDepth, Measured_ref,
                  Measured, lit.PointId, CurKFEpoch))
            continue;

          mpInvDepthFactors[CurKFEpoch] = InvDepthFactor(
              X(mFrames_[StartKFEpoch].first.NavIndex),
              X(mFrames_[CurKFEpoch].first.NavIndex), L(lit.LMIndex),
              Measured_ref, Measured, mNoiseModel_Vision_, gTic);
        }
      }

      if ((int)mpInvDepthFactors.size() >= gMinCoFrames - 1) {
        lit.SolvedFlag = SOLVING;

        mInitValues_.insert(L(lit.LMIndex), lit.InvDepth);
        mFrames_[StartKFEpoch].second[lit.PointId].bOptimized = 1;

        for (auto mpit : mpInvDepthFactors) {
          mpGraph_->add(mpit.second);
          mFrames_[mpit.first].second[lit.PointId].bOptimized = 1;
        }
      } else {
        mLMIndex_--;
        lit.LMIndex = -1;
        continue;
      }
    }
  }
}

void TestSimVIO::ConstructGraph(){
  mStartEpoch_ = mCurNavEpoch_ - mSmootherLag_;
  while ((mStartEpoch_ - mFrames_.begin()->first) > 1e-7) {
    mCachedFrames_[mFrames_.begin()->first] = mFrames_[mFrames_.begin()->first];
    mFrames_.erase(mFrames_.find(mFrames_.begin()->first));
  }

  mpMapper_->MoveSW(mStartEpoch_);

  for (auto mpitF = mFrames_.begin(); mpitF != mFrames_.end(); mpitF++) {
    if (mpitF == mFrames_.begin()) {
      // Nav Index
      int NavInd = mpitF->second.first.NavIndex;

      // Factor Graph
      mpGraph_->add(gtsam::PriorFactor<gtsam::Pose3>(
          X(NavInd), mpitF->second.first.Tnc * gTic.inverse(),
          mInitPoseSigma_));
      mpGraph_->add(gtsam::PriorFactor<gtsam::Vector3>(
          V(NavInd), mpitF->second.first.Vnb, mInitVelSigma_));
      mpGraph_->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
          B(NavInd), mpitF->second.first.IMUBias, mInitBiasSigma_));

      // Initial Values
      mInitValues_.insert(X(NavInd), mpitF->second.first.Tnc * gTic.inverse());
      mInitValues_.insert(V(NavInd), mpitF->second.first.Vnb);
      mInitValues_.insert(B(NavInd), mpitF->second.first.IMUBias);

    } else {
      // Nav Index
      int NavInd = mpitF->second.first.NavIndex;

      // Factor Graph
      assert(mFrames_[mpitF->first].first.pFramePIIMU->deltaTij() > 0);
      AddIMUFactor(NavInd, mFrames_[mpitF->first].first.pFramePIIMU);

      // Initial Values
      mInitValues_.insert(X(NavInd),
                          mFrames_[mpitF->first].first.Tnc * gTic.inverse());
      mInitValues_.insert(V(NavInd), mFrames_[mpitF->first].first.Vnb);
      mInitValues_.insert(B(NavInd), mFrames_[mpitF->first].first.IMUBias);

      // the frame exceed the GNSS range, fix it
      if (mpitF->second.first.bRightNearGNSS) {
        assert(mpitF->first <= mFrames_.rbegin()->first);
        // Nav Index
        int NavInd = mpitF->second.first.NavIndex;

        DiagonalNoise_ptr NoiseGNSS = mpOptimizer_->mNoiseModel_GNSSpos;
        gtsam::Pose3 CurPose = mpitF->second.first.GNSSPredNav_nb.pose();
        gtsam::GPSFactor GPSfactor(X(NavInd), CurPose.translation(), NoiseGNSS);
        mpGraph_->add(GPSfactor);

        // // Factor Graph
        // mpGraph_->add(gtsam::PriorFactor<gtsam::Pose3>(
        //     X(NavInd), mpitF->second.first.Tnc * gTic.inverse(),
        //     mInitPoseSigma_));
        mpGraph_->add(gtsam::PriorFactor<gtsam::Vector3>(
            V(NavInd), mpitF->second.first.GNSSPredNav_nb.v(),
            mInitVelSigma_));
        // mpGraph_->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        //     B(NavInd), mpitF->second.first.IMUBias, mInitBiasSigma_));
      }
    }
  }

  // mFrames_.insert(mCachedFrames_.begin(), mCachedFrames_.end());
  for (auto mpit : mFrames_) {
    PrintTns2PA(mpit.first, mpit.second.first.Tnc * gTic.inverse());
  }

  // Features

  // // Test
  // auto mpit = mFrames_.begin();
  // while (mpit->first < mTestPauseEpoch_) mpit++;
  // double tl = mpit->first;
  // auto mprit = mFrames_.rbegin();
  // mprit++;
  // double tr = mprit->first, InvD;

  // vector<pair<Vector3d, Vector3d>> vprUxy1Pair;
  // vprUxy1Pair = mpMapper_->GetCorresponding(tl, tr);
  // //! 共视的Features应该大于20
  // double sum_parallax = 0, average_parallax;
  // //! 求取匹配的特征点在图像上的视差和(归一化平面上)
  // for (size_t j = 0; j < vprUxy1Pair.size(); j++) {
  //   Vector2d pts_0(vprUxy1Pair[j].first(0), vprUxy1Pair[j].first(1));
  //   Vector2d pts_1(vprUxy1Pair[j].second(0), vprUxy1Pair[j].second(1));
  //   double parallax = (pts_0 - pts_1).norm();
  //   sum_parallax = sum_parallax + parallax;
  // }
  // //! 求取所有匹配的特征点的平均视差
  // average_parallax = 1.0 * sum_parallax / int(vprUxy1Pair.size());

  // // mFrames_[tl].first.Tnc = gtsam::Pose3(mRl_, mFrames_[tl].first.Tnc.translation());
  // // mFrames_[tr].first.Tnc = gtsam::Pose3(mRr_, mFrames_[tr].first.Tnc.translation());
  // PrintTns2PA(tl, mFrames_[tl].first.Tnc * gTic.inverse());
  // PrintTns2PA(tr, mFrames_[tr].first.Tnc * gTic.inverse());
  // gtsam::Pose3 dPose = mFrames_[tl].first.Tnc.inverse() * mFrames_[tr].first.Tnc;
  // PrintTns2PA(tl, dPose);
  // Vector3d Pos3w;
  // for (auto lit : mpMapper_->mMapPoints) {
  //   Vector2d LUxy = lit.mpInfoOnAllFrames[tl].Uxy1.head(2);
  //   Vector2d RUxy = lit.mpInfoOnAllFrames[tr].Uxy1.head(2);
  //   TriangulateOneFt_DualView(LUxy, mFrames_[tl].first.Tnc, RUxy,
  //                             mFrames_[tr].first.Tnc, InvD, Pos3w);

  //   cout << Pos3w.transpose();
  //   cout << endl;
  // }

  // vector<double> vFEpoch;
  // for (auto mpit : mFrames_) vFEpoch.push_back(mpit.first);
  // double NewStart = mFrames_.begin()->first;
  // mpMapper_->MoveSW(NewStart);
  // mFrames_.erase(mFrames_.find(NewStart));
  mpMapper_->TriangulateAllFts_MultView(mFrames_);
  // Factor
  AddVisualFactor();
}

void TestSimVIO::Optimize() {
  // Update Solver
  gtsam::LevenbergMarquardtOptimizer LMOptimizer(*mpGraph_, mInitValues_);

  // mFLSmootherISAM2_.getDelta().print();
  mFLResultValues_ = LMOptimizer.optimize();
  UpdateResults_FLSmoother();

  // mResultValues_.print("Current estimate: ");
  for (auto mpit : mFrames_) {
    PrintTns2PA(mpit.first, mFLResultValues_.at<gtsam::Pose3>(
                                    X(mpit.second.first.NavIndex)));
  }

  mOptCurNavState_ =
      gtsam::NavState(mResultValues_.at<gtsam::Pose3>(X(mNavIndex_)),
                      mResultValues_.at<gtsam::Vector3>(V(mNavIndex_)));
  // mOptCurBias_ = mResultValues_.at<gtsam::imuBias::ConstantBias>(B(mNavIndex_));

  // // clear graph/values and Reset the preintegration object
  // mMarginals = gtsam::Marginals(*mpGraph_, mResultValues_);

  mpGraph_->resize(0);
  mInitValues_.clear();
  // Jin: newed after Vision, but the bias was not up to date
  mpPIIMUMeas_->resetIntegrationAndSetBias(mOptCurBias_);
}

void TestSimVIO::UpdateResults_FLSmoother() {
  for (auto itValue : mFLResultValues_) {
    // update, if the key exsits in the resultvalues
    if (mResultValues_.exists(itValue.key))
      mResultValues_.update(itValue.key, itValue.value);
    // insert, if the key is new
    else
      mResultValues_.insert(itValue.key, itValue.value);
  }
}

void TestSimVIO::UpdateVision() {
  // Update Poses in Frames Queue of Estimator
  for (auto &mpit : mFrames_) {
    int NavInd = mpit.second.first.NavIndex;

    // // Test
    // cout << endl;
    // PrintTns2PA(mpit.first, mpit.second.first.Tnc * gTic.inverse());
    // cout << mpit.second.first.Vnb.transpose() << endl;

    mpit.second.first.Tnc = mResultValues_.at<gtsam::Pose3>(X(NavInd)) * gTic;
    mpit.second.first.Vnb = mResultValues_.at<gtsam::Vector3>(V(NavInd));
    // mpit.second.first.IMUBias =
    //     mResultValues_.at<gtsam::imuBias::ConstantBias>(B(NavInd));

    // Test
    // cout << endl;
    // PrintTns2PA(mpit.first, mpit.second.first.Tnc * gTic.inverse());
    // cout << mpit.second.first.Vnb.transpose() << endl;

    cout << mResultValues_.at<gtsam::imuBias::ConstantBias>(B(NavInd)) << endl;

  }

  if(mFrames_.rbegin()->second.first.bRightNearGNSS){
    mOptCurBias_ = mpOptimizer_->GetCurBias();
    for(auto &mpit : mFrames_){
      mpit.second.first.IMUBias = mOptCurBias_;
      // mpit.second.first.pFramePIIMU->BiasCorrectedDelta(mOptCurBias_);
    }
  }

  mpMapper_->SetPointssDepth(mFLResultValues_, mFrames_);
  mpMapper_->RemoveFailures();
}

}  // namespace VIG
