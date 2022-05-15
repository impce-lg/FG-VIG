#include "TestSynVIO.h"
#include <gtsam/navigation/GPSFactor.h>

namespace VIG {

int TestSynVIO::mNavIndex_ = -1;    // start from 0

TestSynVIO::TestSynVIO(const Configuration *pConfig,
                       const string &ConfigFile) {
  mSmootherLag_ = pConfig->mVIOLag;

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
  mpViewer_ = new Viewer(ConfigFile);
  mpViewer_->InitiViewer(gInitTni);
  
  // Test
  mTestPauseEpoch_ = pConfig->mTestPauseEpoch;
  mRl_ = pConfig->mRl;
  mRr_ = pConfig->mRr;
}

void TestSynVIO::ReadNav(const string &strNavFile) {
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

void TestSynVIO::InitializeVIO(SynSystem *pSynSystem) {
  // Optimizer
  mpVIGInitializer_ = pSynSystem->mpVIGInitializer;
  mpOptimizer_ = pSynSystem->mpOptimizer;

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

  mpTracker_ = pSynSystem->mpTracker;
  mpMapper_ = mpVIGInitializer_->GetMapper();

  for (auto &mpitF : mpOptimizer_->GetFrames()) {
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

bool TestSynVIO::CalcPixelNoise_InvDepth(
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

void TestSynVIO::RunVIO(const double CurIMUEpoch) {
  if (CurIMUEpoch >= mpTracker_->mImageEpoch) {
    // there are some gap in the imu recordings
    mpTracker_->CheckIMUGamp(CurIMUEpoch);

    // 1. get the image of index frmId 0 based
    mpTracker_->ReadCurImage();

    // 3. Track the adjacency frame
    mpTracker_->Track();

    // 4. Add TrackedFrames, FramesId_PosesInKFQue
    ConfigVision(mpTracker_->GetFeatures());

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

void TestSynVIO::ConfigVision(
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

void TestSynVIO::AddIMUFactor(const int NavInd, PIIMU *pPreIntgIMUs) {
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

void TestSynVIO::AddVisualFactor() {
  mLMIndex_ = -1;
  for (auto &lit : mpMapper_->mMapPoints) {
    if (lit.SolvedFlag == FRESH || lit.SolvedFlag == SOLVING) {
      lit.LMIndex = ++mLMIndex_;

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
        lit.SolvedFlag = RAW;
        continue;
      }
    }
  }
}

void TestSynVIO::ConstructGraph(){
  mStartEpoch_ = mCurNavEpoch_ - mSmootherLag_;
  while ((mStartEpoch_ - mFrames_.begin()->first) > 1e-7) {
    mCachedFrames_[mFrames_.begin()->first] = mFrames_[mFrames_.begin()->first];
    mFrames_.erase(mFrames_.find(mFrames_.begin()->first));
  }

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
      assert(mpitF->second.first.pFramePIIMU->deltaTij() > 0);
      AddIMUFactor(NavInd, mpitF->second.first.pFramePIIMU);

      // Initial Values
      mInitValues_.insert(X(NavInd), mpitF->second.first.Tnc * gTic.inverse());
      mInitValues_.insert(V(NavInd), mpitF->second.first.Vnb);
      mInitValues_.insert(B(NavInd), mpitF->second.first.IMUBias);

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

  // Test
  for (auto mpit : mFrames_) {
    PrintTns2PA(mpit.first, mpit.second.first.Tnc * gTic.inverse());
  }

  mpMapper_->FormMapPoints(mFrames_);
  // mpMapper_->MoveSW(mStartEpoch_);
  mpMapper_->TriangulateAllFts_MultView(mFrames_);
  // Factor
  AddVisualFactor();
}

void TestSynVIO::Optimize() {
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

void TestSynVIO::UpdateResults_FLSmoother() {
  for (auto itValue : mFLResultValues_) {
    // update, if the key exsits in the resultvalues
    if (mResultValues_.exists(itValue.key))
      mResultValues_.update(itValue.key, itValue.value);
    // insert, if the key is new
    else
      mResultValues_.insert(itValue.key, itValue.value);
  }
}

void TestSynVIO::UpdateVision() {
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
