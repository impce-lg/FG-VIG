#include "TestVIO.h"

namespace VIG {

int TestVIO::mPoseIndex_ = 0;    // start from 0
int TestVIO::mLMIndex_ = -1;  // start from 0

TestVIO::TestVIO(const Configuration *pConfig,
                 const double FirstNavStateIMUTime,
                 const string &ConfigFile) {
  mNTrue = 0;

  ReadNav(pConfig->mNavFile);

  mpTracker_ =
      new Tracker(pConfig, FirstNavStateIMUTime);
  mpMapper_ = new Mapper(pConfig);

  SetEstimator(pConfig);

  mpViewer_ = new Viewer(ConfigFile);
  mpViewer_->InitiViewer(gInitTni);
}

void TestVIO::ReadNav(const string &strNavFile) {
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

      mTrueNav[t] =
          gtsam::NavState(mTrueTni[t], gtsam::Vector3(ve, vn, vu));
    }
  }
}

void TestVIO::SetEstimator(const Configuration *pConfig) {
  /********* I. Initial Conditions *******/
  mInitPoseSigma_ = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gInitAttSigma * Vector3d(1, 1, 1),
       gInitPosSigma * Vector3d(1, 1, 1))
          .finished());

  /**** II. Camera settings ****/
  mInitWinSize_ = pConfig->mInitWinSize;
  mNoiseModel_Vision_ =
      gtsam::noiseModel::Isotropic::Sigma(2, mInitVSigma);
  geVIGState = UNINITIALIZED;

  /**** III. ISAM2 settings ****/
  mIsamParams_.setOptimizationParams(gtsam::ISAM2GaussNewtonParams());
  mIsamParams_.setFactorization("CHOLESKY");
  mIsamParams_.setRelinearizeSkip(
      pConfig
          ->mIsamParams_relinearizeSkip);  // ISAM2Params set 10 as default
  mIsamParams_.findUnusedFactorSlots =
      pConfig->mbIsamParams_findUnusedFactorSlots;
  mIsamParams_.evaluateNonlinearError =
      pConfig->mbIsamParams_evaluateNonlinearError;
  mSmootherLag_ = pConfig->mSmootherLag;
  mFLSmootherISAM2_ =
      gtsam::IncrementalFixedLagSmoother(mSmootherLag_, mIsamParams_);

  mOptCurTni_ = gInitTni;
  mPredTni_ = mOptCurTni_;
}

bool TestVIO::CalcPixelNoise_InvDepth(
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

  // cout << CurFeatId << "," << setprecision(12) << CurFeatId << "  |  ";
  // cout << ResReprj(0) << "  " << ResReprj(1) << "  |   " << Scale << endl;

  if (bScaleAvaliable) {
    mNoiseModel_Vision_ =
        gtsam::noiseModel::Isotropic::Sigma(2, Scale * mInitVSigma);
  }

  return bScaleAvaliable;
}

void TestVIO::UpdateXVBTimeStamps(double CurPoseTime) {
  mvPosesTimeStamps_.push_back(CurPoseTime);
  mtdmpNewKey_TimeStamps_[X(mPoseIndex_)] = CurPoseTime;
}

void TestVIO::RunVIO(const Configuration *pConfig, const IMU *pIMU,
                     Optimizer *pOptimizer) {
  if (!gbUseCam && pIMU->CurData.T >= mpTracker_->mImageEpoch) {
    // there are some gap in the imu recordings
    mpTracker_->CheckIMUGamp(pIMU->CurData.T);

    // 2. Get current frame pose
    gtsam::Pose3 mPredTni_0 = mTrueTni[mpTracker_->mImageEpoch];
    mPredTni_ = pOptimizer->GetCurPredPose();

    cout << setprecision(13) << mpTracker_->mImageEpoch;
    cout << setprecision(8) << endl;
    mPredTni_0.print("True");
    mPredTni_.print("IG");

    // 3. Track the adjacency frame
    if (gbFirstFrame)  // the first frame
    {
      mPrePubEpoch = mpTracker_->mInitImageEpoch;
      gbFirstFrame = false;
      mpTracker_->GrabNextImage();
      return;
    }

    // 1. get the image of index frmId 0 based
    mpTracker_->ReadCurImage();

    mpTracker_->Track();

    // 4. Add TrackedFrames, FramesId_PosesInKFQue
    ConfigVision(mPredTni_ * gTic, mpTracker_->GetFeatures());

    // Initialize or Add Factors
    if (geVIGState == UNINITIALIZED) {
      if ((int)mFrames_.size() >= mInitWinSize_) {
        // 6. Initialize
        bool bInitializationState = false;
        bInitializationState = InitializeVIO();

        if (bInitializationState) geVIGState = VIG_OK;

        mNTrue = 0;
      }
    } else {
      // 6. Triangulate the Features in navigation Frame which are not
      // triangulated yet
      mpMapper_->TriangulateAllFts_MultView(mFrames_);
      for (auto lit : mpMapper_->mMapPoints) {
        if (lit.SolvedFlag != SOLVING) continue;
        mFLSmootherISAM2_.updateLinearizationPoint(L(lit.LMIndex),
                                                   lit.InvDepth);
      }

      // 7. Optimize
      mPoseIndex_++;
      mNewInitialValues_.insert(X(mPoseIndex_), mPredTni_ * gTic);
      UpdateXVBTimeStamps(pIMU->CurData.T);

      mNTrue++;
      if (mNTrue == 10) {
        mpNewGraph_->add(gtsam::PriorFactor<gtsam::Pose3>(
            X(mPoseIndex_), mPredTni_ * gTic, mInitPoseSigma_));
        mNTrue = 0;
      }

      AddSynVisFactor();
      AddBetwFactorDepend();
      Optimize();
    }

    if (geVIGState == VIG_OK) {
      // 8. Update Poses in KeyFrames Queue of Estimator
      UpdateSynVision();

      // 9. Viewer: TODO
      mpViewer_->RunViewer(mpMapper_, mFrames_,
                           mpTracker_->mCurImage);
    }

    // 10. grab the next image time
    mpTracker_->GrabNextImage();
  }
  // // Test
  // cout << mpViewer->NumOptimized() << endl;
}

void TestVIO::ConfigVision(
    const gtsam::Pose3 &CurTnc,
    const pair<Frame, map<int, Point2D>> &prsCurFrame) {
  if (!mbInitPub_) {
    mbInitPub_ = 1;
  } else if (!mbInitFeature_) {
    mbInitFeature_ = 1;
  } else {
    mCurFrame_ = prsCurFrame;
    mCurFrame_.first.Tnc = CurTnc;
    // mCurFrame_.first.Vnb = ?
    mFrames_[mCurFrame_.first.Epoch] = mCurFrame_;
    mpMapper_->AddMapPoints(mCurFrame_);
  }
}

bool TestVIO::InitializeVIO() {
  /**** 1：Triangulate the Features in KeyFrames Queue ****/
  mpMapper_->TriangulateAllFts_MultView(mFrames_);

  /**** 2：Reinitialize FactorGraph and InitialValues ****/
  mPoseIndex_ = 0;
  // UpdateXVBTimeStamps(gStartTime);
  // mNewInitialValues_.insert(X(0), gInitTni * gTic);
  // mpNewGraph_->add(gtsam::PriorFactor<gtsam::Pose3>(X(0), gInitTni * gTic,
  //                                                   mInitPoseSigma_));

  /**** 3：Add Factors and Optimize ****/
  for (auto &mpitFrame : mFrames_) {
    mPoseIndex_++;
    mNewInitialValues_.insert(X(mPoseIndex_), mpitFrame.second.first.Tnc);
    UpdateXVBTimeStamps(mpitFrame.first);
    mpitFrame.second.first.NavIndex = mPoseIndex_;
    // mNewInitialValues_.print("Initial ");
  }

  mpNewGraph_->add(gtsam::PriorFactor<gtsam::Pose3>(
      X(mKeyFrames_.begin()->second.NavIndex),
      mKeyFrames_.begin()->second.Tnc, mInitPoseSigma_));
  mpNewGraph_->add(gtsam::PriorFactor<gtsam::Pose3>(
      X(mPoseIndex_), mKeyFrames_.rbegin()->second.Tnc,
      mInitPoseSigma_));
  AddVisualFactor_Init();

  mNewInitialValues_.print("mNewInitialValues_");
  // mpNewGraph_->print("mpNewGraph_");

  Optimize();

  return true;
}

void TestVIO::AddVisualFactor_Init() {
  for (auto &lit : mpMapper_->mMapPoints) {
    if (lit.SolvedFlag == FRESH) {
      mLMIndex_++;
      lit.LMIndex = mLMIndex_;

      double StartKFEpoch = lit.mpInfoOnAllFrames.begin()->first;
      gtsam::Point2 Measured_ref = lit.Uxy1_ref.head(2);
      vector<InvDepthFactor> vInvDepthFactors;

      for (auto &mpitInfoPerFrame : lit.mpInfoOnAllFrames) {
        double CurKFEpoch = mpitInfoPerFrame.first;

        if (CurKFEpoch == StartKFEpoch) {
          continue;
        } else {
          gtsam::Point2 Measured = mpitInfoPerFrame.second.Uxy1.head(2);

          if (!CalcPixelNoise_InvDepth(
                  mKeyFrames_[StartKFEpoch].Tnc,
                  mKeyFrames_[CurKFEpoch].Tnc, lit.InvDepth, Measured_ref,
                  Measured, lit.PointId, CurKFEpoch))
            continue;

          vInvDepthFactors.push_back(InvDepthFactor(
              X(mKeyFrames_[StartKFEpoch].NavIndex),
              X(mKeyFrames_[CurKFEpoch].NavIndex), L(lit.LMIndex),
              Measured_ref, Measured, mNoiseModel_Vision_));
        }
      }

      if ((int)vInvDepthFactors.size() >= gMinCoFrames - 1) {
        lit.SolvedFlag = SOLVING;

        mtdmpNewKey_TimeStamps_[L(lit.LMIndex)] =
            lit.mpInfoOnAllFrames.begin()->first;
        mNewInitialValues_.insert(L(lit.LMIndex), lit.InvDepth);

        if (!mKeyFrames_[StartKFEpoch].bIsRef)
          mKeyFrames_[StartKFEpoch].bIsRef = true;

        for (size_t i = 0; i < vInvDepthFactors.size(); i++)
          mpNewGraph_->add(vInvDepthFactors[i]);
      } else {
        mLMIndex_--;
        lit.LMIndex = -1;
        continue;
      }
    }
  }
}

void TestVIO::AddBetwFactorDepend() {
  auto mpitKFLast = mKeyFrames_.rbegin();
  for (int i = 0; i < gMinCoFrames - 1; i++) mpitKFLast++;

  if (!mpitKFLast->second.bIsRef) {
    // gtsam::Pose3 Tnl, Tnr;
    int LNavIndex, RNavIndex;
    // Tnl = mpitKFLast->second.Tnc;
    LNavIndex = mpitKFLast->second.NavIndex;
    mpitKFLast--;
    // Tnr = mpitKFLast->second.Tnc;
    RNavIndex = mpitKFLast->second.NavIndex;

    // gtsam::Pose3 Tlr = Tnl.inverse() * Tnr;
    // DiagonalNoise_ptr NoiseModel_BetwPose =
    // gtsam::noiseModel::Diagonal::Sigmas(
    //     (gtsam::Vector(6) << 0.05 * Vector3d(1, 1, 1), 0.05 * Vector3d(1, 1,
    //     1))
    //         .finished());

    // mpNewGraph_->add(gtsam::BetweenFactor<gtsam::Pose3>(
    //     X(LNavIndex), X(RNavIndex), Tlr, NoiseModel_BetwPose));
    mtdmpNewKey_TimeStamps_[X(LNavIndex)] = mpitKFLast->first;
  }
}

void TestVIO::AddSynVisFactor() {
  mKeyFrames_.rbegin()->second.NavIndex = mPoseIndex_;

  for (auto &mpitFt : mCurFrame_.second) {
    int PointId = mpitFt.first;
    auto lit = find_if(mpMapper_->mMapPoints.begin(),
                       mpMapper_->mMapPoints.end(),
                       [PointId](const Point3D &lit) {
                         return lit.PointId == PointId;
                       });
    assert(lit != mpMapper_->mMapPoints.end());

    if (lit->SolvedFlag == FRESH) {
      mLMIndex_++;
      lit->LMIndex = mLMIndex_;

      double StartKFEpoch = lit->mpInfoOnAllFrames.begin()->first;
      gtsam::Point2 Measured_ref = lit->Uxy1_ref.head(2);
      vector<InvDepthFactor> vInvDepthFactors;
      for (auto &mpitInfoPerFrame : lit->mpInfoOnAllFrames) {
        double CurKFEpoch = mpitInfoPerFrame.first;

        if (CurKFEpoch == StartKFEpoch) {
          continue;
        } else {
          gtsam::Point2 Measured = mpitInfoPerFrame.second.Uxy1.head(2);
          if (!CalcPixelNoise_InvDepth(
                  mKeyFrames_[StartKFEpoch].Tnc,
                  mKeyFrames_[CurKFEpoch].Tnc, lit->InvDepth, Measured_ref,
                  Measured, lit->PointId, CurKFEpoch))
            continue;
          // Notice that mNoiseModel_Vision_ has been changed
          vInvDepthFactors.push_back(InvDepthFactor(
              X(mKeyFrames_[StartKFEpoch].NavIndex),
              X(mKeyFrames_[CurKFEpoch].NavIndex), L(lit->LMIndex),
              Measured_ref, Measured, mNoiseModel_Vision_));
        }
      }

      if ((int)vInvDepthFactors.size() >= gMinCoFrames - 1) {
        lit->SolvedFlag = SOLVING;
        mtdmpNewKey_TimeStamps_[L(lit->LMIndex)] =
            lit->mpInfoOnAllFrames.begin()->first;
        mNewInitialValues_.insert(L(lit->LMIndex), lit->InvDepth);

        if (!mKeyFrames_[StartKFEpoch].bIsRef)
          mKeyFrames_[StartKFEpoch].bIsRef = true;

        for (size_t i = 0; i < vInvDepthFactors.size(); i++)
          mpNewGraph_->add(vInvDepthFactors[i]);
      } else {
        mLMIndex_--;
        lit->LMIndex = -1;
        continue;
      }

      mpitFt.second.bOptimized = 1;
      continue;

    } else if (lit->SolvedFlag == SOLVING) {
      double StartEpoch = lit->mpInfoOnAllFrames.begin()->first;
      gtsam::Point2 Measured_ref = lit->Uxy1_ref.head(2);
      gtsam::Point2 Measured = mpitFt.second.Uxy1.head(2);
      if (!CalcPixelNoise_InvDepth(
              mKeyFrames_[StartEpoch].Tnc, mKeyFrames_.rbegin()->second.Tnc,
              lit->InvDepth, Measured_ref, Measured, lit->PointId,
              mKeyFrames_.rbegin()->first))
        continue;
      // Notice that mNoiseModel_Vision_ has been changed
      mpNewGraph_->add(InvDepthFactor(
          X(mKeyFrames_[StartEpoch].NavIndex), X(mPoseIndex_),
          L(lit->LMIndex), Measured_ref, Measured, mNoiseModel_Vision_));

      mpitFt.second.bOptimized = 1;

    } else
      continue;
  }
}

void TestVIO::Optimize() {
  // Update Solver
  mFLSmootherISAM2_.update(*mpNewGraph_, mNewInitialValues_,
                           mtdmpNewKey_TimeStamps_);

  // Test
  if (mPoseIndex_ >= 139) {
    gtsam::KeyVector vKeys = mFLSmootherISAM2_.getFactors().keyVector();
    for (auto itKey : vKeys) {
      gtsam::Symbol itSymbol(itKey);
      if (itSymbol.chr() == 'x' && itSymbol.index() == 74) {
        cout << "Wait! Error here!";
      }
    }
  }

  mFLSmootherISAM2_.getDelta().print();
  mFLResultValues_ = mFLSmootherISAM2_.calculateEstimate();
  UpdateResults_FLSmoother();

  // mResultValues_.print("Current estimate: ");

  mOptCurTni_ = mResultValues_.at<gtsam::Pose3>(X(mPoseIndex_));

  // // clear graph/values and Reset the preintegration object
  // mMarginals = gtsam::Marginals(*mpNewGraph_, mResultValues_);

  mtdmpNewKey_TimeStamps_.clear();
  mpNewGraph_->resize(0);
  mNewInitialValues_.clear();
}

void TestVIO::UpdateResults_FLSmoother() {
  for (auto itValue : mFLResultValues_) {
    // update, if the key exsits in the resultvalues
    if (mResultValues_.exists(itValue.key))
      mResultValues_.update(itValue.key, itValue.value);
    // insert, if the key is new
    else
      mResultValues_.insert(itValue.key, itValue.value);
  }
}

void TestVIO::UpdateSynVision() {
  // Update Poses in KeyFrames Queue of Estimator
  // for (auto &mpitFrame : mKeyFrames_) {
  //   if (mpitFrame.second.NavIndex < 0) continue;

  //   mpitFrame.second.Tnc =
  //       mResultValues_.at<gtsam::Pose3>(X(mpitFrame.second.NavIndex));
  // }

  mpMapper_->SetPointssDepth(mFLResultValues_, mFrames_);
  mpMapper_->RemoveFailures();
}

}  // namespace VIG
