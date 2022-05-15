#include "Optimizer.h"

namespace VIG {
#define MAX_ITERATION_ISAM_UPD_CAL 3

Optimizer::Optimizer(const Configuration *pConfig) {
  /**** 1. set initial parameters ****/
  SetInitParas(pConfig);
  SetIMUParas(pConfig);
  if (gbUseCam) SetCamParas(pConfig);

  /**** 2. Prepare Factor Graph and Inital Values ****/
  mSmootherLag = pConfig->mSmootherLag;
  SetPrior();
  
  /**** 3. 打开结果文件 ****/
  mofNavState.open(pConfig->mNavResult);
  mofNavState << fixed;
  mofNavError.open(pConfig->mNavError);
  mofNavError<< fixed;
  mofIMUBias.open(pConfig->mIMUBias);
  mofIMUBias << fixed;

  mbWriteMapPoints = pConfig->mbWriteMapPoints;
  if (mbWriteMapPoints) {
    mofMapPoints.open(pConfig->mMapPoints);
    mofMapPoints << fixed;
  }
}

void Optimizer::SetInitParas(const Configuration *pConfig) {
  mt_ig = gtsam::Point3(pConfig->mt_ig);
  mTic = pConfig->mTic;

  // Get initial navstates
  ifstream ifNav(pConfig->mNavFile);

  bool bStartTime = true;
  while (!ifNav.eof()) {
    string strLine;
    getline(ifNav, strLine);
    if (!strLine.empty()) {
      if (strLine.at(0) < '0' || strLine.at(0) > '9') continue;

      double t, Nx, Ny, Nz, v_x, v_y, v_z, roll, pitch, yaw;
      stringstream ss(strLine);
      ss >> t;

      if (t < gStartTime)
        continue;
      else {
        ss >> Nx;
        ss >> Ny;
        ss >> Nz;
        ss >> v_x;
        ss >> v_y;
        ss >> v_z;
        ss >> roll;
        ss >> pitch;
        ss >> yaw;

        // 记录初始状态真值
        if (bStartTime) {
          mInitVel_i2n << v_x, v_y, v_z;
          Vector3d rpy_rad = Vector3d(roll, pitch, yaw) * kDeg2Rad;
          mInitTni = gtsam::Pose3(gtsam::Rot3::RzRyRx(rpy_rad),
                                  gtsam::Point3(Vector3d(Nx, Ny, Nz)));

          gStartTime = t;
          bStartTime =false;
        }

        // 存储真值
        Eigen::Matrix<double, 9, 1> NavTrue;
        NavTrue << Nx, Ny, Nz, v_x, v_y, v_z, roll, pitch, yaw;
        mpNavTrue[t] = NavTrue;
      }
    }
  }
  ifNav.close();

  // Noise
  mInitPoseSigma = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << pConfig->mInitAttSigma * Vector3d(1, 1, 2),
       pConfig->mInitPosSigma * Vector3d(1, 1, 1))
          .finished());
  mInitVelSigma =
      gtsam::noiseModel::Isotropic::Sigma(3, pConfig->mInitVelSigma);
  mInitBiasSigma = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << pConfig->mInitAccBiasSigma * Vector3d(1, 1, 1),
       pConfig->mInitGyroBiasSigma * Vector3d(1, 1, 1))
          .finished());
}

void Optimizer::SetIMUParas(const Configuration *pConfig) {
  if (pConfig->mNavFrame == "NED") {
    gG = Vector3d(0, 0, pConfig->mNormG);
    mIMU_params = PIIMUParams::MakeSharedD(pConfig->mNormG);  // input norm g
  } else if (pConfig->mNavFrame == "ENU") {
    gG = Vector3d(0, 0, -pConfig->mNormG);
    mIMU_params = PIIMUParams::MakeSharedU(pConfig->mNormG);  // input norm g
  } else {
    cerr << "Unknown navframe!" << endl;
    return;
  }

  // IMU measurement noise
  mIMU_params->accelerometerCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(pConfig->mAccSigma, 2);
  // PreintegratedRotation params:
  mIMU_params->gyroscopeCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(pConfig->mGyroSigma, 2);

  // Bias noise
  mIMU_params->biasAccCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(pConfig->mAccBiasRW, 2);
  mIMU_params->biasOmegaCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(pConfig->mGyroBiasRW, 2);

  // Integration noise used in gtsam
  mIMU_params->integrationCovariance =
      gtsam::Matrix33::Identity(3, 3) * pConfig->mIntgSigma;
  mIMU_params->biasAccOmegaInt =
      gtsam::Matrix::Identity(6, 6) * pConfig->mBiasAccOmegaInt;

  mInitSigma6_BetwBias << pConfig->mInitAccBetwBiasSigma * Vector3d(1, 1, 1),
      pConfig->mInitGyroBetwBiasSigma * Vector3d(1, 1, 1);
  mNoiseModel_BetwBias =
      gtsam::noiseModel::Diagonal::Sigmas(mInitSigma6_BetwBias);

  mCurPre_State.OptBias = gtsam::imuBias::ConstantBias();  // zero
  mpPIIMUMeas = new PIIMU(mIMU_params, mCurPre_State.OptBias);
  mCurPre_State.pPIIMU4Prd = new PIIMU(mIMU_params, mCurPre_State.OptBias);
}

void Optimizer::SetCamParas(const Configuration *pConfig) {
  mInitVSigma = pConfig->mSigma_Pixel / gFocalLength;
  // 真实帧的误差模型
  mNoiseModel_KF = gtsam::noiseModel::Isotropic::Sigma(2, mInitVSigma);
  // GNSS内插帧的误差模型
  mNoiseModel_KF4G = gtsam::noiseModel::Isotropic::Sigma(2, 1.5 * mInitVSigma);

  mpWinMapper = new Mapper(pConfig);
  mpMargMapper = new Mapper(pConfig);
}

void Optimizer::SetPrior() {
  mGraph.resize(0);
  mInitValues.clear();

  // Factor Graph
  mGraph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), mInitTni, mInitPoseSigma));
  mGraph.add(
      gtsam::PriorFactor<gtsam::Vector3>(V(0), mInitVel_i2n, mInitVelSigma));
  mGraph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      B(0), mCurPre_State.OptBias, mInitBiasSigma));

  // Initial Values
  mInitValues.insert(X(0), mInitTni);
  mInitValues.insert(V(0), mInitVel_i2n);
  mInitValues.insert(B(0), mCurPre_State.OptBias);

  // 当前状态
  mCurPre_State.OptEpoch = gStartTime;
  mCurPre_State.OptNav = gtsam::NavState(mInitTni, mInitVel_i2n);
  mCurPre_State.PredNav = mCurPre_State.OptNav;
  // 当重置Prior信息时，bias可能改变了
  mCurPre_State.pPIIMU4Prd->resetIntegrationAndSetBias(mCurPre_State.OptBias);
  mpPIIMUMeas->resetIntegrationAndSetBias(mCurPre_State.OptBias);

  // State序列
  mpNavInds.clear();
  mpNavInds[gStartTime] = 0;
}

void Optimizer::PushIMU(const IMU &sIMU) {
  mIMUs[sIMU.T] = sIMU;
}

double Optimizer::CalNoiseScale(const double error) {
  double Scale = 1000.;
  // cout << CurLMIndex << "," << error.transpose() << endl;

  if (error < mInitVSigma) {
    Scale = 1.0;
  } else if (error < 1.5 * mInitVSigma) {
    Scale = 1.5;
  } else if (error < 2 * mInitVSigma) {
    Scale = 2.;
  } else if (error < 4 * mInitVSigma) {
    Scale = 4.;
  } else if (error < 8 * mInitVSigma) {
    Scale = 10.;
  } else if (error < 16 * mInitVSigma) {
    Scale = 30.;
  } else {
    Scale = 100.;
  }

  return Scale;
}

void Optimizer::PreIntegrate(const double PreEpoch, const double CurEpoch,
                             PIIMU *pPIIMU) {
  assert(mIMUs.begin()->first <= PreEpoch && mIMUs.rbegin()->first >= CurEpoch);
  // 指定时段预积分，所以这里要清空
  pPIIMU->resetIntegration();

  // 如果时间出错，可能是使用不当，也可能是某些不可控因素，但都会原样返回输入的空pPIIMU
  if (PreEpoch > CurEpoch) {
    cout << "PreEpoch is larger than CurEpoch, you may need to check! "
            "We have returned the PreIntegration safely, it is zero just as "
            "its input."
         << endl;
    return;
  } else if (PreEpoch == CurEpoch) {
    cout << "PreEpoch is equal to CurEpoch, you may need to check! "
            "We have returned the PreIntegration safely, it is zero just as "
            "its input."
         << endl;
    return;
  }

  auto LIt = mIMUs.find(PreEpoch), RIt = mIMUs.find(PreEpoch);
  for (RIt++; RIt != mIMUs.end(); LIt++, RIt++) {
    double dt = RIt->first - LIt->first;
    Vector3d MeanAcc = (LIt->second.Acc + RIt->second.Acc) * 0.5;
    Vector3d MeanGyro = (LIt->second.Gyro + RIt->second.Gyro) * 0.5;

    pPIIMU->integrateMeasurement(MeanAcc, MeanGyro, dt);

    if (RIt->first >= CurEpoch) break;
  }
}

void Optimizer::Predict(double CurEpoch) {
  // 注意bias只在开始和Optimize后才会改变，这里不需要担心其是否应重置
  PreIntegrate(mCurPre_State.OptEpoch, CurEpoch, mCurPre_State.pPIIMU4Prd);

  mCurPre_State.PreEpoch = CurEpoch;
  mCurPre_State.PredNav =
      mCurPre_State.pPIIMU4Prd->predict(mCurPre_State.OptNav, mCurPre_State.OptBias);

  // // Test
  // mCurPre_State.pPIIMU4Prd->print();
  // PrintTns2PA(CurEpoch, mCurPre_State.PredNav.pose());

  mCurPre_State.pPIIMU4Prd->resetIntegrationAndSetBias(mCurPre_State.OptBias);
}

void Optimizer::ConfigGNSS(const int NavIndex, GNSSSol &sGNSSSol) {
  sGNSSSol.NavIndex = NavIndex;

  Predict(sGNSSSol.Epoch);
  sGNSSSol.SetState(mCurPre_State);

  mGNSSs[sGNSSSol.Epoch] = sGNSSSol;
}

void Optimizer::ConfigCurFrame(const int NavIndex,
                               pair<Frame, map<int, Point2D>> &CurFrame) {
  // Add Current Tracked Frame
  CurFrame.first.NavIndex = NavIndex;

  Predict(CurFrame.first.Epoch);
  CurFrame.first.SetState(mCurPre_State, mTic);

  mFrames[CurFrame.first.Epoch] = CurFrame;
}

bool Optimizer::CheckNewKeyFrame(const double CurEpoch){
  auto CurFrame = mFrames.at(CurEpoch);

  // 第一帧
  if (mFrames.size() == 1) {
    mKeyFrames[CurEpoch] = CurFrame;
    return true;
  } 
  // 非第一帧
  else {
    auto PreFrame = mKeyFrames.rbegin()->second;

    // 真实帧需要判断和前一帧的视差，内插到GNSS时刻的帧默认为关键帧
    int NCorr;
    if (EnoughParallax(PreFrame, CurFrame, NCorr) || CurFrame.first.bOnGNSS) {

      CurFrame.first.bKeyFrame = true;
      mKeyFrames[CurEpoch] = CurFrame;

      return true;
    }
    else
      return false;
  }
}

void Optimizer::InterplKFOnGNSS(const double GNSSEpoch) {
  auto sGNSSSol = mGNSSs.at(GNSSEpoch);
  auto GNSS_RF = mFrames.lower_bound(GNSSEpoch);
  auto GNSS_LF = GNSS_RF;
  GNSS_LF--;

  pair<Frame, map<int, Point2D>> Frame_G;
  // first部分
  Frame_G.first.FrameId = GNSS_LF->second.first.FrameId;
  Frame_G.first.Epoch = GNSSEpoch;
  Frame_G.first.NavIndex = sGNSSSol.NavIndex;
  Frame_G.first.Tnc = sGNSSSol.Tnb * mTic;
  Frame_G.first.Vnb = sGNSSSol.Vnb;
  Frame_G.first.IMUBias = sGNSSSol.IMUBias;
  Frame_G.first.bOnGNSS = true;
  // second部分
  double K =
      (GNSSEpoch - GNSS_LF->first) / (GNSS_RF->first - GNSS_LF->first);
  for (auto LitFt : GNSS_LF->second.second) {
    if (GNSS_RF->second.second.find(LitFt.first) !=
        GNSS_RF->second.second.end()) {
      auto RitFt = GNSS_RF->second.second.at(LitFt.first);
      Point2D sFeature;

      sFeature.Duv = LitFt.second.Duv + K * (RitFt.Duv - LitFt.second.Duv);
      sFeature.Uuv = LitFt.second.Uuv + K * (RitFt.Uuv - LitFt.second.Uuv);
      sFeature.Uxy1 = LitFt.second.Uxy1 + K * (RitFt.Uxy1 - LitFt.second.Uxy1);
      sFeature.TrackNum = LitFt.second.TrackNum;
      Frame_G.second[LitFt.first] = sFeature;
    }
  }

  mFrames[GNSSEpoch] = Frame_G;

  CheckNewKeyFrame(GNSSEpoch);
}

bool Optimizer::LMOptimizePoint(Point3D &SubPoint) {
  if (SubPoint.SolvedFlag == SOLVING) return true;

  // 之前用多视角几何为地图点位赋值，这里是为其重构的点（all，part1，part2）赋值，
  // 只选首尾两帧，使用简单的双视几何
  Vector3d LUxy1 = SubPoint.mpPt2DKFs.begin()->second.Uxy1;
  Vector3d RUxy1 = SubPoint.mpPt2DKFs.rbegin()->second.Uxy1;

  double InvDepth_Init = 0.0;
  Eigen::Vector3d Pos3w_Init;
  TriangulateOneFt_DualView(
      LUxy1.head(2),
      mKeyFrames.at(SubPoint.mpPt2DKFs.begin()->first).first.Tnc,
      RUxy1.head(2),
      mKeyFrames.at(SubPoint.mpPt2DKFs.rbegin()->first).first.Tnc,
      InvDepth_Init, Pos3w_Init);

  if (InvDepth_Init < 0.0) return false;

  // 优化点位
  gtsam::NonlinearFactorGraph *pNewGraph = new gtsam::NonlinearFactorGraph();
  gtsam::Values NewInitValues;

  //构建因子图
  double StartEpoch = SubPoint.mpPt2DKFs.begin()->first;
  double k = 1.0;
  for (auto &mpit : SubPoint.mpPt2DKFs) {
    double CurEpoch = mpit.first;

    NewInitValues.insert(X(mKeyFrames.at(CurEpoch).first.NavIndex),
                         mKeyFrames.at(CurEpoch).first.Tnc);

    auto Noise_FirstPose = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.001 * k * Vector3d(1, 1, 1),
         0.001 * k * Vector3d(1, 1, 1))
            .finished());
    pNewGraph->add(gtsam::PriorFactor<gtsam::Pose3>(
        X(mKeyFrames.at(CurEpoch).first.NavIndex), mKeyFrames.at(CurEpoch).first.Tnc,
        Noise_FirstPose));

    // PrintTns2PA(mpit.first, mCVTnc.at(mpit.first) * mTic.inverse());
    // PrintTns2PA(mpit.first, mCVTnc.at(mpit.first));// * mTic.inverse());
    // PrintTns2PA(mpit.first, mpit.second.first.Tnc * mTic.inverse());
    // cout << endl;

    if (CurEpoch > StartEpoch) {
      gtsam::Point2 Measured = mpit.second.Uxy1.head(2);

      auto factor = InvDepthFactor(X(mKeyFrames.at(StartEpoch).first.NavIndex),
                                   X(mKeyFrames.at(CurEpoch).first.NavIndex),
                                   L(SubPoint.PointId), SubPoint.Uxy1_ref.head(2), Measured,
                                   mNoiseModel_KF);
      auto error = factor.evaluateError(mKeyFrames.at(StartEpoch).first.Tnc,
                                        mKeyFrames.at(CurEpoch).first.Tnc,
                                        InvDepth_Init);

      double scale = CalNoiseScale(error.norm());
      auto Noise = gtsam::noiseModel::Isotropic::Sigma(2, scale * mInitVSigma);
      factor.updateNoiseModel(Noise);

      pNewGraph->add(factor);
    }

    k = k + 1;
  }
  // 不要忘了最关键的点位变量
  NewInitValues.insert(L(SubPoint.PointId), InvDepth_Init);

  // gtsam::LevenbergMarquardtOptimizer LMOptimizer(*pNewGraph, NewInitValues);
  // gtsam::Values LMResultValues = LMOptimizer.optimize();
  // auto LMInvDepth = LMResultValues.at<double>(L(SubPoint.PointId));

  jBatchFLSmoother FLOptimizer;
  FLOptimizer.update(*pNewGraph, NewInitValues);
  auto FLValues = FLOptimizer.calculateEstimate();
  auto FLInvDepth = FLValues.at<double>(L(SubPoint.PointId));

  // gtsam::NonlinearISAM isam(3);
  // isam.update(*pNewGraph, NewInitValues);
  // auto ISValues = isam.estimate();
  // auto ISInvDepth = ISValues.at<double>(L(SubPoint.PointId));

  SubPoint.InvDepth = FLValues.at<double>(L(SubPoint.PointId));
  SubPoint.Pos3w =
      mKeyFrames.at(StartEpoch)
          .first.Tnc.transformFrom(1. / SubPoint.InvDepth * SubPoint.Uxy1_ref);

  // 深度值相对误差
  double delta_depth =
      fabs(1. / InvDepth_Init - 1. / SubPoint.InvDepth) * SubPoint.InvDepth;

  double err = pNewGraph->error(FLValues);
  // auto nit = LMOptimizer.getInnerIterations();
  if (delta_depth > 0.25 ||                              // 相对误差
      pNewGraph->error(FLValues) > 4e-2 ||               // 优化残差
      SubPoint.InvDepth < 0.01 || SubPoint.InvDepth > 1  //||  // 深度过大或过小
      /*LMOptimizer.getInnerIterations() > 10*/) {  // 优化迭代次数
    SubPoint.Pos3w *= 0;
    return false;
  }

  return true;
}

bool Optimizer::LMOptimizePointAndCheck(Point3D &CurPoint) {
  if ((int)CurPoint.mpPt2DKFs.size() < gMinCoFrames) return false;

  gtsam::Pose3 Tnc_s =
      mKeyFrames.at(CurPoint.mpPt2DKFs.begin()->first).first.Tnc;

  Point3D all_point(CurPoint.PointId, CurPoint.Uxy1_ref);
  Point3D part1_point(CurPoint.PointId, CurPoint.Uxy1_ref);
  Point3D part2_point(CurPoint.PointId, CurPoint.Uxy1_ref);

  size_t i = 0;
  auto iter = CurPoint.mpPt2DKFs.begin();
  for (; iter != CurPoint.mpPt2DKFs.end(); iter++) {
    all_point.mpPt2DKFs[iter->first] = iter->second;
    if (i % 3 == 0) {
      part1_point.mpPt2DKFs[iter->first] = iter->second;
      part2_point.mpPt2DKFs[iter->first] = iter->second;
    } else if (i % 3 == 1) {
      part1_point.mpPt2DKFs[iter->first] = iter->second;
    } else {
      part2_point.mpPt2DKFs[iter->first] = iter->second;
    }
    i++;
  }

  if (LMOptimizePoint(all_point) &&    //
      LMOptimizePoint(part1_point) &&  //
      LMOptimizePoint(part2_point)) {
    Eigen::Vector3d diff1 = all_point.Pos3w - part1_point.Pos3w;
    Eigen::Vector3d diff2 = all_point.Pos3w - part2_point.Pos3w;

    if (abs(diff1(2)) < 8 && abs(diff2(2)) < 8 &&  //
        abs(diff1(1)) < 2 && abs(diff2(1)) < 2 &&  //
        abs(diff1(0)) < 2 && abs(diff2(0)) < 2) {
      // CurPoint.InvDepth = all_point.InvDepth;
      // CurPoint.Pos3w = all_point.Pos3w;
      CurPoint.SolvedFlag = SOLVING;
      return true;
    } 
    // 如果不满足条件，就改为RAW状态
    else
      CurPoint.SolvedFlag = RAW;
  }
  return false;
}

void Optimizer::SelectNewPoints() {
  // 筛选优化的Point
  mpWinMapper->ClearState();
  for (auto itKF : mKeyFrames) mpWinMapper->AddMapPoints(itKF.second);

  vector<int> vEraseIds;
  // 这里只能用迭代器，如果用增强for循环（冒号）遍历会卡顿
  for (auto mpitMP = mpWinMapper->mNewMapPoints.begin();
       mpitMP != mpWinMapper->mNewMapPoints.end(); mpitMP++) {
    auto &MP = mpitMP->second;

    if ((int)MP.mpPt2DKFs.size() >= gMinCoFrames) {
      mpWinMapper->TriangulateMP_MultView(MP, mKeyFrames);

      // cout << MP.PointId << ": " << MP.Pos3w.transpose() << endl;

      // 错误的和过远的点都被剔除
      if (MP.SolvedFlag != SOLVING) {
        vEraseIds.push_back(mpitMP->first);
        continue;
      }

    }
    // 观测帧数不足，且最后一帧也没有该点，被剔除
    else if (mKeyFrames.rbegin()->second.second.find(mpitMP->first) ==
             mKeyFrames.rbegin()->second.second.end())
      vEraseIds.push_back(mpitMP->first);
  }

  // 处理NewMapper，只留下能被最新帧观测的RAW点
  for (auto vit = vEraseIds.begin(); vit != vEraseIds.end(); vit++) {
    auto mpitMP = mpWinMapper->mNewMapPoints.find(*vit);
    // mpWinMapper->mIdleMapPoints.insert(*mpitMP);  // 只是为了画图用的
    mpWinMapper->mNewMapPoints.erase(mpitMP);
  }
}

void Optimizer::AddIMUFactor(const uint64 PreNavInd, const uint64 CurNavInd,
                             PIIMU *pPreIntgIMUs) {
  PIIMU *Preint_imu = dynamic_cast<PIIMU *>(pPreIntgIMUs);
  gtsam::ImuFactor IMU_Factor(X(PreNavInd), V(PreNavInd), X(CurNavInd),
                              V(CurNavInd), B(PreNavInd), *Preint_imu);
  mGraph.add(IMU_Factor);
  gtsam::imuBias::ConstantBias Zero_bias(Vector3d(0, 0, 0), Vector3d(0, 0, 0));
  // mNoiseModel_BetwBias =
  // gtsam::noiseModel::Diagonal::Sigmas(pow(mIMUCounterBetwDualObss, 0.5) *
  //                                                               mInitSigma6_BetwBias);

  mGraph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(PreNavInd), B(CurNavInd), Zero_bias, mNoiseModel_BetwBias));
}

void Optimizer::AddGNSSFactor(const GNSSSol &InGNSSSol) {
  // Add GNSS
  mNoiseModel_GNSSpos = gtsam::noiseModel::Diagonal::Sigmas(InGNSSSol.Std);
  // Add GNSS Factor
  gtsam::GPSFactorLA GNSSFactor(X(InGNSSSol.NavIndex),
                                gtsam::Point3(InGNSSSol.t_ng),
                                mNoiseModel_GNSSpos, mt_ig);
  mGraph.add(GNSSFactor);

  // // Test
  // cout << "GNSS res: " << GNSSFactor.evaluateError(InGNSSSol.Tnb).transpose();
  // cout << endl;
}

void Optimizer::AddCurKFFactor(
    const pair<Frame, map<int, Point2D>> &CurFrame) {

  for (auto &mpitFt : CurFrame.second) {
    auto mpit = mpWinMapper->mNewMapPoints.find(mpitFt.first);

    // 只接受flag为SOLVING的点
    if (mpit->second.SolvedFlag == SOLVING) {
      // 新点
      // if (mpit->second.LMIndex == -1)
      if (mResultValues.exists(L(mpit->second.PointId))) {
        // 将该点插入状态量
        mInitValues.insert(L(mpit->second.PointId), mpit->second.InvDepth);

        double StartKFEpoch = mpit->second.mpPt2DKFs.begin()->first;
        // 现在再来插入因子
        gtsam::Point2 Measured_ref = mpit->second.Uxy1_ref.head(2);
        for (auto &mpitInfoPerKF : mpit->second.mpPt2DKFs) {
          double CurKFEpoch = mpitInfoPerKF.first;
          auto Noise = mNoiseModel_KF;
          if (mKeyFrames.at(CurKFEpoch).first.bOnGNSS) Noise = mNoiseModel_KF4G;

          if (CurKFEpoch == StartKFEpoch) {
            continue;
          } else {
            gtsam::Point2 Measured = mpitInfoPerKF.second.Uxy1.head(2);

            auto factor = boost::allocate_shared<InvDepthFactor>(
                Eigen::aligned_allocator<InvDepthFactor>(),
                X(mFrames.at(StartKFEpoch).first.NavIndex),
                X(mFrames.at(CurKFEpoch).first.NavIndex),
                L(mpit->second.PointId), Measured_ref, Measured, Noise, mTic);

            mGraph.add(factor);
            mvInvDepthFactors.push_back(factor);
          }
        }
      }
      // 旧点
      else {
        double StartKFEpoch = mpit->second.mpPt2DKFs.begin()->first;
        gtsam::Point2 Measured_ref = mpit->second.Uxy1_ref.head(2);
        double CurKFEpoch = CurFrame.first.Epoch;
        gtsam::Point2 Measured = mpitFt.second.Uxy1.head(2);
        auto Noise = mNoiseModel_KF;
        if (mKeyFrames.at(CurKFEpoch).first.bOnGNSS) Noise = mNoiseModel_KF4G;

        auto factor = boost::allocate_shared<InvDepthFactor>(
            Eigen::aligned_allocator<InvDepthFactor>(),
            X(mFrames.at(StartKFEpoch).first.NavIndex),
            X(mFrames.at(CurKFEpoch).first.NavIndex), L(mpit->second.PointId),
            Measured_ref, Measured, Noise, mTic);

        mGraph.add(factor);
        mvInvDepthFactors.push_back(factor);
      }
    } else
      continue;
  }
}

void Optimizer::AddMapperFactor() {
  mvInvDepthFactors.clear();
  SelectNewPoints();

  // 只能用迭代器
  for (auto mpit = mpWinMapper->mNewMapPoints.begin();
       mpit != mpWinMapper->mNewMapPoints.end(); mpit++) {
    // SOLVING is setted in SFM optimization
    if (mpit->second.SolvedFlag == SOLVING) {
      // 将该点插入状态量
      mInitValues.insert(L(mpit->second.PointId), mpit->second.InvDepth);

      double StartKFEpoch = mpit->second.mpPt2DKFs.begin()->first;
      // 现在再来插入因子
      gtsam::Point2 Measured_ref = mpit->second.Uxy1_ref.head(2);
      for (auto &mpitInfoPerKF : mpit->second.mpPt2DKFs) {
        double CurKFEpoch = mpitInfoPerKF.first;

        if (CurKFEpoch == StartKFEpoch) {
          continue;
        } else {
          gtsam::Point2 Measured = mpitInfoPerKF.second.Uxy1.head(2);

          auto factor = boost::allocate_shared<InvDepthFactor>(
              Eigen::aligned_allocator<InvDepthFactor>(),
              X(mKeyFrames.at(StartKFEpoch).first.NavIndex),
              X(mKeyFrames.at(CurKFEpoch).first.NavIndex),
              L(mpit->second.PointId), Measured_ref, Measured, mNoiseModel_KF,
              mTic);

          mGraph.add(factor);
          mvInvDepthFactors.emplace_back(factor);
        }
      }
    }
  }
}

void Optimizer::UpdateGNSS4Solver() {
  // State序列
  mpNavInds[mGNSSs.rbegin()->first] = mGNSSs.rbegin()->second.NavIndex;

  // 前一个Index可能是从先验开始，故设为0
  uint64 PreNavInd = 0, CurNavInd;
  double PreEpoch = gStartTime, CurEpoch;
  for (auto &itGNSS : mGNSSs) {
    auto &CurGNSS = itGNSS.second;
    CurNavInd = CurGNSS.NavIndex;
    CurEpoch = itGNSS.first;

    // Initial Values
    mInitValues.insert(X(CurNavInd), CurGNSS.Tnb);
    mInitValues.insert(V(CurNavInd), CurGNSS.Vnb);
    mInitValues.insert(B(CurNavInd), CurGNSS.IMUBias);

    // IMU factor
    // 如果是第一个GNSS，就不要添加预积分因子了
    if (itGNSS.first != mpNavInds.begin()->first) {
      PreIntegrate(PreEpoch, CurEpoch, mpPIIMUMeas);
      CurGNSS.pGNSSPIIMU = mpPIIMUMeas;
      // 注意这里一定要新建指针变量，否则之前由之赋值的变量会变化
      mpPIIMUMeas = new PIIMU(mIMU_params, mCurPre_State.OptBias);

      AddIMUFactor(PreNavInd, CurNavInd, CurGNSS.pGNSSPIIMU);
    }

    // GNSS factor
    AddGNSSFactor(CurGNSS);

    PreNavInd = CurNavInd;
    PreEpoch = CurEpoch;
  }
}

void Optimizer::UpdateVision4Solver() {
  // 不要直接在后面插入新的KF，GNSS出现的时候可能会漏一个
  mpNavInds.clear();
  for (auto itKF : mKeyFrames)
    mpNavInds[itKF.first] = itKF.second.first.NavIndex;

  // cout << "边缘化因子的key个数：" << mGraph.keys().size() << endl;

  // 前一个Index可能是从先验开始，故设为0
  uint64 PreNavInd = 0, CurNavInd;
  double PreEpoch = gStartTime, CurEpoch;
  for (auto &itKF : mKeyFrames) {
    auto &CurKF = itKF.second;
    CurNavInd = itKF.second.first.NavIndex;
    CurEpoch = itKF.first;

    // Initial Values
    mInitValues.insert(X(CurNavInd), CurKF.first.Tnc * mTic.inverse());
    mInitValues.insert(V(CurNavInd), CurKF.first.Vnb);
    mInitValues.insert(B(CurNavInd), CurKF.first.IMUBias);

    // IMU factor
    // 如果不是第一帧，添加预积分因子
    if (itKF.first > mpNavInds.begin()->first) {
      PreIntegrate(PreEpoch, CurEpoch, mpPIIMUMeas);
      CurKF.first.pFramePIIMU = mpPIIMUMeas;
      // 注意这里一定要新建指针变量，否则之前由之赋值的变量会变化
      mpPIIMUMeas = new PIIMU(mIMU_params, mCurPre_State.OptBias);

      AddIMUFactor(PreNavInd, CurNavInd, CurKF.first.pFramePIIMU);
    }

    // GNSS factor
    if (gbUseGNSS && itKF.second.first.bOnGNSS
      // && (mKeyFrames.begin()->first == 5 && itKF.first!=6)
    ) {
      AddGNSSFactor(mGNSSs.at(itKF.first));
    }

    PreNavInd = CurNavInd;
    PreEpoch = CurEpoch;
  }

  // if (mKeyFrames.rbegin()->second.first.NavIndex == 60) 
    // mGraph.print();

  // cout<< "加入视觉前的因子个数：" << mGraph.size() << endl;

  // 添加视觉factor
  AddMapperFactor();
  // cout << "视觉因子个数：" << mvInvDepthFactors.size() << endl;
}

void Optimizer::Optimize() {
  // mGraph.print();
  // mInitValues.print();
  // FixedLag平滑器，不用设置Lag和TimeStamp，自己管理边缘化
  mMargKeys = GetMargKeys();
  mBFLSmoother = jBatchFLSmoother();
  mBFLSmoother.Update(mGraph, mInitValues, mMargKeys);

  mFLResultValues = mBFLSmoother.calculateEstimate();

  // mBFLSmoother.getFactors().print();
  // mBFLSmoother.update();
  // // 如果有视觉观测，调整方差
  // if (!mvInvDepthFactors.empty()) {
  //   for (auto &factor : mvInvDepthFactors) {
  //     double scale =
  //         CalNoiseScale(factor->unwhitenedError(mFLResultValues).norm());
  //     auto Noise = gtsam::noiseModel::Isotropic::Sigma(2, scale * mInitVSigma);
  //     factor->updateNoiseModel(Noise);
  //   }
  //   jBatchFLSmoother BFLSmoother2;  // = jBatchFLSmoother();
  //   BFLSmoother2.update(mGraph, mInitValues);
  // }

  UpdateNavInd_Results();

  // mBFLSmoother.getDelta().print();  // x: drotvector, dtranslation
  // mResultValues.print("Current estimate: ");
  // for (auto mpit : mpNavInds) {
  //   PrintTns2PA(mpit.first, mResultValues.at<gtsam::Pose3>(X(mpit.second)));
  // }

  auto CurNavInd = mpNavInds.rbegin()->second;
  mCurPre_State.Update(
      mpNavInds.rbegin()->first,
      gtsam::NavState(mResultValues.at<gtsam::Pose3>(X(CurNavInd)),
                      mResultValues.at<gtsam::Vector3>(V(CurNavInd))),
      mResultValues.at<gtsam::imuBias::ConstantBias>(B(CurNavInd)));

  // gtsam::Marginals marginals(mGraph, mFLResultValues);
  // for(auto mmpit : mmpFLNavTimeLine){
  //   // mResultCovs[mmpit.second.NavInd] =
  //   marginals.marginalCovariance(X(mmpit.second.NavInd)); cout <<
  //   mmpit.second.NavInd << " " << endl
  //        << marginals.marginalCovariance(X(mmpit.second.NavInd)) << endl;
  // }

  if (mMargKeys.size() > 0) ProcessMarg();
  mGraph = mMargFactors;
  // mGraph.print();
  mInitValues.clear();
  // 赋值给GNSS或Vision后被new，当时还没有更新Bias
  mpPIIMUMeas->resetIntegrationAndSetBias(mCurPre_State.OptBias);
}

void Optimizer::UpdateNavInd_Results() {
  for (auto itValue : mFLResultValues) {
    // update, if the key exsits in the resultvalues
    if (mResultValues.exists(itValue.key))
      mResultValues.update(itValue.key, itValue.value);
    // insert, if the key is new
    else
      mResultValues.insert(itValue.key, itValue.value);
  }
}

gtsam::KeyVector Optimizer::GetMargKeys() {
  gtsam::KeyVector MargKeys = gtsam::KeyVector();

  // 窗口长度超限
  // TODO: 注意这里每次只边缘化第一个状态，要不要改成超限的都边缘化?
  // if (mpNavInds.rbegin()->first - mpNavInds.begin()->first > mSmootherLag) {
    mMargEpoch = mpNavInds.begin()->first;
    mMargNavInd = mpNavInds.begin()->second;

    MargKeys.push_back(X(mMargNavInd));
    MargKeys.push_back(V(mMargNavInd));
    MargKeys.push_back(B(mMargNavInd));

    // 如果是VIG初始化已完成，并且视觉已经参与优化（初始化时有在IG优化中没有参与）
    if (geVIGState == VIG_OK &&
        mKeyFrames.find(mMargEpoch) != mKeyFrames.end()) {
      assert(mMargEpoch == mKeyFrames.begin()->first);

      for (auto itFeat : mKeyFrames.at(mMargEpoch).second) {
        if (mInitValues.exists(L(itFeat.first)) &&
            mpWinMapper->mNewMapPoints.find(itFeat.first) !=
                mpWinMapper->mNewMapPoints.end()
                //  && mpWinMapper->mNewMapPoints.at(itFeat.first).mpPt2DKFs.size() == 2
            )
          MargKeys.push_back(L(itFeat.first));
      }
    }
  // }
  // // 虽然窗口长度没有超限，但是第一个是先验
  // else if (mpNavInds.begin()->first == gStartTime) {
  //   mMargEpoch = mpNavInds.begin()->first;
  //   mMargNavInd = mpNavInds.begin()->second;

  //   MargKeys.push_back(X(mMargNavInd));
  //   MargKeys.push_back(V(mMargNavInd));
  //   MargKeys.push_back(B(mMargNavInd));
  // }

  return MargKeys;
}

void Optimizer::ProcessMarg() {
  mMargFactors = mBFLSmoother.GetMarginalFactors();

  // 还是等准备好了再输出吧
  if (geVIGState == VIG_OK) SaveNavState(mMargEpoch, mMargNavInd);
  mpNavInds.erase(mpNavInds.find(mMargEpoch));
}

void Optimizer::UpdateGNSS() {
  // Update GNSS NavStates
  for (auto &mpit : mGNSSs) {
    int NavInd = mpit.second.NavIndex;

    mpit.second.Tnb = mResultValues.at<gtsam::Pose3>(X(NavInd));
    mpit.second.Vnb = mResultValues.at<gtsam::Vector3>(V(NavInd));
    mpit.second.IMUBias =
        mResultValues.at<gtsam::imuBias::ConstantBias>(B(NavInd));

    // // 注意这里并不是更新内插Frame，而是用优化的GNSS state为之赋值，即更新其初值。
    // // 下面的UpdateVision才是更新其状态。
    // if (mKeyFrames.find(mpit.first) != mKeyFrames.end()) {
    //   mKeyFrames.at(mpit.first).first.Tnc = mpit.second.Tnb * mTic;
    //   mKeyFrames.at(mpit.first).first.Vnb = mpit.second.Vnb;
    //   mKeyFrames.at(mpit.first).first.IMUBias = mpit.second.IMUBias;
    // }
  }
}

void Optimizer::UpdateVision() {
  // Update Frame Poses
  for (auto &mpitKF : mKeyFrames) {
    int NavInd = mpitKF.second.first.NavIndex;

    if (!mFLResultValues.exists(X(NavInd)))
      continue;
    else {
      mpitKF.second.first.Tnc =
          mFLResultValues.at<gtsam::Pose3>(X(NavInd)) * mTic;
      mpitKF.second.first.Vnb = mFLResultValues.at<gtsam::Vector3>(V(NavInd));
      mpitKF.second.first.IMUBias =
          mFLResultValues.at<gtsam::imuBias::ConstantBias>(B(NavInd));
    }
  }

  // Update Map Points Position
  if (mbWriteMapPoints) {
    mpWinMapper->SetPointsPos(mFLResultValues, mKeyFrames);
    for (auto mpitNewMP : mpWinMapper->mNewMapPoints) {
      auto mpitIdleMP = mpWinMapper->mIdleMapPoints.find(mpitNewMP.first);
      if (mpitIdleMP == mpWinMapper->mIdleMapPoints.end()) {
        mpWinMapper->mIdleMapPoints.insert(mpitNewMP);
      } else
        mpitIdleMP->second.Pos3w = mpitNewMP.second.Pos3w;
    }
  }

  // // Slide Window
  // for (auto mpitFeat : mKeyFrames.begin()->second.second) {
  //   if (mpWinMapper->mNewMapPoints.find(mpitFeat.first) ==
  //       mpWinMapper->mNewMapPoints.end())
  //     continue;

  //   auto &MP = mpWinMapper->mNewMapPoints.at(mpitFeat.first);

  //   // 如果该点第一帧是被边缘化的帧（第一个关键帧是基准帧），并且观测数还足够、正在被优化中，
  //   // 则删除这一帧信息
  //   if (MP.mpPt2DKFs.begin()->first == mMargEpoch && MP.mpPt2DKFs.size() > 2 &&
  //       MP.SolvedFlag == SOLVING) {
  //     // 先删除第一帧
  //     MP.mpPt2DKFs.erase(MP.mpPt2DKFs.begin());
  //     // 再更新深度
  //     MP.Uxy1_ref = MP.mpPt2DKFs.begin()->second.Uxy1;
  //     auto Pos3c = mKeyFrames.at(MP.mpPt2DKFs.begin()->first)
  //                      .first.Tnc.transformTo(MP.Pos3w);
  //     MP.InvDepth = 1 / Pos3c.z();

  //   }
  //   // 如果该点观测数已经不够并且最后一帧已经不是最新关键帧（也就是并非新点，但是又不足以去优化），
  //   // 则存入旧点集
  //   else if (MP.mpPt2DKFs.size() <= 2 &&
  //            MP.mpPt2DKFs.rbegin()->first < mKeyFrames.rbegin()->first) {
  //     mpWinMapper->mIdleMapPoints.insert(make_pair(mpitFeat.first, MP));
  //     mpWinMapper->mNewMapPoints.erase(
  //         mpWinMapper->mNewMapPoints.find(mpitFeat.first));
  //   }
  // }

  // 处理完点再删除当前序列的第一帧
  mMargFrames.insert(*mKeyFrames.begin());
  mKeyFrames.erase(mKeyFrames.begin());
}

void Optimizer::SaveNavState(const double &NavEpoch, const uint64 &NavInd) {
  // 读取结果
  gtsam::Pose3 Tnb = mResultValues.at<gtsam::Pose3>(X(NavInd));
  Vector3d Pos = Tnb.translation();
  Vector3d Vel = mResultValues.at<gtsam::Vector3>(V(NavInd));
  Vector3d AttDeg = Tnb.rotation().rpy() * kRad2Deg;

  // 写入导航结果
  mofNavState << setprecision(9) << NavEpoch << setw(15) <<  // time
      setprecision(5) <<                                     // 5 decimeter
      Pos(0) << setw(11) << Pos(1) << setw(11) << Pos(2) << setw(11) <<  // pos
      Vel(0) << setw(11) << Vel(1) << setw(11) << Vel(2) << setw(11) <<  // vel
      AttDeg(0) << setw(11) << AttDeg(1) << setw(11) << AttDeg(2) << setw(11)
              <<  // att
      // Std(0) << setw(11) << Std(1) << setw(11) << Std(2) << setw(11) <<
      // Std(3) << setw(11) << Std(4) << // std setw(11) << Std(5) << setw(11)
      // << Std(6) << setw(11) << Std(7) << setw(11) << Std(8)
      endl;
  
  // 写入导航误差
  auto mpitTrue = mpNavTrue.find(NavEpoch);
  if (mpitTrue != mpNavTrue.end()) {
    Vector3d ErrP = Pos - mpitTrue->second.topRows(3);
    Vector3d ErrV = Vel - mpitTrue->second.middleRows<3>(3);
    Vector3d ErrA = AttDeg - mpitTrue->second.bottomRows(3);
    if (ErrA.z() > 300) ErrA.z() -= 360;
    if (ErrA.z() < -300) ErrA.z() += 360;

    mofNavError << setprecision(9) << NavEpoch << setw(15) <<  // time
        setprecision(5) <<                                     // 5 decimeter
        ErrP(0) << setw(11) << ErrP(1) << setw(11) << ErrP(2) << setw(11)
                <<  // pos
        ErrV(0) << setw(11) << ErrV(1) << setw(11) << ErrV(2) << setw(11)
                <<  // vel
        ErrA(0) << setw(11) << ErrA(1) << setw(11) << ErrA(2) << setw(11)
                <<  // att
        // Std(0) << setw(11) << Std(1) << setw(11) << Std(2) << setw(11) <<
        // Std(3) << setw(11) << Std(4) << // std setw(11) << Std(5) << setw(11)
        // << Std(6) << setw(11) << Std(7) << setw(11) << Std(8)
        endl;
  } else {
    cout << "Error when calculating nav error, check your navfile" << endl;
    exit(-1);
  }

  // 写入零偏
  auto CurBias = mResultValues.at<gtsam::imuBias::ConstantBias>(B(NavInd));
  gtsam::Vector3 Ba = CurBias.accelerometer();
  gtsam::Vector3 Bg = CurBias.gyroscope();
  mofIMUBias << setprecision(9) << NavEpoch << setw(15) <<  // time
      setprecision(5) <<                                    // 5 decimeter
      Ba(0) << setw(11) << Ba(1) << setw(11) << Ba(2) << setw(11) <<  // Acc
      Bg(0) << setw(11) << Bg(1) << setw(11) << Bg(2) << setw(11) <<  // Gyro
      endl;
}

void Optimizer::SaveTail(){
  for (auto mpit : mpNavInds) {
    SaveNavState(mpit.first, mpit.second);
  }
  mofNavState.close();
  mofIMUBias.close();
}

void Optimizer::SaveTail(const map<int, Eigen::Vector3d> &mpTrueMap){
  for (auto mpit : mpNavInds) {
    SaveNavState(mpit.first, mpit.second);
  }
  mofNavState.close();
  mofIMUBias.close();

  if (mbWriteMapPoints && !mpWinMapper->mIdleMapPoints.empty() &&
      !mpTrueMap.empty()) {

    for (auto mpIdleMP : mpWinMapper->mIdleMapPoints) {
      if (mpIdleMP.second.Pos3w.norm() < 0.001) break;

      Eigen::Vector3d OptMP3w = mpIdleMP.second.Pos3w;
      Eigen::Vector3d TrueMP3w = mpTrueMap.at(mpIdleMP.first);
      Eigen::Vector3d dMP = OptMP3w - TrueMP3w;
      mofMapPoints << std::fixed << std::left << mpIdleMP.first << "  "
                   << std::setprecision(5) << dMP.transpose() << " "
                   << std::setprecision(5) << TrueMP3w.transpose() << " "
                   << std::endl;
    }

    mofMapPoints.close();
  }
}

// // Test
// void Optimizer::PrintCurFactorGraph(gtsam::Values &CurLinearizationPoint) {
//   gtsam::NonlinearFactorGraph CurISAMFactorGraph =
//       LMOptimizer.getFactors();
//   for (size_t i = 0; i < CurISAMFactorGraph.size(); i++) {
//     gtsam::KeyVector itKeyVector;
//     if (CurISAMFactorGraph.at(i))
//       itKeyVector = CurISAMFactorGraph.at(i)->keys();
//     else
//       continue;

//     for (size_t j = 0; j < itKeyVector.size(); j++) {
//       gtsam::Symbol itSymbol(itKeyVector[j]);
//       if (itSymbol.chr() == 'l') {
//         double CurPrjErr =
//             CurISAMFactorGraph.at(i)->error(CurLinearizationPoint);
//         // CurISAMFactorGraph.at(i)->print();
//         cout << i << "  ";
//         itSymbol.print();
//         cout << CurPrjErr << endl;
//       }
//     }
//   }
// }

}  // namespace VIG
