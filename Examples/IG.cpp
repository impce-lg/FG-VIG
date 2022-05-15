/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file imuFactorsExample
 * @brief Test example for using GTSAM ImuFactor and ImuCombinedFactor
 * navigation code.
 * @author Garrett (ghemann@gmail.com), Luca Carlone
 */

/**
 * Example of use of the imuFactors (imuFactor and combinedImuFactor) in
 * conjunction with GPS
 *  - imuFactor is used by default. You can test combinedImuFactor by
 *  appending a `-c` flag at the end (see below for example command).
 *  - we read IMU and GPS data from a CSV file, with the following format:
 *  A row starting with "i" is the first initial position formatted with
 *  N, E, D, qx, qY, qZ, qW, velN, velE, velD
 *  A row starting with "0" is an imu measurement
 *  linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
 *  A row starting with "1" is a gps correction formatted with
 *  N, E, D, qX, qY, qZ, qW
 * Note that for GPS correction, we're only using the position not the
 * rotation. The rotation is provided in the file for ground truth comparison.
 *
 *  Usage: ./ImuFactorsExample [data_csv_path] [-c]
 *  optional arguments:
 *    data_csv_path           path to the CSV file with the IMU data.
 *    -c                      use CombinedImuFactor
 */

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactorLA.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/Marginals.h>

#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>

using namespace gtsam;
using namespace std;

// using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
// using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
// using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
#define B(Int) (2000000 + Int)   // B:  2(A,B,C,...,X,Y,Z), Bias (ax,ay,az,gx,gy,gz)
#define V(Int) (22000000 + Int)  // V: 22(A,B,C,...,X,Y,Z), Vel (xdot,ydot,zdot)
#define X(Int) (24000000 + Int)  // X: 24(A,B,C,...,X,Y,Z), Pose3 (x,y,z,r,p,y)

const double PI = 3.1415926535897932384626433832795;
const double kDeg2Rad = PI / 180.0;  // Radians per degree
const double kRad2Deg = 180.0 / PI;  // Degrees per radian

static double gStartTime, gEndTime;
Point3 mt_ig = Vector3(-0.219, -0.111, -0.096);
map<double, int> gmpObss, mpWinNavInds;
map<double, Eigen::Matrix<double, 9, 1>> mpNavTrue;
double Lag = 5.0;

ofstream fNav;
ofstream fNavErr;
ofstream fBias;

struct IMU {
  double T;
  Vector3 Acc;
  Vector3 Gyro;
};
map<double, IMU> mpIMUs_;

struct GNSSSol {
  double Epoch;
  Vector3 t_ng;
  int Qual;
  int ns;
  Vector3 Std;
  PreintegratedImuMeasurements *pGNSSPIIMU;
  int NavIndex = -1;
  Pose3 Tnb;
  Vector3 Vnb;
  imuBias::ConstantBias IMUBias;
};
map<double, GNSSSol> mpGNSSs_;

NonlinearFactorGraph *graph = new NonlinearFactorGraph();
Values initial_values, currentEstimate;
FixedLagSmoother::KeyTimestampMap newTimestamps;

Pose3 InitiTnb;
Vector3 InitiVnb;
imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias
std::shared_ptr<PreintegrationType> preintegrated = nullptr, PIM4Pred = nullptr;
auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

void SetInitParas(const string &NavFile) {
  // Get initial navstates
  ifstream ifNav(NavFile);

  bool bStartTime = true;
  while (!ifNav.eof()) {
    string strLine;
    std::getline(ifNav, strLine);
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
          InitiVnb << v_x, v_y, v_z;
          Vector3 rpy_rad = Vector3(roll, pitch, yaw) * kDeg2Rad;
          InitiTnb = Pose3(Rot3::RzRyRx(rpy_rad), Point3(Vector3(Nx, Ny, Nz)));

          gStartTime = t;
          bStartTime = false;
        }

        // 存储真值
        Eigen::Matrix<double, 9, 1> NavTrue;
        NavTrue << Nx, Ny, Nz, v_x, v_y, v_z, roll, pitch, yaw;
        mpNavTrue[t] = NavTrue;
      }
    }
  }
  ifNav.close();
}

void LoadIMU(const string &IMUFile) {
  ifstream ifIMU(IMUFile);

  while (!ifIMU.eof()) {
    string strLine;
    std::getline(ifIMU, strLine);

    if (!strLine.empty()) {
      if (strLine.at(0) < '0' || strLine.at(0) > '9') continue;
      double t, wx, wy, wz, ax, ay, az;

      stringstream ss(strLine);
      ss >> t;
      if (t < gStartTime) continue;

      ss >> wx;
      ss >> wy;
      ss >> wz;
      ss >> ax;
      ss >> ay;
      ss >> az;

      IMU sIMU;
      sIMU.T = t;
      sIMU.Gyro << wx, wy, wz;
      sIMU.Acc << ax, ay, az;

      mpIMUs_[t] = sIMU;
    }
  }

  ifIMU.close();
}

void LoadGNSS(const string &GNSSFile) {
  ifstream ifGNSS(GNSSFile);

  int NavInd = 1; // NavInd_prior = 0
  while (!ifGNSS.eof()) {
    string strLine;
    std::getline(ifGNSS, strLine);

    if (!strLine.empty()) {
      if (strLine.at(0) < '0' || strLine.at(0) > '9') continue;

      double Tow, Nx, Ny, Nz, stdNx, stdNy, stdNz;
      stringstream ss(strLine);
      ss >> Tow;
      if (Tow < gStartTime) continue;

      ss >> Nx;
      ss >> Ny;
      ss >> Nz;
      ss >> stdNx;
      ss >> stdNy;
      ss >> stdNz;

      GNSSSol sGNSSSol;
      sGNSSSol.Epoch = Tow;
      sGNSSSol.t_ng << Nx, Ny, Nz;
      sGNSSSol.Std << stdNy, stdNx, stdNz;
      sGNSSSol.NavIndex = NavInd;

      mpGNSSs_[Tow] = sGNSSSol;
      if (gmpObss.find(Tow) == gmpObss.end()) {
        gmpObss[Tow] = NavInd++;
      } else {
        cerr << "Same GNSS and Vision epoch: " << Tow
             << ", please check and retry!" << endl;
        exit(-1);
      }
    }
  }

  ifGNSS.close();
}

void SetIMUParas(const string &ConfigFile) {
  cv::FileStorage fsSettings(ConfigFile, cv::FileStorage::READ);

  if (!fsSettings.isOpened()) {
    cerr << "Error open yaml config file!" << endl;
    exit(-1);
  }

  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = fsSettings["AccSigma"]; // 0.02;
  double gyro_noise_sigma = fsSettings["GyroSigma"]; // 0.0015;
  double accel_bias_rw_sigma = fsSettings["AccBiasRW"]; // 2.666E-04;
  double gyro_bias_rw_sigma = fsSettings["GyroBiasRW"]; // 1.82E-05;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  double IntgSigma = fsSettings["IntgSigma"];
  Matrix33 integration_error_cov =
      I_3x3 * IntgSigma; // 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  double BiasAccOmegaInt = fsSettings["BiasAccOmegaInt"];
  Matrix66 bias_acc_omega_int =
      I_6x6 * BiasAccOmegaInt; //1e-5;  // error in the bias used for preintegration

  string NavFrame;
  double NormG = fsSettings["Norm_g"];
  fsSettings["NavFrame"] >> NavFrame;
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> p;
  if (NavFrame == "NED") {
    p = PreintegratedCombinedMeasurements::Params::MakeSharedD(NormG);
  } else if (NavFrame == "ENU") {
    p = PreintegratedCombinedMeasurements::Params::MakeSharedU(NormG);
  }

  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

  preintegrated =
      std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);
  assert(preintegrated);
  PIM4Pred = std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);
  assert(PIM4Pred);
}

void AddPrior() {
  mpWinNavInds[gStartTime] = 0;

  // Assemble prior noise model and add it the graph.`
  auto pose_noise_model = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.02, 0.02, 0.02, 0.05, 0.05, 0.08)
          .finished());  // rad,rad,rad,m, m, m
  auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.05);  // m/s

  graph->add(PriorFactor<Pose3>(X(0), InitiTnb, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(0), InitiVnb, velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(0), prior_imu_bias,
                                                bias_noise_model));

  // TimeStamp
  newTimestamps[X(0)] = gStartTime;
  newTimestamps[V(0)] = gStartTime;
  newTimestamps[B(0)] = gStartTime;

  // New Initial values
  initial_values.insert(X(0), InitiTnb);
  initial_values.insert(V(0), InitiVnb);
  initial_values.insert(B(0), prior_imu_bias);
}

void SaveNavState(const double &NavEpoch, const int &NavInd) {
  // 读取结果
  gtsam::Pose3 Tnb = currentEstimate.at<gtsam::Pose3>(X(NavInd));
  Vector3 Pos = Tnb.translation();
  Vector3 Vel = currentEstimate.at<gtsam::Vector3>(V(NavInd));
  Vector3 AttDeg = Tnb.rotation().rpy() * kRad2Deg;

  // 写入导航结果
  fNav << setprecision(8) << NavEpoch << setw(15) <<  // time
      setprecision(5) <<                              // 5 decimeter
      Pos(0) << setw(15) << Pos(1) << setw(15) << Pos(2) << setw(15) <<  // pos
      Vel(0) << setw(15) << Vel(1) << setw(15) << Vel(2) << setw(15) <<  // vel
      AttDeg(0) << setw(15) << AttDeg(1) << setw(15) << AttDeg(2) << setw(15)
       << endl;

  // 写入导航误差
  auto mpitTrue = mpNavTrue.find(NavEpoch);
  if (mpitTrue != mpNavTrue.end()) {
    Eigen::Vector3d ErrP = Pos - mpitTrue->second.topRows(3);
    Eigen::Vector3d ErrV = Vel - mpitTrue->second.middleRows<3>(3);
    Eigen::Vector3d ErrA = AttDeg - mpitTrue->second.bottomRows(3);
    if (ErrA.z() > 300) ErrA.z() -= 360;
    if (ErrA.z() < -300) ErrA.z() += 360;

    fNavErr << setprecision(9) << NavEpoch << setw(15) <<  // time
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
  auto CurBias = currentEstimate.at<gtsam::imuBias::ConstantBias>(B(NavInd));
  gtsam::Vector3 Ba = CurBias.accelerometer();
  gtsam::Vector3 Bg = CurBias.gyroscope();
  fBias << setprecision(8) << NavEpoch << setw(15) <<  // time
      setprecision(5) <<                                    // 5 decimeter
      Ba(0) << setw(15) << Ba(1) << setw(15) << Ba(2) << setw(15) <<  // Acc
      Bg(0) << setw(15) << Bg(1) << setw(15) << Bg(2) << setw(15) <<  // Gyro
      endl;
}

void SaveNavState(const double &NavEpoch, const NavState &State,
                  const imuBias::ConstantBias &CurBias) {
  // 读取结果
  Vector3 Pos = State.t();
  Vector3 Vel = State.v();
  Vector3 AttDeg = State.attitude().rpy() * kRad2Deg;

  // 写入导航结果
  fNav << setprecision(8) << NavEpoch << setw(15) <<  // time
      setprecision(5) <<                              // 5 decimeter
      Pos(0) << setw(15) << Pos(1) << setw(15) << Pos(2) << setw(15) <<  // pos
      Vel(0) << setw(15) << Vel(1) << setw(15) << Vel(2) << setw(15) <<  // vel
      AttDeg(0) << setw(15) << AttDeg(1) << setw(15) << AttDeg(2) << setw(15)
       << endl;

  // 写入导航误差
  auto RmpitTrue = mpNavTrue.lower_bound(NavEpoch);
  auto LmpitTrue = RmpitTrue;
  LmpitTrue--;
  if (RmpitTrue != mpNavTrue.end()) {
    auto mpitTrue =
        fabs(NavEpoch - LmpitTrue->first) <= fabs(NavEpoch - RmpitTrue->first)
            ? LmpitTrue
            : RmpitTrue;

    Eigen::Vector3d ErrP = Pos - mpitTrue->second.topRows(3);
    Eigen::Vector3d ErrV = Vel - mpitTrue->second.middleRows<3>(3);
    Eigen::Vector3d ErrA = AttDeg - mpitTrue->second.bottomRows(3);
    if (ErrA.z() > 300) ErrA.z() -= 360;
    if (ErrA.z() < -300) ErrA.z() += 360;

    fNavErr << setprecision(9) << NavEpoch << setw(15) <<  // time
        setprecision(5) <<                                 // 5 decimeter
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
  gtsam::Vector3 Ba = CurBias.accelerometer();
  gtsam::Vector3 Bg = CurBias.gyroscope();
  fBias << setprecision(8) << NavEpoch << setw(15) <<  // time
      setprecision(5) <<                                    // 5 decimeter
      Ba(0) << setw(15) << Ba(1) << setw(15) << Ba(2) << setw(15) <<  // Acc
      Bg(0) << setw(15) << Bg(1) << setw(15) << Bg(2) << setw(15) <<  // Gyro
      endl;
}

int main(int argc, char *argv[]) {

  if (argc != 4) {
    cerr << endl << "Usage: ./IG Inpath Outpath DataId" << endl;
    return 1;
  }

  string ConfigFile = "./config/NavGroup/20210123/" + string(argv[3]) + ".yaml";
  cv::FileStorage fsSettings(ConfigFile, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }
  gStartTime = fsSettings["IMUStartTime"];
  gEndTime = fsSettings["IMUEndTime"];
  Lag = fsSettings["SmootherLag"];

  string Inpath = string(argv[1]) + string(argv[3]);
  SetInitParas(Inpath + "/NavNED/NavNED.txt");
  LoadIMU(Inpath + "/m40/IMU_VIG.txt");
  LoadGNSS(Inpath + "/GNSS/RTK_NED.txt");
  gEndTime = min(mpGNSSs_.rbegin()->first, mpIMUs_.rbegin()->first);

  // Set up output file for plotting errors
  auto Outpath = string(argv[2]) + string(argv[3]);
  fNav.open(Outpath + "/NavState.txt");
  fNav << fixed;
  fNavErr.open(Outpath + "/IG_NavError.txt");
  fNavErr << fixed;
  fBias.open(Outpath + "/Bias.txt");
  fBias << fixed;

  SetIMUParas(ConfigFile);
  AddPrior();

  // Store previous state for imu integration and latest predicted outcome.
  BatchFixedLagSmoother smootherBatch(Lag);
  NavState prev_state(InitiTnb, InitiVnb);
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;

  // All priors have been set up, now iterate through the data file.
  int PreNavInd = 0, CurNavInd;  // after the prior: 0
  double gPreEpoch = gStartTime, gCurEpoch;

  for (auto &mpitObs : gmpObss) {
    CurNavInd = mpitObs.second;
    gCurEpoch = mpitObs.first;

    // 保存结果
    mpWinNavInds.insert(mpitObs);
    while (gCurEpoch - mpWinNavInds.begin()->first > Lag) {
      // 保存要边缘化的首个历元的GNSS，并删除
      SaveNavState(mpWinNavInds.begin()->first, mpWinNavInds.begin()->second);

      // if(gCurEpoch > 531490)
      //   cout << "wait" << endl;

      // 这个GNSS和第二个GNSS之间的IMU时刻也可以计算并存储了
      {
        gtsam::Pose3 Tnb =
            currentEstimate.at<gtsam::Pose3>(X(mpWinNavInds.begin()->second));
        Vector3 Vel =
            currentEstimate.at<gtsam::Vector3>(V(mpWinNavInds.begin()->second));
        auto FstNavState = NavState(Tnb, Vel);
        auto FstBias = currentEstimate.at<gtsam::imuBias::ConstantBias>(
            B(mpWinNavInds.begin()->second));
        PIM4Pred->resetIntegrationAndSetBias(FstBias);

        NavState CurState;
        auto LImu = mpIMUs_.find(mpWinNavInds.begin()->first);
        auto RImu = LImu;
        mpWinNavInds.erase(mpWinNavInds.begin());  // 放在这里方便下面找时间
        for (RImu++; RImu != mpIMUs_.end(); LImu++, RImu++) {
          if (RImu->first >= mpWinNavInds.begin()->first) break;

          double dt = RImu->first - LImu->first;
          Vector3 MeanAcc = (LImu->second.Acc + RImu->second.Acc) * 0.5;
          Vector3 MeanGyro = (LImu->second.Gyro + RImu->second.Gyro) * 0.5;
          PIM4Pred->integrateMeasurement(MeanAcc, MeanGyro, dt);
          CurState = PIM4Pred->predict(FstNavState, FstBias);

          SaveNavState(RImu->first, CurState, FstBias);
        }
      }
    }

    // IMU measurement
    {
      auto LImu = mpIMUs_.find(gPreEpoch), RImu = mpIMUs_.find(gPreEpoch);
      for (RImu++; RImu != mpIMUs_.end(); LImu++, RImu++) {
        double dt = RImu->first - LImu->first;
        Vector3 MeanAcc = (LImu->second.Acc + RImu->second.Acc) * 0.5;
        Vector3 MeanGyro = (LImu->second.Gyro + RImu->second.Gyro) * 0.5;
        preintegrated->integrateMeasurement(MeanAcc, MeanGyro, dt);

        if (RImu->first >= gCurEpoch) break;
      }
    }

    // GPS measurement
    {
      // Adding IMU factor and GPS factor and optimizing.
      {
        auto preint_imu =
            dynamic_cast<const PreintegratedImuMeasurements &>(*preintegrated);
        ImuFactor imu_factor(X(PreNavInd), V(PreNavInd), X(CurNavInd),
                             V(CurNavInd), B(PreNavInd), preint_imu);
        graph->add(imu_factor);

        imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
        double BiasSigma = 1e-3 * preintegrated->deltaTij();
        auto bias_noise_model = noiseModel::Isotropic::Sigma(6, BiasSigma);
        graph->add(BetweenFactor<imuBias::ConstantBias>(
            B(PreNavInd), B(CurNavInd), zero_bias, bias_noise_model));
      }

      {
        auto GNSSNoise =
            noiseModel::Diagonal::Sigmas(1.0 * mpGNSSs_.at(mpitObs.first).Std);
        GPSFactorLA gps_factor(X(CurNavInd),
                               Point3(mpGNSSs_.at(mpitObs.first).t_ng),
                               GNSSNoise, mt_ig);
        Eigen::Vector3d err = gps_factor.evaluateError(mpGNSSs_.at(mpitObs.first).Tnb);
        graph->add(gps_factor);
      }

      // New key time stamps
      newTimestamps[X(CurNavInd)] = gCurEpoch;
      newTimestamps[V(CurNavInd)] = gCurEpoch;
      newTimestamps[B(CurNavInd)] = gCurEpoch;

      // Now optimize and compare results.
      prop_state = preintegrated->predict(prev_state, prev_bias);

      initial_values.insert(X(CurNavInd), prop_state.pose());
      initial_values.insert(V(CurNavInd), prop_state.v());
      initial_values.insert(B(CurNavInd), prev_bias);

      smootherBatch.update(*graph, initial_values, newTimestamps);
      currentEstimate = smootherBatch.calculateEstimate();

      // Overwrite the beginning of the preintegration for the next step.
      prev_state = NavState(currentEstimate.at<Pose3>(X(CurNavInd)),
                            currentEstimate.at<Vector3>(V(CurNavInd)));
      prev_bias = currentEstimate.at<imuBias::ConstantBias>(B(CurNavInd));

      // Reset the preintegration object.
      newTimestamps.clear();
      graph->resize(0);
      initial_values.clear();
      preintegrated->resetIntegrationAndSetBias(prev_bias);
    }

    system("clear");
    cout << "Current/End Time: " <<  gCurEpoch  << "/" << gEndTime << endl;

    gPreEpoch = gCurEpoch;
    PreNavInd = CurNavInd;
  }

  for (auto mpit = mpWinNavInds.begin(); mpit != mpWinNavInds.end();) {
    // 保存要边缘化的首个历元的GNSS，并删除
    SaveNavState(mpit->first, mpit->second);

    // 这个GNSS和第二个GNSS之间的IMU时刻也可以计算并存储了
    {
      gtsam::Pose3 Tnb = currentEstimate.at<gtsam::Pose3>(X(mpit->second));
      Vector3 Vel = currentEstimate.at<gtsam::Vector3>(V(mpit->second));
      auto FstNavState = NavState(Tnb, Vel);
      auto FstBias =
          currentEstimate.at<gtsam::imuBias::ConstantBias>(B(mpit->second));
      PIM4Pred->resetIntegrationAndSetBias(FstBias);

      NavState CurState;
      auto LImu = mpIMUs_.find(mpit->first);
      auto RImu = LImu;
      mpit = mpWinNavInds.erase(mpWinNavInds.begin());  // 放这方便下面找时间
      for (RImu++; RImu != mpIMUs_.end(); LImu++, RImu++) {
        if (RImu->first >= mpit->first) break;

        double dt = RImu->first - LImu->first;
        Vector3 MeanAcc = (LImu->second.Acc + RImu->second.Acc) * 0.5;
        Vector3 MeanGyro = (LImu->second.Gyro + RImu->second.Gyro) * 0.5;
        PIM4Pred->integrateMeasurement(MeanAcc, MeanGyro, dt);
        CurState = PIM4Pred->predict(FstNavState, FstBias);

        SaveNavState(RImu->first, CurState, FstBias);
      }
    }
  }

  return 0;
}
