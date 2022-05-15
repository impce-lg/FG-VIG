#include "VIGCommon.h"

namespace VIG {
double gStartTime, gEndTime;
Vector3d gG;
bool gbUseGNSS;                     // default false
bool gbUseCam;                      // default false

double gFocalLength = 0., gMinKeyFrameParallax = 0.;
Matrix3d gCamK;
Eigen::Matrix<double, 5, 1> gDistCoef;
int gNumUnDisIter;
double gInitDepth = 5., gInvDepthThreshold;
int gFps, gMinCoFrames, gMinCoFeatures;

eVIGState geVIGState;
// Test params
timespec gTestTime1, gTestTime2;
// int gTestFramesThreshold;

bool VectorLessThan(const VectorXd &Vx, const VectorXd &VThres) {
  assert(Vx.size() == VThres.size());

  auto Vx_CwiseAbs = Vx.cwiseAbs();
  auto VThres_CwiseAbs = VThres.cwiseAbs();
  bool bLessThan = true;
  for (unsigned i = 0; i < Vx.size(); i++) {
    if (Vx_CwiseAbs(i) > VThres_CwiseAbs(i)) {
      bLessThan = false;
      break;
    }
  }

  return bLessThan;
}

double GetVariance(const std::vector<double> &vector_)
{
    double sum = std::accumulate(std::begin(vector_), std::end(vector_), 0.0);
    double mean = sum / vector_.size();

    double accum = 0.0;
    std::for_each(std::begin(vector_), std::end(vector_), [&](const double d){
       accum += (d-mean)*(d-mean);
    });

    return accum / vector_.size();
}


double Get1dVectorStd(const vector<double> &V1d){
  std::vector<double> vX;
  
  for(auto it : V1d){
    if (it != it)  // -NaN
      continue;
    vX.push_back(it);
  }

  if (vX.size() < 10)  // keyframe should more than a threshold
    return false;

  return std::sqrt(GetVariance(vX));
}

Vector3d Get3dVectorStd(const vector<Eigen::Vector3d> &V3d) {
  std::vector<double> vX;
  std::vector<double> vY;
  std::vector<double> vZ;
  for (size_t i = 0; i < V3d.size(); i++) {
    Eigen::Vector3d xyz = V3d[i];
    if (xyz(0) != xyz(0) || xyz(1) != xyz(1) || xyz(2) != xyz(2))  // -NaN
      continue;

    vX.push_back(xyz(0));
    vY.push_back(xyz(1));
    vZ.push_back(xyz(2));
  }

  double staDevOfX = std::sqrt(GetVariance(vX));
  double staDevOfY = std::sqrt(GetVariance(vY));
  double staDevOfZ = std::sqrt(GetVariance(vZ));

  return Vector3d(staDevOfX, staDevOfY, staDevOfZ);
}

Matrix3d Skew(const Vector3d &vec3d) {
  Matrix3d SkewMat;
  SkewMat << 0.0, -vec3d(2), vec3d(1), vec3d(2), 0.0, -vec3d(0), -vec3d(1),
      vec3d(0), 0.0;

  return SkewMat;
}

Matrix4d Q1(const Quaterniond Q) {
  double qw = Q.w();
  Vector3d qv = Vector3d(Q.x(), Q.y(), Q.z());

  Matrix4d Q1;
  Q1.block<3, 3>(0, 0) = qw * Matrix3d::Identity() + Skew(qv);
  Q1.block<3, 1>(0, 3) = qv;
  Q1.block<1, 3>(3, 0) = -qv;
  Q1(3, 3) = qw;

  return Q1;
}

Matrix4d Q2(const Quaterniond Q) {
  double qw = Q.w();
  Vector3d qv = Vector3d(Q.x(), Q.y(), Q.z());

  Matrix4d Q2;
  Q2.block<3, 3>(0, 0) = qw * Matrix3d::Identity() - Skew(qv);
  Q2.block<3, 1>(0, 3) = qv;
  Q2.block<1, 3>(3, 0) = -qv;
  Q2(3, 3) = qw;

  return Q2;
}

// maybe unuesd
IMUModel GetIMUModel(
    const int IMUType) {
  double AccSigma(1e-3);
  double GyroSigma(1e-3);
  double IntegCov(1e-6);
  double BiasAccRWSigma(1e-4);
  double BiasOmegaRWSigma(1e-5);
  double BiasAccOmegaInt(1e-6);

  IMUModel sIMUModel;

  switch (IMUType) {
    case 5: {                            // 3dm gx 3 35
      AccSigma = 0.04 * 1.0e-2;          // 2 ug
      GyroSigma = 18 * kDeg2Rad / 3600;  // 2 °/hr
      BiasAccRWSigma = 80 * 1.0e-5;        // 80 ug with 1 Hz
      BiasOmegaRWSigma = 0.03 * kDeg2Rad;  // 0.03 deg/sec/sqrt(Hz)
      IntegCov = 1e-8;
      BiasAccOmegaInt = 1e-5;
      break;
    }
    case 8: {                           // OXTS RT 3003
      AccSigma = 0.005 / 60;            // 0.005 m/s/sqrt(hr)
      GyroSigma = 0.2 * kDeg2Rad / 60;  // 0.2 °/sqrt(hr)
      BiasAccRWSigma = 2 * 9.8 * 1e-6;  // 2 ug
      BiasOmegaRWSigma = 2 * kDeg2Rad / 3600;  // 2 °/hr
      IntegCov = 1e-8;
      BiasAccOmegaInt = 1e-5;
      break;
    }
    case 9: {  // gtsam example
               // double accel_noise_sigma = 0.03924;
               // double gyro_noise_sigma = 0.0205689024915;
               // double accel_bias_rw_sigma = 0.04905;
               // double gyro_bias_rw_sigma = 0.0001454441043;
    }
    case 10: {                            // VIG Simulation
      AccSigma = 0.019;                   // m/(s^2) * 1/sqrt(hz)
      GyroSigma = 0.015 / 3600;           // rad/s * 1/sqrt(hz)
      BiasAccRWSigma = 0.0001;            // 80 ug with 1 Hz
      BiasOmegaRWSigma = 1.0e-5;          // 0.03 deg/sec/sqrt(Hz)
      IntegCov = 1e-8;
      BiasAccOmegaInt = 1e-5;
      break;
    }

    default: {
      cout << "Error IMU type!" << endl;
    }
  }

  sIMUModel.accelerometerCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(AccSigma, 2);
  sIMUModel.gyroscopeCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(GyroSigma, 2);

  sIMUModel.biasAccCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(BiasAccRWSigma, 2);
  sIMUModel.biasOmegaCovariance =
      gtsam::Matrix33::Identity(3, 3) * pow(BiasOmegaRWSigma, 2);

  sIMUModel.integrationCovariance =
      gtsam::Matrix33::Identity(3, 3) *
      IntegCov;  // error committed in integrating position from
  // velocities
  sIMUModel.biasAccOmegaInt =
      gtsam::Matrix::Identity(6, 6) *
      BiasAccOmegaInt;  // error in the bias used for preintegration

  return sIMUModel;
}

Matrix3d R1(double XrotRad) {
  Matrix3d R1;
  R1 << 1, 0, 0, 0, cos(XrotRad), sin(XrotRad), 0, -sin(XrotRad), cos(XrotRad);

  return R1;
}

Matrix3d R2(double YrotRad) {
  Matrix3d R2;
  R2 << cos(YrotRad), 0, -sin(YrotRad), 0, 1, 0, sin(YrotRad), 0, cos(YrotRad);

  return R2;
}

Matrix3d R3(double ZrotRad) {
  Matrix3d R3;
  R3 << cos(ZrotRad), sin(ZrotRad), 0, -sin(ZrotRad), cos(ZrotRad), 0, 0, 0, 1;

  return R3;
}

void RemoveAttAnglesRedundancy(Vector3d &AttRad) {
  // There is some reduncancy in euler angle values((i) is not supported yet):
  // (i)  If m(1)==m(3) then e=[a b c] and e=[a+-pi -b c+-pi] are equivalent.
  //   The output of this routine will always have b>=0;
  // (ii) If m(1)~=m(3) then e=[a b c] and e=[a+-pi pi-b c+-pi] are equivalent.
  //   The output of this routine will always have |b|<=pi/2
  if (fabs(AttRad(1)) > PI / 2) {
    AttRad(0) -= (2 * (AttRad(0) > 0) - 1) * PI;
    AttRad(1) = (2 * (AttRad(1) > 0) - 1) * PI - AttRad(1);
    AttRad(2) -= (2 * (AttRad(2) > 0) - 1) * PI;
  }
}

Vector3d CalgravityN(const Vector3d &BLH) {
  Vector3d gn;
  gn.setZero();

  double m = SQR(kOmega_WGS) * kA_84 * kA_84 * kB_84 / kGM_Earth;
  double g0 = (kA_84 * kTa * cos(BLH(0)) * cos(BLH(0)) +
               kB_84 * kTb * sin(BLH(0)) * sin(BLH(0))) /
              sqrt(kA_84 * kA_84 * cos(BLH(0)) * cos(BLH(0)) +
                   kB_84 * kB_84 * sin(BLH(0)) * sin(BLH(0)));
  gn(2) =
      g0 *
      (1 -
       2 * (1 + kFlattening + m - 2 * kFlattening * sin(BLH(0)) * sin(BLH(0))) *
           BLH(2) / kA_84 +
       3 * BLH(2) * BLH(2) / kA_84 / kA_84);

  return gn;
}

Matrix3d BL2Rne(const Vector2d &BL, const string &strNavFrame) {
  Matrix3d Rne;

  double sB = sin(BL(0));
  double cB = cos(BL(0));
  double sL = sin(BL(1));
  double cL = cos(BL(1));
  if (strNavFrame == "NED")
    Rne << -sB * cL, -sB * sL, cB, -sL, cL, 0.0, -cB * cL, -cB * sL, -sB;
  else if (strNavFrame == "ENU")
    Rne << -sL, cL, 0.0, -sB * cL, -sB * sL, cB, cB * cL, cB * sL, sB;
  else {
    cerr << "Unknown NavFrame in BL2Rne()!" << endl;
    exit(1);
  }

  return Rne;
}

Matrix3d BL2Ren(const Vector2d &BL, const string &strNavFrame) {
  Matrix3d Rne;
  Rne = BL2Rne(BL, strNavFrame);

  return Rne.transpose();
}

// ecef2geo & geo2ecef are refered to gpstk
Vector3d ECEF2BLH(const Vector3d &XYZ) {
  double p, slat, N, htold, latold;
  Vector3d BLH;

  p = XYZ.head(2).norm();
  // if (p < POSITION_TOLERANCE / 5)
  // { // pole or origin
  //     BLH(0) = (XYZ(2) > 0 ? 90.0 : -90.0);
  //     BLH(1) = 0; // lon undefined, really
  //     BLH(2) = fabs(XYZ(2)) - kA_84 * SQRT(1.0 - k_e1_2);
  //     return;
  // }
  BLH(0) = atan2(XYZ(2), p * (1.0 - k_e1_2));
  BLH(2) = 0;
  for (int i = 0; i < 5; i++) {
    slat = sin(BLH(0));
    N = kA_84 / sqrt(1.0 - k_e1_2 * slat * slat);
    htold = BLH(2);
    BLH(2) = p / cos(BLH(0)) - N;
    latold = BLH(0);
    BLH(0) = atan2(XYZ(2), p * (1.0 - k_e1_2 * (N / (N + BLH(2)))));
    if (fabs(BLH(0) - latold) < 1.0e-9 && fabs(BLH(2) - htold) < 1.0e-9 * kA_84)
      break;
  }
  BLH(1) = atan2(XYZ(1), XYZ(0));
  // if (BLH(1) < 0.0)
  //     BLH(1) += TWO_PI;

  return BLH;
}

Vector3d BLH2ECEF(const Vector3d &BLH) {
  Vector3d XYZ;
  double clat = cos(BLH(0));
  double slat = sin(BLH(0));
  double clon = cos(BLH(1));
  double slon = sin(BLH(1));

  double N = kA_84 / sqrt(1.0 - k_e1_2 * slat * slat);
  XYZ(0) = (N + BLH(2)) * clat * clon;
  XYZ(1) = (N + BLH(2)) * clat * slon;
  XYZ(2) = (N * (1.0 - k_e1_2) + BLH(2)) * slat;

  return XYZ;
}

Vector3d PointInLocalFrame(const Vector3d &CurBLH, const Vector3d &BaseXYZ,
                           const string &strNavFrame) {
  Vector3d RovXYZ = BLH2ECEF(CurBLH);
  Vector3d BaseBLH = ECEF2BLH(BaseXYZ);

  Matrix3d baseRne = BL2Rne(BaseBLH.head(2), strNavFrame);
  Vector3d dXYZ = RovXYZ - BaseXYZ;
  Vector3d NavPoint = baseRne * dXYZ;

  return NavPoint;
}

Matrix3d RTKlibStd2CovMatrix(Eigen::Matrix<double, 6, 1> &RTKliblStd) {
  Matrix3d CovMatrix;
  Vector3d atVar;  // autocorelative var
  double maxCov = RTKliblStd.maxCoeff();

  atVar = RTKliblStd.head(3);
  atVar = atVar.cwiseAbs2();
  CovMatrix = atVar.asDiagonal();

  for (int i = 0; i < 6; i++) {
    if (RTKliblStd(i) == 0.0)  // very rarerly, but sometimes var is 0
      RTKliblStd(i) = 2 * maxCov;
  }

  CovMatrix(0, 1) = fabs(RTKliblStd(3)) * RTKliblStd(3);
  CovMatrix(1, 0) = CovMatrix(0, 1);
  CovMatrix(1, 2) = fabs(RTKliblStd(4)) * RTKliblStd(4);
  CovMatrix(2, 1) = CovMatrix(1, 2);
  CovMatrix(0, 2) = fabs(RTKliblStd(5)) * RTKliblStd(5);
  CovMatrix(2, 0) = CovMatrix(0, 2);

  return CovMatrix;
}

Eigen::Matrix<double, 6, 1> CovMatrix2RTKlibStd(const Matrix3d &CovMatrix) {
  Eigen::Matrix<double, 6, 1> RTKliblStd;
  Vector3d atStd;  // autocorelative std

  atStd = CovMatrix.diagonal();
  atStd = atStd.cwiseSqrt();

  // sg=sign(CovMatrix);
  RTKliblStd.head(3) = atStd;
  RTKliblStd(3) = SGN(CovMatrix(0, 1)) * sqrt(fabs(CovMatrix(0, 1)));
  RTKliblStd(4) = SGN(CovMatrix(1, 2)) * sqrt(fabs(CovMatrix(1, 2)));
  RTKliblStd(5) = SGN(CovMatrix(0, 2)) * sqrt(fabs(CovMatrix(0, 2)));

  return RTKliblStd;
}

int CountMatch(const pair<Frame, map<int, Point2D>> &LFrame,
               const pair<Frame, map<int, Point2D>> &RFrame) {
  int NCorres = 0;

  for (auto mpit : LFrame.second) {
    if (RFrame.second.find(mpit.first) == RFrame.second.end()) continue;
    NCorres++;
  }

  return NCorres;
}

bool EnoughParallax(const pair<Frame, map<int, Point2D>> &LFrame,
                    const pair<Frame, map<int, Point2D>> &RFrame) {
  double sum_parallax = 0, avg_parallax;
  int NCorres = 0;

  for (auto mpit : LFrame.second) {
    if (RFrame.second.find(mpit.first) == RFrame.second.end()) continue;

    Vector3d LUxy1 = mpit.second.Uxy1,
             RUxy1 = RFrame.second.at(mpit.first).Uxy1;

    double parallax = (LUxy1.head(2) - RUxy1.head(2)).norm();
    sum_parallax = sum_parallax + parallax;
    NCorres++;
  }

  avg_parallax = sum_parallax / NCorres;

  return NCorres >= gMinCoFeatures && avg_parallax >= gMinKeyFrameParallax;
}

bool EnoughParallax(const pair<Frame, map<int, Point2D>> &LFrame,
                    const pair<Frame, map<int, Point2D>> &RFrame,
                    int &NumCorr) {
  double sum_parallax = 0, avg_parallax;
  int NCorres = 0;

  for (auto mpit : LFrame.second) {
    if (RFrame.second.find(mpit.first) == RFrame.second.end()) continue;

    Vector3d LUxy1 = mpit.second.Uxy1,
             RUxy1 = RFrame.second.at(mpit.first).Uxy1;

    double parallax = (LUxy1.head(2) - RUxy1.head(2)).norm();
    sum_parallax = sum_parallax + parallax;
    NCorres++;
  }

  avg_parallax = sum_parallax / NCorres;

  NumCorr = NCorres;

  return NCorres >= gMinCoFeatures && avg_parallax >= gMinKeyFrameParallax;
}

bool GetMatch(const pair<Frame, map<int, Point2D>> &LFrame,
              const pair<Frame, map<int, Point2D>> &RFrame,
              vector<pair<Vector3d, Vector3d>> &vprCorresUxy1Pair) {
  double sum_parallax = 0, avg_parallax;
  int NCorres = 0;

  for (auto mpit : LFrame.second) {
    if (RFrame.second.find(mpit.first) == RFrame.second.end()) continue;

    Vector3d LUxy1 = mpit.second.Uxy1,
             RUxy1 = RFrame.second.at(mpit.first).Uxy1;
    vprCorresUxy1Pair.push_back(make_pair(LUxy1, RUxy1));

    double parallax = (LUxy1.head(2) - RUxy1.head(2)).norm();
    sum_parallax = sum_parallax + parallax;
    NCorres++;
  }

  avg_parallax = sum_parallax / NCorres;

  return NCorres >= gMinCoFeatures && avg_parallax >= gMinKeyFrameParallax;
}

void GetRoi(const int Col, const int Row, int MaskWidth, cv::Rect &Roi) {
  Roi = cv::Rect(MaskWidth, MaskWidth, Col - MaskWidth, Row - MaskWidth);
}

double CalCVDistance(const cv::Point2f &pt1, const cv::Point2f &pt2) {
  // printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

void GetDistortion(const Vector2d &mxy_d, Vector2d &d_u) {
  double k1 = gDistCoef(0);  // k1
  double k2 = gDistCoef(1);  // k2
  double p1 = gDistCoef(2);  // p1
  double p2 = gDistCoef(3);  // p2

  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

  mx2_u = mxy_d(0) * mxy_d(0);
  my2_u = mxy_d(1) * mxy_d(1);
  mxy_u = mxy_d(0) * mxy_d(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  d_u << mxy_d(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
      mxy_d(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void LiftProjective(const Vector2d &uv, const int NumUnDisIter, Vector3d &xyz) {
  double mx_d, my_d, mx_u, my_u;

  double m_inv_K11 = 1.0 / gCamK(0, 0);
  double m_inv_K13 = -gCamK(0, 2) / gCamK(0, 0);
  double m_inv_K22 = 1.0 / gCamK(1, 1);
  double m_inv_K23 = -gCamK(1, 2) / gCamK(1, 1);
  mx_d = m_inv_K11 * uv(0) + m_inv_K13;
  my_d = m_inv_K22 * uv(1) + m_inv_K23;

  // Lift points to normalised plane
  mx_d = 1 / gCamK(0, 0) * uv(0) - gCamK(0, 2) / gCamK(0, 0);
  my_d = 1 / gCamK(1, 1) * uv(1) - gCamK(1, 2) / gCamK(1, 1);

  // Recursive distortion model
  Vector2d d_u;
  GetDistortion(Vector2d(mx_d, my_d), d_u);
  // Approximate value
  mx_u = mx_d - d_u(0);
  my_u = my_d - d_u(1);

  for (int i = 1; i < NumUnDisIter; ++i) {
    GetDistortion(Vector2d(mx_u, my_u), d_u);
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);
  }
  // Obtain a projective ray
  xyz << mx_u, my_u, 1.0;
}

void ProjectUXYZ2Duv(const Vector3d &UXYZ, Vector2d &Duv) {
  double inv_Z = 1 / UXYZ(2);
  Vector2d Uxy = UXYZ.head(2) * inv_Z;

  // Add distortion:
  double r2 = SQR(Uxy(0)) + SQR(Uxy(1));
  double r4 = SQR(r2), r6 = CUB(r2);

  // Radial distortion:
  double cdist =
      1 + gDistCoef(0) * r2 + gDistCoef(1) * r4 + gDistCoef(4) * r6;
  Vector2d xd1 = Uxy * cdist;

  // tangential distortion:
  double a1 = 2. * Uxy(0) * Uxy(1);
  double a2 = r2 + 2 * SQR(Uxy(0));
  double a3 = r2 + 2 * SQR(Uxy(1));

  Vector2d delta_x;
  delta_x(0) = gDistCoef(2) * a1 + gDistCoef(3) * a2;
  delta_x(1) = gDistCoef(2) * a3 + gDistCoef(3) * a1;

  // Vector3d aa = (2 * gDistCoef(2) * x(1) + 6 * gDistCoef(3) * x(0)) *
  // Vector3d(1, 1, 1); Vector3d bb = (2 * gDistCoef(2) * x(0) + 2 *
  // gDistCoef(3) * x(1)) * Vector3d(1, 1, 1); Vector3d cc = (6 *
  // gDistCoef(2) * x(1) + 2 * gDistCoef(3) * x(0)) * Vector3d(1, 1, 1);

  Vector2d xd2 = xd1 + delta_x;

  // Add Skew:
  Vector2d xd3;
  xd3(0) = xd2(0) + gCamK(0, 1) / gCamK(0, 0) * xd2(1);
  xd3(1) = xd2(1);

  // Pixel coordinates:
  Duv(0) = xd3(0) * gCamK(0, 0) + gCamK(0, 2);
  Duv(1) = xd3(1) * gCamK(1, 1) + gCamK(1, 2);
}

void ProjectUXYZ2Uuv(const Vector3d &UXYZ, Vector2d &Uuv) {
  Vector3d Uxy1 = UXYZ / UXYZ(2);
  // Vector3d FeatureUuvrepj = gCamK * Uxy1;
  // Uuv = FeatureUuvrepj.head(2);

  Uuv(0) = Uxy1(0) * gCamK(0, 0) + gCamK(0, 2);
  Uuv(1) = Uxy1(1) * gCamK(1, 1) + gCamK(1, 2);
}

// UnDis_Dis_Flag, 0： UnDis; 1: Dis
bool ProjectUXYZ2UV(const Vector3d &UXYZ, const eUnDis_Dis_Flag UnDis_Dis_Flag,
                    const int Col, const int Row, Vector2d &UV) {
  // is in front of the camera
  if (UXYZ(2) <= 0) {
    // UV = Vector2d(0, 0);
    return 0;
  }

  // Is in front of the camera?
  if ((atan2(UXYZ(0), UXYZ(2)) * kRad2Deg < -70) ||
      (atan2(UXYZ(0), UXYZ(2)) * kRad2Deg > 70) ||
      (atan2(UXYZ(1), UXYZ(2)) * kRad2Deg < -70) ||
      (atan2(UXYZ(1), UXYZ(2)) * kRad2Deg > 70)) {
    // UV = Vector2d(0, 0);
    return 0;
  }

  if (UnDis_Dis_Flag == UNDIS) {
    Vector2d Uuv;
    ProjectUXYZ2Uuv(UXYZ, Uuv);
    UV = Uuv;
  } else {
    Vector2d Duv;
    ProjectUXYZ2Duv(UXYZ, Duv);
    UV = Duv;
  }

  // Is visible in the image?
  if ((UV(0) <= 0) || (UV(0) >= Col) || (UV(1) <= 0) || (UV(1) >= Row)) {
    // UV = Vector2d(0, 0);
    return 0;
  }

  return 1;
}

void GetXYdistortbyXYnormalized(const Vector2d &xyn, Matrix2d &dxyd_dxyn) {
  double k1 = gDistCoef(0);
  double k2 = gDistCoef(1);
  double p1 = gDistCoef(2);
  double p2 = gDistCoef(3);
  double k3 = gDistCoef(4);

  double xn = xyn(0);
  double yn = xyn(1);

  dxyd_dxyn.setZero();
  // xyd to xyn
  dxyd_dxyn(0, 0) = k2 * SQR(SQR(xn) + SQR(yn)) + k3 * CUB(SQR(xn) + SQR(yn)) +
                    6 * p2 * xn + 2 * p1 * yn +
                    xn * (2 * k1 * xn + 4 * k2 * xn * (SQR(xn) + SQR(yn)) +
                          6 * k3 * xn * SQR(SQR(xn) + SQR(yn))) +
                    k1 * (SQR(xn) + SQR(yn)) + 1;
  // xd to yn
  dxyd_dxyn(0, 1) = 2 * p1 * xn + 2 * p2 * yn +
                    xn * (2 * k1 * yn + 4 * k2 * yn * (SQR(xn) + SQR(yn)) +
                          6 * k3 * yn * SQR(SQR(xn) + SQR(yn)));
  // yd to (xn, yn)
  dxyd_dxyn(1, 0) = 2 * p1 * xn + 2 * p2 * yn +
                    yn * (2 * k1 * xn + 4 * k2 * xn * (SQR(xn) + SQR(yn)) +
                          6 * k3 * xn * SQR(SQR(xn) + SQR(yn)));
  dxyd_dxyn(1, 1) = k2 * SQR(SQR(xn) + SQR(yn)) + k3 * CUB(SQR(xn) + SQR(yn)) +
                    2 * p2 * xn + 6 * p1 * yn +
                    yn * (2 * k1 * yn + 4 * k2 * yn * (SQR(xn) + SQR(yn)) +
                          6 * k3 * yn * SQR(SQR(xn) + SQR(yn))) +
                    k1 * (SQR(xn) + SQR(yn)) + 1;
}

void TriangulateOneFt_DualView(const Vector2d &LUxy, const gtsam::Pose3 &Tw_lf,
                              const Vector2d &RUxy, const gtsam::Pose3 &Tw_rf,
                              double &InvDepth_LFrame, Vector3d &Pos3w) {
  assert(LUxy != RUxy && !(Tw_lf.equals(Tw_rf)));
  gtsam::Pose3 Trl = Tw_rf.inverse() * Tw_lf;

  Eigen::Matrix<double, 4, 4> Tlw_mt, Trw_mt;
  Tlw_mt = Matrix4d::Identity();  // Tw_lf.inverse().matrix();
  Trw_mt = Trl.matrix();          // Tw_rf.inverse().matrix();

  Matrix4d DesignMatrix = Matrix4d::Zero();
  DesignMatrix.row(0) = LUxy[0] * Tlw_mt.row(2) - Tlw_mt.row(0);
  DesignMatrix.row(1) = LUxy[1] * Tlw_mt.row(2) - Tlw_mt.row(1);
  DesignMatrix.row(2) = RUxy[0] * Trw_mt.row(2) - Trw_mt.row(0);
  DesignMatrix.row(3) = RUxy[1] * Trw_mt.row(2) - Trw_mt.row(1);
  Vector4d PointSvd;
  PointSvd =
      DesignMatrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  InvDepth_LFrame = PointSvd(3) / PointSvd(2);

  //! 转为世界坐标(1)，请保留以备与(2)比较
  // Vector3d Point3_LFrame_;
  // Point3_LFrame_(0) = PointSvd(0) / PointSvd(3);
  // Point3_LFrame_(1) = PointSvd(1) / PointSvd(3);
  // Point3_LFrame_(2) = PointSvd(2) / PointSvd(3);
  //! 转为世界坐标(2)
  Vector3d Point3_LFrame;
  Point3_LFrame(0) = LUxy[0] / InvDepth_LFrame;
  Point3_LFrame(1) = LUxy[1] / InvDepth_LFrame;
  Point3_LFrame(2) = 1. / InvDepth_LFrame;
  Pos3w = Tw_lf * Point3_LFrame;
}

// Pose from Lfrm to Rfrm
bool SolveTrlByFMat(const vector<pair<Vector3d, Vector3d>> &vprCorresUxy1Pair,
                    gtsam::Pose3 &Trl) {
  if (vprCorresUxy1Pair.size() >= 15) {
    vector<cv::Point2f> ll, rr;
    for (size_t i = 0; i < vprCorresUxy1Pair.size(); i++) {
      ll.push_back(cv::Point2f(vprCorresUxy1Pair[i].first(0),
                               vprCorresUxy1Pair[i].first(1)));
      rr.push_back(cv::Point2f(vprCorresUxy1Pair[i].second(0),
                               vprCorresUxy1Pair[i].second(1)));
    }
    cv::Mat mask;
    cv::Mat E = cv::findFundamentalMat(ll, rr, cv::FM_RANSAC,
                                       0.5 / gFocalLength, 0.99, mask);
    cv::Mat cameraMatrix =
        (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat rot, trans;
    int inlier_cnt = cv::recoverPose(E, ll, rr, cameraMatrix, rot, trans, mask);
    // cout << "inlier_cnt " << inlier_cnt << endl;

    Eigen::Matrix3d R_ll2rr;
    Eigen::Vector3d T_ll2rr;
    for (int i = 0; i < 3; i++) {
      T_ll2rr(i) = trans.at<double>(i, 0);
      for (int j = 0; j < 3; j++) R_ll2rr(i, j) = rot.at<double>(i, j);
    }

    gtsam::Rot3 Rot_ll2rr = gtsam::Rot3(R_ll2rr);
    gtsam::Point3 Trans_ll2rr = gtsam::Point3(T_ll2rr);
    Trl = gtsam::Pose3(Rot_ll2rr, Trans_ll2rr);

    if (inlier_cnt > 12)
      return true;
    else
      return false;
  }
  return false;
}

bool SolveTncByPnP(const vector<cv::Point3f> &vCVPos3ws,
                   const vector<cv::Point2f> &vCVUxys,
                   gtsam::Pose3 &Tnc) {
  //! PnP的初始值为上一帧的变换矩阵
  gtsam::Pose3 Tcx_w_inital = Tnc.inverse();

  cv::Mat r, rvec, t, D, tmp_r;
  cv::eigen2cv(Tcx_w_inital.rotation().matrix(), tmp_r);
  cv::Rodrigues(tmp_r, rvec);
  cv::eigen2cv(Vector3d(Tcx_w_inital.translation().matrix()), t);
  cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  bool bPnPStatus = cv::solvePnP(vCVPos3ws, vCVUxys, K, D, rvec, t, 1);
  if (!bPnPStatus) return false;

  //! 这里要注意的是PnP求解的结果是将3D点从世界坐标系转到相机坐标系的变换，
  //! 而我们要求解的相机位姿是相机坐标系到世界坐标系的变换
  cv::Rodrigues(rvec, r);
  // cout << "r " << endl << r << endl;
  MatrixXd Rpnp_w2cx;
  cv::cv2eigen(r, Rpnp_w2cx);
  MatrixXd Tpnp_w2cx;
  cv::cv2eigen(t, Tpnp_w2cx);

  gtsam::Rot3 Rot_w2cx = gtsam::Rot3(Rpnp_w2cx);
  gtsam::Point3 Trans_w2cx = gtsam::Point3(Tpnp_w2cx);
  Tcx_w_inital = gtsam::Pose3(Rot_w2cx, Trans_w2cx);

  Tnc = Tcx_w_inital.inverse();

  return true;
}

// Print
void PrintTns2PA(const double Epoch, const gtsam::Pose3 Tns) {
  Vector3d Tran = Tns.translation(), Att = Tns.rotation().rpy() * kRad2Deg;

  cout.setf(ios::fixed,ios::floatfield);
  cout << setprecision(5) << Epoch << " " << Tran.transpose() << " "
       << Att.transpose() << endl;
}

void PrintNavs2PVA(const double Epoch, const gtsam::NavState Navs) {
  Vector3d Tran = Navs.pose().translation(),
           Att = Navs.pose().rotation().rpy() * kRad2Deg, Vel = Navs.v();

  cout.setf(ios::fixed,ios::floatfield);
  cout << setprecision(5) << Epoch << " " << Tran.transpose() << " "
       << Vel.transpose() << " " << Att.transpose() << endl;
}

// // Unused functions
// double InvTPR2Depth(const Vector3d &InvTPR) {
//   double Depth = -1.;
//   double theta = InvTPR(0), phi = InvTPR(1), rho = InvTPR(2);
//   Depth = cos(phi) * cos(theta) / rho;

//   return Depth;
// }

// void InvTPR2Pos3w(const gtsam::Pose3 &Tnc, const Vector3d &InvTPR,
//                     gtsam::Vector3 &Pos3w) {
//   // Calculate the 3D coordinates of the landmark in the Pose1 frame
//   double theta = InvTPR(0), phi = InvTPR(1), rho = InvTPR(2);
//   gtsam::Point3 pose1_P_landmark(cos(phi) * sin(theta) / rho, sin(phi) / rho,
//                                  cos(phi) * cos(theta) / rho);
//   // Convert the landmark to world coordinates
//   Pos3w = Tnc.transform_from(pose1_P_landmark);
// }

// void Pos3w2InvTPR(const gtsam::Pose3 &Tnc,
//                     const gtsam::Vector3 &Pos3w, gtsam::Vector3 &InvTPR,
//                     double &Depth_StartFrame) {
//   Vector3d UXYZ = Tnc.inverse() * Pos3w;
//   Depth_StartFrame = UXYZ(2);

//   double Theta = atan2(UXYZ.x(), UXYZ.z());
//   double Phi = atan2(UXYZ.y(), sqrt(UXYZ.x() * UXYZ.x() +
//                                         UXYZ.z() * UXYZ.z()));
//   double Rho = 1. / (Depth_StartFrame * UXYZ.norm());

//   InvTPR = gtsam::Vector3(Theta, Phi, Rho);
// }

// void UXYZ_Pos3w2InvDepth(const Vector3d UXYZ,
//                                const gtsam::Pose3 &Tnc,
//                                const gtsam::Vector3 &Pos3w,
//                                gtsam::Vector3 &InvTPR,
//                                double &Depth_StartFrame) {
//   Vector3d Point3_StartFrm = Tnc.inverse() * Pos3w;
//   Depth_StartFrame = Point3_StartFrm(2);

//   double Theta = atan2(UXYZ.x(), UXYZ.z());
//   double Phi = atan2(UXYZ.y(), sqrt(UXYZ.x() * UXYZ.x() +
//                                         UXYZ.z() * UXYZ.z()));
//   double Rho = 1. / (Depth_StartFrame * UXYZ.norm());

//   InvTPR = gtsam::Vector3(Theta, Phi, Rho);
// }

// gtsam::Vector3 SetInvDepth(const Vector3d UXYZ, const double Depth) {
//   double Theta = atan2(UXYZ.x(), UXYZ.z());
//   double Phi = atan2(UXYZ.y(), sqrt(UXYZ.x() * UXYZ.x() +
//                                         UXYZ.z() * UXYZ.z()));
//   double Rho = 1. / (Depth * UXYZ.norm());

//   gtsam::Vector3 TPR = gtsam::Vector3(Theta, Phi, Rho);
//   return TPR;
// }

void TestSetTime1() {
  // if (CurImageId - InitImageId >= IdRange_Int2Cur)
  clock_gettime(CLOCK_REALTIME, &gTestTime1);
}

void TestSetTime2AndShowTimeRange() {
  // if (CurImageId - InitImageId >= IdRange_Int2Cur)
  // {
  clock_gettime(CLOCK_REALTIME, &gTestTime2);
  cout /*<< endl*/
      << (gTestTime2.tv_sec - gTestTime1.tv_sec) * 1000 +
             (gTestTime2.tv_nsec - gTestTime1.tv_nsec) / 1000000
      << "  ";  // "ms" << endl;
                // }
}

// void TestSetTime1(double InitiTime, double CurTime, double TimeRange_Int2Cur)
// {
//     if (CurTime - InitiTime >= TimeRange_Int2Cur)
//         clock_gettime(CLOCK_REALTIME, &gTestTime1);
// }

// void TestSetTime2AndShowTimeRange(double InitiTime, double CurTime,
//                                   double TimeRange_Int2Cur)
// {
//     if (CurTime - InitiTime >= TimeRange_Int2Cur)
//     {
//         clock_gettime(CLOCK_REALTIME, &gTestTime2);
//         cout << endl
//              << "Prepare Time: " << (gTestTime2.tv_sec - gTestTime1.tv_sec) *
//              1000 + (gTestTime2.tv_nsec - gTestTime1.tv_nsec) / 1000000 <<
//              "ms" << endl;
//     }
// }

// void TestTxt_CopyAndModifyWhenNeed() {
// //// 1. InitializingStructure of Optimizer.cpp Test
// {
//     for (int i = 0; i < mFrontFrameIdInWin; i++)
//     {
//         cout << endl
//              << i << "th:" << endl;

//         gtsam::Pose3 PoseXth_0th_Win = mdsFramesInWin[i + 1].Tnc.inverse() *
//         mdsFramesInWin[0].Tnc; gtsam::Pose3 PoseXth_0th_CV = arCVPose_x2w[i
//         + 1].inverse() * arCVPose_x2w[0];

//         // PoseXth_0th_Win.print("PoseXth_0th_Win:");
//         cout << endl
//              << "PoseXth_0th_Win Att: "
//              << Vector3d(PoseXth_0th_Win.rotation().roll(),
//                          PoseXth_0th_Win.rotation().pitch(),
//                          PoseXth_0th_Win.rotation().yaw())
//                         .transpose() *
//                     kRad2Deg
//              << endl;
//         // PoseXth_0th_CV.print("PoseXth_0th_CV:");
//         cout << endl
//              << "PoseXth_0th_CV Att: "
//              << Vector3d(PoseXth_0th_CV.rotation().roll(),
//                          PoseXth_0th_CV.rotation().pitch(),
//                          PoseXth_0th_CV.rotation().yaw())
//                         .transpose() *
//                     kRad2Deg
//              << endl;

//         cout << endl
//              << "Diff_Win_CV Att: "
//              << Vector3d((PoseXth_0th_Win.rotation().inverse() *
//              PoseXth_0th_CV.rotation()).roll(),
//                          (PoseXth_0th_Win.rotation().inverse() *
//                          PoseXth_0th_CV.rotation()).pitch(),
//                          (PoseXth_0th_Win.rotation().inverse() *
//                          PoseXth_0th_CV.rotation()).yaw())
//                         .transpose() *
//                     kRad2Deg
//              << endl;

//         Vector3d Scl = Vector3d(PoseXth_0th_Win.translation().matrix()(0) /
//         PoseXth_0th_CV.translation().matrix()(0),
//                                 PoseXth_0th_Win.translation().matrix()(1) /
//                                 PoseXth_0th_CV.translation().matrix()(1),
//                                 PoseXth_0th_Win.translation().matrix()(2) /
//                                 PoseXth_0th_CV.translation().matrix()(2));
//         // double
//         Scl=mdsFramesInWin_ithPoint3.norm()/arCVPose_x2w_ithPoint3.norm();
//         cout << endl
//              << "Scale:" << Scl << endl;
//     }

//     gtsam::Pose3 Tnew_l_Win = mdsFramesInWin.back().Tnc.inverse() *
//                                     mdsFramesInWin.front().Tnc;
//     cout << endl
//          << "Delt_Win_l2latest Att: "
//          << Vector3d(Tnew_l_Win.rotation().roll(),
//                      Tnew_l_Win.rotation().pitch(),
//                      Tnew_l_Win.rotation().yaw())
//                     .transpose() *
//                 kRad2Deg
//          << endl;
//     cout << endl
//          << "Delt_CV_ep_l2latest Att: "
//          << Vector3d(Tnew_l.rotation().roll(),
//                      Tnew_l.rotation().pitch(),
//                      Tnew_l.rotation().yaw())
//                     .transpose() *
//                 kRad2Deg
//          << endl;

//     Vector3d TotalScale =
//     Vector3d(Tnew_l_Win.translation().matrix()(0) /
//     Tnew_l.translation().matrix()(0),
//                                    Tnew_l_Win.translation().matrix()(1)
//                                    /
//                                    Tnew_l.translation().matrix()(1),
//                                    Tnew_l_Win.translation().matrix()(2)
//                                    /
//                                    Tnew_l.translation().matrix()(2));
//     cout << endl
//          << "Delt_Win_l2latest Translation: " <<
//          Tnew_l_Win.translation().matrix().transpose() << endl
//          << "Delt_CV_l2latest Translation: " <<
//          Tnew_l.translation().matrix().transpose() << endl
//          << "Total Scale: " << TotalScale.transpose()
//          << endl;

//     cout << endl
//          << "Test Finished!" << endl;
// }

// // // 2. Test: mpImu_preintegrated_ output of PreIntegrating of
// Optimizer.cpp
// {
//     cout << "J_theta_Bg" << endl
//      << mTic.rotation().matrix() *
//             mpImu_preintegrated_->preintegrated_H_biasOmega().block<3,
//             3>(0, 0)
//      << endl;
//      mpImu_preintegrated_->print(" ");
//      gtsam::Pose3 CurPredPose = GetCurPredPose();
//      CurPredPose.print();
// }

// // 3. pose inference comparison between PreIntegration/EpiGeo/PnP, of
// VIGSystem.cpp
// {
//     if (mpOptimizer->geVIGState == VIG_OK)
//     {
//         mtstCurTnc = mpOptimizer->GetCurPredPose(); //
//         body_P_sensor has been setted
//         // PreIntegration
//         // mtstCurTnc.print("mtstCurTnc ");

//         // gtsam::Pose3 DeltPose_prein = mtstCurTnc.inverse() *
//         mpOptimizer->mCurPre_State.OptNav.pose();
//         // DeltPose_prein.print("DeltPose_prein: ");
//         // Vector3d Att_prein = Vector3d(DeltPose_prein.rotation().roll(),
//         DeltPose_prein.rotation().pitch(),
//         DeltPose_prein.rotation().yaw());
//         // cout << "CurFrameId: " <<
//         mpTracker->msCurFeatures.first.FrameId << endl
//         //      << "Att_prein: " << Att_prein.transpose() * kRad2Deg <<
//         endl;
//         // cout << "Trs_prein: " <<
//         DeltPose_prein.translation().matrix().transpose() << endl;

//         // EpoGeo
//         gtsam::Pose3 DeltPose_epigeo;
//         if (mpTracker->msCurFeatures.first.FrameId ==
//         58)
//             cout << "Wait! EpiGeo Error!" << endl;
//         if
//         (mpMapper->CalRelativeTrl(mpOptimizer->mMaxFrameIdInWin
//         - 1, mpOptimizer->mMaxFrameIdInWin, DeltPose_epigeo))
//         {
//             gtsam::Pose3 CurTnc_epigeo =
//             mpOptimizer->mCurPre_State.OptNav.pose() *
//             DeltPose_epigeo.inverse();
//             // CurTnc_epigeo.print("CurTnc_epigeo ");
//             cout << "Att_epigeo: " <<
//             Rot2AttDeg(CurTnc_epigeo.rotation()).transpose() <<
//             endl;
//             // cout << "Trs_epigeo: " <<
//             CurTnc_epigeo.translation().matrix().transpose() <<
//             endl;

//             // DeltPose_epigeo.print("DeltPose_epigeo: ");
//             // Vector3d Att_epigeo =
//             Vector3d(DeltPose_epigeo.rotation().roll(),
//             DeltPose_epigeo.rotation().pitch(),
//             DeltPose_epigeo.rotation().yaw());
//             // cout << "Att_epigeo: " << Att_epigeo.transpose() * kRad2Deg
//             << endl;
//             // cout << "Trs_epigeo: " <<
//             DeltPose_epigeo.translation().matrix().transpose() << endl;
//         }
//     }

//     if (mpOptimizer->geVIGState == VIG_OK &&
//     mbIsVisionOK)
//     {
//         // PnP
//         gtsam::Pose3 CurTnc_PnP = mtstCurTnc, DeltPose_pnp;
//         bool bPnP = false;
//         if
//         (mpMapper->SolvePnP4CurFrameInKFQue(mpTracker->msCurFeatures,
//                                                        CurTnc_PnP))
//         {
//             bPnP = true;
//             // CurTnc_PnP.print("CurTnc_PnP ");

//             // DeltPose_pnp = CurTnc_PnP.inverse() *
//             mpOptimizer->mCurPre_State.OptNav.pose();
//             // DeltPose_pnp.print("DeltPose_pnp: ");
//             // Vector3d Att_pnp = Vector3d(DeltPose_pnp.rotation().roll(),
//             DeltPose_pnp.rotation().pitch(),
//             DeltPose_pnp.rotation().yaw());
//             // cout << "Att_pnp: " << Att_pnp.transpose() * kRad2Deg <<
//             endl;
//             // cout << "Trs_pnp: " <<
//             DeltPose_pnp.translation().matrix().transpose() << endl;
//         }
//         cout << "CurFrameId: " <<
//         mpTracker->msCurFeatures.first.FrameId << endl
//              << "Att_prein: " <<
//              Rot2AttDeg(mtstCurTnc.rotation()).transpose() << endl;
//         if (bPnP)
//             cout << "Att_pnp: " <<
//             Rot2AttDeg(CurTnc_PnP.rotation()).transpose() <<
//             endl;

//         cout << "Trs_prein: " <<
//         mtstCurTnc.translation().matrix().transpose() << endl; if
//         (bPnP)
//             cout << "Trs_pnp: " <<
//             CurTnc_PnP.translation().matrix().transpose() << endl;
//         cout << endl;
//     }
// }

// // 4. TriangulateRestFts_MultFrames test of fun
// TriangulateRestFts_MultFrames of Featuremanager.cpp
// // Test1
// int LastKFId = XthKFId;
// for (int itFtOnXthFrameInKFQue = StartKFId, itFtOnXthFrame = 0;
//      itFtOnXthFrameInKFQue <= LastKFId;
//      itFtOnXthFrameInKFQue++, itFtOnXthFrame++)
// {
//     gtsam::Point3 Point3_XthKFId =
//     dKeyFrames[itFtOnXthFrameInKFQue].Tnc.inverse() *
//                                              lit.Pos3w;
//     Point3_XthKFId /= Point3_XthKFId(2);
//     gtsam::Point3 Uuvrep_homo = mCamK * Point3_XthKFId;

//     gtsam::Point2 ErrorRepj = Uuvrep_homo.head(2) -
//     lit.vInfoOnAllFrames[itFtOnXthFrame].Uuv;
//     ErrorRepj.print("ErrorRepj ");
//     cout << endl;
// }
//
// // Test2
// gtsam::Pose3 Tcx_c0_prein = dKeyFrames[XthKFId].Tnc.inverse()
// *
//                                dKeyFrames[StartKFId].Tnc;
// gtsam::Pose3 Tcx_c0_epgeo_cal, Tcx_c0_epgeo;
// if (CalRelativeTrl(StartKFId, XthKFId,
// Tcx_c0_epgeo_cal))
//     Tcx_c0_epgeo = Tcx_c0_epgeo_cal;

// Vector3d Point3c0;
// TriangulateOneFt_DualView(gtsam::Pose3::identity(), Tcx_c0_epgeo,
//                            lit.vInfoOnAllFrames[StartKFId].Uxy1.head(2),
//                            lit.vInfoOnAllFrames[XthKFId].Uxy1.head(2),
//                            Point3c0);

// Vector3d Altc02cx_prein = Vector3d(Tcx_c0_prein.rotation().roll(),
// Tcx_c0_prein.rotation().pitch(), Tcx_c0_prein.rotation().yaw());
// Altc02cx_prein *= kRad2Deg;
// Vector3d Altc02cx_epgeo = Vector3d(Tcx_c0_epgeo.rotation().roll(),
// Tcx_c0_epgeo.rotation().pitch(), Tcx_c0_epgeo.rotation().yaw());
// Altc02cx_epgeo *= kRad2Deg;
// gtsam::Rot3 DeltRot = Tcx_c0_prein.rotation() *
//                       Tcx_c0_epgeo.rotation().inverse();
// Vector3d DeltAlt = Vector3d(DeltRot.roll(), DeltRot.pitch(),
// DeltRot.yaw()); DeltAlt *= kRad2Deg; Vector3d Scl; Vector3d Pos3w =
// dKeyFrames[XthKFId].Tnc * Point3c0; for (int i = 0; i < 3; i++)
//     Scl(i) = Point3_StartKFId(i) / Point3c0(i);

// double svd_method = svd_V[2] / svd_V[3];

// if (lit.EstimatedDepth_StartKFId < 0.1)
// {
//     lit.EstimatedDepth_StartKFId = gInitiDepth;
// }

// // 5. remove unused factors_
// if (gbUseCam)
// {
//     for (int i = 0; i < MAX_ITERATION_ISAM_UPD_CAL; i++)
//     {
//         gtsam::VectorValues CurDeltas = mFLSmootherISAM2.getDelta();
//         // Test
//         CurDeltas.print();
//         cout << endl;
//         GetFactorsWtLandMarkToRemove(CurDeltas);

//         if (mFactorsWtLandMarkToRemove.empty())
//             break;

//         gtsam::NonlinearFactorGraph nullFactorGraph;
//         gtsam::Values nullValues;
//         gtsam::FixedLagSmootherKeyTimestampMap nullKey_TimeStamps;
//         mFLSmootherISAM2.update(nullFactorGraph, nullValues,
//         nullKey_TimeStamps, mFactorsWtLandMarkToRemove); FLResultValues =
//         mFLSmootherISAM2.calculateEstimate();
//     }
// }
// void Optimizer::GetFactorsWtLandMarkToRemove(
//     const gtsam::VectorValues &CurDeltas) {
//   mFactorsWtLandMarkToRemove.clear();
//   gtsam::NonlinearFactorGraph CurISAMFactorGraph =
//       mFLSmootherISAM2.getFactors();

//   for (auto itDelta : CurDeltas) {
//     // gtsam::Key key = itDelta.first;
//     gtsam::Symbol itSymbol(itDelta.first);
//     if (itSymbol.chr() == 'l') {
//       gtsam::Vector3 CurLandMarkDelta = itDelta.second.cwiseAbs();

//       if (CurLandMarkDelta(0) >= 3.0 || CurLandMarkDelta(1) >= 3.0 ||
//           CurLandMarkDelta(2) >= 3.0) {
//         if (mNewFactorIndex >= (int)CurISAMFactorGraph.size()) {
//           cerr << "Wrong New Factor Index!";
//           exit(1);
//         }

//         // Mark it in mpMapper
//         int PointId = itSymbol.index();
//         auto lit = find_if(mpMapper->mNewMapPoints.begin(),
//                            mpMapper->mNewMapPoints.end(),
//                            [PointId](const Point3D &lit) {
//                              return lit.PointId == PointId;
//                            });
//         lit->SolvedFlag = FAILED;

//         // Gather the involved factors
//         for (size_t i = mNewFactorIndex; i < CurISAMFactorGraph.size(); i++) {
//           if (CurISAMFactorGraph.at(i)->find(itDelta.first) !=
//               CurISAMFactorGraph.at(i)->keys().end())
//             mFactorsWtLandMarkToRemove.push_back(i);
//         }
//       }
//     }
//   }

//   // Bug here
//   if (!mFactorsWtLandMarkToRemove.empty())
//     mNewFactorIndex = mFactorsWtLandMarkToRemove.back() + 1;
// }

/// remove unused keys
// for (auto itUnusedLandMarkKey :
// mFLSmootherISAM2.getISAM2Result().unusedKeys)
// {
//     if (mResultValues.exists(itUnusedLandMarkKey))
//         mResultValues.erase(itUnusedLandMarkKey);
// }

// // 2*. Sovle scale,gravity,Acc bias
// Eigen::Matrix<double, Dynamic, 4> A;
// Eigen::Matrix<double, Dynamic, 3> A3;
// Eigen::Matrix<double, Dynamic, 1> B;
// Eigen::Matrix<double, Dynamic, 1> B3;
// Matrix3d I3 = Matrix3d::Identity();
// A.setZero(3 * (NTrackedFrms - 2), 4);
// A3.setZero(3 * (NTrackedFrms - 2), 3);
// B.setZero(3 * (NTrackedFrms - 2), 1);
// B3.setZero(3 * (NTrackedFrms - 2), 1);
// // double s_star0 = 0.20803365278239949;
// // Vector3d g_w_star0 =
// //     Vector3d(-0.77205850928756448, 1.1962859828478352,
// -9.715290726491423); double s_star0 = 1.; Vector3d g_w_star0 = Vector3d(0,
// 0, -9.81);
// // 2.1 Approx Scale and Gravity vector in world frame (nav frame)
// for (FId4IMUPre = 1; FId4IMUPre < NTrackedFrms - 1; FId4IMUPre++) {
//   pair<Frame, map<int, Point2D>> TrackedFrm1 =
//       mdprFrames[FId4IMUPre - 1];
//   pair<Frame, map<int, Point2D>> TrackedFrm2 =
//   mdprFrames[FId4IMUPre]; pair<Frame, map<int, Point2D>>
//   TrackedFrm3 =
//       mdprFrames[FId4IMUPre + 1];
//   // Delta time between frames
//   double dt12 = TrackedFrm2.first.Epoch -
//   TrackedFrm1.first.Epoch; double dt23 = TrackedFrm3.first.Epoch
//   - TrackedFrm2.first.Epoch;
//   // Pre-integrated measurements
//   Vector3d dp12 = mIMUPreIntBetwDualFrms[FId4IMUPre].deltaPij();
//   Vector3d dv12 = mIMUPreIntBetwDualFrms[FId4IMUPre].deltaVij();
//   Vector3d dp23 = mIMUPreIntBetwDualFrms[FId4IMUPre + 1].deltaPij();
//   Matrix3d Jpba12 = mIMUPreIntBetwDualFrms[FId4IMUPre].delPdelBiasAcc();
//   Matrix3d Jvba12 = mIMUPreIntBetwDualFrms[FId4IMUPre].delVdelBiasAcc();
//   Matrix3d Jpba23 = mIMUPreIntBetwDualFrms[FId4IMUPre +
//   1].delPdelBiasAcc();
//   // Pose of camera in world frame
//   Vector3d pc1 = TrackedFrm1.first.Tnc.translation().matrix();
//   Vector3d pc2 = TrackedFrm2.first.Tnc.translation().matrix();
//   Vector3d pc3 = TrackedFrm3.first.Tnc.translation().matrix();
//   Matrix3d Rc1 = TrackedFrm1.first.Tnc.rotation().matrix();
//   Matrix3d Rc2 = TrackedFrm2.first.Tnc.rotation().matrix();
//   Matrix3d Rc3 = TrackedFrm3.first.Tnc.rotation().matrix();

//   Vector3d vb1 = TrackedFrm1.first.Vnb;
//   Vector3d vb2 = TrackedFrm2.first.Vnb;
//   Vector3d vb2_pre_gG = vb1 + gG * dt12 + Rc1 * Rcb * dv12;
//   // cout << "vb2" << vb2.transpose() << endl;
//   // cout << "vb2_pre_gstar" << vb2_pre_gstar.transpose() << endl;
//   // cout << "vb2_pre_gG" << vb2_pre_gG.transpose() << endl;

//   // lambda*s + beta*g = gamma
//   A.block<3, 1>(3 * (FId4IMUPre - 1), 0) =
//       (pc2 - pc1) * dt23 + (pc2 - pc3) * dt12;
//   A.block<3, 3>(3 * (FId4IMUPre - 1), 1) =
//       // Rc1 * Jpba12 * dt23 - Rc1 * Jvba12 * dt12 * dt23 - Rc2 * Jpba23 *
//       // dt12;
//       0.5 * I3 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23);
//   B.middleRows<3>(3 * (FId4IMUPre - 1)) =
//       (Rc3 - Rc2) * pcb * dt12 + (Rc1 - Rc2) * pcb * dt23 +
//       Rc1 * Rcb * dp12 * dt23 - Rc2 * Rcb * dp23 * dt12 -
//       Rc1 * Rcb * dv12 * dt12 * dt23;
//   // - 0.5 * gG * dt12 * dt23 * (dt12 + dt23);

//   Vector3d L1_0 = (pc2 - pc1) * s_star0 - 0.5 * g_w_star0 * dt12 * dt12;
//   // Vector3d R1_0 = vb1 * dt12 + Rc1 * Rcb * dp12 + (Rc1 - Rc2) * pcb;
//   Vector3d R1_0 = Rc1 * Rcb * dp12 + (Rc1 - Rc2) * pcb;
//   Vector3d v1cal_1 = (L1_0 - R1_0) / dt12;
//   Vector3d L2_0 = (pc3 - pc2) * s_star0 - 0.5 * g_w_star0 * dt23 * dt23;
//   // Vector3d R2_0 = (vb1 + g_w_star0 * dt12 + Rc1 * Rcb * dv12) * dt23 +
//   //                 Rc2 * Rcb * dp23 + (Rc2 - Rc3) * pcb;
//   Vector3d R2_0 = (g_w_star0 * dt12 + Rc1 * Rcb * dv12) * dt23 +
//                   Rc2 * Rcb * dp23 + (Rc2 - Rc3) * pcb;
//   Vector3d v1cal_2 = (L2_0 - R2_0) / dt23;
//   // cout << "L1_0 " << L1_0.transpose() << endl;
//   // cout << "R1_0 " << R1_0.transpose() << endl;
//   // cout << "L2_0 " << L2_0.transpose() << endl;
//   // cout << "R2_0 " << R2_0.transpose() << endl;
//   cout << "v1cal_1 " << v1cal_1.transpose() << endl;
//   cout << "v1cal_2 " << v1cal_2.transpose() << endl;
//   cout << endl;

//   // Vector3d L1_1 =
//   //     ((pc2 - pc1) * s_star0 - 0.5 * g_w_star0 * dt12 * dt12) * dt23;
//   // Vector3d R1_1 = (vb1 * dt12 + Rc1 * Rcb * dp12 + (Rc1 - Rc2) * pcb) *
//   // dt23; Vector3d L2_1 = ((pc3 - pc2) * s_star0 - 0.5 * g_w_star0 * dt23
//   *
//   // dt23 -
//   //                  g_w_star0 * dt12 * dt23) *
//   //                 dt12;
//   // Vector3d R2_1 = ((vb1 + Rc1 * Rcb * dv12) * dt23 + Rc2 * Rcb * dp23 +
//   //                  (Rc2 - Rc3) * pcb) *
//   //                 dt12;
//   // cout << "L1_1 " << L1_0.transpose() << endl;
//   // cout << "R1_1 " << R1_0.transpose() << endl;
//   // cout << "L2_1 " << L2_0.transpose() << endl;
//   // cout << "R2_1 " << R2_0.transpose() << endl;
//   // cout << endl;

//   // A3.block<3, 3>(3 * (FId4IMUPre - 1), 0) =
//   //     Rc1 * Jpba12 * dt23 - Rc1 * Jvba12 * dt12 * dt23 - Rc2 * Jpba23 *
//   //     dt12;
//   // B3.middleRows<3>(3 * (FId4IMUPre - 1)) =
//   //     (Rc3 - Rc2) * pcb * dt12 + (Rc1 - Rc2) * pcb * dt23 +
//   //     Rc1 * Rcb * dp12 * dt23 - Rc2 * Rcb * dp23 * dt12 -
//   //     Rc1 * Rcb * dv12 * dt12 * dt23 -
//   //     0.5 * gG * dt12 * dt23 * (dt12 + dt23) - (pc2 - pc1) * dt23 -
//   //     (pc2 - pc3) * dt12;

//   // Matrix3d A3_i = A3.block<3, 3>(3 * (FId4IMUPre - 1), 0);
//   // Vector3d B3_i = B3.middleRows<3>(3 * (FId4IMUPre - 1));
//   // Vector3d ba_i = A3_i.ldlt().solve(B3_i);
//   // cout << "ba_i" << ba_i.transpose() << endl;
//   // cout << endl;
// }

// // Solve Approx scale and gravity
// cout << A << endl << endl;
// cout << B << endl << endl;
// Matrix4d ATA = A.transpose() * A;
// Vector4d ATB = A.transpose() * B;
// Vector4d s_g_w_star = ATA.ldlt().solve(ATB);
// double s_star = s_g_w_star(0);
// Vector3d mG_w_star = s_g_w_star.tail(3);
// if (s_star < 0) {
//   cerr << "Wrong Scale!" << endl;
//   exit(1);
// }
// Matrix3d ATA3 = A3.transpose() * A3;
// Vector3d ATB3 = A3.transpose() * B3;
// Vector3d b3 = ATA3.ldlt().solve(ATB3);

// // Test	2.1, rough scale and gravity star
//   Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
//   Eigen::Matrix<double, Dynamic, 4> As_g;
//   Eigen::Matrix<double, Dynamic, 1> Bs_g;
//   As_g.setZero(3 * (NTrackedFrms - 1), 4);
//   Bs_g.setZero(3 * (NTrackedFrms - 1), 1);
//   for (FId4IMUPre = 1; FId4IMUPre < NTrackedFrms - 1; FId4IMUPre++) {
//     pair<Frame, map<int, Point2D>> TrackedFrm1 =
//         dFrames[FId4IMUPre - 1];
//     pair<Frame, map<int, Point2D>> TrackedFrm2 = dFrames[FId4IMUPre];
//     pair<Frame, map<int, Point2D>> TrackedFrm3 =
//         dFrames[FId4IMUPre + 1];
//     // Delta time between frames
//     double dt12 = TrackedFrm2.first.Epoch - TrackedFrm1.first.Epoch;
//     double dt23 = TrackedFrm3.first.Epoch - TrackedFrm2.first.Epoch;
//     // Pre-integrated measurements
//     Vector3d dp12 = mIMUPreIntBetwDualFrms[FId4IMUPre].deltaPij();
//     Vector3d dv12 = mIMUPreIntBetwDualFrms[FId4IMUPre].deltaVij();
//     Vector3d dp23 = mIMUPreIntBetwDualFrms[FId4IMUPre + 1].deltaPij();
//     // Pose of camera in world frame
//     Vector3d pc1 = TrackedFrm1.first.Tnc.translation().matrix();
//     Vector3d pc2 = TrackedFrm2.first.Tnc.translation().matrix();
//     Vector3d pc3 = TrackedFrm3.first.Tnc.translation().matrix();
//     Matrix3d Rc1 = TrackedFrm1.first.Tnc.rotation().matrix();
//     Matrix3d Rc2 = TrackedFrm2.first.Tnc.rotation().matrix();
//     Matrix3d Rc3 = TrackedFrm3.first.Tnc.rotation().matrix();

//     // Vector3d vb1 = TrackedFrm1.first.Vnb;
//     // Vector3d vb2 = TrackedFrm2.first.Vnb;
//     // Vector3d vb2_pre_gstar = vb1 + mG_w_star * dt12 + Rc1 * Rcb * dv12;
//     // Vector3d vb2_pre_gG = vb1 + gG * dt12 + Rc1 * Rcb * dv12;
//     // cout << "vb2" << vb2.transpose() << endl;
//     // cout << "vb2_pre_gstar" << vb2_pre_gstar.transpose() << endl;
//     // cout << "vb2_pre_gG" << vb2_pre_gG.transpose() << endl;
//     // // Vector3d deltv = vb2_pre - vb2;
//     // // cout << "deltv" << deltv.transpose() << endl;

//     // // lambda*s + beta*g = gamma
//     // gtsam::Pose3 Pose_b1 = mdInitKeyFrames[FId4IMUPre - 1].Tnc * Tci;
//     // gtsam::Pose3 Pose_b2 = mdInitKeyFrames[FId4IMUPre].Tnc * Tci;
//     // Vector3d pb1 = Pose_b1.translation().matrix();
//     // Vector3d L_star = s_star * (pc2 - pc1) - 0.5 * mG_w_star * dt12 * dt12;
//     // Vector3d L_s1_g_star = 1 * (pc2 - pc1) - 0.5 * mG_w_star * dt12 * dt12;
//     // Vector3d L_true = 1 * (pc2 - pc1) - 0.5 * gG * dt12 * dt12;
//     // Vector3d R = (Rc1 - Rc2) * pcb + vb1 * dt12 + Rc1 * Rcb * dp12;
//     // cout << "L_star" << L_star.transpose() << "      " << dt12 << endl;
//     // cout << "L_s1_g_star" << L_s1_g_star.transpose() << endl;
//     // cout << "L_true" << L_true.transpose() << endl;
//     // cout << "R" << R.transpose() << endl << endl;

//     As_g.block<3, 1>(3 * (FId4IMUPre - 1), 0) =
//         (pc2 - pc1) * dt23 - (pc3 - pc2) * dt12;
//     As_g.block<3, 3>(3 * (FId4IMUPre - 1), 1) =
//         0.5 * I3 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23);
//     Bs_g.middleRows<3>(3 * (FId4IMUPre - 1)) =
//         Rc1 * Rcb * dp12 * dt23 + (Rc1 - Rc2) * pcb * dt23 -
//         Rc1 * Rcb * dv12 * dt12 * dt23 - Rc2 * Rcb * dp23 * dt12 -
//         (Rc2 - Rc3) * pcb * dt12 + (pc1 - pc2) * dt23 - (pc2 - pc3) * dt12;

//     int a = 1;
//   }
//   cout << As_g << endl << endl;
//   cout << Bs_g << endl << endl;
//   // Solve Approx scale and gravity
//   Matrix4d ATA_s_g = As_g.transpose() * As_g;
//   Vector4d ATB_s_g = As_g.transpose() * Bs_g;
//   Vector4d s_g_w_star_ = ATA_s_g.ldlt().solve(ATB_s_g);
//   Vector3d mG_w_star = s_g_w_star_.tail(3);

//   // Test 2.2, refine scale and gravity, estimate acc bias
//   Vector3d g_I = Vector3d(0., 0., -1.), g_wn = mG_w_star / mG_w_star.norm();
//   Vector3d g_Ixg_wn = g_I.cross(g_wn);
//   double norm_g_Ixg_wn = g_Ixg_wn.norm();
//   Vector3d v_hat = g_Ixg_wn / norm_g_Ixg_wn;
//   double theta = atan2(norm_g_Ixg_wn, g_I.dot(g_wn));
//   Matrix3d Rwi = gtsam::Rot3::Expmap(gtsam::Vector3(v_hat * theta)).matrix();
  
//   Vector3d gw = Rwi * gG;

//   Eigen::Matrix<double, Dynamic, 6> C;
//   Eigen::Matrix<double, Dynamic, 1> D;
//   C.setZero(3 * (NTrackedFrms - 2), 6);
//   D.setZero(3 * (NTrackedFrms - 2), 1);
//   for (FId4IMUPre = 1; FId4IMUPre < NTrackedFrms - 1; FId4IMUPre++) {
//     pair<Frame, map<int, Point2D>> TrackedFrm1 =
//         dFrames[FId4IMUPre - 1];
//     pair<Frame, map<int, Point2D>> TrackedFrm2 = dFrames[FId4IMUPre];
//     pair<Frame, map<int, Point2D>> TrackedFrm3 =
//         dFrames[FId4IMUPre + 1];
//     // Delta time between frames
//     double dt12 = TrackedFrm2.first.Epoch - TrackedFrm1.first.Epoch;
//     double dt23 = TrackedFrm3.first.Epoch - TrackedFrm2.first.Epoch;
//     // Pre-integrated measurements
//     Vector3d dp12 = mIMUPreIntBetwDualFrms[FId4IMUPre].deltaPij();
//     Vector3d dv12 = mIMUPreIntBetwDualFrms[FId4IMUPre].deltaVij();
//     Vector3d dp23 = mIMUPreIntBetwDualFrms[FId4IMUPre + 1].deltaPij();
//     Matrix3d Jpba12 = mIMUPreIntBetwDualFrms[FId4IMUPre].delPdelBiasAcc();
//     Matrix3d Jvba12 = mIMUPreIntBetwDualFrms[FId4IMUPre].delVdelBiasAcc();
//     Matrix3d Jpba23 = mIMUPreIntBetwDualFrms[FId4IMUPre + 1].delPdelBiasAcc();
//     // Pose of camera in world frame
//     Vector3d pc1 = TrackedFrm1.first.Tnc.translation().matrix();
//     Vector3d pc2 = TrackedFrm2.first.Tnc.translation().matrix();
//     Vector3d pc3 = TrackedFrm3.first.Tnc.translation().matrix();
//     Matrix3d Rc1 = TrackedFrm1.first.Tnc.rotation().matrix();
//     Matrix3d Rc2 = TrackedFrm2.first.Tnc.rotation().matrix();
//     Matrix3d Rc3 = TrackedFrm3.first.Tnc.rotation().matrix();

//     C.block<3, 1>(3 * (FId4IMUPre - 1), 0) =
//         (pc2 - pc1) * dt23 - (pc3 - pc2) * dt12;
//     C.block<3, 3>(3 * (FId4IMUPre - 1), 1) =
//         -0.5 * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23) * Rwi *
//         Skew(gG);  // note: this has a '-', different to paper
//     C.block<3, 3>(3 * (FId4IMUPre - 1), 3) = Rc2 * Rcb * Jpba23 * dt12 +
//                                              Rc1 * Rcb * Jvba12 * dt12 * dt23 -
//                                              Rc1 * Rcb * Jpba12 * dt23;
//     D.middleRows<3>(3 * (FId4IMUPre - 1)) =
//         (Rc1 - Rc2) * pcb * dt23 + Rc1 * Rcb * dp12 * dt23 -
//         (Rc2 - Rc3) * pcb * dt12 - Rc2 * Rcb * dp23 * dt12 -
//         Rc1 * Rcb * dv12 * dt23 * dt12 -
//         0.5 * Rwi * gG *
//             (dt12 * dt12 * dt23 + dt12 * dt23 * dt23);  // note:  - paper
//   }
//   cout << C << endl << endl;
//   cout << D << endl << endl;
//   Eigen::Matrix<double, 6, 6> CTC = C.transpose() * C;
//   Eigen::Matrix<double, 6, 1> CTD = C.transpose() * D;
//   Eigen::Matrix<double, 6, 1> y = CTC.ldlt().solve(CTD);

//   double s = y(0);
//   Vector3d delta_theta = Vector3d(y(1), y(2), 0);
//   Vector3d ba = y.tail(3);

//   Matrix3d Rwi_ = Rwi * gtsam::Rot3::Expmap(-delta_theta).matrix();
//   Vector3d gw_1 = Rwi_ * gG, gw_2 = Rwi * gG - Rwi * Skew(gG) * delta_theta;

// for (FId4IMUPre = 1; FId4IMUPre < NTrackedFrms - 1; FId4IMUPre++) {
//   pair<Frame, map<int, Point2D>> TrackedFrm1 =
//       mdprFrames[FId4IMUPre - 1];
//   pair<Frame, map<int, Point2D>> TrackedFrm2 =
//   mdprFrames[FId4IMUPre];
//   // Delta time between frames
//   double dt12 = TrackedFrm2.first.Epoch -
//   TrackedFrm1.first.Epoch;
//   // Pre-integrated measurements
//   Vector3d dp12 = mIMUPreIntBetwDualFrms[FId4IMUPre].deltaPij();
//   Matrix3d Jpba12 = mIMUPreIntBetwDualFrms[FId4IMUPre].delPdelBiasAcc();

//   // Pose of camera in world frame
//   Vector3d pc1 = TrackedFrm1.first.Tnc.translation().matrix();
//   Vector3d pc2 = TrackedFrm2.first.Tnc.translation().matrix();
//   Vector3d v1 = TrackedFrm1.first.Vnb;
//   Matrix3d Rc1 = TrackedFrm1.first.Tnc.rotation().matrix();
//   Matrix3d Rc2 = TrackedFrm2.first.Tnc.rotation().matrix();

//   Vector3d L = s * pc2;
//   Vector3d R = s * pc1 + v1 * dt12 -
//                0.5 * Rwi * Skew(gG) * dt12 * dt12 * delta_theta +
//                Rc1 * Rcb * (dp12 + Jpba12 * ba) + (Rc1 - Rc2) * pcb +
//                0.5 * Rwi * gG * dt12 * dt12;
//   cout << "L" << L.transpose() << endl;
//   cout << "R" << R.transpose() << endl << endl;
//   int a = 1;
// }
// }

// void Optimizer::UpdateGraph_Win() {
//   assert(meCurObTp != NONE);
//   mmpFLNavTimeLine.insert(
//       make_pair(mCurNavEpoch, NavTimeLine(mNavIndex, meCurObTp)));
//   mStartEpoch = mCurNavEpoch - mSmootherLag;

//   // 1. Margin out 
//   for (auto mpitNavTL = mmpFLNavTimeLine.begin();
//        mpitNavTL != mmpFLNavTimeLine.end(); mpitNavTL++) {
//     if (geVIGState == VIG_OK && (mStartEpoch - mpitNavTL->first) > 1e-7) {
//       if (mpitNavTL->second.ObsType == GNSS) {
//         mGNSSs.erase(mGNSSs.find(mpitNavTL->first));
//       } else if (mpitNavTL->second.ObsType == VISION) {
//         mMargFrames[mpitNavTL->first] = mFrames[mpitNavTL->first];
//         mFrames.erase(mFrames.find(mpitNavTL->first));
//       }
//       mmpMargNavTimeLine.insert(make_pair(mpitNavTL->first, mpitNavTL->second));
//       mmpFLNavTimeLine.erase(mpitNavTL);

//     } else
//       break;
//   }

//   // 2. Construct Factor Graph
//   // 2.1 Nav State Time Line
//   for (auto mpitNavTL = mmpFLNavTimeLine.begin();
//        mpitNavTL != mmpFLNavTimeLine.end(); mpitNavTL++) {

//     if (mpitNavTL->second.ObsType == PRIOR) {
//       // Nav Index
//       int NavInd = mpitNavTL->second.NavInd;

//       // Factor Graph
//       mGraph->add(gtsam::PriorFactor<gtsam::Pose3>(X(NavInd), gInitTni,
//                                                     mInitPoseSigma));
//       mGraph->add(gtsam::PriorFactor<gtsam::Vector3>(V(NavInd), gInitVel_i2n,
//                                                       mInitVelSigma));
//       mGraph->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
//           B(NavInd), mCurPre_State.OptBias, mInitBiasSigma));

//       // Initial Values
//       mInitValues.insert(X(NavInd), gInitTni);
//       mInitValues.insert(V(NavInd), gInitVel_i2n);
//       mInitValues.insert(B(NavInd), mCurPre_State.OptBias);

//     } else if (mpitNavTL == mmpFLNavTimeLine.begin()) {
//       // Nav Index
//       int NavInd = mpitNavTL->second.NavInd;
//       assert(mResultValues.exists(X(NavInd)) &&
//              mResultValues.exists(V(NavInd)) &&
//              mResultValues.exists(B(NavInd)));

//       // Factor Graph
//       mGraph->add(gtsam::PriorFactor<gtsam::Pose3>(
//           X(NavInd), mResultValues.at<gtsam::Pose3>(X(NavInd)),
//           mInitPoseSigma));
//       mGraph->add(gtsam::PriorFactor<gtsam::Vector3>(
//           V(NavInd), mResultValues.at<gtsam::Vector3>(V(NavInd)),
//           mInitVelSigma));
//       mGraph->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
//           B(NavInd), mResultValues.at<gtsam::imuBias::ConstantBias>(B(NavInd)),
//           mInitBiasSigma));

//       // Initial Values
//       mInitValues.insert(X(NavInd), mResultValues.at<gtsam::Pose3>(X(NavInd)));
//       mInitValues.insert(V(NavInd),
//                          mResultValues.at<gtsam::Vector3>(V(NavInd)));
//       mInitValues.insert(
//           B(NavInd), mResultValues.at<gtsam::imuBias::ConstantBias>(B(NavInd)));

//       // the first GNSS should be added
//       if (mpitNavTL->second.ObsType == GNSS) {
//         AddGNSSFactor(mGNSSs[mpitNavTL->first]);
//       }

//     }
//     else if (mpitNavTL->second.ObsType == GNSS ||
//              mpitNavTL->second.ObsType == VG) {
//       // Nav Index
//       int NavInd = mpitNavTL->second.NavInd;
//       assert(mGNSSs[mpitNavTL->first].NavIndex == NavInd);
//       if (mpitNavTL->second.ObsType == VG)
//         assert(mFrames[mpitNavTL->first].first.NavIndex == NavInd);

//       // Factor Graph
//       if (mGNSSs[mpitNavTL->first].pGNSSPIIMU->deltaTij() != 0)
//         AddIMUFactor(NavInd, mGNSSs[mpitNavTL->first].pGNSSPIIMU);
//       AddGNSSFactor(mGNSSs[mpitNavTL->first]);

//       // Initial Values
//       mInitValues.insert(X(NavInd), mGNSSs[mpitNavTL->first].Tnb);
//       mInitValues.insert(V(NavInd), mGNSSs[mpitNavTL->first].Vnb);
//       mInitValues.insert(B(NavInd), mGNSSs[mpitNavTL->first].IMUBias);

//     } else if (mpitNavTL->second.ObsType == VISION && geVIGState == VIG_OK) {
//       // Nav Index
//       int NavInd = mpitNavTL->second.NavInd;
//       assert(mFrames[mpitNavTL->first].first.NavIndex == NavInd);

//       // Factor Graph
//       if (mFrames[mpitNavTL->first].first.pFramePIIMU->deltaTij() > 0)
//         AddIMUFactor(NavInd, mFrames[mpitNavTL->first].first.pFramePIIMU);

//       // Initial Values
//       mInitValues.insert(X(NavInd),
//                          mFrames[mpitNavTL->first].first.Tnc * gTic.inverse());
//       mInitValues.insert(V(NavInd), mFrames[mpitNavTL->first].first.Vnb);
//       mInitValues.insert(B(NavInd), mFrames[mpitNavTL->first].first.IMUBias);
//     }
//   }

//   // 2.2. Features
//   if (gbUseCam && geVIGState == VIG_OK) {
//     // Features
//     mpMapper->MoveWin(mStartEpoch);
//     mpMapper->TriangulateOptMP_MultView(mFrames);

//     // Factor
//     AddCurKFFactor();
//   }

//   // reset obs type
//   meCurObTp = NONE;
// }

}  // namespace VIG
