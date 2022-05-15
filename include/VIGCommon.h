#pragma once

#include <assert.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>

// #include <fstream>
#include <iosfwd>
// #include <istream>
#include <list>
#include <map>
#include <numeric>
#include <queue>
#include <string>
#include <vector>

#include "jPreintegratedIMUs.h"

#include <math.h>
#include <time.h>

#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

// using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
// using gtsam::symbol_shorthand::L;  // LandMark (X,Y,Z)
// using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
// using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
// x is integer
#define B(x) (2000000 + x)   // B:  2(A,B,C,...,X,Y,Z), Bias (ax,ay,az,gx,gy,gz)
#define L(x) (12000000 + x)  // L: 12(A,B,C,...,X,Y,Z), LandMark (X,Y,Z)
#define V(x) (22000000 + x)  // V: 22(A,B,C,...,X,Y,Z), Vel (xdot,ydot,zdot)
#define X(x) (24000000 + x)  // X: 24(A,B,C,...,X,Y,Z), Pose3 (x,y,z,r,p,y)

const double PI = 3.1415926535897932384626433832795;
const double TWO_PI = 2.0 * PI;      // 2PI
const double kDeg2Rad = PI / 180.0;  // Radians per degree
const double kRad2Deg = 180.0 / PI;  // Degrees per radian
const double kA_84 = 6378137.0;      // WGS84 long radius（m）
const double kB_84 = 6356752.3142;   // WGS84 short radius（m）
const double kFlattening = (kA_84 - kB_84) / kA_84;  // WGS84 falatenning
const double kRavg = 6317000.0;                      // Earth Average Radius
const double kOmega_WGS =
    7.292115e-5;  // [rad/s], earth autogiration angular rate
const double k_e1_2 =
    (kA_84 * kA_84 - kB_84 * kB_84) /
    (kA_84 * kA_84);  // First eccentricity's square of ellipsoid WGS84
const double k_e2_2 =
    (kA_84 * kA_84 - kB_84 * kB_84) /
    (kB_84 * kB_84);  // Second eccentricity's square of ellipsoid WGS84
const double kGM_Earth =
    3.986004418e+14;               // [m^3/s]; WGS84 Gravitational constant
const double kU0 = 62636860.8497;  // WGS84 Normal gravity potential
const double kGe = 9.7803267714;   // WGS84 nominal g on earth surface
const double kJ = -484.16685e-6;   // Second order harmonic coefficient of
                                   // gravitational potential
const double kTa = 9.7803267715;   // Gravity of the equator（m/s^2）
const double kTb = 9.8321863685;   // Gravity of the pole point（m/s^2）

const Vector3d kWie_e = Vector3d(0.0, 0.0, kOmega_WGS);
const Vector3d GNSSPosStdQ1 = Vector3d(0.05, 0.05, 0.15);
const Vector3d GNSSPosStdQ2 = Vector3d(1.0, 1.0, 2.0);
const Vector3d GNSSPosStdQ36 = Vector3d(8, 8, 15);

namespace VIG {

extern double gStartTime, gEndTime;
extern Vector3d gG;
extern bool gbUseGNSS;
extern bool gbUseCam;

extern double gFocalLength, gMinKeyFrameParallax;
extern Matrix3d gCamK;
extern Eigen::Matrix<double, 5, 1> gDistCoef;
extern double gInitDepth, gInvDepthThreshold;
extern int gFps, gMinCoFrames, gMinCoFeatures;

// Test params
extern timespec gTestTime1, gTestTime2;
// static int gTestFramesThreshold;

typedef gtsam::PreintegratedCombinedMeasurements::Params PIIMUParams;
typedef jPreintegratedIMUs PIIMU;

enum eUnDis_Dis_Flag { UNDIS, DIS };

enum eMapPointSolveFlag {
  RAW,
  FAILED,
  OVERSIZE,
  SOLVING,
  SOLVED
};

// enum eMarginFlag { MARGIN_OLD, MARGIN_SECOND_NEW };
enum eObsType { NONE, PRIOR, GNSS, VISION, VG };
struct NavTimeLine {
  eObsType ObsType;
  int NavInd;

  NavTimeLine(eObsType InObsType, int InNavInd)
      : ObsType(InObsType), NavInd(InNavInd) {}
};

enum eVIGState { UNINITIALIZED, VIG_OK };
extern eVIGState geVIGState;

struct IMUModel {
  gtsam::Matrix33 accelerometerCovariance;
  gtsam::Matrix33 gyroscopeCovariance;
  gtsam::Matrix33 integrationCovariance;  // error committed in integrating
                                          // position from velocities
  gtsam::Matrix33 biasAccCovariance;
  gtsam::Matrix33 biasOmegaCovariance;
  gtsam::Matrix66 biasAccOmegaInt;  // error in the bias used for preintegration
};

struct IMU {
  double T;
  Vector3d Acc;
  Vector3d Gyro;
};

// 绑定这几个参数
struct SingleEpochState {
  double OptEpoch, PreEpoch;
  PIIMU *pPIIMU4Prd;
  gtsam::imuBias::ConstantBias OptBias;
  gtsam::NavState PredNav, OptNav;

  // void Predict(double CurEpoch) {
  //   PreIntegrate(OptEpoch, CurEpoch, pPIIMU4Prd);
  //   PredNav = pPIIMU4Prd->predict(OptNav, OptBias);
  // }

  void Update(const double OptCurEpoch, const gtsam::NavState &OptNavState,
              const gtsam::imuBias::ConstantBias &OptCurBias) {
    OptEpoch = OptCurEpoch;
    PreEpoch = OptEpoch;

    OptNav = OptNavState;
    PredNav = OptNav;

    OptBias = OptCurBias;
    pPIIMU4Prd->resetIntegrationAndSetBias(OptBias);
  }
};

struct GNSSSol {
  double Epoch;
  Vector3d t_ng;
  int Qual;
  int ns;
  Eigen::Vector3d Std;
  PIIMU *pGNSSPIIMU;
  int NavIndex = -1;
  gtsam::Pose3 Tnb;
  Eigen::Vector3d Vnb;
  gtsam::imuBias::ConstantBias IMUBias;

  void SetState(const SingleEpochState &CurState) {
    Tnb = CurState.PredNav.pose();
    Vnb = CurState.PredNav.v();
    IMUBias = CurState.OptBias;
  }
};

struct Frame {
  int FrameId;
  double Epoch;
  int NavIndex = -1;
  PIIMU *pFramePIIMU;
  gtsam::Pose3 Tnc;
  Eigen::Vector3d Vnb;
  gtsam::imuBias::ConstantBias IMUBias;
  bool bOnGNSS = false, bKeyFrame = false;

  void SetState(const SingleEpochState &CurState, const gtsam::Pose3 &Tic) {
    Tnc = CurState.PredNav.pose() * Tic;
    Vnb = CurState.PredNav.v();
    IMUBias = CurState.OptBias;
  }
};

struct Point2D {
  Vector2d Duv;
  Vector2d Uuv;
  Vector3d Uxy1;
  int TrackNum;
  bool bOptimized = false;
};

struct Pt2DInfo {
  int FrameId;
  Vector2d Uuv;
  Vector3d Uxy1;
  // bool IsUsed;
};

struct Point3D {
  int PointId;
  map<double, Pt2DInfo> mpPt2DKFs;

  gtsam::Point3 Uxy1_ref;
  gtsam::Point3 Pos3w;
  double InvDepth;
  eMapPointSolveFlag SolvedFlag;
  // Vector3d gt_p;
  Point3D(int InPointId, Vector3d InUxy1_ref)
      : PointId(InPointId),
        Uxy1_ref(InUxy1_ref),
        InvDepth(-1000.0),
        SolvedFlag(RAW) {}
};

template <typename T>
int SGN(T x) {
  return fabs(x) < 1e-10 ? 0 : (x < .0 ? -1 : 1);
};

template <typename T>
T SQR(T x) {
  return x * x;
};
template <typename T>
T CUB(T x) {
  return x * x * x;
};

Matrix3d Skew(const Vector3d &vec3d);
Matrix4d Q1(const Quaterniond Q);
Matrix4d Q2(const Quaterniond Q);
bool VectorLessThan(const VectorXd &Vx, const VectorXd &VThres);
double GetVariance(const vector<double> &vector_);
double Get1dVectorStd(const vector<double> &V1d);
Vector3d Get3dVectorStd(const vector<Eigen::Vector3d > &V3d);
IMUModel GetIMUModel(const int IMUType);
Matrix3d R1(double XrotRad);
Matrix3d R2(double YrotRad);
Matrix3d R3(double ZrotRad);
void RemoveAttAnglesRedundancy(Vector3d &AttRad);
Vector3d CalgravityN(const Vector3d &BLH);

// strNavFrame is "NED" or "ENU"
Matrix3d BL2Rne(const Vector2d &BL, const string &strNavFrame);
// strNavFrame is "NED" or "ENU"
Matrix3d BL2Ren(const Vector2d &BL, const string &strNavFrame);

Vector3d ECEF2BLH(const Vector3d &XYZ);
Vector3d BLH2ECEF(const Vector3d &BLH);

Vector3d PointInLocalFrame(const Vector3d &CurBLH, const Vector3d &BaseXYZ,
                           const string &strNavFrame);

Matrix3d RTKlibStd2CovMatrix(Eigen::Matrix<double, 6, 1> &RTKliblStd);
Eigen::Matrix<double, 6, 1> CovMatrix2RTKlibStd(const Matrix3d &CovMatrix);
int CountMatch(const pair<Frame, map<int, Point2D>> &LFrame,
               const pair<Frame, map<int, Point2D>> &RFrame);
bool EnoughParallax(const pair<Frame, map<int, Point2D>> &LFrame,
                    const pair<Frame, map<int, Point2D>> &RFrame);
bool EnoughParallax(const pair<Frame, map<int, Point2D>> &LFrame,
                    const pair<Frame, map<int, Point2D>> &RFrame, int &NumCorr);
bool EnoughParallax(const pair<Frame, map<int, Point2D>> &LFrame,
                    const pair<Frame, map<int, Point2D>> &RFrame, int &NumCorr);
bool GetMatch(const pair<Frame, map<int, Point2D>> &LFrame,
              const pair<Frame, map<int, Point2D>> &RFrame,
              vector<pair<Vector3d, Vector3d>> &vprCorresUxy1Pair);
void GetRoi(const int Col, const int Row, int MaskWidth, cv::Rect &Roi);
double CalCVDistance(const cv::Point2f &pt1, const cv::Point2f &pt2);
void GetDistortion(const Vector2d &mxy_d, Vector2d &d_u);
void LiftProjective(const Vector2d &uv, const int NumUnDisIter, Vector3d &xyz);
void ProjectUXYZ2Duv(const Vector3d &UXYZ, Vector2d &Duv);
void ProjectUXYZ2Uuv(const Vector3d &UXYZ, Vector2d &Uuv);
// UnDis_Dis_Flag, 0： UnDis; 1: Dis
bool ProjectUXYZ2UV(const Vector3d &UXYZ, const eUnDis_Dis_Flag UnDis_Dis_Flag,
                    const int Col, const int Row, Vector2d &UV);
void GetXYdistortbyXYnormalized(const Vector2d &xyn, Matrix2d &dxyd_dxyn);

void TriangulateOneFt_DualView(const Vector2d &LUxy, const gtsam::Pose3 &Tw_lf,
                              const Vector2d &RUxy, const gtsam::Pose3 &Tw_rf,
                              double &InvDepth_LFrame, Vector3d &Pos3w);
bool SolveTrlByFMat(const vector<pair<Vector3d, Vector3d>> &vprCorresUxy1Pair,
                    gtsam::Pose3 &Trl);
bool SolveTncByPnP(const vector<cv::Point3f> &vCVPos3ws,
                   const vector<cv::Point2f> &vCVUxys,
                   gtsam::Pose3 &Tnc);

//Print
void PrintTns2PA(const double Epoch, const gtsam::Pose3 Tns);
void PrintNavs2PVA(const double Epoch, const gtsam::NavState Navs);

// Test func
void TestSetTime1();  // int InitImageId, int CurImageId, int IdRange_Int2Cur
void TestSetTime2AndShowTimeRange();

// void TestSetTime1(double InitiTime, double CurTime, double
// TimeRange_Int2Cur); void TestSetTime2AndShowTimeRange(double InitiTime,
// double CurTime,
//                                   double TimeRange_Int2Cur);

void TestTxt_CopyAndModifyWhenNeed();
}  // namespace VIG
