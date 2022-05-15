#pragma once

#include <gtsam/nonlinear/Values.h>

#include "Configuration.h"

namespace VIG {
class Configuration;

class Mapper {
 public:
  Mapper(const Configuration *pConfig);

  void ClearState();

  void AddMapPoints(const pair<Frame, map<int, Point2D>> &CurFrame);
  void FormMapPoints(
      const map<double, pair<Frame, map<int, Point2D>>> &mpFrames);

  map<int, Point3D> SelectSFMPoints();
  int CountRefCurFrame(const double Epoch);
  int CountCurFrame(const double Epoch);
  int CountMatch(const double LEpoch, const double REpoch);
  vector<pair<Vector3d, Vector3d>> GetMatch(const double LEpoch,
                                            const double REpoch);
  bool CalRelativeTrl(const double LEpoch, const double REpoch,
                      gtsam::Pose3 &Trl);
  bool CheckEnableTriangulate(
      const Point3D &CurPoint,
      const map<double, pair<Frame, map<int, Point2D>>> &mpFrames);
  bool CheckMotionStatus(const Point3D &CurPoint);
  void TriangulateMP_MultView(
      Point3D &CurPoint,
      const map<double, pair<Frame, map<int, Point2D>>> &mpFrames);
  void TriangulateOptMPs_MultView(
      const map<double, pair<Frame, map<int, Point2D>>> &mpFrames);
  bool SolvePnP4AllKeyFrames(
      deque<pair<Frame, map<int, Point2D>>> &dprTrackedFrames,
      const map<double, pair<Frame, map<int, Point2D>>> &mpFrames,
      const gtsam::Pose3 *arCVTn_fx, map<int, Vector3d> &mpSFMFeatIds_Pos3ws);
  bool SolvePnP4CurFrameInKFQue(
      const pair<Frame, map<int, Point2D>> &prCurTrackedFrame,
      gtsam::Pose3 &CurTn_fx_inital);
  void SetPointsPos(
      gtsam::Values &ResultValues,
      map<double, pair<Frame, map<int, Point2D>>> &mpKeyFrames);
  void MoveWin(const double newStart);
  void RemoveFailures();

 public:
  map<int, Point3D> mNewMapPoints;  // 新的地图点(刚刚加入地图的点)
  map<int, Point3D> mIdleMapPoints;  // 闲置点(包括深度超限的点和被边缘化的点)
  int mLastTrackNum;

 private:
  double compensatedParallax2(const Point3D &it_per_id);
};

}  // namespace VIG
