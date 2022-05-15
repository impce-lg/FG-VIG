/**
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
             佛祖保佑       永无BUG
*/
//
// Created by Ronghe Jin on 11/1/20.
//

#include "Tracker.h"
#include "VIGInitializer.h"
#include "Viewer.h"

using namespace std;
using namespace VIG;

map<double, NavTimeLine> mpObss_;
map<double, int> mpImages_;
map<double, GNSSSol> mpGNSSs_;

Tracker *pTracker_;
Optimizer *pOptimizer_;
VIGInitializer *pVIGInitializer_;
Viewer *pViewer_;

void LoadImages(const string &ImageFile) {
  ifstream ifImage(ImageFile);
  map<double, int> mpImages_Init;

  while (!ifImage.eof()) {
    string strLine;
    getline(ifImage, strLine);

    if (!strLine.empty()) {
      if (strLine.at(0) < '0' || strLine.at(0) > '9') continue;

      int ImgID;
      double ImgEpoch;
      stringstream ss(strLine);
      ss >> ImgID;
      ss >> ImgEpoch;
      if (ImgEpoch < gStartTime) {
        mpImages_Init[ImgEpoch] = ImgID;
        continue;
      }  // add one

      mpImages_[ImgEpoch] = ImgID;
      // avoid vision when time same as GNSS
      if (mpObss_.find(ImgEpoch) == mpObss_.end())
        mpObss_.insert(make_pair(ImgEpoch, NavTimeLine(VISION, -1)));
      else {
        cerr << "Same GNSS and Vision epoch: " << ImgEpoch
             << ",Image Id: " << ImgID << ", please check and retry!" << endl;
        exit(-1);
      }
    }
  }

  // 在起始时刻之前加两帧，作为引入
  auto mpLastIt = mpImages_Init.rbegin();
  for (int i = 0; i < 2; i++) {
    mpImages_[mpLastIt->first] = mpLastIt->second;

    mpLastIt++;
    if (mpLastIt == mpImages_Init.rend()) {
      cerr << "Wrong Image Epoch, not enough for initial tracking!" << endl;
      exit(-1);
    }
  }

  ifImage.close();
}

void LoadGNSS(const string &GNSSFile) {
  ifstream ifGNSS(GNSSFile);

  while (!ifGNSS.eof()) {
    string strLine;
    getline(ifGNSS, strLine);
    
    if (!strLine.empty()) {
      if (strLine.at(0) < '0' || strLine.at(0) > '9') continue;

      double Tow, Nx, Ny, Nz, stdNx, stdNy, stdNz;
      stringstream ss(strLine);
      ss >> Tow;
      if(Tow < gStartTime) continue;
      
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

      mpGNSSs_[Tow] = sGNSSSol;
      if (mpObss_.find(Tow) == mpObss_.end()) {
        mpObss_.insert(make_pair(Tow, NavTimeLine(GNSS, -1)));
      }
      else {
        cerr << "Same GNSS and Vision epoch: " << Tow
             << ", please check and retry!" << endl;
        exit(-1);
      }
    }
  }

  ifGNSS.close();
}

void ReadIMULine(const string &strIMULine, IMU &sIMU) {
  double t, wx, wy, wz, ax, ay, az;

  stringstream ss(strIMULine);
  ss >> t;
  ss >> wx;
  ss >> wy;
  ss >> wz;
  ss >> ax;
  ss >> ay;
  ss >> az;

  sIMU.T = t;
  sIMU.Gyro << wx, wy, wz;
  sIMU.Acc << ax, ay, az;
}

IMU GetFirstIMU(ifstream &fIMU) {
  IMU sIMU;
  
  while (!fIMU.eof()) {
    string strLine;

    if (!fIMU.eof()) getline(fIMU, strLine);
    if (!strLine.empty()) {
      if (strLine.at(0) < '0' || strLine.at(0) > '9') continue;

      double t;
      stringstream ss(strLine);
      ss >> t;

      if (t < gStartTime) continue;

      if (t != gStartTime) {
        cerr << "Initial time of imu and navblh are different, please check!"
             << endl;
        exit(-1);
      }
      ReadIMULine(strLine, sIMU);
      break;
    }
  }
  return sIMU;
}

void MoveOnIMU(ifstream &fIMU, IMU &sIMU) {
  if (!fIMU.eof()) {
    string strCurLine;
    getline(fIMU, strCurLine);
    if (!strCurLine.empty()) {
      ReadIMULine(strCurLine, sIMU);
      return;
    }
  }
}

void ReadIMU(const double CurEpoch, ifstream &ifIMU, IMU &sIMU) {
  while (sIMU.T <= CurEpoch) {
    pOptimizer_->PushIMU(sIMU);
    MoveOnIMU(ifIMU, sIMU);
  }

  // 预积分根据需要在Optimizer中计算，这里不再事先计算
  // pOptimizer_->PreIntegrate(gPreEpoch, CurEpoch);
}

void CoupleGNSS() {
  pOptimizer_->UpdateGNSS4Solver();
  pOptimizer_->Optimize();
  pOptimizer_->UpdateGNSS();
}

void CoupleVision() {
  pOptimizer_->UpdateVision4Solver();
  pOptimizer_->Optimize();
  pOptimizer_->UpdateVision();
  pOptimizer_->UpdateGNSS();
}

void SaveTailAndExit() {
  pOptimizer_->SaveTail();
  cout << endl << "Finihing..." << endl;
}

int main(int argc, char* argv[]) {

  if (argc < 2) {
    cerr << endl << "Usage: ./VIG_FactorGraph path_to_config_yaml" << endl;
    return 1;
  }
  
  cout << R"(____   ____.___  ________          ___________________ )" << endl;
  cout << R"(\   \ /   /|   |/  _____/          \_   _____/  _____/ )" << endl;
  cout << R"( \   Y   / |   /   \  ___   ______  |    __)/   \  ___ )" << endl;
  cout << R"(  \     /  |   \    \_\  \ /_____/  |     \ \    \_\  \)" << endl;
  cout << R"(   \___/   |___|\______  /          \___  /  \______  /)" << endl;
  cout << R"(                       \/               \/          \/ )" << endl;

  string strConfigFile = string(argv[1]);
  /**** Load Configuration config file ****/
  Configuration *pConfig = new Configuration(strConfigFile);
  pConfig->SetGlobalParas();

  /**** Initialize Vision and Optimizer ****/
  if (gbUseCam) {
    LoadImages(pConfig->mImageFile);
    pTracker_ = new Tracker(pConfig);

    for (int i = 0; i < 2; i++) {
      pTracker_->Track(mpImages_.begin()->first, mpImages_.begin()->second);
      mpImages_.erase(mpImages_.begin());
    }

    // Initialize Optimizer
    geVIGState = UNINITIALIZED;
    pVIGInitializer_ = new VIGInitializer(pConfig);
    pOptimizer_ = pVIGInitializer_;
  } else {
    // Initialize Optimizer
    pOptimizer_ = new Optimizer(pConfig);
  }

  /**** Initialize Viewer ****/
  if (gbUseCam) {
    pViewer_ = new Viewer(strConfigFile);
    pViewer_->InitiViewer(pOptimizer_->mInitTni);
  }

  /**** Initialize IMU ****/
  ifstream ifIMU(pConfig->mIMUFile);
  IMU sIMU = GetFirstIMU(ifIMU);

  /**** Initialize GNSS files ****/
  if (gbUseGNSS) LoadGNSS(pConfig->mGNSSFile);

  /**** 分配状态索引序号 ****/
  {
    int NavIndex = 0;  // after the prior: 0
    // for (auto &mpitObs : mpObss_) mpitObs.second.NavInd = ++NavIndex;
    for (auto mpitObs = mpObss_.begin(); mpitObs != mpObss_.end(); mpitObs++)
      mpitObs->second.NavInd = ++NavIndex;
  }

  /**** main loop ****/
  for (auto mpitObs = mpObss_.begin(); mpitObs != mpObss_.end(); mpitObs++) {
    double CurEpoch = mpitObs->first;
    auto CurNavIndex = mpitObs->second.NavInd;
    auto CurObsType = mpitObs->second.ObsType;
    ReadIMU(CurEpoch, ifIMU, sIMU);

    // ************** IG ************** //
    if (!gbUseCam && gbUseGNSS) {
      assert(CurObsType == GNSS);

      pOptimizer_->ConfigGNSS(CurNavIndex, mpGNSSs_.at(CurEpoch));
      CoupleGNSS();
    }
    // ************** VIG ************** //
    else if (gbUseCam && gbUseGNSS) {
      // 观测值为GNSS
      if (CurObsType == GNSS) {
        pOptimizer_->ConfigGNSS(CurNavIndex, mpGNSSs_.at(CurEpoch));

        // 如果VIG没有初始化，就计算IG组合
        if (geVIGState == UNINITIALIZED) {
          CoupleGNSS();
        }
        // 如果已经完成VIG初始化
        else if (geVIGState == VIG_OK) {
          double CurGNSSEpoch = CurEpoch;

          // 往前移动一帧
          mpitObs++;
          // 如果没有下一帧，就结束
          if (mpitObs == mpObss_.end() || mpitObs->second.ObsType == GNSS)
            break;

          CurEpoch = mpitObs->first;
          CurNavIndex = mpitObs->second.NavInd;
          CurObsType = mpitObs->second.ObsType;
          ReadIMU(CurEpoch, ifIMU, sIMU);

          auto CurFrame = pTracker_->Track(CurEpoch, mpImages_.at(CurEpoch));
          pOptimizer_->ConfigCurFrame(CurNavIndex, CurFrame);
          pOptimizer_->InterplKFOnGNSS(CurGNSSEpoch);

          CoupleVision();
        }
      }
      // 观测值为VISION
      else if (CurObsType == VISION) {
        auto CurFrame = pTracker_->Track(CurEpoch, mpImages_.at(CurEpoch));
        pOptimizer_->ConfigCurFrame(CurNavIndex, CurFrame);

        // 初始化
        if (geVIGState == UNINITIALIZED) {
          if (pOptimizer_->CheckNewKeyFrame(CurEpoch)) {
            pVIGInitializer_->TrackMonoInit();

            if (pVIGInitializer_->IsKFsEnough()) {
              if (pVIGInitializer_->Initialize_Loosely())
                // 初始化中已有优化操作
                continue;
            }
          }
        }
      }

      // 只优化关键帧
      if (geVIGState == VIG_OK) {
        if (pOptimizer_->CheckNewKeyFrame(CurEpoch)) {
          CoupleVision();
        }

        // Viewer要写视频，放在这里还是原始均匀的采样
        // 放到后面改变了采样率（关键帧），写的视频忽快忽慢
        pViewer_->RunViewer(pOptimizer_->GetMapPoints(),
                            pOptimizer_->GetKeyFrames(),
                            pTracker_->mCurImage);
      }
    }
  }

  ifIMU.close();
  SaveTailAndExit();

  return 0;
}
