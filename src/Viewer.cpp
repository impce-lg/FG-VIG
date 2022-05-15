#include "Viewer.h"

namespace VIG {

Viewer::Viewer(const string &ConfigFile) {
  std::string config_file = ConfigFile;
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

  float fps = fsSettings["Cam.fps"];
  if (fps < 1) fps = 30;
  mT = 1e3 / fps;

  mImageWidth = fsSettings["Cam.nCols"];
  mImageHeight = fsSettings["Cam.nRows"];
  if (mImageWidth < 1 || mImageHeight < 1) {
    cout << "Unknown Camera Size!" << endl;
  }

  fsSettings["Viewer.bFollow"] >> mbFollow;
  fsSettings["Viewer.bShowKeyFrames"] >> mbShowKeyFrames;
  fsSettings["Viewer.bShowMapPoints"] >> mbShowMapPoints;
  fsSettings["Viewer.bSaveCurMap"] >> mbSaveCurMap;

  mCameraSize = fsSettings["Viewer.CameraSize"];
  mCameraLineWidth = fsSettings["Viewer.CameraLineWidth"];
  mKeyFrameSize = fsSettings["Viewer.KeyFrameSize"];
  mKeyFrameLineWidth = fsSettings["Viewer.KeyFrameLineWidth"];
  mPointSize = fsSettings["Viewer.PointSize"];

  mViewpointX = fsSettings["Viewer.ViewpointX"];
  mViewpointY = fsSettings["Viewer.ViewpointY"];
  mViewpointZ = fsSettings["Viewer.ViewpointZ"];
  mViewpointF = fsSettings["Viewer.ViewpointF"];

  fsSettings["WriteVideo"] >> mbWriteVideo;
  if (mbWriteVideo) {
    string strVideo = fsSettings["Video"];

    stringstream s;
    s << "LOCALIZATION | ";
    int baseline = 0;
    cv::Size textSize =
        cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

    mVideoWriter = cv::VideoWriter(
        strVideo,                                     //写入视频的目录
        cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),  //-1时弹出选择编码对话框
        fps,                                          //帧率
        cv::Size(mImageWidth, mImageHeight + textSize.height + 10), 1);
  }
}

void Viewer::InitiViewer(const gtsam::Pose3 &InitFrameTnc) {
  mNumMPs = 0;

  cv::namedWindow("VIG: CurFrame");
  pangolin::CreateWindowAndBind("VIG: Mapper", 1024, 600);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Define Camera Render Object (for view / scene browsing)
  // set a camera, pangolin::ProjectionMatrix: intrinsic parameters
  // pangolin::ProjectionMatrix(w,h,fu,fv,u0,v0,zNear,zFar);
  // pangolin::ModelViewLookAt()
  mS_cam = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(1024, 600, mViewpointF, mViewpointF, 512, 389,
                                 0.1, 1000),
      pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0,
                                pangolin::AxisNegY));

  // Add named OpenGL viewport to window and provide 3D Handler
  // creat a Viewport
  mD_cam = pangolin::CreateDisplay()
               .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0,
                          -1024.0f / 600.0f)
               .SetHandler(new pangolin::Handler3D(mS_cam));

  // GetCurrentOpenGLCameraMatrix(InitFrameTnc);
  mTnc.SetIdentity();
}

// Note: CurFrameTnc and CurKeyFramesPoses defines the transformation from
// CAMERA to WORLD coordinate frame WORLD Frame: c0 frame,i.e,the initial frame
// of camera frame
void Viewer::RunViewer(
    const map<int, Point3D> &MapPoints,
    const map<double, pair<Frame, map<int, Point2D>>> &mpKeyFrames,
    const cv::Mat &CurImg) {
  mNumKFs = (int)mpKeyFrames.size();
  mNumMPs = (int)MapPoints.size();

  /**** VIG-FactorGraph: Map Viewer ****/
  // clear color buffer, depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  GetCurrentOpenGLCameraMatrix(gtsam::Pose3::identity());

  // Follow Camera
  if (mbFollow) {
    mS_cam.Follow(mTnc);
  } else if (!mbFollow) {
    mS_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
        mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, 0.0, 1.0));
    mS_cam.Follow(mTnc);
    mbFollow = true;
  } else if (!mbFollow) {
    mbFollow = false;
  }

  mD_cam.Activate(mS_cam);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  DrawCurrentCamera();
  if (mbShowKeyFrames) DrawKeyFrames(mpKeyFrames);
  if (mbShowMapPoints)
    DrawMapPoints(mpKeyFrames.rbegin()->second.first.Tnc, MapPoints);

  pangolin::FinishFrame();

  /**** VIG-FactorGraph: Frame Viewer ****/
  mCurFrameId = mpKeyFrames.rbegin()->second.first.FrameId;  //+ to_string()
  cv::Mat im = DrawFrame(MapPoints, mpKeyFrames.rbegin()->second.second, CurImg);
  cv::imshow("VIG: CurFrame", im);
  if (mbWriteVideo) mVideoWriter.write(im);
  cv::waitKey(mT);

  if (mbSaveCurMap)  //
    pangolin::SaveWindowOnRender("window");
}

void Viewer::GetCurrentOpenGLCameraMatrix(const gtsam::Pose3 &CurFrameTnc) {
  Matrix4d eCurFrameTnc = CurFrameTnc.matrix();
  Matrix3d Rnc = eCurFrameTnc.block<3, 3>(0, 0).transpose();
  Vector3d pnc = eCurFrameTnc.block<3, 1>(0, 3);

  mTnc.m[0] = Rnc(0, 0);
  mTnc.m[1] = Rnc(1, 0);
  mTnc.m[2] = Rnc(2, 0);
  mTnc.m[3] = 0.0;

  mTnc.m[4] = Rnc(0, 1);
  mTnc.m[5] = Rnc(1, 1);
  mTnc.m[6] = Rnc(2, 1);
  mTnc.m[7] = 0.0;

  mTnc.m[8] = Rnc(0, 2);
  mTnc.m[9] = Rnc(1, 2);
  mTnc.m[10] = Rnc(2, 2);
  mTnc.m[11] = 0.0;

  mTnc.m[12] = pnc(0);
  mTnc.m[13] = pnc(1);
  mTnc.m[14] = pnc(2);
  mTnc.m[15] = 1.0;
}

void Viewer::DrawCurrentCamera() {
  const float &w = mCameraSize;
  const float h = w * 0.75;
  const float z = w * 0.6;

  glPushMatrix();
  // cout<<"mTnc.m:"<<endl<<mTnc<<endl;

#ifdef HAVE_GLES
  glMultMatrixf(mTnc.m);
#else
  glMultMatrixd(mTnc.m);
#endif

  glLineWidth(mCameraLineWidth);
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);
  glEnd();

  glPopMatrix();
}

void Viewer::DrawKeyFrames(
    const map<double, pair<Frame, map<int, Point2D>>> &mpKeyFrames) {
  const float &w = mKeyFrameSize;
  const float h = w * 0.75;
  const float z = w * 0.6;

  for (auto &mpitFrame : mpKeyFrames) {
    // Matrix<double, 4, 4> TncEgTr =
    //     mpitFrame.second.Tnc.matrix().transpose();
    // cv::Mat Tnc;
    // cv::eigen2cv(TncEgTr, Tnc);

    auto gtTwc = mpKeyFrames.rbegin()->second.first.Tnc.inverse() *
                 mpitFrame.second.first.Tnc;
    Matrix3d Rwc = gtTwc.rotation().matrix();
    Vector3d pwc = gtTwc.translation();

    std::vector<GLdouble> Twc = {Rwc(0, 0), Rwc(1, 0), Rwc(2, 0), 0.,
                                 Rwc(0, 1), Rwc(1, 1), Rwc(2, 1), 0.,
                                 Rwc(0, 2), Rwc(1, 2), Rwc(2, 2), 0.,
                                 pwc.x(),   pwc.y(),   pwc.z(),   1.};

    glPushMatrix();
    glMultMatrixd(Twc.data());
    glLineWidth(mKeyFrameLineWidth);
    glColor3f(0.0f, 0.0f, 1.0f);
    
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);
    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);
    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);
    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);

    glEnd();

    glPopMatrix();
  }
}

void Viewer::DrawMapPoints(const gtsam::Pose3 &CurTnc,
                           const map<int, Point3D> &MapPoints) {
  glPointSize(mPointSize);
  glBegin(GL_POINTS);

  Vector3d Pos3w;
  for (auto mpit : MapPoints) {
    Pos3w = CurTnc.transformTo(mpit.second.Pos3w);

    if (mpit.second.SolvedFlag == FAILED)
      continue;
    else if (mpit.second.SolvedFlag == OVERSIZE)
      glColor3f(1.0, 0.0, 1.0);
    else if (mpit.second.SolvedFlag == SOLVING)
      glColor3f(0.0, 1.0, 0.0);
    else
      glColor3f(0.0, 0.0, 0.0);

    glVertex3f(Pos3w(0), Pos3w(1), Pos3w(2));
  }
  glEnd();
}

cv::Mat Viewer::DrawFrame(const map<int, Point3D> &MapPoints,
                          const map<int, Point2D> &mpCurFrameFeatures,
                          const cv::Mat &CurImg) {
  cv::Mat im;
  int state = 1;  // Tracking state

  CurImg.copyTo(im);
  if (im.channels() < 3)  // this should be always true
    cvtColor(im, im, cv::COLOR_GRAY2BGR);

  // Draw
  mNumOptimized = 0;
  mNumTracked = 0;
  mNumRejected = 0;
  const float r = 5;
  for (auto mpitFt : mpCurFrameFeatures) {
    mNumTracked++;

    cv::Point2f FeatMeas, pt1, pt2;
    FeatMeas.x = mpitFt.second.Duv(0);
    FeatMeas.y = mpitFt.second.Duv(1);
    pt1.x = mpitFt.second.Duv(0) - r;
    pt1.y = mpitFt.second.Duv(1) - r;
    pt2.x = mpitFt.second.Duv(0) + r;
    pt2.y = mpitFt.second.Duv(1) + r;

    auto mpit = MapPoints.find(mpitFt.first);
    if (mpit != MapPoints.end() && mpit->second.SolvedFlag != SOLVING) {
      cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 255));
      cv::circle(im, FeatMeas, 2, cv::Scalar(255, 0, 255), -1);
      mNumOptimized++;
    } else {
      cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
      cv::circle(im, FeatMeas, 2, cv::Scalar(0, 255, 0), -1);
    }
    // }
  }

  cv::Mat imWithInfo;
  DrawTextInfo(im, state, imWithInfo);

  return imWithInfo;
}

void Viewer::DrawTextInfo(const cv::Mat &im, const int nState,
                          cv::Mat &imText) {
  stringstream s;
  if (nState == 1) {
    s << "LOCALIZATION | ";

    s << "CurFrameId: " << mCurFrameId << ", MPs: " << mNumMPs;
      // << ", Optimized: " << mNumOptimized;
    if (mNumTracked > 0) s << ", + Tracked: " << mNumTracked;
    // if (mNumRejected > 0) s << ", + Rejected: " << mNumRejected;
  } else {
    s << " TRACK LOST. TRYING TO RELOCALIZE ";
  }

  int baseline = 0;
  cv::Size textSize =
      cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

  imText = cv::Mat(im.rows + textSize.height + 10, im.cols, im.type());
  im.copyTo(imText.rowRange(0, im.rows).colRange(0, im.cols));
  imText.rowRange(im.rows, imText.rows) =
      cv::Mat::zeros(textSize.height + 10, im.cols, im.type());
  cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}

};  // namespace VIG
