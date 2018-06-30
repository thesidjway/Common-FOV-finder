#include <mvtlc/FeatureTrailManager.h>
#include <iostream>
#include <chrono>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;
using namespace gtsam;

void generatePoints (float focal, float height, float width, float max_dis, vector<Point3>& points) {
  float cx = width/2;
  float cy = height/2;
  for(int x = 10; x < width - 10 ; x+= 5 ) {
    for(int y = 10; y < height - 10; y+= 5 ) {
      for (double z = 1 ; z < 5; z += 0.25) {
	double distmod = z + 0.25;
	points.push_back( Point3((x-cx)/focal*distmod, (y-cy)/focal*distmod, distmod) );
      }
    }
  }
}

void projectPointsToCamera (PinholeCameraCal3_S2 camera, vector<Point3> points, vector<Point2>& image_pts, vector<Point2>& ori_pts) {
  for (auto x:points) {
    try {
      Point2 pt = camera.project(x);
      if (pt.x() < 310 && pt.x() > 10 && pt.y() > 10 && pt.y() < 230) {
	PinholeCameraCal3_S2 camera_orig(Pose3(), camera.calibration());
	Point2 ori_pt = camera_orig.project(x);
	ori_pts.push_back(ori_pt);
	image_pts.push_back(pt);
	//cout << pt << ", while prev was: " << ori_pt << endl;
      }
    } catch (...) { 
    }
  }
}

int main() {
  Rot3 r = Rot3::Quaternion(0.4, -0.2, -0.4, 0.2);
  Point3 t = Point3(0, 0, 0.46);
  Pose3 p(r,t);
  PinholeCameraCal3_S2 camera(p, Cal3_S2(300, 300, 0 , 160, 120));
  
  vector<Point3> pts;
  generatePoints(300, 240, 320, 5, pts);
  vector<Point2> image_pts, orig_pts;
  vector<cv::Point2f> image_pts_cv, orig_pts_cv;
  projectPointsToCamera (camera, pts, image_pts, orig_pts);
  cv::Mat image1 = cv::Mat::zeros(240, 320, CV_32FC1);
  cv::Mat image2 = cv::Mat::zeros(240, 320, CV_32FC1);
  for(auto x : image_pts) {
    image_pts_cv.push_back(cv::Point2f(x.x(), x.y()));
    image2.at<float>(cv::Point2f(x.x(), x.y())) = 255;
  }
  for(auto x : orig_pts) {
    orig_pts_cv.push_back(cv::Point2f(x.x(), x.y()));
    image1.at<float>(cv::Point2f(x.x(), x.y())) = 255;
  }
//   vector<cv::Point2f> hull1, hull2;
//   convexHull(orig_pts_cv, hull1);
//   convexHull(image_pts_cv, hull2);
//   for(auto x : hull1) {
//     image1.at<float>(x) = 255;
//   }
//   for(auto x : hull2) {
//     image2.at<float>(x) = 255;
//   }
  cv::imshow("image1" , image1);
  cv::imshow("image2" , image2);
  cv::waitKey(1000000);
}

