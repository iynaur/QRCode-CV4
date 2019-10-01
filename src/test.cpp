#include <opencv2/opencv.hpp>

#include <QString>
#include <QDir>
#include <QString>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "qrcode.h"
using namespace cv;
using namespace std;

#define CERR(x) std::cerr<<#x<<" = "<<(x)<<std::endl;//CERR(1<<2)


int test_qrcode_cv(cv::Mat src, std::vector<Point2f> &points) {
#if (CV_VERSION_MAJOR > 3)
    cv::QRCodeDetector qrdc;

    qrdc.detect(src, points);
    std::cerr<<points.size()<<std::endl;
    for (Point2f p : points)
    {
      std::cerr<<p.x<<" "<<p.y<<"\n";
    }
#endif
    return points.size();
}

int test_qrcode(cv::Mat src) {
    pc::QRCodeDetector qrdc;
    std::vector<Point2f> points;
    qrdc.detect(src, points);
    std::cerr<<points.size()<<std::endl;
    for (Point2f p : points)
    {
      std::cerr<<p.x<<" "<<p.y<<"\n";
    }
    return 0;
}

double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

int main(int argc, char *argv[]) {
    string filename = "../QRCode-CV4/tys_data.txt";
    string line;
    ifstream myfile (filename.c_str());

    vector<array<float, 6>> allposes;
    if (myfile.is_open())
    {
        std::array<float, 6> pos;
        while(getline(myfile, line))
        {
            if (line.empty()) continue;
            stringstream ss;
            ss<<line;
            for (int i = 0; i<6; ++i){
                ss>>pos[i];
                if (i > 2) pos[i] = pos[i]/180*M_PI;
            }
            allposes.push_back(pos);
        }
        myfile.close();
    }
  string folderPath = "../tys-data3/tys-data2";

  vector<vector<Point2f>> pointslist;
  vector<array<float, 6>> goodposes;
  for (int i  = 1; i<= 18 ; ++i)
  {
//      system(("cp " + folderPath + "/"+std::to_string(i) +"/2.jpg " + "./robot" + std::to_string(i)
//             + ".jpg").c_str());
          std::string one_file = folderPath + "/"+std::to_string(i) +"/rgb_0.png";
          Mat image = imread(one_file);
          std::vector<Point2f> points;
          if (test_qrcode_cv(image, points) == 4) {
              pointslist.push_back(points);
              if (i<allposes.size()) goodposes.push_back(allposes[i]);
              std::cerr<<one_file<<std::endl;

          }
  }
  vector<vector<Point3f> > object_pointsL;
  for (auto x : pointslist)
  {
      float a = 0.5;
//      vector<Point3f> obj({Point3f(0,0,0), Point3f(1,0,0), Point3f(1,1,0), Point3f(0,1,0)});
      vector<Point3f> obj({Point3f(0,0,0), Point3f(0,a,0), Point3f(a,a,0), Point3f(a,0,0)});
      object_pointsL.push_back(obj);
  }

  Mat KL;
  Mat DL(5, 1, CV_32F);
  DL = 0.0;
  vector<Mat> rvecsL, tvecsL;
  int flagL = 0;
  if (1){
      flagL |= CALIB_ZERO_TANGENT_DIST;
      flagL |= CALIB_FIX_K3;
  }

  double errorL;
  errorL = calibrateCamera(object_pointsL, pointslist, Size(1920, 1080), KL, DL, rvecsL, tvecsL, flagL);
  CERR(errorL)
  vector<float> reprojErrsL;
  errorL = computeReprojectionErrors(object_pointsL, pointslist,
              rvecsL, tvecsL, KL, DL, reprojErrsL);

  CERR(KL);
  CERR(DL);
  for (auto x : reprojErrsL)
  {
      CERR(x);
  }

  int n = goodposes.size();

  auto abc2rot = [](array<float, 6> pos)->Eigen::Matrix3f
  {
      Eigen::Vector3f axis(pos[3], pos[4], pos[5]);
//      CERR(axis);
      float angle = axis.norm();
      axis = axis/angle;
//      return (Eigen::AngleAxisf(angle, axis)) .matrix();//mirror
      return (Eigen::AngleAxisf(pos[3], Eigen::Vector3f::UnitX())*
              Eigen::AngleAxisf(pos[4], Eigen::Vector3f::UnitY())*
              Eigen::AngleAxisf(pos[5], Eigen::Vector3f::UnitZ())) .matrix();
  };
  auto rvec2rot = [](Mat rvec)->Eigen::Matrix3f
  {
//      CERR(rvec);
      //why double? how to get scalar type?
      Eigen::Vector3f axis(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0));
//      CERR(axis);
      float angle = axis.norm();
      axis = axis/angle;
      return (Eigen::AngleAxisf(angle, axis)) .matrix();//mirror
  };
  for (int i = 0; i< n; ++i)//robot pose
  {
      Eigen::Matrix3f base2handi = abc2rot(goodposes[i]);
      Eigen::Matrix3f cam2pati = rvec2rot(rvecsL[i]);
      CERR(cam2pati)
      for (int j=0; j<n; ++j)// cam pose
      {
          if (i==j) continue;

          Eigen::Matrix3f base2handj = abc2rot(goodposes[j]);

          Eigen::Matrix3f cam2patj = rvec2rot(rvecsL[j]);

          Eigen::Matrix3f A = cam2patj*cam2pati.inverse();
          Eigen::Matrix3f B = base2handj.inverse()*base2handi;

          Eigen::AngleAxisf ax(A);
          Eigen::AngleAxisf bx(B);

          float x = ax.angle() - floor(ax.angle()/(2*M_PI))*2*M_PI - M_PI;
          float y = bx.angle() - floor(bx.angle()/(2*M_PI))*2*M_PI - M_PI;

          cerr<<i<<" "<<j<<" "<<((fabs(x) - fabs(y))/M_PI*180)<<endl;
      }
  }



    return 0;
}
