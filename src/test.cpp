#include <opencv2/opencv.hpp>
#include "qrcode.h"
#include <QString>
#include <QDir>
#include <QString>

using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {

  float max = 0;
  for (int cnt = 0; cnt < 32; ++cnt)
  {
    std::string one_file ="ir_" + to_string(cnt) + ".png";
    Mat image = imread(one_file, IMREAD_UNCHANGED);

    cv::Mat gray(image.rows, image.cols, CV_32F);

    for (int i = 0; i < image.rows; ++i)
      for (int j = 0; j < image.cols; ++j)
      {
        float f;
        uchar *pos = (uchar*)&f;
        for (int k = 0; k < 4; ++k)
        {
          *(pos + k) = image.at<Vec4b>(i, j)(k);
        }
        cerr<<f<<endl;
        max= std::max(max, f);
        gray.at<float>(i, j) = min(f/4000, 1.0f);

      }
    imshow("", gray);
//    waitKey();
    Mat1b imageF_8U;
    gray.convertTo(imageF_8U, CV_8U, 255);
    cv::imwrite(to_string(cnt) + "gray.png", imageF_8U);
    cerr<<max<<endl;
  }

    return 0;
}
