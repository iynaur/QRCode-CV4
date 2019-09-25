#include <opencv2/opencv.hpp>
#include "qrcode.h"
#include <QString>
#include <QDir>
#include <QString>

using namespace cv;

int test_qrcode_cv(cv::Mat src) {
//#if (CV_VERSION_MAJOR > 3)
    cv::QRCodeDetector qrdc;
    std::vector<Point2f> points;
    qrdc.detect(src, points);
    cv::Mat code;
    qrdc.decode(src, points, code);
    if (code.cols) {
      cv::resize(code, code, cv::Size((code.cols-1)*8+1, (code.rows-1)*8+1),cv::INTER_NEAREST);

      cv::imshow("Matches", code);
      cv::waitKey(0);

    }
    else {
      std::cerr<<"BAD IMAGE\n";
    }
    std::cerr<<points.size()<<std::endl;
    for (Point2f p : points)
    {
      std::cerr<<p.x<<" "<<p.y<<"\n";
    }

    std::string dec = qrdc.detectAndDecode(src);
    std::cerr << "decode: " << dec<<" from\n";
//#endif
    return 0;
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
    std::string dec = qrdc.detectAndDecode(src);
    std::cerr << "decode: " << dec<<" from\n";
    return 0;
}

int main(int argc, char *argv[]) {
  QString folderPath = "../QRCode-CV4/images";
  QDir dir(folderPath);
  QStringList list = dir.entryList();

  for (QString file : list)
  {

      if (file.endsWith(".jpg", Qt::CaseInsensitive) ||
          file.endsWith(".png", Qt::CaseInsensitive) ||
          file.endsWith(".jpeg", Qt::CaseInsensitive))
      {
          std::string one_file = (folderPath + "/"+file).toStdString();
          Mat image = imread(one_file);
          test_qrcode_cv(image);
          std::cerr<<one_file<<std::endl;
          test_qrcode(image);
      }
  }

    return 0;
}
