#include <iostream>
#include "rostalk.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

//--------------------------------------
using std::placeholders::_1;
using namespace cv;

RosTalk::~RosTalk() = default;

std::string base64Encode(const unsigned char *Data, int DataByte)
{
  // 编码表
  const char EncodeTable[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  // 返回值
  std::string strEncode;
  unsigned char Tmp[4] = {0};
  int LineLength = 0;
  for (int i = 0; i < (int)(DataByte / 3); i++)
  {
    Tmp[1] = *Data++;
    Tmp[2] = *Data++;
    Tmp[3] = *Data++;
    strEncode += EncodeTable[Tmp[1] >> 2];
    strEncode += EncodeTable[((Tmp[1] << 4) | (Tmp[2] >> 4)) & 0x3F];
    strEncode += EncodeTable[((Tmp[2] << 2) | (Tmp[3] >> 6)) & 0x3F];
    strEncode += EncodeTable[Tmp[3] & 0x3F];
    if (LineLength += 4, LineLength == 76)
    {
      strEncode += "\r\n";
      LineLength = 0;
    }
  }
  // 对剩余数据进行编码
  int Mod = DataByte % 3;
  if (Mod == 1)
  {
    Tmp[1] = *Data++;
    strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
    strEncode += EncodeTable[((Tmp[1] & 0x03) << 4)];
    strEncode += "==";
  }
  else if (Mod == 2)
  {
    Tmp[1] = *Data++;
    Tmp[2] = *Data++;
    strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
    strEncode += EncodeTable[((Tmp[1] & 0x03) << 4) | ((Tmp[2] & 0xF0) >> 4)];
    strEncode += EncodeTable[((Tmp[2] & 0x0F) << 2)];
    strEncode += "=";
  }

  return strEncode;
}

// imgType 包括png bmp jpg jpeg等opencv能够进行编码解码的文件
std::string Mat2Base64(const cv::Mat &img, std::string imgType)
{
  // Mat转base64
  std::string img_data;
  std::vector<uchar> vecImg;
  std::vector<int> vecCompression_params;
  vecCompression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  vecCompression_params.push_back(90);
  imgType = "." + imgType;
  cv::imencode(imgType, img, vecImg, vecCompression_params);
  img_data = base64Encode(vecImg.data(), vecImg.size());
  return img_data;
}

void RosTalk::init()
{

  // // 读取视频或摄像头
  // //  VideoCapture capture(0);

  // std::string source = "rtsp://admin:ls123456@192.168.2.127:554/h265/ch1/main/av_stream"; // 摄像头的rtsp地址
  // VideoCapture capture(source, CAP_FFMPEG);

  // // while (true)
  // // {
  // //   Mat frame;
  // //   capture >> frame;
  // //   // imshow("读取视频", frame);
  // //   // waitKey(30); // 延时30

  // //   static int a{3};
  // //   a--;
  // //   if (a == 0)
  // //   {
  // //     a = 1;
  // //     std::vector<std::string> data;
  // //     cv::Mat shrink;
  // //     cv::resize(frame, shrink, cv::Size(frame.cols / 2, frame.rows / 2));
  // //     std::string camera_data = "data:image/png;base64," + Mat2Base64(shrink, "jpeg");
  // //     data.push_back("image");
  // //     data.push_back(camera_data);
  // //     this->websocket->pushData(data);
  // //   }
  // //   // usleep(1*1000);

  // //   // std::vector<std::string> data;
  // //   // data.push_back("image");
  // //   // data.push_back("data");
  // //   // this->websocket->pushData(data);
  // //   // usleep(1000*1000);
  // // }

  lidar_datas_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ch128x1/lslidar_point_cloud",
      rclcpp::QoS{10},
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
      { lidarDatasCallback(msg); });
}

void RosTalk::lidarDatasCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

  // 子节点
  Json::Value root;
  Json::Value friends;
  Json::FastWriter writer;

  auto caculate2Bit = [](const double &val)
  {
    char *chCode;
    chCode = new char[20];
    sprintf(chCode, "%.2lf", int(val * 100) * 0.01);
    std::string str(chCode);
    delete[] chCode;
    return str;
    // return std::to_string(int(data * 100) * 0.01);
  };

  // sensor_msgs::PointCloud out_pointcloud;
  // sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);

  auto cloud =
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud);

  for (int i = 0; i < cloud->points.size(); i++)
  {
    // cout << out_pointcloud.points[i].x << ", " << out_pointcloud.points[i].y << ", " << out_pointcloud.points[i].z << endl;
    friends["x"] = Json::Value(caculate2Bit(cloud->points[i].x));
    friends["y"] = Json::Value(caculate2Bit(cloud->points[i].y));
    friends["z"] = Json::Value(caculate2Bit(cloud->points[i].z));

    // friends["z"] = Json::Value(lidardata.mark);
    root.append(friends);
  }

  std::string lidar_data = writer.write(root);
  std::vector<std::string> data;
  data.push_back("lidar");
  data.push_back(lidar_data);
  this->websocket->pushData(data);

  std::cout << "1111111111111111111111111111" << std::endl;
}