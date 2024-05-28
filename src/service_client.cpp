

/*
 * @Author: Jianheng Liu
 * @Date: 2022-01-08 18:25:31
 * @LastEditors: Jianheng Liu
 * @LastEditTime: 2022-01-09 16:56:56
 * @Description: Description
 */
//
// Created by ou on 2021/3/14.
//
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <yolo_ros/yolo.h>
#include <yolo_ros/yoloAction.h>

#include <yolo_ros/DetectionMessage.h>
#include <yolo_ros/DetectionMessages.h>

using namespace std;

ros::ServiceClient client;
// ros::Publisher yolo_results_pub;
ros::Publisher yolo_results_pub_left;
ros::Publisher yolo_results_pub_right;

void img_callback(sensor_msgs::Image::ConstPtr img_msg,const std::string& camera_name) {
  double start_t = ros::Time::now().toSec();
  cv::Mat img = cv_bridge::toCvCopy(img_msg, img_msg->encoding)->image;

  // 如果图像是单通道，转换为三通道 shuangmu
  if (img.channels() == 1) {
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  }

  yolo_ros::yolo srv;
  srv.request.image =
      *cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  if (client.call(srv)) {
    yolo_ros::DetectionMessages detection_msgs;
    detection_msgs.header = img_msg->header;
    for (auto &result : srv.response.results) {
      // auto xyxy = result.bbox.xyxy;

      //收集检测信息
      yolo_ros::DetectionMessage dmsg;
      dmsg.x1 = result.bbox.xyxy[0];
      dmsg.y1 = result.bbox.xyxy[1];
      dmsg.x2 = result.bbox.xyxy[2];
      dmsg.y2 = result.bbox.xyxy[3];
      dmsg.class_pred = result.id;
      dmsg.score = result.prob;
      dmsg.label = result.label;
      if (dmsg.score < 0.5) {
        continue;
      }
      detection_msgs.data.push_back(dmsg);
      detection_msgs.detection_num++;

      //可视化
      // cv::Point p1(dmsg.x1, dmsg.y1), p2(dmsg.x2, dmsg.y2), wh = p2 - p1;
      // auto thickness = cv::min(wh.x, wh.y);
      // cv::rectangle(img, p1, p2, cv::Scalar(255, 0, 0), thickness / 40 + 1);
      // cv::putText(img, result.label, p1, cv::FONT_HERSHEY_COMPLEX, 1,
      //             cv::Scalar(0, 0, 255), 1, 0);
      //            cout << result.label << endl;
      

    }
    if (camera_name == "left") {
      yolo_results_pub_left.publish(detection_msgs);
    } else if (camera_name == "right") {
      yolo_results_pub_right.publish(detection_msgs);
    }
    // yolo_results_pub.publish(detection_msgs);
    // printf("YOLOv5 detect cost: %fs\n", ros::Time::now().toSec() - start_t);
  }
  //可视化
      // cv::imshow("img", img);
      // cv::waitKey(1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "yolo_client_node");
  ros::NodeHandle n;

  // yolo_results_pub =
  //     n.advertise<yolo_ros::DetectionMessages>("/untracked_info", 10);
  // ros::Subscriber imu_sub =
  //     n.subscribe("/camera/color/image_raw", 10, img_callback);
  yolo_results_pub_left = n.advertise<yolo_ros::DetectionMessages>("/untracked_info_left", 10);
  yolo_results_pub_right = n.advertise<yolo_ros::DetectionMessages>("/untracked_info_right", 10);

  ros::Subscriber img_left_sub = n.subscribe<sensor_msgs::Image>("/camera/infra1/image_rect_raw", 10, boost::bind(img_callback, _1, "left"));
  ros::Subscriber img_right_sub = n.subscribe<sensor_msgs::Image>("/camera/infra2/image_rect_raw", 10, boost::bind(img_callback, _1, "right"));

  cv::Mat frame;
  bool isAction = false;
  n.getParam("yolov5/action", isAction);
  if (isAction) {
    ROS_ERROR("no in service mode, please modify the .yaml file and reload");
    return 0;
  }


  client = n.serviceClient<yolo_ros::yolo>("yolo_service");
  client.waitForExistence(ros::Duration(30e-3));

  ros::spin();
  return 0;
}
