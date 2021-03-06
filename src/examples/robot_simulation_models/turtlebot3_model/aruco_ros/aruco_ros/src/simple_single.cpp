#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>
#include <aruco_msgs/CheckActionEffect.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <aruco_msgs/CheckActionEffect.h>

using namespace aruco;

class ArucoSimple
{
private:
  cv::Mat inImage;
  cv::Mat actionDetectInImage;
  ros::Time action_curr_stamp;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  vector<Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub; 
  ros::Publisher position_pub;
  ros::Publisher marker_pub; //rviz visualization marker
  ros::Publisher pixel_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;

  double marker_size;
  int marker_id;
  bool rotate_marker_axis_;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub, action_check_sub;

  tf::TransformListener _tfListener;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

public:
    bool action_effect_achieved;
    bool aruco_detected;

public:
  ArucoSimple(ros::NodeHandle& nh)
    : cam_info_received(false),
      nh("~"),
      it(nh)
  {
    std::string refinementMethod;
    nh.param<std::string>("corner_refinement", refinementMethod, "LINES");
    if ( refinementMethod == "SUBPIX" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
    else if ( refinementMethod == "HARRIS" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::HARRIS);
    else if ( refinementMethod == "NONE" )
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::NONE); 
    else      
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES); 

    //Print parameters of aruco marker detector:
    ROS_INFO_STREAM("Corner refinement method: " << mDetector.getCornerRefinementMethod());
    ROS_INFO_STREAM("Threshold method: " << mDetector.getThresholdMethod());
    double th1, th2;
    mDetector.getThresholdParams(th1, th2);
    ROS_INFO_STREAM("Threshold method: " << " th1: " << th1 << " th2: " << th2);
    float mins, maxs;
    mDetector.getMinMaxSize(mins, maxs);
    ROS_INFO_STREAM("Marker size min: " << mins << "  max: " << maxs);
    ROS_INFO_STREAM("Desired speed: " << mDetector.getDesiredSpeed());

    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    action_check_sub = it.subscribe("/image", 1, &ArucoSimple::action_check_image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
    pixel_pub = nh.advertise<geometry_msgs::PointStamped>("pixel", 10);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<int>("marker_id", marker_id, 300);
    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);
    nh.param<bool>("rotate_marker_axis", rotate_marker_axis_, true);

    ROS_ASSERT(camera_frame != "" && marker_frame != "");

    if ( reference_frame.empty() )
      reference_frame = camera_frame;

    ROS_INFO("Aruco node started with marker size of %f m and marker id to track: %d",
             marker_size, marker_id);
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
             reference_frame.c_str(), marker_frame.c_str());

    dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));

    action_effect_achieved = false;
    aruco_detected = false;
  }


  bool GoToActionEffectCheck(aruco_msgs::CheckActionEffect::Request &req, aruco_msgs::CheckActionEffect::Response &res)
  {
      std::cout << "Before: GoToActionEffectCheck response: " << req.aruco_id << std::endl;
      aruco_detection(actionDetectInImage, action_curr_stamp);
      std::cout << "After: GoToActionEffectCheck response: " << action_effect_achieved << std::endl;
      res.detection_result = action_effect_achieved;
      return true;
  }


  bool getTransform(const std::string& refFrame, const std::string& childFrame, tf::StampedTransform& transform)
  {
    std::string errMsg;
    if ( !_tfListener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5), ros::Duration(0.01), &errMsg))
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform( refFrame, childFrame, ros::Time(0), transform);  //get latest available
      }
      catch ( const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }
    }
    return true;
  }


  void aruco_detection(cv::Mat inImage, ros::Time curr_stamp)
  {
      action_effect_achieved = false;
      aruco_detected = false;
      static tf::TransformBroadcaster br;
      //detection results will go into "markers"
      markers.clear();        //Ok, let's detect
      mDetector.detect(inImage, markers, camParam, marker_size, false);
      //for each marker, draw info and its boundaries in the image
      for(size_t i=0; i<markers.size(); ++i)
      {
          // only publishing the selected marker
          if(markers[i].id == marker_id)
          {
              action_effect_achieved = true;
              aruco_detected = true;
              tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_);
              tf::StampedTransform cameraToReference;
              cameraToReference.setIdentity();

              if ( reference_frame != camera_frame )
              {
                  getTransform(reference_frame, camera_frame, cameraToReference);
              }

              transform =
                      static_cast<tf::Transform>(cameraToReference) * static_cast<tf::Transform>(rightToLeft) * transform;

              tf::StampedTransform stampedTransform(transform, curr_stamp, reference_frame, marker_frame);
              br.sendTransform(stampedTransform);
              geometry_msgs::PoseStamped poseMsg;
              tf::poseTFToMsg(transform, poseMsg.pose);
              poseMsg.header.frame_id = reference_frame;
              poseMsg.header.stamp = curr_stamp;
              pose_pub.publish(poseMsg);

              geometry_msgs::TransformStamped transformMsg;
              tf::transformStampedTFToMsg(stampedTransform, transformMsg);
              transform_pub.publish(transformMsg);

              geometry_msgs::Vector3Stamped positionMsg;
              positionMsg.header = transformMsg.header;
              positionMsg.vector = transformMsg.transform.translation;
              position_pub.publish(positionMsg);

              geometry_msgs::PointStamped pixelMsg;
              pixelMsg.header = transformMsg.header;
              pixelMsg.point.x = markers[i].getCenter().x;
              pixelMsg.point.y = markers[i].getCenter().y;
              pixelMsg.point.z = 0;
              pixel_pub.publish(pixelMsg);

              //Publish rviz marker representing the ArUco marker patch
              visualization_msgs::Marker visMarker;
              visMarker.header = transformMsg.header;
              visMarker.id = 1;
              visMarker.type   = visualization_msgs::Marker::CUBE;
              visMarker.action = visualization_msgs::Marker::ADD;
              visMarker.pose = poseMsg.pose;
              visMarker.scale.x = marker_size;
              visMarker.scale.y = 0.001;
              visMarker.scale.z = marker_size;
              visMarker.color.r = 1.0;
              visMarker.color.g = 0;
              visMarker.color.b = 0;
              visMarker.color.a = 1.0;
              visMarker.lifetime = ros::Duration(3.0);
              marker_pub.publish(visMarker);
          }
          // but drawing all the detected markers
          markers[i].draw(inImage,cv::Scalar(0,0,255),2);
      }
      //draw a 3d cube in each marker if there is 3d info
      if(camParam.isValid() && marker_size>0)
      {
          for(size_t i=0; i<markers.size(); ++i)
          {
              CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
      }
  }

  void action_check_image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        ros::Time curr_stamp = msg->header.stamp;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            actionDetectInImage = cv_ptr->image;
            action_curr_stamp = curr_stamp;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s in action_check_image_callback", e.what());
            return;
        }
    }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if ((image_pub.getNumSubscribers() == 0) &&
        (debug_pub.getNumSubscribers() == 0) &&
        (pose_pub.getNumSubscribers() == 0) &&
        (transform_pub.getNumSubscribers() == 0) &&
        (position_pub.getNumSubscribers() == 0) &&
        (marker_pub.getNumSubscribers() == 0) &&
        (pixel_pub.getNumSubscribers() == 0))
    {
      ROS_DEBUG("No subscribers, not looking for aruco markers");
      return;
    }
    static tf::TransformBroadcaster br;
    if(cam_info_received)
    {
      ros::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;
//        aruco_detected = false;
//        aruco_detection(inImage, curr_stamp);
        markers.clear();
        mDetector.detect(inImage, markers, camParam, marker_size, false);
        for (size_t i = 0; i < markers.size(); ++i)
        {
            if (markers[i].id == marker_id)
            {
                aruco_detected = true;
                tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_);
                tf::StampedTransform cameraToReference;
                cameraToReference.setIdentity();

                if ( reference_frame != camera_frame )
                {
                    getTransform(reference_frame,
                                 camera_frame,
                                 cameraToReference);
                }

                transform =
                        static_cast<tf::Transform>(cameraToReference)
                        * static_cast<tf::Transform>(rightToLeft)
                        * transform;

                tf::StampedTransform stampedTransform(transform, curr_stamp,
                                                      reference_frame, marker_frame);
                br.sendTransform(stampedTransform);
                geometry_msgs::PoseStamped poseMsg;
                tf::poseTFToMsg(transform, poseMsg.pose);
                poseMsg.header.frame_id = reference_frame;
                poseMsg.header.stamp = curr_stamp;
                pose_pub.publish(poseMsg);
                std::cout << "poseMsg = " << poseMsg << std::endl;

//                std::string actionserver = "/rosoclingo";
//                actionlib::SimpleActionClient<rosoclingo::ROSoClingoAction> rosoclingo_action_client_(actionserver, true);
//                rosoclingo_action_client_.waitForServer();
//                rosoclingo::ROSoClingoGoal asp_request_goal;
//                std::vector<std::string> object_names;
//                object_names.push_back("box");
//                asp_request_goal.request = "possibleOrientation(red,a1)";
//                asp_request_goal.information = object_names;
//                rosoclingo_action_client_.sendGoal(asp_request_goal);

                geometry_msgs::TransformStamped transformMsg;
                tf::transformStampedTFToMsg(stampedTransform, transformMsg);
                transform_pub.publish(transformMsg);

                geometry_msgs::Vector3Stamped positionMsg;
                positionMsg.header = transformMsg.header;
                positionMsg.vector = transformMsg.transform.translation;
                position_pub.publish(positionMsg);

                geometry_msgs::PointStamped pixelMsg;
                pixelMsg.header = transformMsg.header;
                pixelMsg.point.x = markers[i].getCenter().x;
                pixelMsg.point.y = markers[i].getCenter().y;
                pixelMsg.point.z = 0;
                pixel_pub.publish(pixelMsg);

                //Publish rviz marker representing the ArUco marker patch
                visualization_msgs::Marker visMarker;
                visMarker.header = transformMsg.header;
                visMarker.id = 1;
                visMarker.type   = visualization_msgs::Marker::CUBE;
                visMarker.action = visualization_msgs::Marker::ADD;
                visMarker.pose = poseMsg.pose;
                visMarker.scale.x = marker_size;
                visMarker.scale.y = 0.001;
                visMarker.scale.z = marker_size;
                visMarker.color.r = 1.0;
                visMarker.color.g = 0;
                visMarker.color.b = 0;
                visMarker.color.a = 1.0;
                visMarker.lifetime = ros::Duration(3.0);
                marker_pub.publish(visMarker);
            }
            markers[i].draw(inImage,cv::Scalar(0,0,255),2);
        }

        std::cout << "In aruco_single: topic result = " << aruco_detected << std::endl;

        if(camParam.isValid() && marker_size>0)
        {
            for(size_t i=0; i<markers.size(); ++i)
            {
                CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
            }
        }

        if(image_pub.getNumSubscribers() > 0)
        {
          //show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if(debug_pub.getNumSubscribers() > 0)
        {
          //show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3]/msg.P[0],
            -msg.P[7]/msg.P[5],
            0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();
  }


  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    mDetector.setThresholdParams(config.param1,config.param2);
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }
};


int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_simple");
  ros::NodeHandle nh("~");

  ArucoSimple node(nh);
  ros::ServiceServer action_effect_check_server = nh.advertiseService("action_effect_check", &ArucoSimple::GoToActionEffectCheck, &node);

  ros::spin();
  return 0;
}
