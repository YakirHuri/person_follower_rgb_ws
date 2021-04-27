/*
 * person_follower_rgb_node.cpp
 *
 *  Created on: April 26, 2021
 *      Author: yakir huri
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <object_msgs/ObjectInBox.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <object_msgs/ObjectsInBoxes.h>

#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class PersonFollowerRgb
{

public:
    PersonFollowerRgb()
    {

        initParams();

        rgbSubscriber_ = node_.subscribe("/openvino_toolkit/images", 1,
                                         &PersonFollowerRgb::imageCallback, this);

        objectSubscriber_ = node_.subscribe("openvino_toolkit/detected_objects", 1,
                                            &PersonFollowerRgb::detectedObjectsCallback, this);

        twistCommandPublisher_ 
            = node_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1, false);                                    
    } 

    ~PersonFollowerRgb() {}

    void initParams()
    {

        trackingTargetClassName_ = node_.param("target", string("person"));

        focalLength_ = node_.param("focal_length", 747.0); // (pixel_w * real_cm_distance) / real_w_cm
        knownWidthCm_ = node_.param("known_target_width_cm", 50.0);

        minDistance_ = node_.param("min_distance", 0.3);
        maxDistance_ = node_.param("max_distance", 1.5);

        minSpeed_ = node_.param("min_speed", 1.0);
        maxSpeed = node_.param("max_speed", 1.0);

        minAngleDegRotation_ = node_.param("min_degree_angle_rotation", 10.0);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            currentImg_ = cv_bridge::toCvShare(msg, "bgr8")->image;
            // cv::imwrite("/home/yakir/person_follower_rgb_ws/src/person_follower_rgb/imgs/"+to_string(count)+".jpg",currentImg_);
            // count++;
            // cv::imshow("view", currentImg_);
            // cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void detectedObjectsCallback(const object_msgs::ObjectsInBoxes::Ptr &objects)
    {
     
        if (!currentImg_.empty())
        {
            object_msgs::ObjectInBox nearestObjectInBox;
            double maxWidth = 0;
            bool foundTarget = false;
            for (auto &&obj : objects->objects_vector)
            {
                if (obj.object.object_name == trackingTargetClassName_)
                {
                   
                    foundTarget = true;

                    double width = obj.roi.width;
                    double height = obj.roi.height;
                    //finding the nearest target
                    if (width > maxWidth)
                    {
                        nearestObjectInBox = obj;
                    }
                }
            }
            if (foundTarget)
            {

                double distance = calculateDistanceFromObjectInMeter(nearestObjectInBox);
                double degAngle = calculateAngleFromObject(nearestObjectInBox);

                //showDebugImg(distance, degAngle, nearestObjectInBox);

                if( distance >= minDistance_ && distance <= maxDistance_ ){

                    sendVelCmd(distance, degAngle);                   
                }
            }
        }
    }

    void showDebugImg(double distance, double degAngle,
                      object_msgs::ObjectInBox nearestObjectInBox)
    {
       
        if (!currentImg_.empty())
        {   

            
            //Mat debugImg = currentImg_.clone();
            cv::Mat debugImg(currentImg_.rows, currentImg_.cols,CV_8UC3, cv::Scalar(0));
          
            cv::Point topLeft1(50,50);
           
            cv::Point topLeft2(50,100);

            circle(debugImg, cv::Point(debugImg.cols / 2, debugImg.rows / 2),
                3, Scalar(255,0,255),FILLED, 8,0);

          
            circle(debugImg, 
                cv::Point(nearestObjectInBox.roi.x_offset + (nearestObjectInBox.roi.width / 2),
                currentImg_.rows / 2),
                //nearestObjectInBox.roi.y_offset + (nearestObjectInBox.roi.height / 2 ) ),
                 3, Scalar(0,255,255),FILLED, 8,0);
            
                
            cv::Rect r(nearestObjectInBox.roi.x_offset,nearestObjectInBox.roi.y_offset,
                nearestObjectInBox.roi.width, nearestObjectInBox.roi.height);

            cv::rectangle(debugImg, r, cv::Scalar(0,255,0), 2);

            putText(debugImg, "distance(M): " + to_string(distance), topLeft1,
                    0.2, 1, Scalar(255,255,255), 3);
            putText(debugImg, "deg_angle: " + to_string(degAngle), topLeft2,
                    0.2, 1, Scalar(255,255,255), 3);

            cv::imshow("debugImg", debugImg);
            cv::waitKey(1);
        }
    }

    double calculateDistanceFromObjectInMeter(object_msgs::ObjectInBox object)
    {
        double distanceCm = (knownWidthCm_ * focalLength_) / object.roi.width; //in cm

        //convert to meters

        return (distanceCm / 100.0);
    }

    double calculateAngleFromObject(object_msgs::ObjectInBox object)
    {

        double image_width_pixels = currentImg_.cols; //CAMERA_HORIZONTAL_RESOLUTION_PIXELS;
        int x = object.roi.x_offset + (object.roi.width / 2);
        double angle_radians = atan((x - 0.5 *  currentImg_.cols) / focalLength_);

        return angles::to_degrees(angle_radians);
    }

    void sendVelCmd(double distance, double degAngle) {

        if( degAngle > fabs(minAngleDegRotation_)) {

            geometry_msgs::Twist command;
            command.linear.x = minSpeed_;
            command.angular.z = -(angles::from_degrees(degAngle));
            twistCommandPublisher_.publish(command);

        } else {
            geometry_msgs::Twist command;
            command.linear.x = minSpeed_;
            command.angular.z = 0;
            twistCommandPublisher_.publish(command);
        }
     
    }
   

private:
    ros::NodeHandle node_;

    ros::Subscriber rgbSubscriber_;

    ros::Subscriber objectSubscriber_;

    ros::Publisher twistCommandPublisher_;

    string trackingTargetClassName_;

    double focalLength_;

    double minDistance_;
    double maxDistance_;

    double minSpeed_;
    double maxSpeed;

    double minAngleDegRotation_;

    cv::Mat currentImg_;

    double knownWidthCm_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "person_follower_rgb_node");
    ros::NodeHandle node;
    PersonFollowerRgb follower;
    ros::spin();
    return 0;
}
