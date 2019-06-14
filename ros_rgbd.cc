/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/TransformStamped.h"
#include<opencv2/core/core.hpp>
#include"../../../include/System.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include<vector>

#include<opencv2/opencv.hpp>



using namespace std;
using namespace tf;
using namespace cv;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

tf::Quaternion hamiltonProduct(tf::Quaternion a, tf::Quaternion b) {

        tf::Quaternion c;

                c[0] = (a[0]*b[0]) - (a[1]*b[1]) - (a[2]*b[2]) - (a[3]*b[3]);
                c[1] = (a[0]*b[1]) + (a[1]*b[0]) + (a[2]*b[3]) - (a[3]*b[2]);
                c[2] = (a[0]*b[2]) - (a[1]*b[3]) + (a[2]*b[0]) + (a[3]*b[1]);
                c[3] = (a[0]*b[3]) + (a[1]*b[2]) - (a[2]*b[1]) + (a[3]*b[0]);

        return c;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //detect chessboard
    bool patternfound;
    std::vector<cv::Point2d> corners; //this will be filled by the detected corners, row by row left to right
    cv::Size patternsize(3,4); //interior number of corners (columes, rows)
    cv_bridge::CvImageConstPtr cv_ptrGray = cv_bridge::toCvShare(msgRGB, "mono8"); //source image

    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners
    patternfound = cv::findChessboardCorners(cv_ptrGray->image, patternsize, corners, 0);

    //camera parameters
    double coeff[]={631.4, 0, 303.5,
                    0, 631.4, 228.5,
                    0, 0, 1};

    cv::Mat cameraMatrix=cv::Mat(3,3,CV_64F,coeff);

    //distortion coefficients
    cv::Mat distCoeffs=cv::Mat::zeros(1,4,CV_64F);

    //rotation translation matrix
    cv::Mat rvec, tvec;

    //calculata camera enternal parameter with chessboard
    bool success=false;

    //objectPoints of corners on chessboard in chessboard coordination, origin:ART markers
    double three[12][3]={{0.279, 0.279, -0.271},{0.297, 0.254, -0.198},{0.313, 0.228, -0.125},

                             {0.211, 0.249, -0.270},{0.224, 0.221, -0.196},{0.237, 0.194, -0.124},

                             {0.136, 0.215, -0.269},{0.152, 0.189, -0.196},{0.165, 0.160, -0.122},

                             {0.068, 0.186, -0.269},{0.080, 0.156, -0.194},{0.090, 0.126, -0.121}

                            };

    std::vector<cv::Point3d> objectPoints;

    for(int i=0;i<12;i++)
    {
       objectPoints.push_back(Point3d(three[i][0]*1000,three[i][1]*1000,three[i][2]*1000));
    }

  cv::Mat op=cv::Mat(12, 3,CV_64F);
  op=cv::Mat(objectPoints);

    //imagePoints of corners on chessboard
    cv::Mat inDim=cv::Mat(12,2,CV_64F);
    inDim=cv::Mat(corners);


    int flags=CV_EPNP;
    bool useE=false;

    cv::Mat Rvec=cv::Mat(3,3, CV_64F);
    cv::Mat Rvec_inv=cv::Mat(3,3, CV_64F);


    if(patternfound)
    {
       //solvePnP,update translation vector tvec and rotation vector(RPY) rvec
       //from camera coordination to chessboard origin

       //potential error: op.checkVetor, makesure op and inDim have same dimension before use this function,
       //1*12 2 or 3 channels

        success=cv::solvePnP(op, inDim, cameraMatrix,
            distCoeffs, rvec, tvec, useE, flags);

        //depth cale factor 1000
        tvec=tvec/1000;

     }

       //R: 3x3 rotation matrix
       //r: 3x1 rotation vector
       //t: 3x1 translation vector
       //b9tcl: chessboard to camera link
        cv::Mat t_optical_link, rb9tcl, Rb9tcl;
        cv::Mat r_optical_link, tb9tcl;
        cv::Mat R_optical_link;

        tf::TransformListener listener2;

        //cof_cl: color optical frame to camera link
        tf::StampedTransform trans_cof_cl;

        tf::Vector3 tf_tcof_cl;
        tf::Matrix3x3 tf_Rcof_cl;
        cv::Mat tcof_link=cv::Mat(3, 1, CV_64F);
        cv::Mat Rcof_link=cv::Mat(3, 3, CV_64F);

        tb9tcl=cv::Mat(3, 1, CV_64F);
        Rb9tcl=cv::Mat(3, 3, CV_64F);

        bool calibrated;
        tcof_link.at<double>(0,0)=0.015;
        tcof_link.at<double>(0,1)=0;
        tcof_link.at<double>(0,2)=0;

        Rcof_link.at<double>(0,0)=0; Rcof_link.at<double>(0,1)=-1; Rcof_link.at<double>(0,2)=0;
        Rcof_link.at<double>(1,0)=0; Rcof_link.at<double>(1,1)=0; Rcof_link.at<double>(1,2)=-1;
        Rcof_link.at<double>(2,0)=1; Rcof_link.at<double>(2,1)=0; Rcof_link.at<double>(2,2)=0;

        if(success)
            {

            //change rvec tvec, represented in chessboard system
            cv::Rodrigues(rvec, Rvec);
            Rvec_inv=Rvec.inv();
            tvec=-1*Rvec_inv*tvec;
            tb9tcl=tvec+Rvec_inv*tcof_link;
            Rb9tcl=Rvec_inv*Rcof_link;

            }


            tf::Transform trans_body10_cl;

            static tf::TransformBroadcaster br1, br2;

            //listen ART to chessboard/body9

            tf::TransformListener listener1;

            tf::StampedTransform trans_body9;

            tf::Vector3 tf_tb9;
            tf::Matrix3x3 tf_Rb9;
            cv::Mat tb9=cv::Mat(3,1, CV_64F);
            cv::Mat Rb9=cv::Mat(3, 3, CV_64F);

            if(success)
            {
            try{

                    listener1.waitForTransform( "/art", "/body10",
                                              ros::Time(0), ros::Duration(1.0));
                    listener1.lookupTransform("/art", "/body10",
                                           ros::Time(0), trans_body9);
             }
               catch (tf::TransformException ex)
            {
               ROS_ERROR("%s",ex.what());
                  ros::Duration(0.1).sleep();
            }


            tf_tb9=trans_body9.getOrigin();
            tf_Rb9=trans_body9.getBasis();

            for(int i=0;i<3;i++)
            {

                tb9.at<double>(0,i)=tf_tb9[i];
                for(int j=0;j<3;j++)
                {
                    Rb9.at<double>(i,j)=tf_Rb9[i][j];
                }
            }
           }


            //t r_slam: slam origin to current camera_color_optical_frame, read form:pose
            cv::Mat pose;

            pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
            if (pose.empty())
                return;

            tf::Matrix3x3 tf3d;
            tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
            pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
            pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

            tf::Quaternion tfqt;
            tf3d.getRotation(tfqt);
            double aux = tfqt[0];
                tfqt[0]=-tfqt[2];
                tfqt[2]=tfqt[1];
                tfqt[1]=aux;

            //Translation for camera
            tf::Vector3 origin;
            origin.setValue(pose.at<float>(0,3),pose.at<float>(1,3),pose.at<float>(2,3));
            //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
            const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                                    0, 0, 1,
                                                    -1, 0, 0);

            tf::Vector3 translationForCamera = origin * rotation270degXZ;

            //Hamilton (Translation for world)
            tf::Quaternion quaternionForHamilton(tfqt[3], tfqt[0], tfqt[1], tfqt[2]);
            tf::Quaternion secondQuaternionForHamilton(tfqt[3], -tfqt[0], -tfqt[1], -tfqt[2]);
            tf::Quaternion translationHamilton(0, translationForCamera[0], translationForCamera[1], translationForCamera[2]);

            tf::Quaternion translationStepQuat;
            translationStepQuat = hamiltonProduct(hamiltonProduct(quaternionForHamilton, translationHamilton), secondQuaternionForHamilton);

            tf::Vector3 translation(translationStepQuat[1], translationStepQuat[2], translationStepQuat[3]);


            //Creates transform and populates it with translation and quaternion
            tf::Transform transformCurrent;
            transformCurrent.setOrigin(translation);
            transformCurrent.setRotation(tfqt);
            br2.sendTransform(tf::StampedTransform(transformCurrent, ros::Time(0), "slam_origin", "camera_link"));

            // transform coordinate from quaternion form to rotation matrix form
            tf::Vector3 tf_tslam;
            tf::Matrix3x3 tf_Rslam;
            cv::Mat tslam=cv::Mat(3, 1, CV_64F);
            cv::Mat Rslam=cv::Mat(3, 3, CV_64F);

            tf_tslam=transformCurrent.getOrigin();
            tf_Rslam=transformCurrent.getBasis();

            for(int i=0;i<3;i++)
            {

                tslam.at<double>(0,i)=tf_tslam[i];
                for(int j=0;j<3;j++)
                {

                    Rslam.at<double>(i,j)=tf_Rslam[i][j];
                }
            }

            Rslam=Rslam.inv();
            tslam=-1*Rslam*tslam;


            //calculate Art to slam origin and add fixed frame
            // art_origin: Art to slam origin
            cv::Mat t_art_origin=cv::Mat(3, 1, CV_64F);
            cv::Mat R_art_origin=cv::Mat(3, 3, CV_64F);

            tf::TransformBroadcaster br3;
            tf::Transform transform_ART_origin;

            tf::Vector3 tf_t_art_origin;
            tf::Matrix3x3 tf_R_art_origin;

            tf::Quaternion q3;

          if(success)
          {
                R_art_origin=Rb9*Rb9tcl*Rslam;
                t_art_origin=tb9+Rb9*tb9tcl+Rb9*Rb9tcl*tslam;

                tf_R_art_origin.setValue(R_art_origin.at<double>(0,0),R_art_origin.at<double>(0,1),R_art_origin.at<double>(0,2),
                               R_art_origin.at<double>(1,0),R_art_origin.at<double>(1,1),R_art_origin.at<double>(1,2),
                               R_art_origin.at<double>(2,0),R_art_origin.at<double>(2,1),R_art_origin.at<double>(2,2));
                tf_t_art_origin.setValue(t_art_origin.at<double>(0,0),t_art_origin.at<double>(0,1),t_art_origin.at<double>(0,2));
                tf_R_art_origin.getRotation(q3);
                q3.normalize();
                transform_ART_origin.setOrigin(tf_t_art_origin);
                transform_ART_origin.setRotation(q3);
                br3.sendTransform(tf::StampedTransform(transform_ART_origin, ros::Time(0), "art", "slam_origin"));

          }


}



