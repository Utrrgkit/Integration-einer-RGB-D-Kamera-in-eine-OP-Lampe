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
    //std::vector<std::vector<cv::Vec2f>> corners;
    std::vector<cv::Point2d> corners; //this will be filled by the detected corners, row by row left to right
    cv::Size patternsize(3,4); //interior number of corners (columes, rows)
    //cv::Size patternsize(4,3); //interior number of corners (columes, rows)
    cv_bridge::CvImageConstPtr cv_ptrGray = cv_bridge::toCvShare(msgRGB, "mono8"); //source image

    //CALIB_CB_FAST_CHECK saves a lot of time on images
    //that do not contain any chessboard corners
    patternfound = cv::findChessboardCorners(cv_ptrGray->image, patternsize, corners, 0);

    //cout<<"corners:"<<corners<<endl;

    //camera parameters
    double coeff[]={631.4, 0, 303.5,
                    0, 631.4, 228.5,
                    0, 0, 1};
    //cv::Mat cameraMatrix=cv::Mat(3,3,CV_64F,coeff);
    cv::Mat cameraMatrix=cv::Mat(3,3,CV_64F,coeff);

    //distortion coefficients
    cv::Mat distCoeffs=cv::Mat::zeros(1,4,CV_64F);
    //cv::Mat distCoeffs=cv::Mat::zeros(1,4,CV_64F);

    //rotation translation matrix
    cv::Mat rvec, tvec;
    //std::vector<cv::Point3f> rvec, tvec;

    //calculata camera enternal parameter with chessboard
    bool success=false;

    //objectPoints of corners on chessboard in chessboard coordination, origin:ART markers
    /*double three[12][3]={{240,160,0},{240,80,0},{240,0,0},
                         {160,160,0},{160,80,0},{160,0,0},
                         {80,160,0},{80,80,0},{80,0,0},
                         {0,160,0},{0,80,0},{0,0,0}
                        };*/
    double three[12][3]={{0.279, 0.279, -0.271},{0.297, 0.254, -0.198},{0.313, 0.228, -0.125},

                             {0.211, 0.249, -0.270},{0.224, 0.221, -0.196},{0.237, 0.194, -0.124},

                             {0.136, 0.215, -0.269},{0.152, 0.189, -0.196},{0.165, 0.160, -0.122},

                             {0.068, 0.186, -0.269},{0.080, 0.156, -0.194},{0.090, 0.126, -0.121}

                            };
    /*    double three[12][3]={{0,160,0},{80,160,0},{160,160,0},{240,160,0},
                        {0,80,0},{80,80,0}, {160,80,0},{240,80,0},
                        {0,0,0},{80,0,0}, {160,0,0}, {240,0,0}};*/
    //double three[4][3]={{0,160,0},{240,160,0},{0,0,0},{240,0,0}};
    std::vector<cv::Point3d> objectPoints;

       // std::vector<std::vector<cv::Vec3f>> objectPoints;
    for(int i=0;i<12;i++)
    {
       objectPoints.push_back(Point3d(three[i][0]*1000,three[i][1]*1000,three[i][2]*1000));
    }

    //std::vector<std::vector<cv::Point3f> > objectPoints2;
    //objectPoints2.push_back(objectPoints);

  //cv::Mat op=cv::Mat(1,12,CV_32F);
    //const int sop[]={1,3};
  cv::Mat op=cv::Mat(12, 3,CV_64F);
  op=cv::Mat(objectPoints);


    //imagePoints of corners on chessboard
    cv::Mat inDim=cv::Mat(12,2,CV_64F);
    inDim=cv::Mat(corners);

    /*double alf=1;
    double bet=0;
    inDim.convertTo(inDim, CV_32FC1,alf,bet);*/

    int flags=CV_EPNP;
    bool useE=false;

    cv::Mat Rvec=cv::Mat(3,3, CV_64F);
    cv::Mat Rvec_inv=cv::Mat(3,3, CV_64F);

    //cv::Size size(-1, -1);
    //cv::Size imagesize(640, 480);
    //int califlag=CV_CALIB_USE_INTRINSIC_GUESS;

    if(patternfound)
    {
       //solvePnP,update translation vector tvec and rotation vector(RPY) rvec
       //from camera coordination to chessboard origin

       //potential error: op.checkVetor, makesure op and inDim have same dimension before use this function,
       //1*12 2 or 3 channels

        //postprocessing corners, no obvious improvement
        //cv::cornerSubPix(cv_ptrGray->image, inDim, patternsize,
                         //size, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

       //inDim.convertTo(inDim, CV_64FC2,alf,bet);

        success=cv::solvePnP(op, inDim, cameraMatrix,
            distCoeffs, rvec, tvec, useE, flags);


        //cv::calibrateCamera(objectPoints2, corners, imagesize, cameraMatrix,
                            //distCoeffs, rvec, tvec, califlag,cv::TermCriteria( TermCriteria::COUNT+TermCriteria::EPS, 30, DBL_EPSILON));


        //depth cale factor 1000
        tvec=tvec/1000;

     }

        //coloroptical->camera_link: t[0.015, -0.000, -0.000] r[0.129, -1.563, 1.441]
        //r tb9tcl: body9 to cameralink
        cv::Mat t_optical_link, rb9tcl, Rb9tcl;
        cv::Mat r_optical_link, tb9tcl;

        cv::Mat R_optical_link;

        tf::TransformListener listener2;

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

        //if(!sucesss&&calibrated)
        //{calibrated=false;}

        if(success)
            {

            /*calibrated=true;

            //listen  camera_color_optical_frame to camera_link

            try{

                    listener2.waitForTransform( "/camera_color_optical_frame", "/camera_link",
                                              ros::Time(0), ros::Duration(3.0));
                    listener2.lookupTransform("/camera_color_optical_frame", "/camera_link",
                                           ros::Time(0), trans_cof_cl);
             }
               catch (tf::TransformException ex)
            {
               ROS_ERROR("%s",ex.what());
                  ros::Duration(0.1).sleep();
            }


            tf_tcof_cl=trans_cof_cl.getOrigin();
            tf_Rcof_cl=trans_cof_cl.getBasis();



            for(int i=0;i<3;i++)
            {

                tcof_cl.at<double>(0,i)=tf_tcof_cl[i];
                //cout<<"tclcof="<<tf_tcof_cl[i]<<endl;
                for(int j=0;j<3;j++)
                {

                    //cout<<"Rclcof="<<tf_Rcof_cl[i][j]<<endl<<endl;
                    Rcof_cl.at<double>(i,j)=tf_Rcof_cl[i][j];
                }
            }

            cout<<"t:"<<tcof_cl<<endl;
            cout<<Rcof_cl<<endl<<endl;*/

            //change rvec tvec, represented in body9 system
            cv::Rodrigues(rvec, Rvec);
            Rvec_inv=Rvec.inv();
            //cv::Rodrigues(Rvec_inv,rvec);
            tvec=-1*Rvec_inv*tvec;


            tb9tcl=tvec+Rvec_inv*tcof_link;

            //left multiply?
            Rb9tcl=Rvec_inv*Rcof_link;


            }


            tf::Transform trans_body10_cl;//, trans_ccof_orig;

            static tf::TransformBroadcaster br1, br2;

            //Translation for camera
            /*tf::Vector3 body10_cl;
            tf::Matrix3x3 tf_Rb9tcl;


            tf::Quaternion q1,q2;

            if(success)
            {
              tf_Rb9tcl.setValue(Rb9tcl.at<double>(0,0),Rb9tcl.at<double>(0,1),Rb9tcl.at<double>(0,2),
                                 Rb9tcl.at<double>(1,0),Rb9tcl.at<double>(1,1),Rb9tcl.at<double>(1,2),
                                 Rb9tcl.at<double>(2,0),Rb9tcl.at<double>(2,1),Rb9tcl.at<double>(2,2));
              //broadcast tf: body9 to camera_link
              body10_cl.setValue(tb9tcl.at<double>(0,0),tb9tcl.at<double>(0,1),tb9tcl.at<double>(0,2));
              tf_Rb9tcl.getRotation(q1);

              trans_body10_cl.setOrigin(body10_cl);
              trans_body10_cl.setRotation(q1);

              //br1.sendTransform(tf::StampedTransform(trans_body10_cl, ros::Time(0),  "body10", "camera_link"));

            }*/


             //listen ART to body9

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


/*            tf3d.setValue(Rslam.at<double>(0,0),Rslam.at<double>(0,1),Rslam.at<double>(0,2),
                               Rslam.at<double>(1,0),Rslam.at<double>(1,1),Rslam.at<double>(1,2),
                               Rslam.at<double>(2,0),Rslam.at<double>(2,1),Rslam.at<double>(2,2));

            //broadcast tf: camera_link to origin
            origin.setValue(tslam.at<double>(0,0),tslam.at<double>(0,1),tslam.at<double>(0,2));
            tf3d.getRotation(translationStepQuat);

            transformCurrent.setOrigin(origin);
            transformCurrent.setRotation(translationStepQuat);

            br2.sendTransform(tf::StampedTransform(transformCurrent, ros::Time(0), "camera_link", "origin"));

*/







            //calculate Art to slam origin and add fixed frame

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


/*       //broadcast slamorigin to camera pose


           cv::Mat pose, tvec_slam, Rvec_slam;
           Rvec_slam=cv::Mat(3, 3, CV_64F);
           tvec_slam=cv::Mat(3, 1, CV_64F);

           pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
           if (pose.empty())
               return;


           double alpha=1;
           double beta=0;
           pose.convertTo(pose,CV_64F,alpha,beta);

           cout<<pose<<endl<<endl;


           for(int i=0;i<3;i++)
           {
               tvec_slam.at<double>(0,i)=pose.at<double>(i,3);
               for(int j=0;j<3;j++)
               {
                   Rvec_slam.at<double>(i,j)=pose.at<double>(i,j);
               }
           }


           //change t r to camera_color_optical_frame
           //Rvec_slam=Rvec_slam.inv();
           //tvec_slam=Rvec_slam*tvec_slam;


           //cout<<Rvec_slam<<endl;
           //cout<<tvec_slam<<endl<<endl;
           tf::Matrix3x3 tf_Rvec_slam;

           static tf::TransformBroadcaster br2;
           tf_Rvec_slam.setValue(Rvec_slam.at<double>(0,0),Rvec_slam.at<double>(0,1),Rvec_slam.at<double>(0,2),
                            Rvec_slam.at<double>(1,0),Rvec_slam.at<double>(1,1),Rvec_slam.at<double>(1,2),
                            Rvec_slam.at<double>(2,0),Rvec_slam.at<double>(2,1),Rvec_slam.at<double>(2,2));

           tf_Rvec_slam.getRotation(q2);
           origin.setValue(tvec_slam.at<double>(0,0),tvec_slam.at<double>(0,1),tvec_slam.at<double>(0,2));
           //q2.normalize();
           trans_ccof_orig.setOrigin(origin);
           //tf::TransformetOrigin(origin);
           trans_ccof_orig.setRotation(q2);
           br2.sendTransform(tf::StampedTransform(trans_ccof_orig, ros::Time(0), "pose", "SLAM_origin" ));*/

/*rvec.copyTo(r_optical_link);
tvec.copyTo(t_optical_link);
rvec.copyTo(rb9tcl);
tvec.copyTo(tb9tcl);

t_optical_link.at<double>(0,0)=0.015;
t_optical_link.at<double>(0,1)=0;
t_optical_link.at<double>(0,2)=0;

cout<<tvec<<endl;
cout<<t_optical_link<<endl<<endl;

//RPY
r_optical_link.at<double>(0,0)=0;
r_optical_link.at<double>(0,1)=-1.563;
r_optical_link.at<double>(0,2)=1.563;

cv::Rodrigues(r_optical_link, R_optical_link);*/

/*cv::Rodrigues(r_optical_link, R_optical_link);

cout<<"top:"<<t_optical_link<<endl;
cout<<"rop:"<<r_optical_link<<endl;
cout<<"Rop:"<<R_optical_link<<endl;



//reprensent every mat in body 10 cooredination
  cv::Rodrigues(rvec, Rvec);
  Rvec_inv=Rvec.inv();
  tvec=-1*Rvec_inv*tvec;


  cout<<"rvec:"<<rvec<<endl;
  cout<<"Rinv:"<<Rvec_inv<<endl;
  cout<<"tvec:"<<tvec<<endl;

  t_optical_link=Rvec_inv*t_optical_link;
  R_optical_link=Rvec_inv*R_optical_link;
  cv::Rodrigues(R_optical_link, r_optical_link);

  cv::Rodrigues(Rvec_inv, rvec);

  cout<<"top2:"<<t_optical_link<<endl;
  cout<<"rop2:"<<r_optical_link<<endl;
  cout<<"Rop2:"<<R_optical_link<<endl;


  //rvec tvec: coordination between boty10->coloroptical
  //now calculate body10->camera_link=body10->color_optical_frame+coloroptical->camera_link
  //coloroptical->camera_link is known: t_ r_
  tvec=tvec+t_optical_link;
  rvec=rvec+r_optical_link;

  cout<<tvec<<endl;
  cout<<"R:"<<rvec<<endl<<endl;*/

/*
            cv::Rodrigues(rvec, Rvec);
            Rvec_inv=Rvec.inv();


            //tvec=t_optical_link-tvec;
            //cout<<"t1"<<tvec<<endl<<endl;

            tvec=Rvec_inv*tvec;
            //rvec=r_optical_link-rvec;
            cv::Rodrigues(rvec, R);


            Rvec=Rvec_inv*Rvec;
            cv::Rodrigues(Rvec, rvec);
            cout<<"t2"<<tvec<<endl;
            cout<<"R:"<<rvec<<endl<<endl;
                    */

/*        double aux;
 * aux = tfqt[0];
        tfqt[0]=-tfqt[2];
        tfqt[2]=tfqt[1];
        tfqt[1]=aux;

         //rotate 270deg about x and 270deg about z to get ENU: x forward, y left, z up
         const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
                                                 0, 0, 1,
                                                -1, 0, 0);
         const tf::Matrix3x3 rotation(   0, 0, 1,
                                         -1, 0,0,
                                         0, -1, 0);

         tf::Vector3 translationForCamera = origin * rotation270degXZ;

         //Hamilton (Translation for world)
         tf::Quaternion quaternionForHamilton(tfqt[3], tfqt[0], tfqt[1], tfqt[2]);
         tf::Quaternion secondQuaternionForHamilton(tfqt[3], -tfqt[0], -tfqt[1], -tfqt[2]);
         tf::Quaternion translationHamilton(0, translationForCamera[0], translationForCamera[1], translationForCamera[2]);

         tf::Quaternion translationStepQuat;
         translationStepQuat = hamiltonProduct(hamiltonProduct(quaternionForHamilton, translationHamilton), secondQuaternionForHamilton);

         tf::Vector3 translation(translationStepQuat[1], translationStepQuat[2], translationStepQuat[3]);
*/
