


#include <string>
#include <sstream>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}


#include <ros/ros.h>
#include <ros/time.h>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>



using namespace std;
using namespace cv;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  ros::Publisher aruco_point_pub_;

public:
    ImageConverter(string ID)
        : it_(nh_)
    {
         Marker_ID=atoi(ID.c_str());
        
        cout << "JEg hader mit liv" << endl;
        
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        
        
        // For using solvePNP on aruco Marker
//         marker_3D.push_back(Point3f(0,0,0));
//         marker_3D.push_back(Point3f(0.169,0,0));
//         marker_3D.push_back(Point3f(0.169,-0.169,0));
//         marker_3D.push_back(Point3f(0,-0.169,0));

        ROS_INFO("USING marker 2");
        marker_3D.push_back(Point3f(0,0,0));
        marker_3D.push_back(Point3f(0.066,0,0));
        marker_3D.push_back(Point3f(0.066,-0.066,0));
        marker_3D.push_back(Point3f(0,-0.066,0));
        aruco_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco/marker1", 1);
        
        distortion.push_back(-0.228385);
        distortion.push_back(0.266082);
        distortion.push_back(-0.001812);
        distortion.push_back(0.000035);
        distortion.push_back(0.000000);
        
        mark_dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
        
        camera_matrix = (Mat_<float>(3,3) <<    1348.715676, 0.000000, 722.486120,
                                                0.000000, 1347.386029, 495.012476,
                                                0.000000, 0.000000, 1.000000);
        
    }

    ~ImageConverter()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
    }
    
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        
        //cout << "Got image" << endl;

        aruco::detectMarkers(cv_ptr->image, mark_dict, markerCorners, markerIds);

//         if(markerCorners.size()==2) // ugly as fuck
//         {
        Mat rvec, tvec;
        
        for(int i = 0 ; i < markerCorners.size() ; i++)
        {
            if(Marker_ID == markerIds[i] && marker_3D_tvec.size() < 100)
            {
                solvePnP(marker_3D,markerCorners[i],camera_matrix,distortion,rvec,tvec);
                
//                 Point3f estPoint(tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0));
                Point3f estPoint(rvec.at<double>(0,0),rvec.at<double>(1,0),rvec.at<double>(2,0));
                cout << rvec.at<double>(0,0) << " , " << rvec.at<double>(1,0) << " , " << rvec.at<double>(2,0) <<endl;
                marker_3D_tvec.push_back(estPoint);
//                 geometry_msgs::PoseStamped aruco3D_msg;
//                 aruco3D_msg.header.stamp=msg->header.stamp;
//                 aruco3D_msg.pose.position.x=tvec.at<double>(0,0);
//                 aruco3D_msg.pose.position.y=tvec.at<double>(1,0);
//                 aruco3D_msg.pose.position.z=tvec.at<double>(2,0);
//                 aruco3D_msg.pose.orientation.x=rvec.at<double>(0,0);
//                 aruco3D_msg.pose.orientation.y=rvec.at<double>(1,0);
//                 aruco3D_msg.pose.orientation.z=rvec.at<double>(2,0);
                
//                 aruco_point_pub_.publish(aruco3D_msg);
                if( marker_3D_tvec.size() < 100 )
                {
                    cout << "Got measurement" << endl;
                }
            }
        }
        
        if(marker_3D_tvec.size() == 100)
        {
            calc();
        }
        
            

    }
    
    void calc()
    {
        // find mean
        
        Mat mean=Mat::zeros(3, 1, CV_32F);
        
        for(int i=0 ; i<marker_3D_tvec.size() ;i++)
        {
            mean.at<float>(0,0)=mean.at<float>(0,0)+marker_3D_tvec[i].x;
            mean.at<float>(1,0)=mean.at<float>(1,0)+marker_3D_tvec[i].y;
            mean.at<float>(2,0)=mean.at<float>(2,0)+marker_3D_tvec[i].z;
        }
        mean=mean/marker_3D_tvec.size();
        
        
        // find error and then covariance
        Mat error;
        
        error=Mat::zeros(marker_3D_tvec.size(), 3, CV_32F);
        for(int i=0 ; i<marker_3D_tvec.size() ;i++)
        {
            error.at<float>(i,0)=marker_3D_tvec[i].x-mean.at<float>(0,0);
            error.at<float>(i,1)=marker_3D_tvec[i].y-mean.at<float>(1,0);
            error.at<float>(i,2)=marker_3D_tvec[i].z-mean.at<float>(2,0);
        }
        cout << "Errors demeaned:" << endl << error << endl;
        Mat Sigma=Mat::zeros(marker_3D_tvec.size(), 2, CV_64F);
        Sigma=(1.0/((float)(marker_3D_tvec.size())-1.0))*error.t()*error;
        
        cout << "covariance: " <<Sigma << endl;
        
        cout << "Mean: " << mean << endl;
        
    }

    
private:
    cv::Mat current_image;
    vector<Point3f> marker_3D,marker_3D_tvec;
    vector<double> distortion;
    Ptr<aruco::Dictionary> mark_dict;
    Mat_<float> camera_matrix;

    vector< vector<Point2f> > markerCorners; // So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
    vector<int> markerIds;

    int count=0;
    int Marker_ID=2;
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic("2");
    ROS_INFO("Starting camera node up");
    ros::spin();
    return 0;
}

