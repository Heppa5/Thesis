


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
  image_transport::Publisher image_pub2_;
  
  ros::Publisher aruco_point_pub2_;
  ros::Publisher aruco_point_pub3_;

public:
    ImageConverter(string ID)
        : it_(nh_)
    {
        Marker_ID=atoi(ID.c_str());
        
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        image_pub2_ = it_.advertise("/image_converter/output_video2", 1);
        
        // For using solvePNP on aruco Marker
//         marker_3D.push_back(Point3f(0,0,0));
//         marker_3D.push_back(Point3f(0.169,0,0));
//         marker_3D.push_back(Point3f(0.169,-0.169,0));
//         marker_3D.push_back(Point3f(0,-0.169,0));
        
//         marker2_3D.push_back(Point3f(0,0,0));
//         marker2_3D.push_back(Point3f(0.044,0,0));
//         marker2_3D.push_back(Point3f(0.044,-0.044,0));
//         marker2_3D.push_back(Point3f(0,-0.044,0));
        marker3_3D.push_back(Point3f(0,0,0));
        marker3_3D.push_back(Point3f(0.066,0,0));
        marker3_3D.push_back(Point3f(0.066,-0.066,0));
        marker3_3D.push_back(Point3f(0,-0.066,0));
        aruco_point_pub2_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco/marker1", 1);
        aruco_point_pub3_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco/marker2", 1);
//         if (Marker_ID == 2)
//         {
//             ROS_INFO("USING marker 2");
// //             marker2_3D.push_back(Point3f(0,0,0));
// //             marker2_3D.push_back(Point3f(0.044,0,0));
// //             marker2_3D.push_back(Point3f(0.044,-0.044,0));
// //             marker2_3D.push_back(Point3f(0,-0.044,0));
//             aruco_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco/marker1", 1);
//             print_pixel_values=true;;
//         }
//         else if(Marker_ID == 3)
//         {
//             ROS_INFO("USING marker 3");
// //             marker3_3D.push_back(Point3f(0,0,0));
// //             marker3_3D.push_back(Point3f(0.066,0,0));
// //             marker3_3D.push_back(Point3f(0.066,-0.066,0));
// //             marker3_3D.push_back(Point3f(0,-0.066,0));
//             aruco_point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco/marker2", 1);
//             print_pixel_values=false;
//         }
        distortion.push_back(-0.228385);
        distortion.push_back(0.266082);
        distortion.push_back(-0.001812);
        distortion.push_back(0.000035);
        distortion.push_back(0.000000);
        
        print_pixel_values=true;
        
        distortion_mat=(Mat_<float>(5,1) <<    -0.228385,0.266082,-0.001812,0.000035,0.000000);
        mark_dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
        
        camera_matrix = (Mat_<float>(3,3) <<    1348.715676, 0.000000, 722.486120,
                                                0.000000, 1347.386029, 495.012476,
                                                0.000000, 0.000000, 1.000000);
        
//         filtering_constant=3.14/6.0;
        filtering_constant=3.14/4.0;
        
        Mat rvec_marker2,rvec_marker3;
        
        
    }

    ~ImageConverter()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
    }
    
    Mat mean(vector<Mat> data)
    {
        Mat mean=Mat::zeros(3,1,CV_64F);
        for (int i = 0; i<data.size() ; i++)
        {
            mean=mean+data[i];
        }
        mean=mean/data.size();
        return mean;
    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        vector<double> empty_array;
        Mat empty_mat;
        Mat image;
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
        
//         cout << "undistort" << endl;
//         image = cv_ptr->image;
        cv::undistort(cv_ptr->image,image, camera_matrix,distortion_mat,camera_matrix_updated);
//         cout << "wow" << endl;
//         cout << camera_matrix << "\t" << camera_matrix_updated << endl;
        aruco::detectMarkers(image, mark_dict, markerCorners, markerIds);
        aruco::drawDetectedMarkers(image, markerCorners, markerIds);
        
        Mat rvec, tvec;
        vector<Mat> rvecs,tvecs;
        
        
        
        for(int i = 0 ; i < markerCorners.size() ; i++)
        {
            if(2 == markerIds[i])
            {
                solvePnP(marker3_3D,markerCorners[i],camera_matrix,empty_array,rvec,tvec);
                
//                 solvePnP(marker3_3D,markerCorners[i],camera_matrix,empty_array,rvec,tvec);
                if(initialized_marker2==true)
                {
                    if(rvec_marker2.rows==3) // initialized or not
                    {
                        if(norm(rvec_marker2-rvec)<filtering_constant)
                        {
//                             aruco::drawAxis(image, camera_matrix, distortion_mat, rvec, tvec,marker2_size * 0.5f);               
                            aruco::drawAxis(image, camera_matrix, empty_mat, rvec, tvec,marker2_size * 0.5f);
                            geometry_msgs::PoseStamped aruco3D_msg;
                            aruco3D_msg.header.stamp=msg->header.stamp;
                            aruco3D_msg.pose.position.x=tvec.at<double>(0,0);
                            aruco3D_msg.pose.position.y=tvec.at<double>(1,0);
                            aruco3D_msg.pose.position.z=tvec.at<double>(2,0);
                            aruco3D_msg.pose.orientation.x=rvec.at<double>(0,0);
                            aruco3D_msg.pose.orientation.y=rvec.at<double>(1,0);
                            aruco3D_msg.pose.orientation.z=rvec.at<double>(2,0);
                            aruco_point_pub2_.publish(aruco3D_msg);
                        }
                        else
                        {
//                             aruco::drawAxis(image, camera_matrix, distortion_mat, rvec, tvec,marker2_size * 1.0f);
                            aruco::drawAxis(image, camera_matrix, empty_mat, rvec, tvec,marker2_size * 1.0f);
                        
                        }
                    }
                    else
                    {
                        rvec_marker2=rvec;
                    }
                }
                else
                {
                    if(initialize_marker2.size()>20)
                    {
                        rvec_marker2=select_rotation(&initialize_marker2);
                        initialized_marker2=true;
                    }
                    else
                    {
//                         sort_vector(&initialize_marker2,rvec.clone());
                        initialize_marker2.push_back(rvec.clone());
                    }
                }
                
            }
            else if(3 == markerIds[i])
            {
                solvePnP(marker3_3D,markerCorners[i],camera_matrix,empty_array,rvec,tvec);
//                 solvePnP(marker3_3D,markerCorners[i],camera_matrix,empty_array,rvec,tvec);
                if(initialized_marker3==true)
                {
                    if(rvec_marker3.rows==3) // initialized or not
                    {
                        if(norm(rvec_marker3-rvec)<filtering_constant)
                        {
//                             aruco::drawAxis(image, camera_matrix, distortion_mat, rvec, tvec,marker3_size * 0.5f);
                            aruco::drawAxis(image, camera_matrix, empty_mat, rvec, tvec,marker3_size * 0.5f);
                            geometry_msgs::PoseStamped aruco3D_msg;
                            aruco3D_msg.header.stamp=msg->header.stamp;
                            aruco3D_msg.pose.position.x=tvec.at<double>(0,0);
                            aruco3D_msg.pose.position.y=tvec.at<double>(1,0);
                            aruco3D_msg.pose.position.z=tvec.at<double>(2,0);
                            aruco3D_msg.pose.orientation.x=rvec.at<double>(0,0);
                            aruco3D_msg.pose.orientation.y=rvec.at<double>(1,0);
                            aruco3D_msg.pose.orientation.z=rvec.at<double>(2,0);
                            aruco_point_pub3_.publish(aruco3D_msg);
                        }
                        else
                        {
//                             aruco::drawAxis(image, camera_matrix, distortion_mat, rvec, tvec,marker3_size * 1.0f);
                             aruco::drawAxis(image, camera_matrix, empty_mat, rvec, tvec,marker3_size * 1.0f);
                        }
                    }
                    else
                    {
                        rvec_marker3=rvec;
                    }
                }
                else{
                    if(initialize_marker3.size()>20)
                    {    
                        rvec_marker3=select_rotation(&initialize_marker3);
                        initialized_marker3=true;
                    }
                    else
                    {
                        initialize_marker3.push_back(rvec.clone());
//                         sort_vector(&initialize_marker3,rvec.clone());
                    }
                }
            }
        }
        
        if(print_pixel_values)
        {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            image_pub_.publish(msg);
//             msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
//             image_pub2_.publish(msg);
        }
        

    }
    
    Mat select_rotation(vector<Mat>* data)
    {
        double lowest_error=9999999;
        int index=0;
        
        double current_error=0;
        for(int i=0 ; i < data->size() ; i++)
        {
            for(int j=0 ; j < data->size() ; j++)
            {
                current_error=current_error+norm(data->at(j)-data->at(i));
            }
            if(current_error<lowest_error)
            {
                lowest_error=current_error;
                index=i;
            }
            current_error=0;
        }
        cout << lowest_error << endl;
        return data->at(index);
    }
    


    
private:
    cv::Mat current_image;
    vector<Point3f> marker3_3D, marker2_3D;
    vector<double> distortion;
    Ptr<aruco::Dictionary> mark_dict;
    Mat_<float> camera_matrix, camera_matrix_updated;

    vector< vector<Point2f> > markerCorners; // So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
    vector<int> markerIds;

    int count=0;
    int Marker_ID;
    bool print_pixel_values;
    
    double marker2_size=0.066, marker3_size=0.066;
    
    bool chose_a_rotation=false;
    int count_of_rotatations=0;
    
    vector<Mat> initialize_marker2,initialize_marker3;
    bool initialized_marker2=false,initialized_marker3=false;
    
    Mat distortion_mat;
    
    Mat rvec_marker2,rvec_marker3;
    
    float filtering_constant;
    
};

int main(int argc, char** argv)
{
    if (argv[1] == "2" ) 
        ros::init(argc, argv, "image_converter");
    else 
        ros::init(argc, argv, "image_converter2");
    ImageConverter ic(argv[1]);
  ROS_INFO("Starting camera node up");
  ros::spin();
  return 0;
}

