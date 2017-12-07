


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

struct cameraPoint {
  Mat point3D;
  ros::Time stamp;
} ;

struct matchedPoint {
  cameraPoint camera;
  geometry_msgs::PoseStamped robot;
} ;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  ros::Subscriber robot_sub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1,&ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        robot_sub_ = nh_.subscribe("/robot/moved", 1, &ImageConverter::robot_has_moved,this);
        
        // For using solvePNP on aruco Marker
//         marker_3D.push_back(Point3f(0,0,0));
//         marker_3D.push_back(Point3f(0.169,0,0));
//         marker_3D.push_back(Point3f(0.169,-0.169,0));
//         marker_3D.push_back(Point3f(0,-0.169,0));
        marker_3D.push_back(Point3f(0,0,0));
        marker_3D.push_back(Point3f(0.044,0,0));
        marker_3D.push_back(Point3f(0.044,-0.044,0));
        marker_3D.push_back(Point3f(0,-0.044,0));
        
        
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
    
    vector< vector<Point2f> > markerCorners; // So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
    vector<int> markerIds;
    ros::Time time1,time2,time3;
    int count=0;
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
        
        
        time1=ros::Time::now();
        aruco::detectMarkers(cv_ptr->image, mark_dict, markerCorners, markerIds);
        time2=ros::Time::now();
        ROS_INFO("detection is: %f", (time2-time1));
        cout << "detection is: "<<(time2-time1) << endl;
        if(!markerCorners.empty())
        {
            Mat rvec, tvec;
            time1=ros::Time::now();
            solvePnP(marker_3D,markerCorners[0],camera_matrix,distortion,rvec,tvec);
            time2=ros::Time::now();
//             ROS_INFO("solvePNP is: %f", (time2-time1));
            cout << "solvePNP is: "<<(time2-time1) << endl;
            cameraPoint current;
            current.stamp=msg->header.stamp;
            current.point3D=tvec.clone();
            camera_points.insert(camera_points.begin(),current);
            if(camera_points.size() > 7) // only one second of data is stored
            {
                camera_points.pop_back();
            }
            match_points();
        }
        else
        {
            std::string str = patch::to_string(count);
            imwrite("/home/rovi2/fun/lort"+str+".jpg", cv_ptr->image);
            count++;
        }
    }
    void robot_has_moved(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
//         ROS_INFO("We are brilliant");
//         cout << *msg << endl;
//         
//         ROS_INFO("We are brilliant2");
        robot_positions.insert(robot_positions.begin(),*msg);
        if(robot_positions.size() > 7) // only one second of data is stored
        {
            robot_positions.pop_back();
        }
        match_points();
    }
    
    void match_points()
    {
//         ROS_INFO("MATCHING POINTS: Number of robot points: %d  -  Camera points: %d ", robot_positions.size(), camera_points.size());
        for( int i = 0 ; i < robot_positions.size() ; i++)
        {
            for( int j = 0 ; j < camera_points.size() ; j++)
            {
                ros::Duration possible_match(0.0714); // 7 Hz * 2 = 14 HZ
                ros::Time low_bound = camera_points[j].stamp - possible_match;
                ros::Time high_bound = camera_points[j].stamp + possible_match;
                if(robot_positions[i].header.stamp>low_bound && robot_positions[i].header.stamp<high_bound)
                {
//                     cout << camera_points[j].stamp << " ~ " << robot_positions[i].header.stamp << endl;
//                     cout << robot_positions[i].pose << endl;
//                     cout << camera_points[j].point3D << endl;
//                     ROS_INFO("In a match");
                    matchedPoint current;
                    current.camera=camera_points[j];
                    current.robot=robot_positions[i];
                    matchedPoints.insert(matchedPoints.begin(),current);
                    
                    camera_points.erase(camera_points.begin()+j);
                    robot_positions.erase(robot_positions.begin()+i);
                    
                }
            }
        }
        if(matchedPoints.size()>0)
        {
//             ROS_INFO("GOING IN");
            choosePoints();
//             ROS_INFO("GOING OUT");
        }
    }
    
    void choosePoints()
    {
        // if no points has been choosen, then just take the first matched point
        if(chosenMatchedPoints.size() == 0)
        {
            chosenMatchedPoints.insert(chosenMatchedPoints.begin(),matchedPoints[0]);
            matchedPoints.erase(matchedPoints.begin());
        }
//         ROS_INFO("RUNNING CODE AGAIN %d", matchedPoints.size());
        // choosing points based on, whether the robot has moved position 
//         if(matchedPoints.size()>0)
//         {
            while(matchedPoints.size()>0)
            {
//                 ROS_INFO("Hej %f",dist_between_points(matchedPoints[0].robot.pose,chosenMatchedPoints[0].robot.pose));
//                 ROS_INFO("Wow wow wo");
                int i = matchedPoints.size()-1;
                if(compare_3D_points(matchedPoints[i].robot.pose,chosenMatchedPoints[0].robot.pose,0.05))
                {
                    ROS_INFO("Distance between chosen poinsts:  %f",dist_between_points(matchedPoints[0].robot.pose,chosenMatchedPoints[0].robot.pose));
                    
                    chosenMatchedPoints.insert(chosenMatchedPoints.begin(),matchedPoints[i]);
    //                 cout << "hej" << endl;
                    
                    matchedPoints.erase(matchedPoints.end());

                }
                else
                    matchedPoints.erase(matchedPoints.end());
            }
//         }
    }
    
    bool compare_3D_points(geometry_msgs::Pose new_position, geometry_msgs::Pose old_position, double expected_distance)
    {
        double old_dist=sqrt(pow(old_position.position.x,2)+pow(old_position.position.y,2)+pow(old_position.position.z,2));
        double new_dist=sqrt(pow(new_position.position.x,2)+pow(new_position.position.y,2)+pow(new_position.position.z,2));
        
        if(abs(new_dist-old_dist)>=expected_distance)
            return true;
        else 
            return false;
    }
    double dist_between_points(geometry_msgs::Pose new_position, geometry_msgs::Pose old_position)
    {
        double old_dist=sqrt(pow(old_position.position.x,2)+pow(old_position.position.y,2)+pow(old_position.position.z,2));
        double new_dist=sqrt(pow(new_position.position.x,2)+pow(new_position.position.y,2)+pow(new_position.position.z,2));
        return abs(new_dist-old_dist);
    }
    
private:
    cv::Mat current_image;
    vector<Point3f> marker_3D;
    vector<double> distortion;
    Ptr<aruco::Dictionary> mark_dict;
    Mat_<float> camera_matrix;
    vector<cameraPoint> camera_points;
    vector<geometry_msgs::PoseStamped> robot_positions;
    vector<matchedPoint> matchedPoints;
    vector<matchedPoint> chosenMatchedPoints;
    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ROS_INFO("Starting camera node up");
  ros::spin();
  return 0;
}

