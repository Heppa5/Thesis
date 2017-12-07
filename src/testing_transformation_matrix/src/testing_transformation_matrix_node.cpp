


#include <string>
#include <sstream>

#include <tf/transform_broadcaster.h>

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
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose.h"
// #include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <match_points/matched_points.h>
#include "mp_mini_picker/moveToPoint.h"
#include "mp_mini_picker/moveToPose.h"
#include "mp_mini_picker/moveToQ.h"
#include "mp_mini_picker/currentQ.h"


#include <stdlib.h>
#include <stdio.h>
#include <time.h>

using namespace std;
using namespace cv;


// struct matchedPoint {
//   geometry_msgs::PoseStamped camera;
//   geometry_msgs::PoseStamped robot;
// } ;

class TestClass
{
    ros::NodeHandle nh_;
    ros::Subscriber matched_points_sub_;
    ros::Subscriber transformation_sub_;

    ros::Subscriber marker2_sub_;
    ros::Subscriber marker1_sub_;

    ros::Subscriber robot_position_sub_;

    ros::Publisher difference_pub_;

    ros::Publisher marker_dist_pub_;

    ros::ServiceClient serv_move_robot_point_;

    ros::ServiceClient serv_move_robot_pose_;
    
    ros::ServiceClient serv_move_robotQ_;

    ros::ServiceClient serv_get_robotQ_;

    bool debug=false;
public:
    TestClass()
    {
        
        // Subscribe to different topics
        transformation_sub_ = nh_.subscribe("/matched_points/transformation_matrix", 1,&TestClass::update_tranformation,this);
        matched_points_sub_ = nh_.subscribe("/matched_points/all_matched_points", 1, &TestClass::newest_point, this);
        robot_position_sub_ = nh_.subscribe("/robot/moved", 1, &TestClass::update_robot_position, this);
        marker2_sub_ = nh_.subscribe("/kalman_filter/marker2",1,&TestClass::update_marker2_position, this);
        marker1_sub_ = nh_.subscribe("/kalman_filter/marker1",1,&TestClass::update_marker1_position, this);
        
        
        
        // Publishers
        difference_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/matched_points/difference", 1);
        marker_dist_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/test/euclidian_distance_markers", 1);
        
        // services
        serv_move_robot_point_ = nh_.serviceClient<mp_mini_picker::moveToPoint>("/robot/MoveToPoint");
        serv_move_robot_pose_ = nh_.serviceClient<mp_mini_picker::moveToPose>("/robot/MoveToPose");
        serv_move_robotQ_ = nh_.serviceClient<mp_mini_picker::moveToQ>("/robot/MoveToQ");
        serv_get_robotQ_ = nh_.serviceClient<mp_mini_picker::currentQ>("/robot/GetCurrentQ");
        
        
        // create our transformation matrix as homogenous 
        trans_mat = Mat::zeros(4,4,CV_32F);
        trans_mat.at<float>(3,3)=1;
        
        // Set our seed 
        srand(time(NULL));
        
        
// //         Set robot to starting position - old
//         srv_home.request.Q[0]=-0.227210823689596;
//         srv_home.request.Q[1]=-1.2569144407855433;
//         srv_home.request.Q[2]=-1.6682723204242151;
//         srv_home.request.Q[3]=-1.3121197859393519;
//         srv_home.request.Q[4]=2.0358359813690186;
//         srv_home.request.Q[5]=-0.44079381624330694;
        
//         //New home position
        srv_home.request.Q[0]=0.24905504286289215;
        srv_home.request.Q[1]=-1.2299278418170374;
        srv_home.request.Q[2]=-1.6500452200519007;
        srv_home.request.Q[3]=-1.8819416205035608;
        srv_home.request.Q[4]=1.6246130466461182;
        srv_home.request.Q[5]=-0.15994674364198858;
        
        
        ros::Time last_publish=ros::Time::now();
        ros::Duration time_between_publishing(5); // Start up time
        ros::Duration time_difference;
        bool command_send=false;
        bool continue_program=false;
        ROS_INFO("Test node moving robot in 5 seconds");
        while(continue_program == false)
        {
            ros::Time current_time=ros::Time::now();
            time_difference=current_time-last_publish;
            if(time_difference>=time_between_publishing)
            {
                last_publish=ros::Time::now();
                if(command_send==false)
                {
                    serv_move_robotQ_.call(srv_home);
                    if (srv_home.response.ok==1)
                    {
                        ROS_INFO("Moving to start position");
                        command_send=true;
                    }
                    else
                    {
                        ROS_INFO("Did not succeed in moving robot to start position");
                    }
                    ROS_INFO("Waiting another 5 seconds");
                }
                else
                {
                    continue_program=true;
                }
            }
            ros::spinOnce();
        }
        
        
        
        
        
        
    }

    ~TestClass()
    {
        //     cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void update_marker2_position(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        marker2_position=*msg;
        if(got_trans_mat==true)
        {
//             cout << trans_mat << endl;
            Mat cP2 = convert_geomsg_to_hommat(marker2_position);
            Mat wP2=trans_mat*cP2;
            Mat cR2_vec=(Mat_<float>(3,1) <<    marker2_position.pose.orientation.x, marker2_position.pose.orientation.y, marker2_position.pose.orientation.z);
            Mat cR2;
            Rodrigues(cR2_vec,cR2);
            Mat wR2=R_cam_to_world*cR2;
//             cout << wP2 << endl;
//             cout << wR2 << endl;
            sendTransformTf(wP2,wR2,"world","marker2_camera");
        }
    }
    
    void update_marker1_position(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        marker1_position=*msg;
        Mat cam_marker1 = convert_geomsg_to_mat(marker1_position);
        Mat cam_marker2 = convert_geomsg_to_mat(marker2_position);
        
        geometry_msgs::PoseStamped dist_msg;
        dist_msg.header.stamp=ros::Time::now();
        dist_msg.pose.position.x=norm(cam_marker2-cam_marker1);
        marker_dist_pub_.publish(dist_msg);
        
        if(got_trans_mat==true)
        {
//             cout << trans_mat << endl;
            Mat cP1 = convert_geomsg_to_hommat(marker1_position);
            Mat wP1=trans_mat*cP1;
            Mat cR1_vec=(Mat_<float>(3,1) <<    marker1_position.pose.orientation.x, marker1_position.pose.orientation.y, marker1_position.pose.orientation.z);
            Mat cR1;
            Rodrigues(cR1_vec,cR1);
            Mat wR1=R_cam_to_world*cR1;
//             cout << wP1 << endl;
//             cout << wR1 << endl;
            sendTransformTf(wP1,wR1,"world","marker1_camera");
        }
        
    }
    
    void update_robot_position(const geometry_msgs::PoseStamped::ConstPtr msg)
    {
        current_robot_position=*msg;
        sendTransformTf_PoseStamped(current_robot_position,"world","marker1_robot");
    }
    
    void newest_point(const match_points::matched_points::ConstPtr msg)
    {
        Mat cam = convert_geomsg_to_hommat((msg->camera));
        Mat robot = convert_geomsg_to_hommat((msg->robot));
        
        if(got_trans_mat==true)
        {
            
            Mat rob_cam = trans_mat*cam;
            Mat result = robot-rob_cam;
            
            geometry_msgs::PoseStamped msg;
            
            msg.pose.position.x=result.at<float>(0,0);
            msg.pose.position.y=result.at<float>(1,0);
            msg.pose.position.z=result.at<float>(2,0);
            
            difference_pub_.publish(msg);
            
        }
    }
    
    void update_tranformation(const geometry_msgs::Transform::ConstPtr msg)
    {
        if(debug)
            cout << "#######robot_position_sub_########Test node updating its transformation matrix #################### " << endl;
        
        trans_mat_copy=trans_mat.clone();
        
        Mat R;
        Mat R_vector=Mat::zeros(3,1,CV_32F);
        
        R_vector.at<float>(0,0)=msg->rotation.x;
        R_vector.at<float>(1,0)=msg->rotation.y;
        R_vector.at<float>(2,0)=msg->rotation.z;
        
        Rodrigues(R_vector,R);
        
        R_cam_to_world=R;
        
        R.copyTo(trans_mat(Rect(0,0,R.cols,R.rows)));
        trans_mat.at<float>(0,3)=msg->translation.x;
        trans_mat.at<float>(1,3)=msg->translation.y;
        trans_mat.at<float>(2,3)=msg->translation.z;
        
        //cout << "Transformation_matrix at test node:" << endl << trans_mat << endl;
        got_trans_mat=true;
        
        
//         Mat correct_estimate = (Mat_<float>(4,1) <<    0.229208,0.368607,0.84233, 1);
//         Mat cam_marker2 = convert_geomsg_to_hommat(marker2_position);
//         Mat rob_marker2 = trans_mat*cam_marker2;
//         
//         
//         Mat cam_marker1 = convert_geomsg_to_hommat(marker1_position);
//         Mat rob_marker1 = trans_mat*cam_marker1;
        

        if(debug)
        {
            cout << "Change in translation compared to last iteration: " << norm(trans_mat.col(3)-trans_mat_copy.col(3)) << " - current transformation matrix is: " << endl << trans_mat << endl;
        }
        
        if(trans_mat_copy.cols == 4 && trans_mat_copy.rows == 4 && trans_mat.cols == 4 && trans_mat.rows == 4 && move_robot_randomly == true && robot_initialized==true)
        {
            //ROS_INFO("Have a transformation_matrix");
            bool significant_difference=false;
            for(int i = 0 ; i<4 ; i++)
            {
                for(int j=0 ; j<4 ; j++)
                {
                    double comparison=abs((trans_mat.at<float>(i,j)-trans_mat_copy.at<float>(i,j))/trans_mat_copy.at<float>(i,j));
                    if(comparison>0.10)
                    {
//                         cout << i << "," << j << " : " << trans_mat.at<float>(i,j) << " , " << trans_mat_c)opy.at<float>(i,j) << endl;
                        significant_difference = true;
                    }
                }
            }
            if(significant_difference==true)
            {
                ROS_INFO("Got rejected due to too large change compared to last iteration");
            }
            else
            {
                ROS_INFO("Less than 10 percent change for each element compared to last iteration \n ");
                move_robot_randomly=false;
                system_initialized=true;
                                
                if (serv_move_robotQ_.call(srv_home))
                {
                    ROS_INFO("Moving to start position");
                }
                
                ROS_INFO("Waiting for it to move to start position");
                while(robot_at_home_position()==false)
                {
                    // wait
                    usleep(100000);
                    ros::spinOnce();
                }
                move_robot_to_marker2();
                
            }
            
            if(debug)
            {
                cout << "############### Test node finished updating #################### " << endl;
            }
        }
        
        
    }

    
    void move_robot_to_marker2()
    {
        bool use_pose=true;
        if(use_pose==false)
        {
            mp_mini_picker::moveToPoint srv;
                    
            Mat marker2_cam = convert_geomsg_to_hommat(marker2_position);
            Mat rob_marker2_cam = trans_mat*marker2_cam;
            
            srv.request.point[0]=(double)(rob_marker2_cam.at<float>(0,0)-0.05);
            srv.request.point[1]=(double)(rob_marker2_cam.at<float>(1,0)-0.05);
            srv.request.point[2]=(double)(rob_marker2_cam.at<float>(2,0)+0.1);
            cout << "Calculated position of Marker 2 in world frame: "  <<srv.request.point[0]  << " , " << srv.request.point[1]  << " , " << srv.request.point[2]  << endl;
            ROS_INFO("Now moving to marker 2");
            if (serv_move_robot_point_.call(srv))
            {
                ROS_INFO ("Succesful call to robot");
            }
            else
            {
                ROS_INFO("Houston we got a problem");
            }
            
            desired_robot_position.pose.position.x=srv.request.point[0];
            desired_robot_position.pose.position.y=srv.request.point[1];
            desired_robot_position.pose.position.z=srv.request.point[2];
        }
        else
        {
            mp_mini_picker::moveToPose srv;
            Mat wTc=trans_mat;
            Mat wRc=R_cam_to_world;
            
            Mat cP2 = convert_geomsg_to_hommat(marker2_position);
            Mat wP2 = wTc*cP2;
            
            wP2.at<float>(0,0)=wP2.at<float>(0,0)-0.05;
            wP2.at<float>(1,0)=wP2.at<float>(1,0)-0.05;
            wP2.at<float>(2,0)=wP2.at<float>(2,0)+0.1;
            
            srv.request.pose[0]=(double)(wP2.at<float>(0,0));
            srv.request.pose[1]=(double)(wP2.at<float>(1,0));
            srv.request.pose[2]=(double)(wP2.at<float>(2,0));
            
            Mat cR1_vec=(Mat_<float>(3,1) <<    marker1_position.pose.orientation.x, marker1_position.pose.orientation.y, marker1_position.pose.orientation.z);
            Mat cR2_vec= (Mat_<float>(3,1) <<    marker2_position.pose.orientation.x, marker2_position.pose.orientation.y, marker2_position.pose.orientation.z);
            
            Mat cR1,cR2;
            Rodrigues(cR1_vec,cR1);
            Rodrigues(cR2_vec,cR2);
            
            Mat wR1=wRc*cR1;
            Mat wR2=wRc*cR2;
            

            
            Mat _1Rw;
            transpose(wR1,_1Rw);
            
            Mat _1R2=_1Rw*wR2;
            Mat _1R2_vec;
            Rodrigues(_1R2,_1R2_vec);
            
            
            Mat wRt_vec= (Mat_<float>(3,1) <<    current_robot_position.pose.orientation.x, current_robot_position.pose.orientation.y, current_robot_position.pose.orientation.z);
            Mat wRt;
            Rodrigues(wRt_vec,wRt);
            
            // tool(t) is the as marker1
//             Mat _2R1;
//             transpose(_1R2,_2R1);
            Mat wR2_tool=wRt*_1R2;
//             Mat wR2_tool=wRc*cR1*_1R2;
//             Mat wR2_tool=wRt*_2R1;
            Mat wR2_vec;
            Rodrigues(wR2_tool,wR2_vec);
            
            srv.request.pose[3]=(double)(wR2_vec.at<float>(0,0));
            srv.request.pose[4]=(double)(wR2_vec.at<float>(1,0));
            srv.request.pose[5]=(double)(wR2_vec.at<float>(2,0));

            
            cout << "1R2: " << _1R2_vec.at<float>(0,0) << " , " << _1R2_vec.at<float>(1,0) << " , " << _1R2_vec.at<float>(2,0) << " \t" << norm(_1R2_vec)*180/M_PI << endl;
            
            cout << "Current rotation of robot: " << current_robot_position.pose.orientation.x << " , " << current_robot_position.pose.orientation.y << " , " << current_robot_position.pose.orientation.z << endl;
            cout << "Current rotation of marker1: " << wR1.at<float>(0,0)<< " , " <<  wR1.at<float>(1,0) << " , " <<  wR1.at<float>(2,0) << endl;
            
            
            cout << "Calculated position of Marker 2 in world frame: "  <<srv.request.pose[0]  << " , " << srv.request.pose[1]  << " , " << srv.request.pose[2]  << endl;
            cout << "Calculated rotation of tool in world frame: " << srv.request.pose[3]  << " , " << srv.request.pose[4]  << " , " << srv.request.pose[5]  << endl;
            
            
            
            ROS_INFO("Now moving to marker 2");
            if (serv_move_robot_pose_.call(srv))
            {
                ROS_INFO ("Succesful call to robot");
            }
            else
            {
                ROS_INFO("Houston we got a problem");
            }
            
            sendTransformTf(wP2,wR2_tool,"world","desired_location");
//             cout << "WORKED" << endl;
            
            desired_robot_position.pose.position.x=srv.request.pose[0];
            desired_robot_position.pose.position.y=srv.request.pose[1];
            desired_robot_position.pose.position.z=srv.request.pose[2];
            desired_robot_position.pose.orientation.x=srv.request.pose[3];
            desired_robot_position.pose.orientation.y=srv.request.pose[4];
            desired_robot_position.pose.orientation.z=srv.request.pose[5];
        }
    }
    
    
    bool robot_at_home_position()
    {
        mp_mini_picker::currentQ srv;
        srv.request.getQ=1;
        if(serv_get_robotQ_.call(srv))
        {
            bool robot_at_position=true;
            for(int i=0; i<6 ; i++)
            {
                if(srv.response.Q[i] >= (srv_home.request.Q[i]-0.009) && srv.response.Q[i] <= (srv_home.request.Q[i]+0.009))
                {
                    // nothing
                }
                else
                {
                    robot_at_position=false;
                }
            }
            if(robot_at_position==true)
            {
                robot_initialized=true;
                return true;
            }
            else
                return false;
        }
        else 
            return false;
    }
    
    
    Mat convert_geomsg_to_mat(geometry_msgs::PoseStamped msg)
    {
        Mat point=Mat::zeros(3,1,CV_32F);
        point.at<float>(0,0)=(float)msg.pose.position.x;
        point.at<float>(1,0)=(float)msg.pose.position.y;
        point.at<float>(2,0)=(float)msg.pose.position.z;
        
        return point;
    }
    
    Mat convert_geomsg_to_hommat(geometry_msgs::PoseStamped msg)
    {
        Mat point=Mat::zeros(4,1,CV_32F);
        point.at<float>(0,0)=(float)msg.pose.position.x;
        point.at<float>(1,0)=(float)msg.pose.position.y;
        point.at<float>(2,0)=(float)msg.pose.position.z;
        point.at<float>(3,0)=1;
        
        return point;
    }
    
    bool robot_at_asked_position()
    {
        // compare_3D_points checks if the distance is greater than expected_distance. If yes then return true. Since we want these two variables to be close to eachother, then we invert the result. 
        if(compare_3D_points(desired_robot_position.pose,current_robot_position.pose,0.005) == true)
            return false;
        else
            return true;
    }
    long int counter=0;
    void move_robot_random_direction()
    {
        
             
        srand(time(&counter));
        counter++;
        if(move_robot_randomly==true)
        {
             
            mp_mini_picker::moveToPoint srv=generate_random_point(6.5);
            
            
        
//             if (serv_move_robot_point_.call(srv));
            bool got_valid_point=false;
            while(got_valid_point==false)
            {
//                 cout <<"Jeg laver nyt kald (y)";
                srv=generate_random_point(6.5);
                if (serv_move_robot_point_.call(srv))
                {
//                     cout << "Jeg blev accepteret" << endl;
                    got_valid_point=true;
                }
            }
            
//             cout <<" requested point: " << srv.request.point[0] << " , " << srv.request.point[1] << " , " << srv.request.point[2] << " , " << endl;
//             
//             cout <<" current position: " << current_robot_position.pose.position.x << " , " << current_robot_position.pose.position.y << " , " << current_robot_position.pose.position.z << " , " << endl;
            
            
            desired_robot_position.pose.position.x=srv.request.point[0];
            desired_robot_position.pose.position.y=srv.request.point[1];
            desired_robot_position.pose.position.z=srv.request.point[2];
//                 cout << "Succesful call" << endl;
            
        }
    }
    
    mp_mini_picker::moveToPoint generate_random_point(double distance)
    {
        int x_dir=rand()%100+1;
        int y_dir=rand()%100+1;
        int z_dir=rand()%100+1;
        
        double reduc_fac=sqrt(pow(x_dir,2)+pow(y_dir,2)+pow(z_dir,2));        
        double cor_x_dir = distance*((double)x_dir)/reduc_fac/100;
        double cor_y_dir = distance*((double)y_dir)/reduc_fac/100;
        double cor_z_dir = distance*((double)z_dir)/reduc_fac/100;
        
        double cor_reduc_fac=sqrt(pow(cor_x_dir,2)+pow(cor_y_dir,2)+pow(cor_z_dir,2));
        
        mp_mini_picker::moveToPoint srv;
        if (rand()%2==1)
            srv.request.point[0]=current_robot_position.pose.position.x+cor_x_dir;
        else
            srv.request.point[0]=current_robot_position.pose.position.x-cor_x_dir;
        if (rand()%2==1)
            srv.request.point[1]=current_robot_position.pose.position.y+cor_y_dir;
        else
            srv.request.point[1]=current_robot_position.pose.position.y-cor_y_dir;
        if (rand()%2==1)
            srv.request.point[2]=current_robot_position.pose.position.z+cor_z_dir;
        else
            srv.request.point[2]=current_robot_position.pose.position.z-cor_z_dir;
        
        return srv;
    }
    
    bool program_finished()
    {
        return !(move_robot_randomly);
    }
    
    bool is_system_initialized()
    {
        return system_initialized;
    }
    
    void sendTransformTf_PoseStamped(geometry_msgs::PoseStamped pose, string to_frame, string from_frame)
    {
        Mat Rvec=(Mat_<float>(3,1) << pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z);
        Mat R;
        Rodrigues(Rvec,R);
        
        
        tf::Matrix3x3 rot;
        rot.setValue(R.at<float>(0,0),R.at<float>(0,1),R.at<float>(0,2),
                     R.at<float>(1,0),R.at<float>(1,1),R.at<float>(1,2),
                     R.at<float>(2,0),R.at<float>(2,1),R.at<float>(2,2));
        tf::Vector3 t;
        t.setValue(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
        tf::Transform wTc(rot,t);
        
        tf::StampedTransform msg(wTc,ros::Time::now(),to_frame,from_frame);
        static tf::TransformBroadcaster br;
        br.sendTransform(msg);
    }
    
    void sendTransformTf(Mat translation, Mat rotation, string to_frame, string from_frame)
    {
        tf::Matrix3x3 rot;
        rot.setValue(rotation.at<float>(0,0),rotation.at<float>(0,1),rotation.at<float>(0,2),
                     rotation.at<float>(1,0),rotation.at<float>(1,1),rotation.at<float>(1,2),
                     rotation.at<float>(2,0),rotation.at<float>(2,1),rotation.at<float>(2,2));
        tf::Vector3 t;
        t.setValue(translation.at<float>(0,0),translation.at<float>(1,0),translation.at<float>(2,0));
        tf::Transform wTc(rot,t);
        
        tf::StampedTransform msg(wTc,ros::Time::now(),to_frame,from_frame);
        static tf::TransformBroadcaster br;
        br.sendTransform(msg);
    }
    
private:
    
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
    bool got_trans_mat=false,robot_initialized=false, system_initialized=false;

    Mat trans_mat,trans_mat_copy,R_cam_to_world;
    
    bool move_robot_randomly=true;
    
    geometry_msgs::PoseStamped current_robot_position,desired_robot_position,marker2_position,marker1_position;
    

    mp_mini_picker::moveToQ srv_home;
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test matched points");
    TestClass ic;
    ROS_INFO("Starting test node up");
    
//     ros::Time last_publish=ros::Time::now();
//     ros::Duration time_between_publishing(3); // camera has framerate of 7 hz
//     ros::Duration time_difference;
//     while(true)
//     {
//         ros::Time current_time=ros::Time::now();
//         time_difference=current_time-last_publish;
//         if(time_difference>=time_between_publishing)
//         {
//     //             ROS_INFO("I'm inside");
//             last_publish=ros::Time::now();
//              ic.move_robot_random_direction();
//         }
//         
//         ros::spinOnce();
//     }
    
    bool first_iteration=true;
    bool program_done=false;
    ros::Time last_publish=ros::Time::now();
    ros::Duration start_sequence(3);
    ros::Duration finished_program(1);
    
    ros::Duration time_difference;
    
    while(true)
    {
        if(ic.program_finished())
        {
            program_done=true;
        }
        if(first_iteration==true)
        {
            while(ic.robot_at_home_position()==false)
            {
                // wait
                usleep(100000);
                ros::spinOnce();
//                 cout << "Waiting in main loop" << endl;
            }
//             cout << "Sleeping" << endl;
            usleep(1000000); // half a seconds
//             cout << "Done Sleeping" << endl;
            ic.move_robot_random_direction();
            first_iteration=false;
        }
        
        ros::Time current_time=ros::Time::now();
        time_difference=current_time-last_publish;
        if(time_difference>=start_sequence && program_done != true)
        {
            if(ic.robot_at_asked_position())
            {
                ic.move_robot_random_direction();
                last_publish=ros::Time::now();
            }
        }
        
        if(time_difference>=finished_program && program_done == true)
        {
            if(ic.robot_at_asked_position())
            {
                ic.move_robot_to_marker2();
                last_publish=ros::Time::now();
            }
        }
        
        ros::spinOnce();
        
    }

    return 0;
}

