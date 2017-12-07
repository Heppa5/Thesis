#include <ros/ros.h>
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "kalman_filter/turn.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <opencv2/aruco.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

#include <tf/transform_broadcaster.h>

using namespace cv;
using namespace std;

class kalman_filter_class {

    ros::NodeHandle nh_;
    ros::Subscriber measMark1_sub;
    ros::Publisher estMark1_pub;
  
    //cv::KalmanFilter KF;
    std::vector<Point3f> measurev, kalmanv;
    Mat_<float> meas;


    Mat_<float> F,F_rot;
    Mat_<float> F_trans,F_trans_rot;
    Mat_<float> x_cur,x_cur_rot;
    Mat_<float> x_last, x_last_rot;
    Mat_<float> P_cur, P_cur_rot;
    Mat_<float> P_last, P_last_rot;
    Mat_<float> Qk, Qk_rot;
    Mat_<float> Rk, Rk_rot;
    Mat_<float> H;
    Mat_<float> H_trans;
    float processNoise, processNoise_rot , measurementNoise, dt, v, a;
    
    bool eval,use_filter=true,marker1_updated,marker2_updated;
    geometry_msgs::PoseStamped marker1,marker2;
    
    ofstream file_out;
    
    unsigned long long number_of_observations=0;
    ros::Time hej2;

public:
    kalman_filter_class(string useF) {
        if(useF=="1")
        {
            ROS_INFO("Using filter");
            use_filter=true;
        }
        else
        {
            ROS_INFO("Not using filter");
            use_filter=false;
        }
        measMark1_sub = nh_.subscribe("/aruco/marker1", 100, &kalman_filter_class::update_marker1, this);
//         measMark2_sub = nh_.subscribe("/aruco/marker2", 100, &kalman_filter::measCb, this);
        
        estMark1_pub = nh_.advertise<geometry_msgs::PoseStamped>("/kalman_filter/marker1", 100);
//         estMark2_pub = nh_.advertise<geometry_msgs::PoseStamped>("/kalman_filter/marker2", 100);

        processNoise = .0005;
        processNoise_rot = .002; 
        measurementNoise = 5;

        dt = 1.0/7.0; // should have an update rate of 15 frames per second
        v = dt;
        a = .5*dt*dt;
        
        F = (Mat_<float>(9, 9) <<	1, 0, 0, v, 0, 0, a, 0, 0, 
                                        0, 1, 0, 0, v, 0, 0, a, 0, 
                                        0, 0, 1, 0, 0, v, 0, 0, a, 
                                        0, 0, 0, 1, 0, 0, v, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, v, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, v, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);
        F_rot = (Mat_<float>(9, 9) <<	1, 0, 0, v, 0, 0, a, 0, 0, 
                                        0, 1, 0, 0, v, 0, 0, a, 0, 
                                        0, 0, 1, 0, 0, v, 0, 0, a, 
                                        0, 0, 0, 1, 0, 0, v, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, v, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, v, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);


        P_last = (Mat_<float>(9, 9) <<	1, 0, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 1, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 1, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 1, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);
        P_last_rot = (Mat_<float>(9, 9) <<	1, 0, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 1, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 1, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 1, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);
        
        P_cur = (Mat_<float>(9, 9) <<	1, 0, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 1, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 1, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 1, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);
        
        P_cur_rot = (Mat_<float>(9, 9) <<1, 0, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 1, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 1, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 1, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);
        
        Qk = (Mat_<float>(9, 9) <<	1, 0, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 1, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 1, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 1, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);
        Qk_rot = (Mat_<float>(9, 9) <<	1, 0, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 1, 0, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 1, 0, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 1, 0, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 1, 0, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 1, 0, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                        0, 0, 0, 0, 0, 0, 0, 0, 1);

        
        Rk = (Mat_<float>(3, 3) <<      6.3884654e-05, -4.6466208e-05, 0.00028330999,
                                        -4.6466208e-05, 3.4083754e-05, -0.00020681616,
                                        0.00028330999, -0.00020681616, 0.0012589982);

        Rk_rot = (Mat_<float>(3, 3) <<  0.0024434947, 0.0011541061, 0.0045976816,
                                        0.0011541061, 0.00098306185, 0.00037265077,
                                        0.0045976816, 0.00037265077, 0.018606635);
        
        
        
        
        // Qk is the processNoise
        Qk=Qk*processNoise;
        Qk_rot=Qk_rot*processNoise_rot;
        
        // Rk is the noise for our senser
        //Rk=Rk*measurementNoise;
        x_cur= (Mat_<float>(9, 1) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
        x_last= (Mat_<float>(9, 1) << 0, 0, 2, 0, 0, 0, 0, 0, 0);
        
        
        x_cur_rot= (Mat_<float>(9, 1) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
        x_last_rot= (Mat_<float>(9, 1) << -2.0, -2.0, 0, 0, 0, 0, 0, 0, 0);
        
        //H = (Mat_<float>(9, 1) << 1, 1, 1, 0, 0, 0, 0, 0, 0);
        H = (Mat_<float>(3,9) <<    1, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 1, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0, 0, 0, 0);
                                    
        transpose(F,F_trans);
        transpose(F_rot,F_trans_rot);
        transpose(H,H_trans);
        
        
        meas = Mat_<float>(3,1);
        meas.setTo(Scalar(0));
        
        eval = false;
        
        if (eval) {
        	cout << "measx measy measz corx cory corz corx' cory' corz' corx'' cory'' corz'' predx predy predz predx' predy' predz' predx'' predy'' predz'' " << endl;
        }
    }
    
    void update_marker1(const geometry_msgs::PoseStamped& msg)
    {
        if(use_filter==true)
        {
            marker1=msg;
            marker1_updated=true;
            run_iteration();
        }
        else
        {
            marker1=msg;
            marker1_updated=true;
            estMark1_pub.publish(msg);
            sendTransformTf_PoseStamped(marker1,"camera","marker1_camera");
            
        }
        number_of_observations++;
    }
    
//     void update_marker2(const geometry_msgs::PoseStamped& msg)
//     {
//         marker2=*msg;
//         marker2_updated=true;
//     }
  
    void run_iteration() {      
        
            ros::Time hej=ros::Time::now();
            dt = (hej-hej2).toSec();
//             cout << dt << endl;
            v = dt;
            a = .5*dt*dt;
            
            F = (Mat_<float>(9, 9) <<	    1, 0, 0, v, 0, 0, a, 0, 0, 
                                            0, 1, 0, 0, v, 0, 0, a, 0, 
                                            0, 0, 1, 0, 0, v, 0, 0, a, 
                                            0, 0, 0, 1, 0, 0, v, 0, 0, 
                                            0, 0, 0, 0, 1, 0, 0, v, 0, 
                                            0, 0, 0, 0, 0, 1, 0, 0, v, 
                                            0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                            0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                            0, 0, 0, 0, 0, 0, 0, 0, 1);
            F_rot = (Mat_<float>(9, 9) <<   1, 0, 0, v, 0, 0, a, 0, 0, 
                                            0, 1, 0, 0, v, 0, 0, a, 0, 
                                            0, 0, 1, 0, 0, v, 0, 0, a, 
                                            0, 0, 0, 1, 0, 0, v, 0, 0, 
                                            0, 0, 0, 0, 1, 0, 0, v, 0, 
                                            0, 0, 0, 0, 0, 1, 0, 0, v, 
                                            0, 0, 0, 0, 0, 0, 1, 0, 0, 
                                            0, 0, 0, 0, 0, 0, 0, 1, 0, 
                                            0, 0, 0, 0, 0, 0, 0, 0, 1);
            
            
            hej2=hej;
            
            // Make sure that data is still correctly timestamp for later use
            geometry_msgs::PoseStamped msg;
            msg.header=marker1.header;
//             msg.pose.orientation=marker1.pose.orientation;
            
            // inspired from http://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/
            // matrix dimensions are easily given by: www.swarthmore.edu/NatSci/echeeve1/Ref/Kalman/MatrixKalman.html
            
            // prediction
            x_cur=F*x_last; // no external influences
            P_cur=F*P_last*F_trans+Qk; // processNoise as uncertainty
            
            x_cur_rot=F_rot*x_last_rot; // no external influences
            P_cur_rot=F_rot*P_last_rot*F_trans_rot+Qk_rot; // processNoise as uncertainty
            
            
            if(marker1_updated==false) // If updated value, then update prediction and do not publish
            {
                P_last=P_cur;
                x_last = x_cur;
                
                P_last_rot=P_cur_rot;
                x_last_rot = x_cur_rot;
            }
            else{ // else do correction step and publish
                
                // first position
                marker1_updated = false;
                // get measurement
                meas(0) = marker1.pose.position.x;
                meas(1) = marker1.pose.position.y;
                meas(2) = marker1.pose.position.z;
                
                Mat_<float> zk;
                zk = (Mat_<float>(3, 1) << meas(0), meas(1), meas(2));        
                // correction step
                Mat part_of_K=(H*P_cur*H_trans)+Rk;
                Mat K=(P_cur*H_trans)*part_of_K.inv();
                

                Mat x_cur_cor=x_cur+K*(zk-H*x_cur);
                Mat P_cur_cor=P_cur-K*H*P_cur;
                
                
                P_last=P_cur_cor;
                x_last = x_cur_cor;
                
                msg.pose.position.x = x_cur_cor.at<float>(0);
                msg.pose.position.y = x_cur_cor.at<float>(1);
                msg.pose.position.z = x_cur_cor.at<float>(2);
                
                // rotation now
                msg.pose.orientation=marker1.pose.orientation;
                // get measurement
//                 meas(0) = marker1.pose.orientation.x;
//                 meas(1) = marker1.pose.orientation.y;
//                 meas(2) = marker1.pose.orientation.z;
                
                Mat zk_rot = (Mat_<float>(3, 1) << marker1.pose.orientation.x, marker1.pose.orientation.y, marker1.pose.orientation.z);        
//                 // correction step
                Mat part_of_K_rot=(H*P_cur_rot*H_trans)+Rk_rot;
                Mat K_rot=(P_cur_rot*H_trans)*part_of_K_rot.inv();
                

                Mat x_cur_cor_rot=x_cur_rot+K_rot*(zk_rot-H*x_cur_rot);
                Mat P_cur_cor_rot=P_cur_rot-K_rot*H*P_cur_rot;
                
                
                P_last_rot=P_cur_cor_rot;
                x_last_rot = x_cur_cor_rot;
                
                msg.pose.orientation.x = x_cur_cor_rot.at<float>(0);
                msg.pose.orientation.y = x_cur_cor_rot.at<float>(1);
                msg.pose.orientation.z = x_cur_cor_rot.at<float>(2);
                
                
                if(use_filter==true && number_of_observations>30)
                {
                    estMark1_pub.publish(msg);
//                     sendTransformTf_PoseStamped(msg,"camera","marker1_camera");
                }
            }
    }   
    bool turn(kalman_filter::turn::Request  &req,
                   kalman_filter::turn::Response &res)
    {
        use_filter = req.use;
        
        if(use_filter==true)
        {
            ROS_INFO("Using filter");
             res.result=true;
        }
        else
        {
            ROS_INFO("Not using filter");
            res.result=false;
        }
        
        res.result=true;
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

};

int main(int argc, char** argv)
{
        
    ros::init(argc, argv, "kalman_filter_marker1");
    kalman_filter_class kf(argv[1]);
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("/kalman_filter/marker1/turn", &kalman_filter_class::turn, &kf);
    
    
    
    ros::Time last_publish=ros::Time::now();
    ros::Duration time_between_publishing(0.14285); // camera has framerate of 7 hz
    ros::Duration time_difference;
    while(true)
    {
//         ros::Time current_time=ros::Time::now();
//         time_difference=current_time-last_publish;
//         if(time_difference>=time_between_publishing)
//         {
//     //             ROS_INFO("I'm inside");
//             last_publish=ros::Time::now();
//             kf.run_iteration();
//         }
        
        ros::spinOnce();
    }
}
