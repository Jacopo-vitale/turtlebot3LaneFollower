// ROS libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
// Opencv Libraries
#include <opencv2/core/core.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>

// Camera Image defines
#define WIDTH 640
#define HWIDTH 320
#define HEIGTH 480

// Uncomment for setting SIMULATION environment
//#define SIMULATION

// Program Global Variables
double x = 0.0,
       y = 0.0;

double x_0 = 0.0,
       y_0 = 0.0;

double rpx_d = 0.0,
       rpy_d = 0.0;

double gpx_d = 0.0,
       gpy_d = 0.0;

double px_d = 0.0,
       py_d = 0.0;

double x_d = 0.0,
       y_d = 0.0;

double media_x = 0.0,
       media_y = 0.0;

double rnum_x = 0.0,
       rnum_y = 0.0,
       gnum_x = 0.0,
       gnum_y = 0.0;

// Init error vars
double e_x = 0.0,
       e_y = 0.0;

//Stop flag
bool stop  = false;

void imageCallback(const sensor_msgs::ImageConstPtr &img)
{
    try
    {
        /* Get image from Topic */
        // Init bridge pointer
        auto cv_ptr = cv_bridge::toCvCopy(img, "bgr8");

        cv::Mat frame_BGR = cv_ptr->image.clone();
        cv::Mat stop_img = frame_BGR.clone();

        /* Pre-Processing */
        // Resizing
        cv::resize(frame_BGR, frame_BGR, cv::Size(640, 480), 1.0, 1.0, cv::INTER_AREA);

        // cv::imshow("Original", frame_BGR);

        /* Bird View (Perspective) */
#ifdef SIMULATION
        // Select ROI points from RGB Image
        cv::Point2f bottom_left(0, 415), bottom_right(640, 415), top_left(160, 295), top_right(495, 295);
        cv::Point2f src[4] = {bottom_left, bottom_right, top_left, top_right};

        // cv::circle(frame_BGR, bottom_left, 5, cv::Scalar(255, 0, 0), cv::FILLED);
        // cv::circle(frame_BGR, bottom_right, 5, cv::Scalar(255, 0, 0), cv::FILLED);
        // cv::circle(frame_BGR, top_left, 5, cv::Scalar(255, 0, 0), cv::FILLED);
        // cv::circle(frame_BGR, top_right, 5, cv::Scalar(255, 0, 0), cv::FILLED);
        // cv::imshow("Original circle", frame_BGR);

        // Select ROI Point to Destination image
        cv::Point2f tl_w(0, 0), tr_w(WIDTH, 0), bl_w(0, HEIGTH), br_w(WIDTH, HEIGTH);
        cv::Point2f dst[4] = {bl_w, br_w, tl_w, tr_w};

#else
        // Select ROI points from RGB Image
        cv::Point2f bottom_left(10, 396), bottom_right(631, 396), top_left(140, 340), top_right(514, 340);
        cv::Point2f src[4] = {bottom_left, bottom_right, top_left, top_right};

        //cv::circle(frame_BGR, bottom_left, 5, cv::Scalar(255, 0, 0), cv::FILLED);
        //cv::circle(frame_BGR, bottom_right, 5, cv::Scalar(255, 0, 0), cv::FILLED);
        //cv::circle(frame_BGR, top_left, 5, cv::Scalar(255, 0, 0), cv::FILLED);
        //cv::circle(frame_BGR, top_right, 5, cv::Scalar(255, 0, 0), cv::FILLED);
        cv::imshow("Original circle", frame_BGR);
        

        // Select ROI Point to Destination image
        cv::Point2f tl_w(0, 0), tr_w(WIDTH, 0), bl_w(0, HEIGTH), br_w(WIDTH, HEIGTH);
        cv::Point2f dst[4] = {bl_w, br_w, tl_w, tr_w};
#endif
        // Calc Direct and inverse Perspective matrix
        cv::Mat warpMat = cv::getPerspectiveTransform(src, dst);
        cv::Mat warpMatInv = cv::getPerspectiveTransform(dst, src);

        // From BGR to Perspective Image
        cv::Mat rotated_BGR;
        cv::warpPerspective(frame_BGR, rotated_BGR, warpMat, cv::Size(WIDTH, HEIGTH));

        // cv::imshow("Rotated BGR", rotated_BGR);

        // Binarized Bird View
        cv::Mat rotated_HSV;
        cv::cvtColor(rotated_BGR, rotated_HSV, cv::COLOR_BGR2HSV);

#ifdef SIMULATION
        cv::Mat green, red1, red2, rred_bin, rgreen_bin;
        cv::inRange(rotated_HSV, cv::Scalar(40, 145, 65), cv::Scalar(50, 155, 75), green);//cv::Scalar(40, 145, 65), cv::Scalar(50, 155, 75), green)
        cv::inRange(rotated_HSV, cv::Scalar(0, 254, 100), cv::Scalar(10, 255, 110), red1);//cv::Scalar(0, 254, 100), cv::Scalar(10, 255, 110), red1);
        cv::inRange(rotated_HSV, cv::Scalar(170, 254, 100), cv::Scalar(180, 255, 110), red2);//cv::Scalar(170, 254, 100), cv::Scalar(180, 255, 110), red2)

        rred_bin = (red1 ^ red2);
        rgreen_bin = green;
        cv::imshow("Rotated Bin", rred_bin ^ rgreen_bin);
#else
        // Binarization
        cv::Mat yellow, red1, red2, rred_bin, rgreen_bin;
 
        cv::inRange(rotated_HSV, cv::Scalar( 22, 30, 128), cv::Scalar( 45,  65, 255), yellow);
		cv::inRange(rotated_HSV, cv::Scalar(  0, 45,  115),   cv::Scalar( 10, 255, 255), red1);
		cv::inRange(rotated_HSV, cv::Scalar(170, 45,  115), cv::Scalar(180, 255, 255), red2);

        rred_bin = (red1 ^ red2);

        //As to not dubling the lines of code, just apply an alias for yellow <----> green
        rgreen_bin = yellow;
        
        cv::imshow("Rotated Bin", rred_bin ^ rgreen_bin);
#endif

        // Find contours
        std::vector<std::vector<cv::Point>> r_contours, g_contours;
        cv::findContours(rred_bin,   r_contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
        cv::findContours(rgreen_bin, g_contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        // Get rid of small elements, retrieve only the big ones, then find momentum
        int skip_elements = 0;
        for (auto &contour : r_contours)
        {
            float area = cv::contourArea(contour);
            if (area < 1000)
            {
                skip_elements++;
                continue;
            }

            cv::Moments mu = cv::moments(contour);
            rnum_x += (mu.m10 / mu.m00);
            rnum_y += (mu.m01 / mu.m00);
        }

        if (!r_contours.empty()) // r_contours.size() - skip_elements) != 0
        {
            rpx_d = (rnum_x / (r_contours.size() - skip_elements));
            rpy_d = (rnum_y / (r_contours.size() - skip_elements));
        }
        else // set red to negative value when contour vector is empty
        {
            rpx_d = -1;
            rpy_d = -1;
        }

        // ROS_INFO("rpx: %f | rpy: %f", rpx_d, rpy_d);

        // green
        int gskip_elements = 0;
        for (auto &contour : g_contours)
        {
            float area = cv::contourArea(contour);
            if (area < 1000)
            {
                gskip_elements++;
                continue;
            }

            cv::Moments mu = cv::moments(contour);
            gnum_x += (mu.m10 / mu.m00);
            gnum_y += (mu.m01 / mu.m00);
        }
        if (!g_contours.empty()) //(g_contours.size() - gskip_elements) != 0
        {
            gpx_d = (gnum_x / (g_contours.size() - gskip_elements));
            gpy_d = (gnum_y / (g_contours.size() - gskip_elements));
        }
        else // set green to negative value when contour vector is empty
        {
            gpx_d = -1;
            gpy_d = -1;
        }

        // ROS_INFO("gpx: %f | gpy: %f", gpx_d, gpy_d);

        // Build x_d & y_d
        cv::drawContours(rotated_BGR, r_contours, -1, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
        cv::drawContours(rotated_BGR, g_contours, -1, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);

        // When only one line is found, then follow centroid with an offset
        double offset = 200;
        if (rpx_d > 0 && gpx_d > 0)
        {
            px_d = (rpx_d + gpx_d) / 2;
        }
        else if (gpx_d < 0 && gpy_d < 0)
        {
            px_d = rpx_d + offset;
        }
        else if (rpx_d < 0 && rpy_d < 0)
        {
            px_d = gpx_d - offset;
        }
        else{ //No lines are found, error is set to 300
            px_d = 300;
        }

        //Draw desired centroid
        cv::circle(rotated_BGR, cv::Point(px_d, HEIGTH/2), 5, cv::Scalar(0, 255, 255), 2, cv::FILLED);

        //Compute the distance between the image center
        x_d = HWIDTH - px_d;

        // Clear variables
        rnum_x = 0.0;
        rnum_y = 0.0;
        r_contours.clear();

        gnum_x = 0.0;
        gnum_y = 0.0;
        g_contours.clear();

        px_d = 0.0;
        py_d = 0.0;

        cv::imshow("Centroid", rotated_BGR);

        //STOP CONDITION
        cv::cvtColor(stop_img,stop_img,cv::COLOR_BGR2HSV);
        // binarize selecting green ranges
        cv::inRange(stop_img,cv::Scalar(58,43,84),cv::Scalar(78,177,150),stop_img);
        // count non zero pixels
        if(cv::countNonZero(stop_img) > 7000){
            stop = true;
        }

        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }
}
int main(int argc, char **argv)
{

    // Init ros
    ros::init(argc, argv, "pixelpilot");

    // Init Nodes handles
    ros::NodeHandle n_img, n_odom, n_cmd;

    // Init image node handle
    image_transport::ImageTransport it_(n_img);

    // Init Camera image Subscriber
#ifdef SIMULATION
    image_transport::Subscriber sub = it_.subscribe("/camera/rgb/image_raw", 1000, imageCallback);
#else
    image_transport::Subscriber sub = it_.subscribe("/camera/image", 1, imageCallback);
#endif

    // Init command Vel Publisher
    ros::Publisher cmd_pub = n_cmd.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // Define Loop Rate 10 Hz
    ros::Rate loop_rate(10);

    // Init Program Gains
    double k1 = 0.003;

    // Init command message
    geometry_msgs::Twist cmd;

    // Control Loop
    while (ros::ok())
    {
        // Compute errors
        e_x = x_d - x; 

        ROS_INFO("EX: %f\n", e_x);

        // Control Law
        if(stop){
            //stop the program, then the robot
            ros::shutdown();
        }
        else if(std::abs(e_x) > 100){ //Am I in a curve?
            cmd.linear.x  = 0.025;
            cmd.angular.z = k1* 0.45 * e_x;
        }
        else{ 
            cmd.linear.x  =  0.5;
            cmd.angular.z = k1 * e_x;
        }
        
        cmd_pub.publish(cmd);

        loop_rate.sleep();

        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
