#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "apriltag.h"
#include "tag36h11.h"

using namespace cv;

apriltag_family_t *tf = NULL;
apriltag_detector_t *td = NULL;
ros::Publisher aprilTagPub;

//void imageCallback(const sensor_msgs::ImageConstPtr& msg){  //for raw image
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg){  //for compressed image
    //ROS_INFO("got image");
    cv_bridge::CvImagePtr cv_ptr;
    Mat grayMat;
    try{
        //convert image
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cvtColor(cv_ptr->image,grayMat,COLOR_BGR2GRAY);
        image_u8_t im = { .width = grayMat.cols,
            .height = grayMat.rows,
            .stride = grayMat.cols,
            .buf = grayMat.data
        };

        //do apriltag detection
        zarray_t *detections = apriltag_detector_detect(td, &im);
        //ROS_INFO("detect %d tags", zarray_size(detections));
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            line(cv_ptr->image, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
            line(cv_ptr->image, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
            line(cv_ptr->image, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
            line(cv_ptr->image, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);
        }

        apriltag_detections_destroy(detections);

        //show result in opencv
        //imshow("image",cv_ptr->image);
        //waitKey(1);
        cv_bridge::CvImage cvi;
        cvi.encoding = sensor_msgs::image_encodings::BGR8;
        cvi.image = cv_ptr->image;
        //sensor_msgs::ImagePtr msg = cvi.toImageMsg(); //for raw image
        sensor_msgs::CompressedImagePtr msg = cvi.toCompressedImageMsg();   //for compressed image
        aprilTagPub.publish(msg);
  
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "apriltag_node");
    ROS_INFO("apriltag node");
    ros::NodeHandle nh;

    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    //td->quad_decimate = 2.0;
    //td->quad_sigma = 0.0;
    //td->refine_edges = 1;
    //td->decode_sharpening = 0.25;

    cvNamedWindow("image");
    
    aprilTagPub = nh.advertise<sensor_msgs::CompressedImage>("/topower_v1/camera/apriltag/compressed", 1);
    ros::Subscriber sub = nh.subscribe("/topower_v1/camera/image_raw/compressed",1,imageCallback);  //for compressed image
    ros::spin();

    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    return 0;
}
