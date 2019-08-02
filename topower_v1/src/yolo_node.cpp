#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CompressedImage.h"
#include "darknet.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

network* net = NULL;
char** names = NULL;
int classNum = 0;
ros::Publisher yoloPub;

IplImage *image_to_ipl(image im){
    int x,y,c;
    IplImage *disp = cvCreateImage(cvSize(im.w,im.h), IPL_DEPTH_8U, im.c);
    int step = disp->widthStep;
    for(y = 0; y < im.h; ++y){
        for(x = 0; x < im.w; ++x){
            for(c= 0; c < im.c; ++c){
                float val = im.data[c*im.h*im.w + y*im.w + x];
                disp->imageData[y*step + x*im.c + c] = (unsigned char)(val*255);
            }
        }
    }
    return disp;
}

image ipl_to_image(IplImage* src){
    int h = src->height;
    int w = src->width;
    int c = src->nChannels;
    image im = make_image(w, h, c); 
    unsigned char *data = (unsigned char *)src->imageData;
    int step = src->widthStep;
    int i, j, k;

    for(i = 0; i < h; ++i){
        for(k= 0; k < c; ++k){
            for(j = 0; j < w; ++j){ 
                im.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
            }
        }
    }
    return im;
}

//void imageCallback(const sensor_msgs::ImageConstPtr& msg){  //for raw image
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg){  //for compressed image
    //ROS_INFO("got image");
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        //do yolo detection
        IplImage src = (cv_ptr->image);
        image yoloImage = ipl_to_image(&src);
        image sized = letterbox_image(yoloImage, net->w, net->h);
        network_predict(net, sized.data);
        int nboxes = 0;
        float nms = 0.5;
        
        detection *dets = get_network_boxes(net, yoloImage.w, yoloImage.h, 0.5, 0.5, 0, 1, &nboxes);
        if (nms) do_nms_sort(dets, nboxes, classNum, nms);
        draw_detections(yoloImage, dets, nboxes, 0.5, names, NULL, classNum);
        
        //show result in opencv
        IplImage* dst = image_to_ipl(yoloImage);
        //cvShowImage("image",dst);
        //cv::waitKey(1);
        cv_bridge::CvImage cvi;
        cvi.encoding = sensor_msgs::image_encodings::BGR8;
        cvi.image = cv::cvarrToMat(dst);
        //sensor_msgs::ImagePtr msg = cvi.toImageMsg(); //for raw image
        sensor_msgs::CompressedImagePtr msg = cvi.toCompressedImageMsg();   //for compressed image
        yoloPub.publish(msg);
  

        //release memory
        free_detections(dets, nboxes);
        cvReleaseImage(&dst);
        free_image(yoloImage);
        free_image(sized);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "cv_yolo_node");
    ROS_INFO("start cv yolo node");
    ros::NodeHandle nh;

    std::string cfgfile, weightfile,namefile,optionfile;
    std::string nodePath = ros::package::getPath("topower_v1");
    nh.param<std::string>("cfgfile", cfgfile, nodePath+"/src/cfg.fake-tomato/yolov3-tiny.cfg");
    nh.param<std::string>("weightfile", weightfile, nodePath+"/src/cfg.fake-tomato/yolov3-tiny.weight");
    nh.param<std::string>("namefile", namefile, nodePath+"/src/cfg.fake-tomato/obj.names");
    nh.param<std::string>("optionfile", optionfile, nodePath+"/src/cfg.fake-tomato/obj.data");
    
    names = get_labels(const_cast<char*>(namefile.c_str()));
    net = load_network(const_cast<char*>(cfgfile.c_str()), const_cast<char*>(weightfile.c_str()), 0);
    list *options = read_data_cfg(const_cast<char*>(optionfile.c_str()));
    classNum = option_find_int(options, const_cast<char*>("classes"), 0);
    ROS_INFO("class num: %d",classNum);
    set_batch_network(net, 1);
    srand(2222222);

    cvNamedWindow("image");
    //ros::Subscriber sub = nh.subscribe("/topower_v1/camera/image/raw",1,imageCallback); //for raw image
    yoloPub = nh.advertise<sensor_msgs::CompressedImage>("/topower_v1/camera/yolo_result/compressed", 1);
    ros::Subscriber sub = nh.subscribe("/topower_v1/camera/image_raw/compressed",1,imageCallback);  //for compressed image
    ros::spin();
    return 0;
}
