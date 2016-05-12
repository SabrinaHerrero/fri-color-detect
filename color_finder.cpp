#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp> 
#include "std_msgs/ColorRGBA.h" 

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher chatter_pub;
    
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_camera/image_raw", 1, 
      &ImageConverter::imageCb, this);
   // image_pub_ = it_.advertise("/image_converter/output_video", 1);

    chatter_pub = nh_.advertise<std_msgs::ColorRGBA>("color_chatter", 1000);
    
    ros::Rate loop_rate(10);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	ROS_INFO_STREAM("Image encoding: " << msg->encoding);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
	//  ROS_INFO( "HERE" );
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //  ROS_INFO( "HERE NOW" );
    }
    catch (cv_bridge::Exception& e)
    {
	//	ROS_INFO( "HERE?" );
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    
    //declare output image
    cv::Mat outImg;
    outImg = cv_ptr->image.clone();   
    
     std_msgs::ColorRGBA color_msg;

   // namedWindow("opencv", CV_WINDOW_AUTOSIZE);
    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());    
    
        if (outImg.empty())
            return;
 
        vector<Rect> found, found_filtered;
        hog.detectMultiScale(outImg, found, 0, Size(8,8), Size(32,32), 1.05, 2);
        size_t i, j;
        for (i=0; i<found.size(); i++) 
        {
            Rect r = found[i];
            for (j=0; j<found.size(); j++) 
                if (j!=i && (r & found[j]) == r)
                    break;
            if (j== found.size())
                found_filtered.push_back(r);
        }
		
		int r_x = 0;
		int r_y = 0;
		int height_val = 0;
		int width_val = 0;
		int red_pixel = 0; 
		int blue_pixel = 0;
		int green_pixel = 0;
		int max_Area = 0;
 
        for (i=0; i<found_filtered.size(); i++) 
        {
            Rect r = found_filtered[i];
            r.x += cvRound(r.width*0.1);
		    r.width = cvRound(r.width*0.8);
		    r.y += cvRound(r.height*0.07);
		    r.height = cvRound(r.height*0.8);
		    r_x = r.x;
		    r_y = r.y;
		    height_val = r.height;
		    width_val = r.width;
		    int area = r.x * r.y;
		    if ( area > max_Area ) 
				rectangle(outImg, r.tl(), r.br(), Scalar(0,255,0), 3);        
        }
        
        int x_shirt = r_x + (1.0/2.0)*width_val;
        int y_shirt = r_y + (3.0/7.0)*height_val;
        bool okay = false;
      
        if ( found_filtered.size() >= 5 ) {
			
			okay = false;
			
			//color_msg = std_msgs.msg.ColorRGBA(255.0, 0.0, 0.0, 1.0);
			color_msg.r = 255.0;
			color_msg.g = 0.0;
			color_msg.b = 0.0;
			color_msg.a = 1.0;
			
        }
        
        else {
        
			if ( r_x * r_y > max_Area ) {
				cv::circle(outImg, cv::Point( x_shirt, y_shirt ), 25, CV_RGB(255,0,0));
			
				int numPixels = 0;
				red_pixel = 0; 
				blue_pixel = 0;
				green_pixel = 0;
				
				for (int x_pixel = -5; x_pixel < 5; x_pixel++) {
					for (int y_pixel = -5; y_pixel < 5; y_pixel++) {
						
						numPixels++;
						
						int b_ij = (int)outImg.at<cv::Vec3b>(x_pixel + x_shirt, y_pixel + y_shirt)[0];
						int g_ij = (int)outImg.at<cv::Vec3b>(x_pixel + x_shirt, y_pixel + y_shirt)[1];
						int r_ij = (int)outImg.at<cv::Vec3b>(x_pixel + x_shirt, y_pixel + y_shirt)[2];
						
						red_pixel += r_ij;
						green_pixel += g_ij;
						blue_pixel += b_ij;
						
					}
					
				} 
			
				ROS_INFO("RED: %i", red_pixel / numPixels);
				ROS_INFO("GREEN: %i", green_pixel / numPixels);	
				ROS_INFO("BLUE: %i", blue_pixel / numPixels);
				
				double r = (red_pixel * 0.1) / (numPixels * 0.1);
				double g = (green_pixel * 0.1) / (numPixels * 0.1);
				double b = (blue_pixel * 0.1) / (numPixels * 0.1);
				
				//color_msg = std_msgs.msg.ColorRGBA(r, g, b, 1.0);
				color_msg.r = r;
				color_msg.g = g;
				color_msg.b = b;
				color_msg.a = 1.0;
				
			}
			
		}
		
		chatter_pub.publish(color_msg);
 
        imshow("opencv", outImg);
        waitKey(10);

  }
};


int main (int argc, char * argv[])
{
	  ros::init(argc, argv, "image_converter");
	  ImageConverter ic;
	  ros::spin();
		
    
    return 0;
}

