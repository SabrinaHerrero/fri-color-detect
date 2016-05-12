#include <opencv2/opencv.hpp>
 
using namespace cv;
 
int main (int argc, const char * argv[])
{
    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
 
    if (!cap.isOpened())
        return -1;
 
    Mat img;
    namedWindow("opencv", CV_WINDOW_AUTOSIZE);
    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
 
    while (true)
    {
        cap >> img;
        if (img.empty())
            continue;
 
        vector<Rect> found, found_filtered;
        hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);
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
		    rectangle(img, r.tl(), r.br(), Scalar(0,255,0), 3);        
        }
        
        int x_shirt = r_x + (1.0/2.0)*width_val;
        int y_shirt = r_y + (3.0/8.0)*height_val;
        
        int red_pixel = 0;
        int blue_pixel = 0;
        int green_pixel = 0;
        
        
 /*      cv::circle(outImg, cv::Point( x_shirt, y_shirt ), 50, CV_RGB(255,0,0));
        
        for (int x_pixel = -20; x_pixel < 20; x_pixel++) {
			for (int y_pixel = -20; y_pixel < 20; y_pixel++) {
				
				int b_ij = (int)outImg.at<cv::Vec3b>(x_pixel + x_shirt, y_pixel + y_shirt)[0];
				int g_ij = (int)outImg.at<cv::Vec3b>(x_pixel + x_shirt, y_pixel + y_shirt)[1];
				int r_ij = (int)outImg.at<cv::Vec3b>(x_pixel + x_shirt, y_pixel + y_shirt)[2];
				
				red_pixel += r_ij;
				blue_pixel += b_ij;
				green_pixel += g_ij;
				
			}
		} 
		
		ROS_INFO("RED: %i", red_pixel / 40);
		ROS_INFO("BLUE: %i", blue_pixel / 40);
		ROS_INFO("GREEN: %i", green_pixel / 40);	
			
 */
        imshow("opencv", img);
        if (waitKey(10)>=0)
            break;
    }
    return 0;
}
 
