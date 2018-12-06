#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

#include <geometry_msgs/Twist.h> //gives me access to twist

static const std::string OPENCV_WINDOW = "Image window";

int row;
int col;

int sweepTime;

float dist_val;
float current_rotate = 0;

float current_speed = 0;
float target_speed = 0;

bool searching = false;
int searchTime = 0;
bool searchingRight = true;

const static float acc = 0.02;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;  
  image_transport::Publisher image_pub_;

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber depth_sub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 10, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 10);

    depth_sub_ = nh_.subscribe("/camera/depth/image", 1000, &ImageConverter::imgcb, this);

    cv::namedWindow(OPENCV_WINDOW);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1); //sets up publisher
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imgcb(const sensor_msgs::Image::ConstPtr& msg){
	
	try
	{
		cv_bridge::CvImageConstPtr cv_ptr;
	        cv_ptr = cv_bridge::toCvShare(msg);
		double max = 0.0;
		cv::minMaxLoc(cv_ptr->image, 0 , &max, 0, 0);
		cv::Mat normalized;
		cv_ptr->image.convertTo(normalized, CV_32F, 1.0/max, 0);

		dist_val = cv_ptr->image.at<float>(row,col);	
		  
	}
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
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

    row = 0; 
    col = 0; 
    int dis_min = 1000000; 
    for (int i=0; i < cv_ptr->image.rows; i++) {
        for (int j=0; j < cv_ptr->image.cols; j++) {
            int b = cv_ptr->image.at<cv::Vec3b>(i, j).val[0]; 
            int g = cv_ptr->image.at<cv::Vec3b>(i, j).val[1];
            int r = cv_ptr->image.at<cv::Vec3b>(i, j).val[2]; 
            // int dis = pow((r-255)*(r-255) + g*g + b*b, 0.5); 
            int dis = (r-255)*(r-255) + g*g + b*b; 
            // ROS_INFO("b: %d", b); 
            // ROS_INFO("g: %d", g); 
            // ROS_INFO("r: %d", r); 
            // ROS_INFO("%d", dis); 
            if (dis < dis_min) {
                dis_min = dis; 
                row = i; 
                col = j; 
            }
        }
    }

	//assumes 670 columns
	geometry_msgs::Twist base_cmd;
	base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;	

	if(col < 320){
		current_rotate = -0.0078125 * col + 2.5; //left		
	}

	else if(col > 340){
		current_rotate = -0.007575* col + 2.5757; //right
	}
	
	else {
		current_rotate = 0;
	}	

	if(dist_val > 1){
		ROS_INFO_STREAM("forward .1");
		target_speed = .1;
		if(current_speed < target_speed) current_speed += acc;
		if(current_speed > target_speed) current_speed -= acc;
		
	}

	if(dist_val > 1.2){
		ROS_INFO_STREAM("forward .3");
		target_speed = .3;
		if(current_speed < target_speed) current_speed += acc;
		if(current_speed > target_speed) current_speed -= acc;
		
	}

	if(dist_val > 1.4){
		ROS_INFO_STREAM("forward .6");
		target_speed = .8;
		if(current_speed < target_speed) current_speed += acc;
		if(current_speed > target_speed) current_speed -= acc;
		
	}

	if(dist_val < 0.9){
		ROS_INFO_STREAM("backward");
		target_speed = -0.4;
		if(current_speed > target_speed) current_speed -= acc;
	}
	
	if(((dist_val <= 1) && (dist_val >= 0.9)) || isnan(dist_val)){
		ROS_INFO_STREAM("sweet");
		target_speed = 0;
		if(abs(current_speed - target_speed) < 0.02)current_speed = target_speed;
		else if(current_speed > target_speed) current_speed -= acc;
		else if(current_speed < target_speed) current_speed += acc;
	}	
	
	if(dis_min > 4000){ // searching state
		ROS_INFO_STREAM("searching");
		if(searching == false){
			searchTime = 10;					
			searching = true;
			sweepTime = 20;
		}

		if(searchingRight)current_rotate = -.9;
		if(!searchingRight)current_rotate = .9;	

		target_speed = 0;
		if(abs(current_speed - target_speed) < 0.02)current_speed = target_speed;
		else if(current_speed > target_speed) current_speed -= acc;
		else if(current_speed < target_speed) current_speed += acc;	
	
		searchTime++;

		

		if(searchTime > sweepTime){
			searchTime = 0;
			searchingRight = !searchingRight;
			sweepTime += 10;
		}
		
	}
	else {
		searching = false;
	}

	base_cmd.angular.z = current_rotate;
	base_cmd.linear.x = current_speed;


	//ROS_INFO_STREAM(sweepTime);
	//ROS_INFO_STREAM(dis_min);
	//ROS_INFO_STREAM(current_rotate);
	//ROS_INFO_STREAM(target_speed);
	//ROS_INFO_STREAM(col);
	//ROS_INFO("depth: %f", dist_val);

	cmd_vel_pub_.publish(base_cmd); // publish the command

    // ROS_INFO("value: %d", cv_ptr->image.at<cv::Vec3b>(0,0).val[0]); 
    // ROS_INFO("value: %d", cv_ptr->image.at<cv::Vec3b>(0,0).val[1]); 
    // ROS_INFO("value: %d", cv_ptr->image.at<cv::Vec3b>(0,0).val[2]); 
    // ROS_INFO("height: %d", msg->height); 
    // ROS_INFO("width: %d", msg->width); 
    // ROS_INFO_STREAM("encoding: " << msg->encoding); 

   
    // ROS_INFO("r: %d", row); 
    // ROS_INFO("c: %d", col); 

    cv::circle(cv_ptr->image, cv::Point(col, row), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
