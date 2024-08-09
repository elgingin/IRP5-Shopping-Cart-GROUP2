#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class ImageProcessor {
public:
    ImageProcessor(ros::NodeHandle& nh) : it_(nh) {
        // Initialize subscribers and publishers
        image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageProcessor::imageCallback, this);
        image_pub_ = it_.advertise("/camera/image_processed", 1);

        button_sub_ = nh.subscribe("/push_button_state", 10, &ImageProcessor::buttonCallback, this);
        speed_pub_ = nh.advertise<std_msgs::Float64>("/car/vesc/commands/motor/speed", 10);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Process the image (e.g., convert to grayscale)
        cv::Mat gray_image;
        cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
	
	//Generate a unique filename based on the CURRENT TIME
	std::string filename = "images/captured_image_" + std::to_string(ros::Time::now().toSec()) + ".png";
	
	//Saving the grayscale image to the "images" folder
	cv::imwrite(filename, gray_image);

        // Publish the processed image
        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray_image).toImageMsg();
        image_pub_.publish(out_msg);
    }

    void buttonCallback(const std_msgs::Bool::ConstPtr& msg) {
        std_msgs::Float64 speed_msg;
        if (msg->data) {
            speed_msg.data = 0.0;
        } else {
            speed_msg.data = 1.0; // Or some other value for moving
        }
        speed_pub_.publish(speed_msg);
    }

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber button_sub_;
    ros::Publisher speed_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    ImageProcessor ip(nh);
    ros::spin();
    return 0;
}
