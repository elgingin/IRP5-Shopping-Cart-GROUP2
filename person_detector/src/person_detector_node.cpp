

#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Twist.h>

class PersonFollower {
public:
    PersonFollower() {
        // Initialize ROS and set up node handles, publishers, and subscribers
        nh_ = ros::NodeHandle("~");
        sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 10, &PersonFollower::detectionCallback, this);
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    void detectionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
ROS_INFO("callback. Number of bounding boxes: %lu", msg->bounding_boxes.size());
        for (const auto& box : msg->bounding_boxes) {
            if (box.Class == "person" && box.probability > 0.5) {
                // Calculate the center of the bounding box
                int x_center = (box.xmin + box.xmax) / 2;
                int img_center = 640 / 2; // Assuming 640px width image

                // Calculate bounding box size (indicative of distance)
                int box_width = box.xmax - box.xmin;
                int box_height = box.ymax - box.ymin;

                // Initialize Twist message
                geometry_msgs::Twist cmd;

		std::cout << "width: " << box_width << std::endl;             

                // Determine movement direction
                if (x_center < img_center - 50) {
                    // Person is to the left
                    cmd.angular.z = 0.5; // Turn left
                } else if (x_center > img_center + 50) {
                    // Person is to the right
                    cmd.angular.z = -0.5; // Turn right
                } else {
                    cmd.angular.z = 0.0; // No rotation
                }

                // Determine movement speed
                if (box_width < 100) {
                    // Person is far, move faster
                    cmd.linear.x = 0.5;
                } else if (box_width > 200) {
                    // Person is close, move slower or stop
                    cmd.linear.x = 0.1;
                } else {
                    cmd.linear.x = 0.3; // Normal speed
                }

                // Publish the movement command
                pub_.publish(cmd);
                return; // Only act on the first detected person
            }
        }
        
        // If no person is detected, stop the robot
        geometry_msgs::Twist stop_cmd;
        pub_.publish(stop_cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "person_detector");
    PersonFollower pf;
    ros::spin();
    return 0;
}

