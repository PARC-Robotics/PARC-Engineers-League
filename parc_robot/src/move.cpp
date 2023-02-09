#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// Class to move the turtle forward
class Move
{
    public:
        // A constructor of the class which actually does the moving
        Move()
        {
            ros::Rate rate(10); // Publish 10 times in a second
            pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
            while(ros::ok()) // Keep running this code while ros is running
            {
                for(int i = 0; i < 10000; i++){
                    speed.linear.x = 0.5;
                    pub.publish(speed);
                    rate.sleep();
                }
                for(int i = 0; i < 10000; i++){
                    speed.linear.x = -0.5;
                    pub.publish(speed);
                    rate.sleep();
                }
            }
        }
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        geometry_msgs::Twist speed;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "linear");
    Move mv; // An object of the Move class
    ros::spin(); // You only need this if you will receive callback
    ros::shutdown(); // Disconnects from the master. ros::ok() will become false when shutdown is called
    return 0;
}