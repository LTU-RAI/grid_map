#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class OccupancyInvers {
public:
    OccupancyInvers(ros::NodeHandle nh, ros::NodeHandle nh_private);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher out_map_pub;
    ros::Subscriber in_map_sub;
    
    nav_msgs::OccupancyGrid out_map;

    std::string in_map_topic;
    std::string out_map_topic;

    void map_callback(const nav_msgs::OccupancyGridConstPtr& msg);
};
