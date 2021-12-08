#include "occupancy_grid_invers.hpp"

OccupancyInvers::OccupancyInvers(ros::NodeHandle nh, ros::NodeHandle nh_private) :
    nh_(nh),
    nh_private_(nh_private)
{

    if (!nh_private_.getParam("in_map_topic", in_map_topic))
        in_map_topic =  "traversabilety";
    if (!nh_private_.getParam("out_map_topic", out_map_topic))
        out_map_topic =  "inv/traversabilety";

    in_map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>(in_map_topic, 1, &OccupancyInvers::map_callback, this);
    out_map_pub = nh_.advertise<nav_msgs::OccupancyGrid>(out_map_topic, 0);
}

void OccupancyInvers::map_callback(const nav_msgs::OccupancyGridConstPtr& msg){
    out_map.header = msg->header;
    out_map.info = msg->info;
    out_map.data = msg->data;
    
    int length = msg->info.width * msg->info.height;
    for(int i = 0; i < length; i++){
        if(msg->data[i] == 0){
            out_map.data[i] = -1;
        }else{
            out_map.data[i] = 100 - msg->data[i];
    }
    }
    out_map_pub.publish(out_map);
}

int main (int argc, char **argv){
    ros::init(argc, argv, "ocupancy_invers");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    OccupancyInvers inverse(nh, nh_private);

    ros::spin();

    return 0;
}
