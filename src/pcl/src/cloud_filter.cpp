#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
 #include <visualization_msgs/Marker.h>


ros::Publisher left_pub;
ros::Publisher right_pub;
ros::Publisher marker_pub_left;
ros::Publisher marker_pub_right;

pcl::PointCloud<pcl::PointXYZ> cut_pointcloud_box(pcl::PointCloud<pcl::PointXYZ>::Ptr &raw_xyz, float box_bounds[])
{

    pcl::PointCloud<pcl::PointXYZ> cloud_cut;

    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = raw_xyz->begin(); it != raw_xyz->end(); it++){
        
        if(it->x < box_bounds[3] && it->x > box_bounds[0] && it->y < box_bounds[4] && it->y > box_bounds[1] && it->z < box_bounds[5] && it->z > box_bounds[2]){
            cloud_cut.push_back(*it);
        }
    }

    return cloud_cut;
}

void visualize_area(float box_bounds[], float rgb[], bool isLeft)
{
    //rviz visualization 
    visualization_msgs::Marker filtered_area;
    filtered_area.header.frame_id = "velodyne";
    filtered_area.type = visualization_msgs::Marker::CUBE;
    filtered_area.action = visualization_msgs::Marker::ADD;

    filtered_area.pose.position.x = (box_bounds[3] + box_bounds[0]) / 2;
    filtered_area.pose.position.y = (box_bounds[4] + box_bounds[1]) / 2;
    filtered_area.pose.position.z = (box_bounds[5] + box_bounds[2]) / 2;

    filtered_area.pose.orientation.x = 0.0;
    filtered_area.pose.orientation.y = 0.0;
    filtered_area.pose.orientation.z = 0.0;
    filtered_area.pose.orientation.w = 1.0;

    filtered_area.scale.x = (box_bounds[3] - box_bounds[0]);
    filtered_area.scale.y = (box_bounds[4] - box_bounds[1]);
    filtered_area.scale.z = (box_bounds[5] - box_bounds[2]);

    filtered_area.color.a = 0.35;
    filtered_area.color.r = rgb[0];
    filtered_area.color.g = rgb[1];
    filtered_area.color.b = rgb[2];

    if (isLeft)
        marker_pub_left.publish(filtered_area);
    else
        marker_pub_right.publish(filtered_area);
}

void cloud_filter (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud,*xyz_cloud);
    /////////////////////////////////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_right(new pcl::PointCloud<pcl::PointXYZ>);

    float left_box [] = {0, 0, 0, 10, 5, 3};
    float right_box [] = {0, -5, 0, 10, 0, 3};

    *filtered_cloud_left = cut_pointcloud_box(xyz_cloud, left_box);
    *filtered_cloud_right = cut_pointcloud_box(xyz_cloud, right_box);

    float left_color [] = {0.1, 0.3, 0.4};
    visualize_area(left_box, left_color, true);

    float right_color [] = {0.3, 0.4, 0.2};
    visualize_area(right_box, right_color, false);

    /////////////////////////////////////////////////////////////////////////////////
    //To RosMsgs
    pcl::toPCLPointCloud2(*filtered_cloud_left, *cloud);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_left;
    pcl_conversions::fromPCL(*cloud, output_left);

    output_left.header.frame_id = "velodyne";
    left_pub.publish(output_left);

    //To RosMsgs
    pcl::toPCLPointCloud2(*filtered_cloud_right, *cloud);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_right;
    pcl_conversions::fromPCL(*cloud, output_right);

    output_right.header.frame_id = "velodyne";
    right_pub.publish(output_right);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_filter");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/points_raw", 10, cloud_filter);

    left_pub = n.advertise<sensor_msgs::PointCloud2> ("/points/left_filtered", 10);
    right_pub = n.advertise<sensor_msgs::PointCloud2> ("/points/right_filtered", 10);

    marker_pub_left = n.advertise<visualization_msgs::Marker> ("marker_left", 10);
    marker_pub_right = n.advertise<visualization_msgs::Marker> ("marker_right", 10);

    ros::spin();

    return 0;
}