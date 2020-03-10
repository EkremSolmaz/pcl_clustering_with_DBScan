#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud,*xyz_cloud);
    /////////////////////////////////////////////////////////////////////////////////

    int cnt = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = xyz_cloud->begin(); it != xyz_cloud->end(); it++){
        if(abs(it->z) > -1 && abs(it->z) < 0.9)
            it->z = 0.0;
        else
            it->z = 5.0;
        cnt++;
    }

    /////////////////////////////////////////////////////////////////////////////////
    //To RosMsgs
    pcl::toPCLPointCloud2(*xyz_cloud, *cloud);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(*cloud, output);

    pub.publish (output);


}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/points_raw", 10, cloud_cb);

  pub = n.advertise<sensor_msgs::PointCloud2> ("output", 10);

  ros::spin();

  return 0;
}