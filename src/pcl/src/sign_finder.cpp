#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

void visualize_sign(pcl::PointXYZ sign_center, float rgb[], bool isLeft);

ros::Publisher marker_pub_left;
ros::Publisher marker_pub_right;

class DBScan{
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        std::vector<int> visited;

        float epsilon;
        int minNeighbor;
        int minSignPoint;

        int nextGroupID;

    public:
        DBScan(pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_xyz, int eps, int minNeigh, int minPnt)
        {
            cloud = filtered_xyz;
            epsilon = eps;
            minNeighbor = minNeigh;
            minSignPoint = minPnt;
            nextGroupID = 1;

            // std::cout<<cloud->points.size()<<std::endl;
            // std::cout<<"ne"<<std::endl;
            // std::cout<<cloud->begin()->x<<std::endl;
            // std::cout<<"oha"<<std::endl;
            
            visited.reserve(cloud->points.size());
            visited.insert(visited.end(), cloud->size(), 0);

            // std::cout<<"cloud size: "<< cloud->size() <<std::endl;
            // std::cout<<"points size: "<< cloud->points.size() <<std::endl;
            // std::cout<<"Visited size: "<< visited.size() <<std::endl;

        }

        float get_distance_3d(int i1, int i2)
        {
            pcl::PointXYZ num1 = cloud->points[i1];
            pcl::PointXYZ num2 = cloud->points[i2];

            float a = sqrt((num1.x - num2.x) * (num1.x - num2.x) +
                        (num1.y - num2.y) * (num1.y - num2.y) + 
                        (num1.z - num2.z) * (num1.z - num2.z));

            // std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxCounters index: "<< a <<std::endl;

            return sqrt((num1.x - num2.x) * (num1.x - num2.x) +
                        (num1.y - num2.y) * (num1.y - num2.y) + 
                        (num1.z - num2.z) * (num1.z - num2.z));
        }

        std::vector<int> find_neighbors_3d(int home)
        {
            std::vector<int> neighbors;

            for (std::vector<int>::size_type i = 0; i != cloud->points.size(); i++){
                if (get_distance_3d(i, home) < epsilon && i != home and visited[i] < 1){
                    // std::cout<<i<<". neighbor added bitch"<<i<<std::endl;
                    neighbors.push_back(i);
                }
            }
            // std::cout<<"size of Neighbors: "<<neighbors.size()<<std::endl;

            return neighbors;
        }

        void clustering(int home, std::vector<int> neighbors)
        {
            int cluster_id = visited[home];
            std::cout<<"GROUPING"<<std::endl;
            std::cout << home << " -> group " << cluster_id << std::endl;

            for(std::vector<int>::iterator neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor) {
                if(visited[*neighbor] == 0){
                    visited[*neighbor] = cluster_id;

                    std::vector<int> sub_neighbors = find_neighbors_3d(*neighbor);
                    clustering(*neighbor, sub_neighbors);
                }
            }
        }

        void start_clustering()
        {
            for (std::vector<int>::size_type i = 0; i != cloud->points.size(); i++){
                if (visited[i] == 0){
                    // std::cout<<"index: "<<i<<std::endl;
                    std::vector<int> neighbors = find_neighbors_3d(i);
                    std::cout<<"size of Neighbors: "<<neighbors.size()<<std::endl;
                    std::cout<<"size of minNeighbors: "<<minNeighbor<<std::endl;
                    

                    if(neighbors.size() < minNeighbor){
                        // std::cout<<"index: "<<i<<std::endl;
                        visited[i] = -1;
                    }
                    else{
                        std::cout<<"GROUPING"<<std::endl;
                        visited[i] = nextGroupID;
                        nextGroupID++;

                        clustering(i, neighbors);
                    }
                }
            }
        }

        pcl::PointXYZ findAveragePoint(int group_id)
        {
            // std::cout<<"Entered find_avg_points"<<std::endl;

            pcl::PointXYZ avgPoint(0, 0, 0);
            int counter = 0;

            for (std::vector<int>::size_type i = 0; i != cloud->points.size(); i++){
                if (visited[i] == group_id){
                    avgPoint.x += cloud->points[i].x;
                    avgPoint.y += cloud->points[i].y;
                    avgPoint.z += cloud->points[i].z;
                    counter++;
                }
            }

            // std::cout<<"Summation ended"<<std::endl;

            avgPoint.x /= (float)counter;
            avgPoint.y /= (float)counter;
            avgPoint.z /= (float)counter;

            std::cout<<"Dividing ended"<<std::endl;

            return avgPoint;

        }

        pcl::PointXYZ get_sign_position()
        {
            // std::cout<<"Entered get_sign_pos"<<std::endl;
            start_clustering();
            // std::cout<<"Clustering Ended"<<std::endl;
            std::map<int, int> counters;

            // std::cout<<"Visited size: "<< visited.size() <<std::endl;

            for(int i = 0; i < visited.size(); i++){
                if(visited[i] > 0)
                    counters[visited[i]]++;
            }
            std::cout<<"Counters size: "<< counters.size() <<std::endl;

            // std::cout<<"Counters map created"<<std::endl;

            std::vector<int> groups;

            for(std::map<int, int>::iterator it = counters.begin(); it != counters.end(); it++){
                groups.push_back(it->first);
            }
            // std::cout<<"Groups size: "<< groups.size() <<std::endl;
            // std::cout<<"Groups vector created"<<std::endl;

            for (int i = 1; i < groups.size(); i++)
            {
                int j = i - 1;
                int key = groups[i];

                while(j >= 0 && counters[groups[j]] < counters[key]){
                    groups[j+1] = groups[j];
                    j--;
                }
                groups[j+1] = key;
            }

            // std::cout<<"Sorting ended"<<std::endl;
            // std::cout<<"Counters size: "<< groups.size() <<std::endl;
            // if(groups.size() > 0){
            //     std::cout<<"Points count: "<< counters[groups[0]] <<std::endl;
            // }
            for (int i = 1; i < groups.size(); i++)
            {
                std::cout<<groups[i]<<":"<< counters[groups[i]] <<std::endl;
            }

            if (groups.size() > 0 && counters[groups[0]] > minSignPoint){
                // std::cout<<"In the minSgnPoint if"<<std::endl;
                return findAveragePoint(groups[0]);
            }
            else{
                // std::cout<<"In the minSgnPoint else"<<std::endl;
                return pcl::PointXYZ(0, 0, 0);
            }

        }
    
};


void sign_finder_left (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud,*xyz_cloud);
    /////////////////////////////////////////////////////////////////////////////////

    // std::cout<<"Entered sign finder left"<<std::endl;

    if (xyz_cloud->size() > 0){
        DBScan dbscan_finder(xyz_cloud, 1, 1, 1);

        // std::cout<<"Created object left"<<std::endl;

        pcl::PointXYZ sign_pos = dbscan_finder.get_sign_position();

        if (sign_pos.x == 0 && sign_pos.y == 0 && sign_pos.z == 0){
            std::cout<<"No traffic sign"<<std::endl;
            visualization_msgs::Marker filtered_area;
            filtered_area.header.frame_id = "velodyne";
            filtered_area.color.a = 0.0;
            marker_pub_right.publish(filtered_area);
        }
        else{
            float left_color [] = {0.8, 0.3, 0.4};
            visualize_sign(sign_pos, left_color, true);
        }
    }
    else{
        std::cout<<"Left Empty Cloud"<<std::endl;
    }
    

}

void sign_finder_right (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud,*xyz_cloud);
    /////////////////////////////////////////////////////////////////////////////////

    if (xyz_cloud->size() > 0){
        // std::cout<<"Entered sign finder right"<<std::endl;

        DBScan dbscan_finder(xyz_cloud, 1, 1, 1);

        // std::cout<<"Created object right"<<std::endl;

        pcl::PointXYZ sign_pos = dbscan_finder.get_sign_position();

        if (sign_pos.x == 0 && sign_pos.y == 0 && sign_pos.z == 0){
                visualization_msgs::Marker filtered_area;
                filtered_area.header.frame_id = "velodyne";
                filtered_area.color.a = 0.0;
                marker_pub_right.publish(filtered_area);
        }
        else{
            float right_color [] = {0.6, 0.7, 0.5};
            std::cout<<"XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX  WHAT THE FUCK XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<< std::endl;
            std::cout<<"signpos: "<< sign_pos.x<< " " << sign_pos.y << " " << sign_pos.z << std::endl;
            visualize_sign(sign_pos, right_color, false);
        }
    }
    else{
        std::cout<<"Right Empty Cloud"<<std::endl;
    }
    

}

void visualize_sign(pcl::PointXYZ sign_center, float rgb[], bool isLeft)
{
    //rviz visualization 
    visualization_msgs::Marker filtered_area;
    filtered_area.header.frame_id = "velodyne";
    filtered_area.type = visualization_msgs::Marker::CUBE;
    filtered_area.action = visualization_msgs::Marker::ADD;

    filtered_area.pose.position.x = sign_center.x;
    filtered_area.pose.position.y = sign_center.y;
    filtered_area.pose.position.z = sign_center.z;

    filtered_area.pose.orientation.x = 0.0;
    filtered_area.pose.orientation.y = 0.0;
    filtered_area.pose.orientation.z = 0.0;
    filtered_area.pose.orientation.w = 1.0;

    filtered_area.scale.x = 0.1;
    filtered_area.scale.y = 0.5;
    filtered_area.scale.z = 0.8;

    filtered_area.color.a = 0.6;
    filtered_area.color.r = rgb[0];
    filtered_area.color.g = rgb[1];
    filtered_area.color.b = rgb[2];

    if (isLeft)
        marker_pub_left.publish(filtered_area);
    else
        marker_pub_right.publish(filtered_area);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sign_finder");

    ros::NodeHandle n;

    ros::Subscriber sub_left = n.subscribe("/points/left_filtered", 10, sign_finder_left);
    ros::Subscriber sub_right = n.subscribe("/points/right_filtered", 10, sign_finder_right);

    marker_pub_left = n.advertise<visualization_msgs::Marker> ("sign_marker_left", 10);
    marker_pub_right = n.advertise<visualization_msgs::Marker> ("sign_marker_right", 10);

    ros::spin();

    return 0;
}