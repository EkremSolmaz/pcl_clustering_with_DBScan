    
    
ros::Publisher marker_pub;
    
    
    
visualization_msgs::Marker marker;
marker.header.frame_id = "velodyne";
marker.header.stamp = ros::Time();
marker.ns = "my_namespace";
marker.id = 0;
marker.type = visualization_msgs::Marker::CUBE;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = 0;
marker.pose.position.y = 0;
marker.pose.position.z = 0;
marker.pose.orientation.x = 0.0;
marker.pose.orientation.y = 0.0;
marker.pose.orientation.z = 0.0;
marker.pose.orientation.w = 1.0;
marker.scale.x = 1;
marker.scale.y = 2.1;
marker.scale.z = 3.1;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 1.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
marker_pub.publish( marker );


marker_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );