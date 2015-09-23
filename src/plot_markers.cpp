#include "plot_markers.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "plot_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    std::vector<ar_track_alvar::AlvarMarker> way_points;
    MyBag mybag;
    mybag.getAlvarMarkers(way_points);

    int len = way_points.size();

    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.resize(len);


    for (int i = 0; i < len; i++)
    {

        ar_track_alvar::AlvarMarker ar_m = way_points[i];
        // geometry_msgs::PoseStamped w = ar_m.pose;

        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = ar_m.header.frame_id;
        marker.header.stamp = ar_m.header.stamp;

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "ar_demo_markers";
        marker.id = i;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose = ar_m.pose.pose;
        // marker.pose.position.x = 0;
        // marker.pose.position.y = 0;
        // marker.pose.position.z = 0;
        // marker.pose.orientation.x = 0.0;
        // marker.pose.orientation.y = 0.0;
        // marker.pose.orientation.z = 0.0;
        // marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = ((float)i / (float)len) * 1.0f;
        marker.color.g = ((float)(len - i) / (float)len) * 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        markerArray.markers[i] = marker;
        // marker_pub.publish(marker);

    }


    while (ros::ok())
    {
        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        marker_pub.publish(markerArray);

        r.sleep();
    }
    return 0;
}