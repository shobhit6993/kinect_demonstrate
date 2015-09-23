#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "string"
#include "boost/foreach.hpp"
#include "ar_track_alvar/AlvarMarkers.h"
#include "ar_track_alvar/AlvarMarker.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_broadcaster.h>
// #include <visualization_msgs/Marker.h>

const std::string BAG_FILENAME = "/home/shobhit/ros-workspace/src/kinect_demonstrate/bag/ar_test.bag";
const std::string TOPIC1 = "ar_pose_marker";
const int START_MARKER_ID = 0;

class MyBag
{
private:
    rosbag::Bag bag;
    std::vector<std::string> topics;
    void setTopics();
public:
    MyBag();
    void getWayPoints(std::vector<geometry_msgs::Pose> &);
    geometry_msgs::Pose getStartPoint();
    // void getStampedPose(std::vector<geometry_msgs::PoseStamped> &);
    void getAlvarMarkers(std::vector<ar_track_alvar::AlvarMarker> &);
    ~MyBag();
};