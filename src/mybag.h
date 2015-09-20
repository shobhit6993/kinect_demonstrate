#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "string"
#include "boost/foreach.hpp"
#include "ar_track_alvar/AlvarMarkers.h"
#include "ar_track_alvar/AlvarMarker.h"

const std::string BAG_FILENAME = "/home/shobhit/ros-workspace/src/kinect_demonstrate/bag/ar_test.bag";
const std::string TOPIC1 = "ar_pose_marker";

class MyBag
{
private:
    rosbag::Bag bag;
    std::vector<std::string> topics;
    void setTopics();
public:
    MyBag();
    void readMessages();
    ~MyBag();
};