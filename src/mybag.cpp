#include "mybag.h"

MyBag::MyBag()
{
    try
    {
        bag.open(BAG_FILENAME, rosbag::bagmode::Read);

    } catch (rosbag::BagIOException e)
    {
        std::cout << e.what();
        bag.close();
    }

}

// reads a bagfile recorded using ar_track_alvar package
// extracts pose information from each message
// fills the argument vector with waypoints for the robot's end-effector
void MyBag::getWayPoints(std::vector<geometry_msgs::Pose> &way_points)
{
    setTopics();
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // messages are of type ar_track_alvar::AlvarMarkers
    // Header h
    // AlvarMarker[] markers
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        ar_track_alvar::AlvarMarkers::ConstPtr i = m.instantiate<ar_track_alvar::AlvarMarkers>();
        if (i != NULL)
        {
            if (i->markers.size() != 0)
            {
                //add geometry_msgs::Pose not geometry_msgs::PoseStamped
                way_points.push_back(i->markers[0].pose.pose);
            }
        }
    }
}

void MyBag::getAlvarMarkers(std::vector<ar_track_alvar::AlvarMarker> &way_points)
{
    setTopics();
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // messages are of type ar_track_alvar::AlvarMarkers
    // Header h
    // AlvarMarker[] markers
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        ar_track_alvar::AlvarMarkers::ConstPtr i = m.instantiate<ar_track_alvar::AlvarMarkers>();
        if (i != NULL)
        {
            if (i->markers.size() != 0)
            {
                //add geometry_msgs::PoseStamped not geometry_msgs::Pose
                way_points.push_back(i->markers[0]);
            }
        }
    }
}

// sets topic for the messages to be read from bagfile
void MyBag::setTopics()
{
    topics.push_back(std::string(TOPIC1));
}

geometry_msgs::Pose MyBag::getStartPoint()
{
    setTopics();
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    geometry_msgs::Pose start;

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        ar_track_alvar::AlvarMarkers::ConstPtr i = m.instantiate<ar_track_alvar::AlvarMarkers>();
        if (i != NULL)
        {
            if (i->markers.size() != 0 && i->markers[0].id == START_MARKER_ID)
            {
                // first instance of the pose of start marker found. Return this as start position
                std::cout << "Start Marker" << i->markers[0].id << std::endl;
                std::cout << i->markers[0].pose.pose << std::endl;
                start = i->markers[0].pose.pose;
                break;
            }
        }
    }
    std::cout << "BYE" << std::endl;
    return start;
}

MyBag::~MyBag()
{
    bag.close();
}

// int main(int argc, char const * argv[])
// {
//     MyBag mybag;

//     std::vector<geometry_msgs::Pose> way_points;
//     mybag.getWayPoints(way_points);

//     // std::cout << "Size:" << way_points.size() << std::endl;
//     BOOST_FOREACH(geometry_msgs::Pose const w, way_points)
//     {
//         std::cout << w << std::endl;
//     }
//     return 0;
// }