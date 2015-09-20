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
void MyBag::getWayPoints(std::vector<geometry_msgs::PoseStamped> &way_points)
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
                way_points.push_back(i->markers[0].pose);
            }
        }
    }
}

// sets topic for the messages to be read from bagfile
void MyBag::setTopics()
{
    topics.push_back(std::string(TOPIC1));
}

MyBag::~MyBag()
{
    bag.close();
}

int main(int argc, char const * argv[])
{
    MyBag mybag;

    std::vector<geometry_msgs::PoseStamped> way_points;
    mybag.getWayPoints(way_points);

    BOOST_FOREACH(geometry_msgs::PoseStamped const w, way_points)
    {
        std::cout << w << std::endl;
    }
    return 0;
}