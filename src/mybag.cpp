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

void MyBag::readMessages()
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
            std::cout << i->markers[0].pose << std::endl;
    }
}

void MyBag::setTopics()
{
    topics.push_back(std::string(TOPIC1));
}

MyBag::~MyBag()
{
    bag.close();
}

int main(int argc, char const *argv[])
{
    MyBag mybag;
    mybag.readMessages();
    return 0;
}