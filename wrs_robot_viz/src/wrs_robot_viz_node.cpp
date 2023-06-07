#include "sketch.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wrs_robot_viz_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    Sketch * s = new Sketch();
    s->run();
    delete(s);
}