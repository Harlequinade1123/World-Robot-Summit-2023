#include "sketch.h"

int main()
{
    ros::init(argc, argv, "wrs_robot_viz");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    Sketch * s = new Sketch();
    s->run();
    delete(s);
}