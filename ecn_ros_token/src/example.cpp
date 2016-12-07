#include <ecn_ros_token/ecn_token.h>
#include <sstream>
#include <time.h>

using namespace std;

int main(int argc, char** argv)
{
    // this node name
    stringstream ss;
    srand (time(NULL));
    ss << rand() % 255;
    string node_name = "node_" + ss.str();
    string group_name = "group_" + ss.str();

    ros::init(argc, argv, node_name.c_str());
    ros::NodeHandle nh;

    TokenManager tm(group_name);

    ros::Rate loop(1);

    while(ros::ok())
    {
        cout << group_name << " doing its job" << endl;

        tm.updateToken();


        loop.sleep();
        ros::spinOnce();
    }








}
