#ifndef ECN_TOKEN_H
#define ECN_TOKEN_H

#include <ros/ros.h>
#include <std_msgs/String.h>


class TokenManager
{
public:

    TokenManager(std::string group_name)
    {

        msg_.data = group_name;

        pub_ = nh_.advertise<std_msgs::String>("/token_manager/requested", 1);
        sub_ = nh_.subscribe("/token_manager/current", 100, &TokenManager::tokenCallBack, this);

        // wait for token
        ros::Rate loop(1);
        while(ros::ok() && current_token_ != group_name)
        {
            updateToken();
            if(current_token_ != "")
                std::cout << "Current token is for group " << current_token_ << std::endl;

            loop.sleep();
            ros::spinOnce();
        }
    }

    void updateToken()
    {
        pub_.publish(msg_);
    }


protected:
    ros::NodeHandle nh_;
    std::string current_token_;
    std_msgs::String msg_;
    ros::Publisher pub_;
    ros::Subscriber sub_;


    void tokenCallBack(const std_msgs::StringConstPtr &_msg)
    {
        current_token_ = _msg->data;
    }


};



#endif // ECN_TOKEN_H
