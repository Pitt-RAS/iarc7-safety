#ifndef SAFETY_CLIENT_HPP
#define SAFETY_CLIENT_HPP

////////////////////////////////////////////////////////////////////////////
//
// SafetyClient
//
// Class implements an easy way to use the node_monitor to notify of safety events
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bondcpp/bond.h>

namespace Iarc7Safety
{

    class SafetyClient
    {
    public:

        SafetyClient(ros::NodeHandle& nh, const std::string bond_id);

        SafetyClient() = delete;
        ~SafetyClient() = default;

        // Don't allow the copy constructor or assignment.
        SafetyClient(const SafetyClient& rhs) = delete;
        SafetyClient& operator=(const SafetyClient& rhs) = delete;


        // returns true on success
        bool __attribute__((warn_unused_result)) formBond();

        bool isSafetyActive();
        bool isFatalActive();

    private:

        void processSafetyMessage(const std_msgs::String::ConstPtr& message);

        ros::Subscriber safety_subscriber_;

        const std::string bond_id_;
        bond::Bond bond_;

        bool fatal_active_{false};
        bool safety_active_{false};

        const std::string fatal_message_{"FATAL"};
    };

}

#endif
