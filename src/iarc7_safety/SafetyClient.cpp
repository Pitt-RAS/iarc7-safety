////////////////////////////////////////////////////////////////////////////
//
// SafetyClient
//
// Class implements an easy way to use the node_monitor to notify of safety events
//
////////////////////////////////////////////////////////////////////////////

// Associated header
#include "iarc7_safety/SafetyClient.hpp"

// ROS message headers

using namespace Iarc7Safety;

SafetyClient::SafetyClient(std::string bond_id) :
bond_id_(bond_id),
bond_("bond_topic", bond_id)
{

}

bool SafetyClient::init()
{
    // Try to start the bond
    bond_.start();
    // Wait a certain amount of time for the bond to form (5 seconds)
    if (!bond_.waitUntilFormed(ros::Duration(5.0))) 
    {
        // The bond didn't form,  post an error and stop creating bonds
        // node_monitor should exit now which will break all the bonds it had
        ROS_ERROR("BondId: %s could not be formed client side", bond_.getId().c_str());
        return false;
    }
    return true;
}

void SafetyClient::processSafetyMessage(const std_msgs::String::ConstPtr& message)
{
    if(message->data == bond_id_)
    {
        safety_active_ = true;
    }
    else if(message->data == fatal_message_)
    {
        safety_active_ = true;
        fatal_active_ = true;
    }
}

bool SafetyClient::isSafetyActive()
{
    return safety_active_;
}

bool SafetyClient::isFatalActive()
{
    return fatal_active_;
}