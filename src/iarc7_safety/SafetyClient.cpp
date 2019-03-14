////////////////////////////////////////////////////////////////////////////
//
// SafetyClient
//
// Class implements an easy way to use iarc7_safety to notify of safety events
//
////////////////////////////////////////////////////////////////////////////

// System header
#include <functional>

// Associated header
#include "iarc7_safety/SafetyClient.hpp"

// ROS message headers

using namespace Iarc7Safety;

SafetyClient::SafetyClient(ros::NodeHandle& nh,
                           const std::string bond_id,
                           bool log_for_client)
    : bond_id_(bond_id),
      bond_("bond_topic",
            bond_id_,
            std::bind(&SafetyClient::onBroken, this),
            std::bind(&SafetyClient::onFormed, this)),
      log_for_client_(log_for_client)
{
    safety_subscriber_ = nh.subscribe("safety",
                                      100,
                                      &SafetyClient::processSafetyMessage,
                                      this);
    bond_.setHeartbeatPeriod(0.2);
    bond_.setHeartbeatTimeout(1.5);
    bond_.setConnectTimeout(60.0);
}

bool SafetyClient::formBond()
{
    if (log_for_client_) {
        ROS_INFO("safety_client: trying to form bond %s", bond_.getId().c_str());
    }

    // Try to start the bond
    bond_.start();

    return waitUntilSafe();
}

// This function is a workaround since Bond::waitUntilFormed doesn't work because it doesn't spin
bool SafetyClient::waitUntilSafe()
{
    while(ros::ok())
    {
        if(broken_)
        {
            return false;
        }

        if(formed_)
        {
            return true;
        }

        ros::spinOnce();

        ros::Duration(0.1).sleep();
    }

    // This can only happen if ros::ok() was false which means the bond was not formed
    return false;
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

void SafetyClient::setSafetyResponseActive()
{
    safety_response_activated_ = true;
}

bool SafetyClient::isSafetyResponseActive()
{
    return safety_response_activated_;
}

void SafetyClient::onBroken()
{
    broken_ = true;
    formed_ = false;

    fatal_active_ = true;
    safety_active_ = true;
}

void SafetyClient::onFormed()
{
    broken_ = false;
    formed_ = true;
}

const std::string SafetyClient::getId() const
{
    return bond_id_;
}
