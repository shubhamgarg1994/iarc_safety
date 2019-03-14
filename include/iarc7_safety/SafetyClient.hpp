#ifndef SAFETY_CLIENT_HPP
#define SAFETY_CLIENT_HPP

////////////////////////////////////////////////////////////////////////////
//
// SafetyClient
//
// Class implements an easy way to use the iarc7_safety to notify of safety events
//
////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <bondcpp/bond.h>
#include <std_msgs/String.h>

namespace Iarc7Safety
{

class SafetyClient
{
public:

    SafetyClient(ros::NodeHandle& nh,
                 const std::string bond_id,
                 bool log_for_client=true);

    SafetyClient() = delete;
    ~SafetyClient() = default;

    // Don't allow the copy constructor or assignment.
    SafetyClient(const SafetyClient& rhs) = delete;
    SafetyClient& operator=(const SafetyClient& rhs) = delete;


    // returns true on success
    bool __attribute__((warn_unused_result)) formBond();

    bool isSafetyActive();
    bool isFatalActive();
    void setSafetyResponseActive();
    bool isSafetyResponseActive();

    const std::string getId() const;

private:

    bool waitUntilSafe();

    void processSafetyMessage(const std_msgs::String::ConstPtr& message);

    void onBroken();
    void onFormed();

    ros::Subscriber safety_subscriber_;

    const std::string bond_id_;
    bond::Bond bond_;

    const bool log_for_client_;

    bool fatal_active_{false};
    bool safety_active_{false};
    bool safety_response_activated_{false};

    bool formed_{false};
    bool broken_{false};

    const std::string fatal_message_{"FATAL"};
};

}

#endif
