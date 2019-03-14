////////////////////////////////////////////////////////////////////////////
//
// Safety Node
//
// This node monitors other nodes for failures using bonds
//
// Note that a bond has to be created in both the watched node, and this
// safety node with matching names and IDs (See the Bond Example region below) 
//
// Bond should already exist as a package in ROS. Docs at:
// http://docs.ros.org/api/bondcpp/html/classbond_1_1Bond.html
////////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include <regex>
#include <ros/ros.h>
#include <vector>

#include "iarc7_safety/SafetyClient.hpp"

#include <bondcpp/bond.h>

#include "std_msgs/String.h"

// Hearbeat interval in seconds
const float kHeartbeatSec = 0.2;

// Rate in hz to check the bonds. Make it loop three times faster than the heartbeats
const float kLoopFrequencyHz = 1.0/(kHeartbeatSec/3.0);

/**
 * Starts a new Safety Node
 */
int main(int argc, char **argv)
{
   // Register this node with ROS, naming it "CORE_safety"
   ros::init(argc, argv, "iarc7_safety");
   
   // Print out that the node has started
   ROS_DEBUG("node_monitor has started.");

   // Create a handle for this particular node, which is
   // responsible for handling all of the ROS communications
   ros::NodeHandle nh;
   ros::NodeHandle private_nh ("~");
   ros::NodeHandle bond_names_nh (private_nh.param("bond_id_namespace",
                                                   std::string("safety_bonds")));

   // Create a publisher to advertise this node's presence.
   // This node should only publish in case of emergency, so queue length is 100
   // TODO : Change std_msgs::String to a custom type
   ros::Publisher safety_publisher = nh.advertise<std_msgs::String>("safety", 100);

   // Specify a time for the message loop to wait between each cycle (ms)
   ros::Rate loop_rate(kLoopFrequencyHz);

   // Read in all the bonds and their properties from the parameter server
   std::vector<std::pair<std::string, int>> prioritized_bond_ids;
   std::string resolved_bond_namespace = bond_names_nh.getNamespace();

   std::vector<std::string> param_names;
   ROS_ASSERT_MSG(nh.getParamNames(param_names),
                  "Can't load parameter list from parameter server");

   // for all parameters on the server:
   for (const std::string& param_name : param_names) {
       std::smatch match_results;
       // if this parameter looks like /bond_ns/bond_name/form_bond:
       if (std::regex_match(param_name,
                            match_results,
                            std::regex(resolved_bond_namespace
                                     + "/([[:alnum:]_]+)/form_bond"))) {
           // if the parameter is true:
           if (bond_names_nh.param(param_name, false)) {
               // add this bond and its priority to the list
               std::string bond_full_name = resolved_bond_namespace
                                          + "/"
                                          + std::string(match_results[1]);
               prioritized_bond_ids.emplace_back(
                       match_results[1],
                       bond_names_nh.param(bond_full_name + "/priority", -1));
           }
       }
   }

   std::ostringstream bond_stream;
   bond_stream << "Bonds found: ";
   for (const auto& p : prioritized_bond_ids) {
       bond_stream << p.first << " " << p.second << ", ";
   }
   ROS_INFO_STREAM(bond_stream.str());

   std::sort(prioritized_bond_ids.begin(),
             prioritized_bond_ids.end(),
             [](const auto& p1, const auto& p2) -> bool {
                 return p1.second < p2.second;
             });

   if (!prioritized_bond_ids.empty()
           && prioritized_bond_ids.front().second <= 0) {
       ROS_ASSERT_MSG(false, "Bond requested with invalid priority");
   }

   if (std::adjacent_find(prioritized_bond_ids.begin(),
                          prioritized_bond_ids.end(),
                          [](const auto& p1, const auto& p2) -> bool {
                              return p1.second == p2.second;
                          }) != prioritized_bond_ids.end()) {
       ROS_ASSERT_MSG(false, "Two bonds requested with same priority");
   }

   std::vector<std::string> bond_ids;
   for (const auto& p : prioritized_bond_ids) {
       bond_ids.push_back(p.first);
   }

   ROS_ASSERT_MSG(bond_ids.size() > 0, "iarc7_safety: bondId list is empty");

   // This is the lowest priority that is still safe. It should never be incremented.   
   int32_t lowest_safe_priority{static_cast<int32_t>(bond_ids.size()) - 1};
   int32_t last_lowest_safe_priority{lowest_safe_priority};

   // Initialize all the bonds   
   std::vector<std::unique_ptr<Iarc7Safety::SafetyClient>> bonds;
   for(std::string bond_id : bond_ids)
   {
      ROS_INFO("iarc7_safety: Starting bond: %s", bond_id.c_str());

      std::unique_ptr<Iarc7Safety::SafetyClient> bond_ptr(new Iarc7Safety::SafetyClient(nh, bond_id, false));

      // Start the bond
      bool success = bond_ptr->formBond();
      if(success)
      {
         ROS_INFO("iarc7_safety: Made bond: %s", bond_ptr->getId().c_str());
      }
      else
      {
         ROS_ERROR("iarc7_safety: Could not make bond: %s", bond_ptr->getId().c_str());

         // Stop making bonds and immediately set the lowest_safe_priority to fatal.
         // The program did not start correctly.
         lowest_safe_priority = -1;
         bonds.push_back(std::move(bond_ptr));
         break;
      }

      bonds.push_back(std::move(bond_ptr));
   }

   // Continuously loop the node program
   while(true)
   {
      // Go through every node
      for(int32_t i = 0; i < static_cast<int32_t>(bonds.size()); i++)
      {
         // If the safety is on just make sure our priority is as low as that ones
         if (bonds[i]->isSafetyActive())
         {
            // Set lowest safe priority accordingly
            lowest_safe_priority = std::min(i, lowest_safe_priority);
            ROS_ERROR_COND(lowest_safe_priority < last_lowest_safe_priority, "iarc7_safety: Safety status read when checking bond: %s", bonds[i]->getId().c_str());
         }

         // If fatal is active the node can no longer have control. Move to priority to next available priority.
         if (bonds[i]->isFatalActive())
         {
            // Make the lowest safe priority one lower, 
            lowest_safe_priority = std::min(i-1, lowest_safe_priority);
            ROS_ERROR_COND(lowest_safe_priority < last_lowest_safe_priority, "iarc7_safety: Fatal status read when checking bond: %s", bonds[i]->getId().c_str());
         }
      }

      // Make sure the lowest_safe_priority is in a legal range
      ROS_ASSERT_MSG((lowest_safe_priority > -2) && (lowest_safe_priority < static_cast<int32_t>(bonds.size())),
                     "node_monitor: Lowest safe priority is outside of possible range, value: %d", lowest_safe_priority);

      // If lowest_safe_priority is not fatal and not the lowest priority
      // we have a safety event, publish the name of the node to take safety control 
      if(lowest_safe_priority > -1 && lowest_safe_priority < static_cast<int32_t>(bonds.size()) - 1)
      {
         // Publish the current highest level safe node
         // If a node hears its name it should take appropriate action
         std_msgs::String safe_node_name;
         safe_node_name.data = bonds[lowest_safe_priority]->getId();
         safety_publisher.publish(safe_node_name);

         ROS_ERROR_COND(lowest_safe_priority < last_lowest_safe_priority, "iarc7_safety: safety event: current: priority: %d bondId: %s",
         lowest_safe_priority, bonds[lowest_safe_priority]->getId().c_str());
      }
      // Check for a fatal event
      else if(lowest_safe_priority < 0 )
      {
         // All nodes should try to exit at this point as they are not safe.
         std_msgs::String safe_node_name;
         safe_node_name.data = std::string("FATAL");
         safety_publisher.publish(safe_node_name);

         ROS_ERROR_COND(lowest_safe_priority < last_lowest_safe_priority, "iarc7_safety: FATAL event: current: priority: %d", lowest_safe_priority);
      }

      // Ensure that the node hasn't been shut down.
      if (!ros::ok()) {
         // If node is shutdown the bonds will break and 
         // any listening nodes will default to a fatal state.
         break;
      }

      last_lowest_safe_priority = lowest_safe_priority;

      // Give callback to subscribed events
      ros::spinOnce();
      
      loop_rate.sleep();
   }

   return 0;
}
