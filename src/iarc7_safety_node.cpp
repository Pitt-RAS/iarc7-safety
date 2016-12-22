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

#include <ros/ros.h>
#include <bondcpp/bond.h>

#include "iarc7_safety/SafetyBonds.hpp"

#include "std_msgs/String.h"

// The time in ms between each safety check
#define LOOPTIME 200

// The time in sec to wait for watched nodes
#define INITTIME 5.0

/**
 * Starts a new Safety Node
 */
int main(int argc, char **argv)
{
   // Register this node with ROS, naming it "CORE_safety"
   ros::init(argc, argv, "node_monitor");

   // Create a handle for this particular node, which is
   // responsible for handling all of the ROS communications
   ros::NodeHandle nHandle;
   
   // Create a publisher to advertise this node's presence.
   // This node should only publish in case of emergency, so queue length is 100
   // TODO : Change std_msgs::String to a custom type
   ros::Publisher safety_publisher = nHandle.advertise<std_msgs::String>("safety", 100);
   
   // Specify a time for the message loop to wait between each cycle (ms)
   ros::Rate loop_rate(LOOPTIME);
   
   // Specify a time for the node connection to wait unit an error is thrown (sec)
   ros::Duration init_dur(INITTIME);
   
   // Print out that the node has started
   ROS_INFO("node_monitor has started.");

   // Initialize bonds
   // Go through all the bonds to initialize them
   for(int32_t i = 0; i < num_bonds; i++)
   {
      // Get a bond
      bond::Bond& bond = bonds[i];

      // Try to start the bond
      bond.start();

      // Wait a certain amount of time for the bond to form
      if (!bond.waitUntilFormed(init_dur))
      {
         // The bond didn't form,  post an error and stop creating bonds
         // node_monitor should exit now which will break all the bonds it had
         ROS_ERROR("node_monitor: BondId %s could not be formed", bond.getId().c_str());
         return 0;
      }
   }
   // End intializing bonds

   // This is the lowest priority that is still safe. It should never be incremented.   
   int32_t lowest_safe_priority{num_bonds - 1};

   // Continuously loop the node program
   while(true)
   {
      // Begin checking bonds
      // Go through every node
      for(int32_t i = 0; i < num_bonds; i++)
      {         
         // If the bond is broken stop looping
         if (bonds[i].isBroken()){
            break;
         }
         else
         {
            // Set the current safe priority. Never go to a less safe priority than previously recorded.
            int32_t safe_priority = i - 1;
            lowest_safe_priority = safe_priority < lowest_safe_priority ? safe_priority : lowest_safe_priority;
         }
      }

      ROS_ASSERT_MSG(lowest_safe_priority < num_bonds || lowest_safe_priority > -2,
                     "node_monitor: Lowest safe priority is outside of possible range");

      // If the lowest_safe_priority is less than the number of safety bonds, we have
      // a safety event
      if(lowest_safe_priority > -1)
      {
         // Publish the current highest level safe node
         // If a node hears its name it should respond with a safety response
         std_msgs::String safe_node_name;
         safe_node_name.data = bonds[lowest_safe_priority].getId();
         safety_publisher.publish(safe_node_name);
      }
      else
      {
         // Fatal bond breakage
         // All nodes should try to exit at this point as they are not safe.
         std_msgs::String safe_node_name;
         safe_node_name.data = std::string("FATAL");
         safety_publisher.publish(safe_node_name);
      }
      // End checking bonds

      // Ensure that the node hasn't been shut down.
      if (!ros::ok()) {
         // If shutdown the bonds will break and any listening nodes will default to a fatal state.
         break;
      }
      
      // Give callback to subscribed events
      ros::spinOnce();
      
      // Sleep for a bit 
      loop_rate.sleep();
   } // End main loop

    return 0;
}
