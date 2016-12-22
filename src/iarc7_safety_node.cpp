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
#include <std_msgs/Int32.h>

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
   ros::Publisher safety_publisher = nHandle.advertise<std_msgs::Int32>("safety", 100);
   
   // Specify a time for the message loop to wait between each cycle (ms)
   ros::Rate loop_rate(LOOPTIME);
   
   // Specify a time for the node connection to wait unit an error is thrown (sec)
   ros::Duration init_dur(INITTIME);
   
   // Print out that the node has started
   ROS_INFO("node_monitor has started.");
   
   // Each bond is listed here. The lower the index of the array to the bond the higher priority that bond has.
   // Priority is used to determine which node is requested to take control upon a safety event.
   // If a node becomes unsafe (usually due to crashing), the safety node will request via the
   // safety topic that the node represented by the next higher priority bond in the table to take control.
   bond::Bond bonds[]{{"low_level_motion_bond_topic", "low_level_motion_bond"}
                      };

   // This is the lowest priority that is still safe.
   // It is set based on the amount of bonds safely formed in order.
   int32_t lowest_safe_priority{-1};
   for(bond::Bond &bond : bonds)
   {
      bond.start();
      if (!bond.waitUntilFormed(init_dur)){
         ROS_ERROR("node_monitor: BondId %s could not be formed", bond.getId().c_str());
         return -1;
      }
      {
         ++lowest_safe_priority;
      }
   }

   // Continuously loop the node program
   while (true) {
   
      int32_t current_safe_node{-1};
      for(bond::Bond &bond : bonds)
      {
         // If the bond is not broken
         if (!bond.isBroken()){
            // We can progress to giving control to a less safe priority
            // Bound at the lowest_safe_priority
            current_safe_node = current_safe_node < lowest_safe_priority ? current_safe_node + 1 : lowest_safe_priority;
         }
         else
         {
            lowest_safe_priority = current_safe_node;
         }
      }

      // Publish the current safe priority
      std_msgs::Int32 safe_node_id;
      safe_node_id.data = current_safe_node;
      safety_publisher.publish(safe_node_id);
      
      // Ensure that everything is okay
      // Note that if this returns false, all ROS calls will fail
      if (!ros::ok()) {
         
         // TODO : This is only called when ROS fails.
         //        Figure out what can be salvaged at this point.
         break;
      }
      
      // Give callback to subscribed events
      ros::spinOnce();
      
      // Sleep for a bit 
      loop_rate.sleep();
   }

    // Error has been handled, so all is good.
    return 0;
}
