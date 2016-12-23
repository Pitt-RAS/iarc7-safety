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

#include <deque>

#include <ros/ros.h>
#include <bondcpp/bond.h>

#include "std_msgs/String.h"

// Rate in hz to check the bonds at
#define LOOPTIME 5

// The time in sec to wait for watched nodes
#define INITTIME 5.0

/**
 * Starts a new Safety Node
 */
int main(int argc, char **argv)
{
   // Register this node with ROS, naming it "CORE_safety"
   ros::init(argc, argv, "iarc7_safety");
   
   // Print out that the node has started
   ROS_INFO("node_monitor has started.");

   // Create a handle for this particular node, which is
   // responsible for handling all of the ROS communications
   ros::NodeHandle nh;
   ros::NodeHandle param_nh ("iarc7_safety");

   // Read in parameter and create the bond table
   std::vector<std::string> bond_ids;
   ROS_ASSERT_MSG(param_nh.getParam("bondIds", bond_ids), "iarc7_safety: Can't load bond id list from parameter server");

   int32_t num_bonds = bond_ids.size();
   std::vector<bond::Bond*> bonds;
   for(std::string bond_id : bond_ids)
   {
      bond::Bond* bond = new bond::Bond("bond_topic", bond_id);
      bonds.push_back(bond);
   }

   // Create a publisher to advertise this node's presence.
   // This node should only publish in case of emergency, so queue length is 100
   // TODO : Change std_msgs::String to a custom type
   ros::Publisher safety_publisher = nh.advertise<std_msgs::String>("safety", 100);
   
   // Specify a time for the message loop to wait between each cycle (ms)
   ros::Rate loop_rate(LOOPTIME);
   
   // Specify a time for the node connection to wait unit an error is thrown (sec)
   ros::Duration init_dur(INITTIME);

   // Initialize bonds
   // Go through all the bonds to initialize them
   for(int32_t i = 0; i < num_bonds; i++)
   {
      // Get a bond
      bond::Bond& bond = *bonds[i];
      ROS_INFO("iarc7_safety: Starting bond: %s", bond.getId().c_str());
      
      // Start the bond
      bond.start();
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
         if (bonds[i]->isBroken()){
            // Set the current safe priority. Never go to a less safe priority than previously recorded.
            lowest_safe_priority = i-1 < lowest_safe_priority ? i-1 : lowest_safe_priority;
         }
      }

      ROS_ASSERT_MSG(lowest_safe_priority < num_bonds || lowest_safe_priority > -2,
                     "node_monitor: Lowest safe priority is outside of possible range");

      // If the lowest_safe_priority is less than the number of safety bonds, we have
      // a safety event
      if(lowest_safe_priority > -1 && lowest_safe_priority < num_bonds - 1)
      {
         // Publish the current highest level safe node
         // If a node hears its name it should respond with a safety response
         std_msgs::String safe_node_name;
         safe_node_name.data = bonds[lowest_safe_priority]->getId();
         safety_publisher.publish(safe_node_name);
      }
      else if(lowest_safe_priority < 0 )
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
