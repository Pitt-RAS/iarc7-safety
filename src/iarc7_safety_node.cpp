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
#include <std_msgs/String.h>

// The time in ms between each safety check
#define LOOPTIME 10

// The time in sec to wait for watched nodes
#define INITTIME 5.0

/**
 * Starts a new Safety Node
 */
int main(int argc, char **argv)
{
   // Register this node with ROS, naming it "CORE_safety"
   ros::init(argc, argv, "CORE_safety");

   // Create a handle for this particular node, which is
   // responsible for handling all of the ROS communications
   ros::NodeHandle nHandle;
   
   // Create a publisher to advertise this node's presence.
   // This node should only publish in case of emergency, so queue length is 100
   // TODO : Change std_msgs::String to a custom type
   ros::Publisher publisher = nHandle.advertise<std_msgs::String>("CORE_safety_pub", 100);
   
   // Specify a time for the message loop to wait between each cycle (ms)
   ros::Rate loop_rate(LOOPTIME);
   
   // Specify a time for the node connection to wait unit an error is thrown (sec)
   ros::Duration init_dur(INITTIME);
   
   // Print out that the node has started
   ROS_INFO("iarc7_safety has started.");
   
   // Create a new bond for other processes to attach to (ID "test")
   bond::Bond bond("iarc7_safety_bond_topic", "test"); 
   bond.start();
   if (!bond.waitUntilFormed(init_dur)){
      ROS_ERROR("ERROR! A bond could not be formed!");
      return -1;
   }

   // Continuously loop the node program
   while (true) {
      
      // Ensure that the Bond has not been broken
      if (bond.isBroken()){
      
         // TODO : Restart the node and the bond
         break;
      }
      
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
