#include "iarc7_safety/SafetyBonds.hpp"

// Each bond is listed here. The lower the index of the array to the bond the higher priority that bond has.
// Priority is used to determine which node is requested to take control upon a safety event.
// If a node becomes unsafe (usually due to crashing), the safety node will request via the
// safety topic that the node represented by the next higher priority bond in the table to take control.
bond::Bond bonds[]{{"low_level_motion_bond_topic", "low_level_motion_bond"}
                  };

// Used for iterating over bonds
const int32_t num_bonds{sizeof(bonds)/sizeof(bonds[0])};
