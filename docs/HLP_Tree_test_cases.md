Asssuming:
- closeness threshold = 2


Test:
AddPoint(0, 0) // Start Point
// Tree now contains 0, 0
GetNext() -> (0, 0)
// Add next open points
AddPoint(3, 0.5)
AddPoint(3, -0.5)
// Tree now contains one of the two nodes from above
// One is removed due to radius constraints
GetNext() -> (3, {-0.5, 0.5})
// Add hallway points
AddPoint(4, -3)
AddPoint(4, 4)
// Pick new direction to go in
GetNext() -> (4, 4)
// See end of hallway
AddPoint(5, 8)
// Pick new direction to go in
GetNext() -> (5, 8)
// Reach end of hallway
// See beginning of hallway
AddPoint(4, 6)
// Node is not added, node is within the distance threshold of the path
// Distance threshold of path is distance from point to line
// No new nodes
// Leaf node reached, go back n
GetNext() -> (4, 4)
// go back n
GetNext() -> (3, {-0.5, 0.5})
// This node still has (3, -3) unexplored
GetNext() -> (3, -3)
// Reach end of hallway
// No new nodes
// Leaf node reached, go back n
GetNext() -> (3, {-0.5, 0.5})
// go back n
GetNext() -> (0, 0)
// At end
GetNext() -> (0, 0)
GetNext() -> (0, 0)
