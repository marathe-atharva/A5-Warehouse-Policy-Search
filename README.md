- warehouse_state: a mxn grid representing the warehouse state indicating which cell contains wall/box/dropzone/traversable area
- warehouse_cost: a mxn grid representing the cost for accessing the cell at (i, j) position
- dropzone: indicating the index(i, j) where the boxes need to be dropped off, this is also the starting position for the robot
- boxes: a dictionary where box(i, j) indices are stored with key being box_id

Warehouse state:
- #: for wall
- @: dropzone
- .: traversable area

Movements/Actions:
n - north/up, s - south/down, e - east/right, w - west/left
- Orthogonal (n, s, e, w): 2
- Diagonal (ne, nw, se, sw): 3
- Lift the box up (from any direction): 4
- Put the box down (from any direction): 2
- Illegal move: 100 + movement cost
- The total cost for any move = motion_cost + floor_cost
e.g
warehouse = ['*..1','....','.##.']
warehouse_cost = [[ 1, 95, 50, 1],[ 1, 1, 1, 1],[ 1, w, w, 1]]
- If the robot enters (0,1) from (0,0) then the total action cost will be:
  total action cost = motion cost (horizontal movement) + floor cost (destination)
  = 2 + 95
  = 97
- If the robot enters (0,1) from (1,0) then the total action cost will be:
  total action cost = motion cost (diagonal movement) + floor cost (destination)
  = 3 + 95
  = 98
- Note that the floor cost to move into cell (0,1) is 95 regardless of the direction the robot is entering from

Robot Constraints:
- Box can only be placed in any adjacent empty space or the dropzone
- For moving the box it needs to picked up first
- Robot always starts from the dropzone, and after placing all the boxes in correct order in the dropzone it should again go back to the dropzone

Action Format:
- "move <direction>", direction could be n, s, e, w, ne, nw, se, sw
- "lift <box_id>", pick up the box_id box which needs to be adjacent to the robot
- "down <direction>", puts the box down in the given direction empty space