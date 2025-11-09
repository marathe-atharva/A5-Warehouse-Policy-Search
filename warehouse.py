######################################################################
# This file copyright the Georgia Institute of Technology
#
# Permission is given to students to use or modify this file (only)
# to work on their assignments.
#
# You may NOT publish this file or make it available to others not in
# the course.
#
######################################################################
import heapq
import math

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib

    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f"Unique file ID: {file_hash}")


class DeliveryPlanner_PartA:
    """
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

        generate_policies(self, debug = False):
         Stubbed out below. You may not change the method signature
         as it will be called directly by the autograder but you
         may modify the internals as needed.

        __init__:
         Required to initialize the class.  Signature can NOT be changed.
         Basic template starter code is provided.  You may choose to
         use this starter code or modify and replace it based on
         your own solution.

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, warehouse_cost, todo):
        self.rows = len(warehouse)
        self.cols = len(warehouse[0])

        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.todo = todo

        self.actions = [
            (-1, 0, 2, "move s"),
            (1, 0, 2, "move n"),
            (0, -1, 2, "move e"),
            (0, 1, 2, "move w"),
            (-1, -1, 3, "move se"),
            (-1, 1, 3, "move sw"),
            (1, -1, 3, "move ne"),
            (1, 1, 3, "move nw"),
        ]

        self.neighbors = [
            (-1, 0, "n"),
            (1, 0, "s"),
            (0, -1, "w"),
            (0, 1, "e"),
            (-1, -1, "nw"),
            (-1, 1, "ne"),
            (1, -1, "sw"),
            (1, 1, "se"),
        ]

        self.opposite_directions = {
            "n": "s",
            "s": "n",
            "w": "e",
            "e": "w",
            "nw": "se",
            "se": "nw",
            "ne": "sw",
            "sw": "ne",
        }

    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """

        self.warehouse_state = [
            [None for _ in range(self.cols)] for _ in range(self.rows)
        ]
        self.dropzone = None
        self.boxes = dict()

        for r in range(self.rows):
            for c in range(self.cols):
                this_square = warehouse[r][c]

                if this_square == ".":
                    self.warehouse_state[r][c] = "."

                elif this_square == "#":
                    self.warehouse_state[r][c] = "#"

                elif this_square == "@":
                    self.warehouse_state[r][c] = "@"
                    self.dropzone = (r, c)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[r][c] = box_id
                    self.boxes[box_id] = (r, c)

    def plan_path(self, goals):
        value_grid = [
            [float("inf") for _ in range(self.cols)] for _ in range(self.rows)
        ]
        policy_grid = [["-1" for _ in range(self.cols)] for _ in range(self.rows)]

        open_list = []

        for r, c, cost, action in goals:
            value_grid[r][c] = cost
            policy_grid[r][c] = action
            heapq.heappush(open_list, (cost, r, c))

        while open_list:
            cost, r, c = heapq.heappop(open_list)

            if cost > value_grid[r][c]:
                continue

            for dr, dc, move_cost, action in self.actions:
                nr = r + dr
                nc = c + dc

                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if self.warehouse_state[nr][nc] != "#":
                        floor_cost = self.warehouse_cost[r][c]
                        new_cost = cost + move_cost + floor_cost

                        if new_cost < value_grid[nr][nc]:
                            value_grid[nr][nc] = new_cost
                            policy_grid[nr][nc] = action

                            # potential neighbors to explore its neighbors
                            heapq.heappush(open_list, (new_cost, nr, nc))

        return policy_grid

    def get_box_policy(self, box_id, box_pos):
        (br, bc) = box_pos
        lift_cost = 4 + self.warehouse_cost[br][bc]

        goals = list()
        for dr, dc, _ in self.neighbors:
            nr = br + dr
            nc = bc + dc

            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.warehouse_state[nr][nc] != "#":
                    goals.append((nr, nc, lift_cost, f"lift {box_id}"))

        policy_grid = self.plan_path(goals)
        policy_grid[br][bc] = "B"
        return policy_grid

    def get_dropzone_policy(self):
        (dr, dc) = self.dropzone
        down_cost = 2 + self.warehouse_cost[dr][dc]

        goals = []
        for dr_n, dc_n, action in self.neighbors:
            nr = dr + dr_n
            nc = dc + dc_n

            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.warehouse_state[nr][nc] != "#":
                    opposite_action = self.opposite_directions[action]
                    goals.append((nr, nc, down_cost, f"down {opposite_action}"))

        policy_grid = self.plan_path(goals)

        min_move_cost = float("inf")
        best_move_action = "-1"

        outgoing_moves = [
            (-1, 0, 2, "n"),
            (1, 0, 2, "s"),
            (0, -1, 2, "w"),
            (0, 1, 2, "e"),
            (-1, 1, 3, "ne"),
            (-1, -1, 3, "nw"),
            (1, 1, 3, "se"),
            (1, -1, 3, "sw"),
        ]

        for dr_n, dc_c, move_cost, action in outgoing_moves:
            nr = dr + dr_n
            nc = dc + dc_c

            if (
                0 <= nr < self.rows
                and 0 <= nc < self.cols
                and self.warehouse_state[nr][nc] != "#"
            ):
                cost_to_step_aside = move_cost + self.warehouse_cost[nr][nc]

                if cost_to_step_aside < min_move_cost:
                    min_move_cost = cost_to_step_aside
                    best_move_action = f"move {action}"

        # dropzone cell - "step aside" move
        policy_grid[dr][dc] = best_move_action

        return policy_grid

    def generate_policies(self, debug=False):
        """
        generate_policies() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """

        box_id = self.todo[0]
        box_pos = self.boxes[box_id]

        to_box_policy = self.get_box_policy(box_id, box_pos)

        deliver_policy = self.get_dropzone_policy()

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nDeliver Policy:")
            for i in range(len(deliver_policy)):
                print(deliver_policy[i])

        return to_box_policy, deliver_policy


class DeliveryPlanner_PartB:
    """
    [Doc string same as Part A]
    Note: All print outs must be conditioned on the debug parameter.

    Required methods in this class are:

        generate_policies(self, debug = False):
         Stubbed out below. You may not change the method signature
         as it will be called directly by the autograder but you
         may modify the internals as needed.

        __init__:
         Required to initialize the class.  Signature can NOT be changed.
         Basic template starter code is provided.  You may choose to
         use this starter code or modify and replace it based on
         your own solution.

    The following method is starter code you may use.
    However, it is not required and can be replaced with your
    own method(s).

        _set_initial_state_from(self, warehouse):
         creates structures based on the warehouse map

    """

    def __init__(self, warehouse, warehouse_cost, todo, stochastic_probabilities):
        self.rows = len(warehouse)
        self.cols = len(warehouse[0])

        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.todo = todo
        self.stochastic_probabilities = stochastic_probabilities

        self.actions = [
            (-1, 0, 2, "move s"),
            (1, 0, 2, "move n"),
            (0, -1, 2, "move e"),
            (0, 1, 2, "move w"),
            (-1, -1, 3, "move se"),
            (-1, 1, 3, "move sw"),
            (1, -1, 3, "move ne"),
            (1, 1, 3, "move nw"),
        ]

        self.neighbors = [
            (-1, 0, "n"),
            (1, 0, "s"),
            (0, -1, "w"),
            (0, 1, "e"),
            (-1, -1, "nw"),
            (-1, 1, "ne"),
            (1, -1, "sw"),
            (1, 1, "se"),
        ]

        self.opposite_directions = {
            "n": "s",
            "s": "n",
            "w": "e",
            "e": "w",
            "nw": "se",
            "se": "nw",
            "ne": "sw",
            "sw": "ne",
        }

    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """

        self.warehouse_state = [
            [None for _ in range(self.cols)] for _ in range(self.rows)
        ]
        self.dropzone = None
        self.boxes = dict()

        for r in range(self.rows):
            for c in range(self.cols):
                this_square = warehouse[r][c]

                if this_square == ".":
                    self.warehouse_state[r][c] = "."

                elif this_square == "#":
                    self.warehouse_state[r][c] = "#"

                elif this_square == "@":
                    self.warehouse_state[r][c] = "@"
                    self.dropzone = (r, c)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[r][c] = box_id
                    self.boxes[box_id] = (r, c)

    def plan_path(self, goals):
        value_grid = [
            [float("inf") for _ in range(self.cols)] for _ in range(self.rows)
        ]
        policy_grid = [["-1" for _ in range(self.cols)] for _ in range(self.rows)]

        open_list = []

        for r, c, cost, action in goals:
            value_grid[r][c] = cost
            policy_grid[r][c] = action
            heapq.heappush(open_list, (cost, r, c))

        while open_list:
            cost, r, c = heapq.heappop(open_list)

            if cost > value_grid[r][c]:
                continue

            for dr, dc, move_cost, action in self.actions:
                nr = r + dr
                nc = c + dc

                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    if self.warehouse_state[nr][nc] != "#":
                        floor_cost = self.warehouse_cost[r][c]
                        new_cost = cost + move_cost + floor_cost

                        if new_cost < value_grid[nr][nc]:
                            value_grid[nr][nc] = new_cost
                            policy_grid[nr][nc] = action

                            # potential neighbors to explore its neighbors
                            heapq.heappush(open_list, (new_cost, nr, nc))

        return policy_grid

    def get_box_policy(self, box_id, box_pos):
        (br, bc) = box_pos
        lift_cost = 4 + self.warehouse_cost[br][bc]

        goals = list()
        for dr, dc, _ in self.neighbors:
            nr = br + dr
            nc = bc + dc

            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.warehouse_state[nr][nc] != "#":
                    goals.append((nr, nc, lift_cost, f"lift {box_id}"))

        policy_grid = self.plan_path(goals)
        policy_grid[br][bc] = "B"
        return policy_grid

    def get_dropzone_policy(self):
        (dr, dc) = self.dropzone
        down_cost = 2 + self.warehouse_cost[dr][dc]

        goals = []
        for dr_n, dc_n, action in self.neighbors:
            nr = dr + dr_n
            nc = dc + dc_n

            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.warehouse_state[nr][nc] != "#":
                    opposite_action = self.opposite_directions[action]
                    goals.append((nr, nc, down_cost, f"down {opposite_action}"))

        policy_grid = self.plan_path(goals)

        min_move_cost = float("inf")
        best_move_action = "-1"

        outgoing_moves = [
            (-1, 0, 2, "n"),
            (1, 0, 2, "s"),
            (0, -1, 2, "w"),
            (0, 1, 2, "e"),
            (-1, 1, 3, "ne"),
            (-1, -1, 3, "nw"),
            (1, 1, 3, "se"),
            (1, -1, 3, "sw"),
        ]

        for dr_n, dc_c, move_cost, action in outgoing_moves:
            nr = dr + dr_n
            nc = dc + dc_c

            if (
                0 <= nr < self.rows
                and 0 <= nc < self.cols
                and self.warehouse_state[nr][nc] != "#"
            ):
                cost_to_step_aside = move_cost + self.warehouse_cost[nr][nc]

                if cost_to_step_aside < min_move_cost:
                    min_move_cost = cost_to_step_aside
                    best_move_action = f"move {action}"

        # dropzone cell - "step aside" move
        policy_grid[dr][dc] = best_move_action

        return policy_grid

    def generate_policies(self, debug=False):
        """
        generate_policies() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        All print outs must be conditioned on the debug flag.
        """

        box_id = self.todo[0]
        box_pos = self.boxes[box_id]

        # The following is the hard coded solution to test case 1
        to_box_policy = self.get_box_policy(box_id, box_pos)

        to_zone_policy = self.get_dropzone_policy()

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nTo Zone Policy:")
            for i in range(len(to_zone_policy)):
                print(to_zone_policy[i])

        # For debugging purposes you may wish to return values associated with each policy.
        # Replace the default values of None with your grid of values below and turn on the
        # VERBOSE_FLAG in the testing suite.
        to_box_values = None
        to_zone_values = None
        return to_box_policy, to_zone_policy, to_box_values, to_zone_values


def who_am_i():
    # Please specify your GT login ID in the whoami variable (ex: jsmith225).
    whoami = "amarathe38"
    return whoami


if __name__ == "__main__":
    """
    You may execute this file to develop and test the search algorithm prior to running
    the delivery planner in the testing suite.  Copy any test cases from the
    testing suite or make up your own.
    Run command:  python warehouse.py
    """

    # Test code in here will NOT be called by the autograder
    # This section is just a provided as a convenience to help in your development/debugging process

    # Testing for Part A
    # testcase 1
    print("\n~~~ Testing for part A: ~~~")
    warehouse = ["1..", ".#.", "..@"]

    warehouse_cost = [[3, 5, 2], [10, math.inf, 2], [2, 10, 2]]

    todo = ["1"]

    partA = DeliveryPlanner_PartA(warehouse, warehouse_cost, todo)
    partA.generate_policies(debug=True)

    # Testing for Part B
    # testcase 1
    print("\n~~~ Testing for part B: ~~~")
    warehouse = ["1..", ".#.", "..@"]

    warehouse_cost = [[13, 5, 6], [10, math.inf, 2], [2, 11, 2]]

    todo = ["1"]

    stochastic_probabilities = {
        "as_intended": 0.70,
        "slanted": 0.1,
        "sideways": 0.05,
    }

    partB = DeliveryPlanner_PartB(
        warehouse, warehouse_cost, todo, stochastic_probabilities
    )
    partB.generate_policies(debug=True)
