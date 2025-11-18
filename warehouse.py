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

        self.moves = [
            (-1, 0, 2, "move n"),  # 0
            (-1, 1, 3, "move ne"),  # 1
            (0, 1, 2, "move e"),  # 2
            (1, 1, 3, "move se"),  # 3
            (1, 0, 2, "move s"),  # 4
            (1, -1, 3, "move sw"),  # 5
            (0, -1, 2, "move w"),  # 6
            (-1, -1, 3, "move nw"),  # 7
        ]

        self.stochastic_offsets = {
            "as_intended": 0,
            "slanted": [1, -1],  # +1 index, -1 index
            "sideways": [2, -2],  # +2 index, -2 index
        }

        self.max_cost = 0
        for i in range(self.rows):
            for j in range(self.cols):
                if self.warehouse_cost[i][j] > self.max_cost:
                    self.max_cost = self.warehouse_cost[i][j]

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

    def _solve_mdp(self, goals):
        """
        Runs Value Iteration to solve the Stochastic Shortest Path problem.

        Args:
            goals: list of tuples (r, c, cost, action_str)
                   These are the terminal states (neighbors of target).
        """
        value_grid = [[25 for _ in range(self.cols)] for _ in range(self.rows)]
        policy_grid = [["-1" for _ in range(self.cols)] for _ in range(self.rows)]

        goal_coords = set()
        for r, c, cost, action in goals:
            if cost < value_grid[r][c]:
                value_grid[r][c] = cost
                policy_grid[r][c] = action
                goal_coords.add((r, c))

        p_intended = self.stochastic_probabilities["as_intended"]
        p_slanted = self.stochastic_probabilities["slanted"]
        p_sideways = self.stochastic_probabilities["sideways"]

        max_iterations = 1000
        epsilon = 1e-6  # Convergence threshold

        for _ in range(max_iterations):
            max_change = 0.0

            for r in range(self.rows):
                for c in range(self.cols):
                    if self.warehouse_state[r][c] == "#" or (r, c) in goal_coords:
                        continue

                    floor_cost = self.warehouse_cost[r][c]
                    best_action_value = self.max_cost

                    for move_idx, (dr, dc, move_cost, action_name) in enumerate(
                        self.moves
                    ):
                        current_action_expected_cost = 0.0

                        outcomes = [
                            (0, p_intended),
                            (1, p_slanted),
                            (-1, p_slanted),
                            (2, p_sideways),
                            (-2, p_sideways),
                        ]

                        for offset, prob in outcomes:
                            actual_idx = (move_idx + offset) % 8
                            adr, adc, amove_cost, _ = self.moves[actual_idx]

                            nr, nc = r + adr, c + adc

                            hit_wall = False
                            if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                                hit_wall = True
                            elif self.warehouse_state[nr][nc] == "#":
                                hit_wall = True

                            if hit_wall:
                                penalty_cost = 100 + amove_cost + floor_cost
                                term = penalty_cost + value_grid[r][c]
                            else:
                                step_cost = amove_cost + floor_cost
                                term = step_cost + value_grid[nr][nc]

                            current_action_expected_cost += prob * term

                        if current_action_expected_cost < best_action_value:
                            best_action_value = current_action_expected_cost

                    if abs(best_action_value - value_grid[r][c]) > max_change:
                        max_change = abs(best_action_value - value_grid[r][c])

                    if best_action_value != float("inf"):
                        value_grid[r][c] = best_action_value

            if max_change < epsilon:
                break

        for r in range(self.rows):
            for c in range(self.cols):
                if self.warehouse_state[r][c] == "#" or (r, c) in goal_coords:
                    continue

                floor_cost = self.warehouse_cost[r][c]
                best_val = float("inf")
                best_action_str = "-1"

                for move_idx, (dr, dc, move_cost, action_name) in enumerate(self.moves):
                    expected_cost = 0.0
                    outcomes = [
                        (0, p_intended),
                        (1, p_slanted),
                        (-1, p_slanted),
                        (2, p_sideways),
                        (-2, p_sideways),
                    ]
                    for offset, prob in outcomes:
                        actual_idx = (move_idx + offset) % 8
                        adr, adc, amove_cost, _ = self.moves[actual_idx]
                        nr, nc = r + adr, c + adc

                        hit_wall = False
                        if not (0 <= nr < self.rows and 0 <= nc < self.cols):
                            hit_wall = True
                        elif self.warehouse_state[nr][nc] == "#":
                            hit_wall = True

                        if hit_wall:
                            penalty = 100 + amove_cost + floor_cost
                            expected_cost += prob * (penalty + value_grid[r][c])
                        else:
                            step = amove_cost + floor_cost
                            expected_cost += prob * (step + value_grid[nr][nc])

                    if expected_cost <= best_val:
                        best_val = expected_cost
                        best_action_str = action_name

                policy_grid[r][c] = best_action_str

        return policy_grid, value_grid

    def generate_policies(self, debug=False):
        box_id = self.todo[0]
        box_pos = self.boxes[box_id]

        box_goals = []
        br, bc = box_pos

        adjacents = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]

        for dr, dc in adjacents:
            nr, nc = br + dr, bc + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.warehouse_state[nr][nc] != "#":
                    cost = 4 + self.warehouse_cost[br][bc]
                    box_goals.append((nr, nc, cost, f"lift {box_id}"))

        to_box_policy, to_box_values = self._solve_mdp(box_goals)
        to_box_policy[br][bc] = "B"

        dr, dc = self.dropzone
        zone_goals = []

        opposites = {
            (-1, 0): "s",
            (1, 0): "n",
            (0, -1): "e",
            (0, 1): "w",
            (-1, -1): "se",
            (-1, 1): "sw",
            (1, -1): "ne",
            (1, 1): "nw",
        }

        for dr_n, dc_n in adjacents:
            nr, nc = dr + dr_n, dc + dc_n
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.warehouse_state[nr][nc] != "#":
                    down_cost = 2 + self.warehouse_cost[dr][dc]
                    direction_str = opposites[(dr_n, dc_n)]
                    zone_goals.append((nr, nc, down_cost, f"down {direction_str}"))

        to_zone_policy, to_zone_values = self._solve_mdp(zone_goals)

        if debug:
            print("\nTo Box Policy:")
            for row in to_box_policy:
                print(row)
            print("\nTo Zone Policy:")
            for row in to_zone_policy:
                print(row)

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
