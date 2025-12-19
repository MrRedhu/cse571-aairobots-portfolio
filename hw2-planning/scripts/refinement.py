#!/usr/bin/env python3
# encoding: utf-8

__copyright__ = "Copyright 2021, AAIR Lab, ASU"
__authors__ = ["Namn Shah", "Rushang Karia"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = "reStructuredText"

import rospy
import problem
import heapq
import argparse
import os
import json
import random
from std_msgs.msg import String

from parser import parse_args
from server import initialize_planning_server
from server import generate_maze
from utils import initialize_ros
from utils import cleanup_ros

from hw2.srv import MoveActionMsg
from hw2.srv import PlaceActionMsg
from hw2.srv import PickActionMsg

import planner
import utils
import search
import traceback
import sys

"""
Do not change anything above this line, except if you want to import some package.
"""


class Refine:
    """
    This class has code to refine the high level actions used by PDDL
    to the low level actions used by the TurtleBot.
    """

    def __init__(self, action_list):
        """
        :param plan_file: Path to the file where plan is stored.
        :type plan_file: str
        :param planner: Planner that was used to generate this plan.
        :type planner: str
        """

        self.helper = problem.Helper()
        with open("%s/objects.json" % (utils.ROOT_PATH)) as f:
            self.env_data = json.load(f)

        self.action_index = 0
        self.actions_queue = self.refine_plan(action_list)

    def get_load_locations(self, location):
        """
        Returns a set of coordinates that correspond to the given location.
        """
        obj = location[: location.index("_iloc")]

        if obj in self.env_data["object"].keys():
            return self.env_data["object"][obj]["load_loc"]
        elif obj in self.env_data["goal"].keys():
            return self.env_data["goal"][obj]["load_loc"]
        else:
            exit(-1)

    def _simplify_primitives(self, prims):
        """
        Peephole simplifier for low-level primitives.
        - Cancels opposite turns back-to-back (TurnCW, TurnCCW) or (TurnCCW, TurnCW).
        - Compresses triple turns: TurnCW x3 -> TurnCCW, TurnCCW x3 -> TurnCW.
        """
        stack = []
        for p in prims:
            if stack and (
                (stack[-1] == "TurnCW" and p == "TurnCCW")
                or (stack[-1] == "TurnCCW" and p == "TurnCW")
            ):
                stack.pop()
            else:
                stack.append(p)

        changed = True
        while changed:
            changed = False
            i = 0
            while i <= len(stack) - 3:
                tri = stack[i:i + 3]
                if tri == ["TurnCW", "TurnCW", "TurnCW"]:
                    stack[i:i + 3] = ["TurnCCW"]
                    changed = True
                elif tri == ["TurnCCW", "TurnCCW", "TurnCCW"]:
                    stack[i:i + 3] = ["TurnCW"]
                    changed = True
                else:
                    i += 1

        return stack

    def refine_plan(self, action_list):
        """
        Perform downward refinement to convert the high level plan into a low level executable plan.
        """
        current_state = self.helper.get_initial_state()
        actions = []

        for high_level_action in action_list:
            # This step performs downward refinement.
            if high_level_action[0] == "move":
                target_loc = high_level_action[3]
                candidates = list(self.get_load_locations(target_loc))
                if not candidates:
                    raise ValueError(f"No load locations known for '{target_loc}'")

                ll_actions, reached_state, ok = self.get_path(current_state, candidates)
                if not ok:
                    cs = current_state
                    raise RuntimeError(
                        f"Downward refinement failed: cannot reach {target_loc} from "
                        f"({cs.x},{cs.y},{cs.orientation})"
                    )

                ll_actions = self._simplify_primitives(ll_actions)

                if ll_actions:
                    actions.append(("move", ll_actions, reached_state.get_gazebo_repr()))

                current_state = reached_state

            elif high_level_action[0] == "pick":
                obj = high_level_action[1]
                actions.append(("pick", obj))
                self.helper.remove_edge(obj)

            elif high_level_action[0] == "place":
                obj = high_level_action[1]
                target = high_level_action[3]
                actions.append(("place", obj, target))

        return actions

    # --------------- HELPER FUNCTIONS --------------- #

    def is_goal_state(self, current_state, goal_state):
        """
        Checks if the current_state is goal_state or not.
        """
        return (
            current_state.x == goal_state.x
            and current_state.y == goal_state.y
            and current_state.orientation == goal_state.orientation
        )

    def get_manhattan_distance(self, from_state, to_state):
        """
        Returns the manhattan distance between 2 states.
        """
        return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)

    def build_goal_states(self, locations):
        """
        Creates State representations for given list of locations.
        """
        states = []
        for location in locations:
            states.append(problem.State(location[0], location[1], "EAST"))
            states.append(problem.State(location[0], location[1], "WEST"))
            states.append(problem.State(location[0], location[1], "NORTH"))
            states.append(problem.State(location[0], location[1], "SOUTH"))
        return states

    def get_path(self, init_state, goal_locations):
        """
        Searches for a path from init_state to one of the possible goal_locations.
        """
        final_state = None
        goal_states = self.build_goal_states(goal_locations)

        for goal_state in goal_states:
            action_list, nodes_expanded = search.search(
                init_state, goal_state, self.helper, "astar"
            )
            if action_list is not None:
                return action_list, goal_state, True

        return [], init_state, False


def generate_plan(env, planner_name):
    """
    Run a planner to evaluate problem.pddl and domain.pddl to generate a plan.
    Writes the plan to an output file.
    """
    domain_file_path = utils.DOMAIN_FILEPATH[:-5] + "_" + env + ".pddl"
    action_list = planner.run_planner(
        planner_name, domain_file_path, utils.PROBLEM_FILEPATH
    )
    return action_list


if __name__ == "__main__":
    random.seed(0xDEADC0DE)

    args = parse_args()
    roscore_process = initialize_ros()
    server_process = initialize_planning_server()

    # Generate the world.
    generate_maze(args.objtypes, args.objcount, args.seed, args.env)

    if args.generate_only:
        sys.exit(0)

    FILE_PATH = utils.ROOT_PATH + "/" + args.file_name
    if os.path.exists(FILE_PATH) and args.clean:
        assert not os.path.isdir(FILE_PATH)
        os.remove(FILE_PATH)

    if os.path.exists(FILE_PATH):
        assert not os.path.isdir(FILE_PATH)
        file_handle = open(FILE_PATH, "a")
    else:
        file_handle = open(FILE_PATH, "w")
        file_handle.write(
            "Object_types; object_count; seed; exception; refined_plan; env\n"
        )

    try:
        # Call the external planner for finding a high level plan.
        action_list = generate_plan(args.env, args.planner)

        # Perform downward refinement.
        refinement = Refine(action_list)

        file_handle.write(
            "%u; %u; %u; %s; %s; %s\n"
            % (
                args.objtypes,
                args.objcount,
                args.seed,
                None,
                str(refinement.actions_queue),
                args.env,
            )
        )
    except Exception as e:
        print("Exception caught!")
        file_handle.write(
            "%u; %u; %u; %s; %s; %s\n"
            % (args.objtypes, args.objcount, args.seed, type(e), [], args.env)
        )
        traceback.print_exc()

    # Cleanup ROS core.
    cleanup_ros(roscore_process.pid, server_process.pid)

