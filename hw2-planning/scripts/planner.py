import subprocess
import re
import utils
import time
import sys
import os

# Default matches the starter
DEFAULT_FD_SEARCH = os.environ.get(
    "HW2_FD_SEARCH",
    'lazy_greedy([ff()], preferred=[ff()])'
)

def run_planner(planner_name, domain_filepath, problem_filepath,
                log_filepath=utils.PLANNER_LOG_FILEPATH):
    """
    Runs the planner and parses its output.

    Parameters
    ===========
        planner_name: str
            The name of the planner.
        domain_filepath: str
            The domain.pddl filepath.
        problem_filepath: str
            The problem.pddl filepath.
        log_filepath: str
            The path to the planner log file.

    Returns
    ========
        list
            The list of actions as a list of parameterized actions.
            Example: 2 actions move A B, pick B would be returned as
            [[move, A, B], [pick, B]] where all the members are strings.
    """
    if planner_name.lower() == "fd":
        planner = FD()
    else:
        raise Exception("Unsupported Planner!")
        return None

    command = planner.get_planner_command(domain_filepath, problem_filepath)
    log_filehandle = open(log_filepath, "w")

    planner_process = subprocess.Popen(
        command, shell=True,
        stdout=log_filehandle, stderr=log_filehandle
    )

    end_time = time.time() + utils.PLANNER_TIMEOUT_IN_SECS
    while time.time() < end_time and planner_process.returncode is None:
        planner_process.poll()

    try:
        planner_process.terminate()
        planner_process.kill()
    except Exception:
        pass

    log_filehandle.close()

    if time.time() >= end_time:
        raise Exception("Timeout in high level planner!")

    return planner.parse_output(log_filepath)


class Planner:
    def __init__(self):
        pass

    def get_planner_command(self, domain_filepath, problem_filepath, *args, **kwargs):
        """
        Parameters
        ===========
            domain_filepath: str
                The path to the domain.pddl file.
            problem_filepath: str
                The path to the problem.pddl file.

        Returns
        ========
            str
                The string command that can be used to execute the planner
                using subprocess.
        """
      

        command = "python3 "
        command += utils.ROOT_PATH + "/planners/FD/fast-downward.py "
        command += "--plan-file /tmp/plan.txt "
        command += f"{domain_filepath} {problem_filepath} "
        command += f'--search "{DEFAULT_FD_SEARCH}"'
        return command

    def parse_output(self, log_filepath):
        """
        Parameters
        ===========
            log_filepath: str
                The path to the planner output log.

        Returns
        ========
            list(list)
                A list of list of parameterized actions.
        """
        import os
        import re

        plan_path = "/tmp/plan.txt"
        actions = []

        # Preferred: parse the explicit plan file
        if os.path.exists(plan_path):
            with open(plan_path, "r") as pf:
                for raw in pf:
                    line = raw.strip()
                    if not line or line.startswith(";"):
                        continue
                    # Strip trailing " (cost: n)" if present
                    if line.startswith("(") and ")" in line:
                        line = line.split(")")[0] + ")"
                    if line.startswith("(") and line.endswith(")"):
                        inner = line[1:-1].strip()
                        parts = inner.split()
                        if parts:
                            parts[0] = parts[0].lower()
                            actions.append(parts)
            return actions

        # Fallback: scrape the log (your original approach)
        PLAN_REGEX = re.compile(r"(?P<action>(\w|\W)*) \(\d+\)($|\n)")
        with open(log_filepath, "r") as f:
            for line in f:
                m = PLAN_REGEX.match(line)
                if m:
                    action = m.group("action").strip().lower().split()
                    actions.append(action)
        return actions


class FD(Planner, object):
    def __init__(self):
        super(FD, self).__init__()

    def get_planner_command(self, domain_filepath, problem_filepath, *args, **kwargs):
        # Tip: Leave spaces at the end so that the next += does not have to worry about them.
        command = "python3 "
        command += utils.ROOT_PATH + "/planners/FD/fast-downward.py "
        command += "--plan-file /tmp/plan.txt "
        command += f"{domain_filepath} {problem_filepath} "
        command += f'--search "{DEFAULT_FD_SEARCH}"'
        return command

    def parse_output(self, log_filepath):
        PLAN_REGEX = re.compile(r"(?P<action>(\w|\W)*) \(\d+\)($|\n)")

        with open(log_filepath, "r") as log_filehandle:
            action_list = []
            for line in log_filehandle:
                action_match = PLAN_REGEX.match(line)
                if action_match is not None:
                    action = action_match.group("action").strip().lower().split()
                    # We use append() instead of += since we want a list of lists.
                    action_list.append(action)

        return action_list

