#!/usr/bin/env python3
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Abhyudaya Srinet", "Rushang Karia", "Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import os
USE_QUANTIFIED_GOAL = os.environ.get("HW2_QUANTIFIED", "0") == "1"
def get_goal_string(object_dict, obj_list, obj_loc_list, goal_list,
                    goal_loc_list, env):
    """
    Returns
    ========
        str:
            A generic goal condition that will place every object based on
            its type and size at the correct goal.
    """

    # Append your goal condition to this string.
    #
    # Below are a few hints to help you along the way:
    # =================================================
    #
    # You can print the parameters to help you in coming up with a generalized
    # goal condition.
    #
    # You can also look at the (:objects) and (:init) of problem.pddl to give
    # you an idea of where your goal string is going wrong.
    #
    # Keep in mind the subject type and sizes of the bins and the books must
    # match.
    #
    # High level plans do not need actual co-ordinates, rather they use the
    # high-level locations found in the parameters of this method.
    #
    # Finally, there exists a way to write the goal condition using universal
    # and existential quantifiers.
    #
    # Executing wrong plans on Gazebo might not give you the right execution!
    #
    # Remember that the two environments have unique objects, goals and predicates!

    # Helper to read possibly-variant keys robustly
    USE_QUANTIFIED_GOAL = True  # set False to revert to conjunctive goals

    def _pddl_atom(x):
        return "_".join(str(x).split())

    goal_string = "(:goal "

    # ---------- Conjunctive STRIPS (fallback) ----------
    if not USE_QUANTIFIED_GOAL:
        goal_string += "(and\n"

        if env == "bookWorld":
            for obj, goal, loc in zip(obj_list, goal_list, goal_loc_list):
                oinfo = object_dict["object"][obj]
                ginfo = object_dict["goal"][goal]
                if oinfo["size"] == ginfo["size"] and oinfo["obj_type"] == ginfo["obj_type"]:
                    subj = _pddl_atom(ginfo["obj_type"])
                    size = ginfo["size"]
                    goal_string += (
                        f"  (Book_At {obj} {loc})\n"
                        f"  (Bin_At {goal} {loc})\n"
                        f"  (Book_Subject {obj} {subj})\n"
                        f"  (Bin_Subject {goal} {subj})\n"
                        f"  (Book_Size {obj} {size})\n"
                        f"  (Bin_Size {goal} {size})\n"
                        f"  (not (In_Basket {obj}))\n"
                    )

        elif env == "cafeWorld":
            for obj, goal, loc in zip(obj_list, goal_list, goal_loc_list):
                oinfo = object_dict["object"][obj]
                ginfo = object_dict["goal"][goal]
                if oinfo["size"] == ginfo["size"] and oinfo["obj_type"] == ginfo["obj_type"]:
                    ftype = _pddl_atom(ginfo["obj_type"])
                    size = ginfo["size"]
                    goal_string += (
                        f"  (Food_At {obj} {loc})\n"
                        f"  (Table_At {goal} {loc})\n"
                        f"  (Ordered {goal} {ftype})\n"
                        f"  (Portion_Size {obj} {size})\n"
                        f"  (Food_of_Type {obj} {ftype})\n"
                        f"  (Ordered_Portion {goal} {size})\n"
                        f"  (not (In_Basket {obj}))\n"
                    )

        goal_string += "  (Empty_Basket tbot3)\n"
        goal_string += "))\n"
        return goal_string

    # ---------- Quantified goals (extra credit) ----------
    if env == "bookWorld":
        goal_string += """
(and
  (forall (?b - book)
    (exists (?n - bin ?l - location ?s - subject ?z - size)
      (and
        (Book_Subject ?b ?s)
        (Book_Size ?b ?z)
        (Bin_Subject ?n ?s)
        (Bin_Size ?n ?z)
        (Bin_At ?n ?l)
        (Book_At ?b ?l)
        (not (In_Basket ?b))
      )
    )
  )
  (Empty_Basket tbot3)
)
"""
    elif env == "cafeWorld":
        goal_string += """
(and
  (forall (?f - food)
    (exists (?t - table ?l - location ?k - food_type ?z - size)
      (and
        (Food_of_Type ?f ?k)
        (Portion_Size ?f ?z)
        (Ordered ?t ?k)
        (Ordered_Portion ?t ?z)
        (Table_At ?t ?l)
        (Food_At ?f ?l)
        (not (In_Basket ?f))
      )
    )
  )
  (Empty_Basket tbot3)
)
"""

    goal_string += ")\n"
    return goal_string



def sample_goal_condition(object_dict, obj_list, obj_loc_list, goal_list, 
    goal_loc_list):
    """
        Returns
        ========
            str:
                A generic goal condition that moves the robot to any one of
                the object locations.

        
        .. note ::

            You can replace the contents of get_goal_string() with the text below
            to get an idea of what is expected.
            
            The goal condition in the stock task here is VASTLY different from the
            expectation from you. Please review the homework documentation to identify
            your task.
            
            Here are some instructions to run this in Gazebo.
            1. Replace the content of get_goal_string() with this method.
            2. rosrun hw2 refinement.py \
                --objtypes <object types> \
                --objcount <number of objects> \
                --seed <seed>
            3. rosrun hw2 gazebo.py

            The generic goal condition here is to move the robot to a object location.
            
            The stock task below generates a generic goal condition that moves the
            robot to a random object location and this is independent of the total 
            number of locations and objects. 
            
    """

    import random
    assert len(obj_loc_list) > 0
    i = random.randint(0, len(obj_loc_list) - 1)
    
    goal_string = "(:goal (and "
    goal_string += "(Robot_At tbot3 %s)" % (obj_loc_list[i])
    goal_string += "))\n"
    
    return goal_string
