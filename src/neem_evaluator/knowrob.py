from typing import Dict, List, Callable

import rospy
import rosservice
from .mongo import restore, tf, get_tf_for_object
from datetime import datetime

try:
    from rosprolog_client import Prolog
    # prolog = Prolog()
except ImportError:
    rospy.logerr("No Knowrob services found")

rospy.init_node("NEEM_Evaluator")

prolog = None


def init_prolog(func: Callable) -> Callable:
    """
    Decorator for initializing the prolog client if it is not already initialized.

    :param func: Function that should be decorated
    :return: The decorated function
    """

    def wrapper(*args, **kwargs):
        global prolog
        if not prolog:
            prolog = Prolog()
        rospy.wait_for_service("/rosprolog/query")
        return func(*args, **kwargs)

    return wrapper


@init_prolog
def remember_neem(path: str):
    """
    Loads the neem in the given path to knowrob

    :param path: Path to the neem
    :return: None
    """
    # Loads the NEEM into Knowrob
    prolog.once(f"remember('{path}')")
    # Is needed since Knowrob does not load the TF data to Mongo and without it
    # the queries do not work
    # restore(path)


@init_prolog
def get_event_intervals() -> Dict[str, Dict[str, int]]:
    """
    Returns the start and end times for all actions

    :return: A dict with action as key and start and end times as value
    """
    intervals = prolog.once("findall([Begin, End, Evt], event_interval(Evt, Begin, End), Times)")
    result = {}
    for interval in intervals["Times"]:
        result[interval[2]] = {"start": interval[0], "end": interval[1]}
        # result.append({"event": interval[2], "start": interval[0], "end": interval[1]})
    return result


@init_prolog
def get_all_actions_in_neem() -> Dict[str, List]:
    """
    Returns a dictionary which maps all actions happening in this NEEM to the human-readable name of the action.

    :return: A dict of action instances to human-readable action names
    """
    actions = prolog.once("findall([Act, Task], (is_action(Act), executes_task(Act, Task)), Act)")["Act"]
    res = {}
    for act in actions:
        # res[act[0]] = act[1]
        action_type = act[1].split('#')[1].split("_")[0]
        if action_type in res.keys():
            res[action_type].append(act[0])
        else:
            res[action_type] = [act[0]]
    return res


@init_prolog
def get_objects_for_action(action: str) -> List[str]:
    """
    Returns a list of objects which are part of the given action.

    :param action: The action for which objects should be returned
    :return: An action instance
    """
    mapping = prolog.once("findall([Act, Obj], has_participant(Act, Obj), Obj)")
    act_to_obj = {}
    for m in mapping["Obj"]:
        if m[0] not in act_to_obj.keys():
            act_to_obj[m[0]] = [m[1]]
        else:
            act_to_obj[m[0]].append(m[1])
    return act_to_obj[action]


@init_prolog
def get_actions_for_object() -> Dict[str, List[str]]:
    """
    Returns a dictionary which maps all available knowrob objetct instances to the actions which they are part of.

    :return: A dict mapping objects to a list of actions
    """
    all_actions = get_all_actions_in_neem()
    action_to_objects = {}
    object_to_actions = {}
    for act in all_actions.keys():
        action_to_objects[act] = get_objects_for_action(act)
    for act, objs in action_to_objects.items():
        for obj in objs:
            if obj not in object_to_actions.keys():
                object_to_actions[obj] = [act]
            if act not in object_to_actions[obj]:
                object_to_actions[obj].append(act)
    return object_to_actions


@init_prolog
def get_link_name_for_object(object: str) -> str:
    """
    Returns the link name for a knowrob object instance.

    :param object: The knowrob instance
    :return:  The link name as used in the TFs
    """
    query = prolog.once(f"has_base_link_name('{object}', A)")
    if query:
        link_name = query["A"]
        return query["A"]
        # return link_name.split("#")[1]
    else:
        query = prolog.once(f"has_base_link('{object}', A)")



@init_prolog
def get_all_tf_for_action(action: str) -> Dict[str, Dict]:
    """
    Get all TFs for objects that are part of the given action.

    :param action: Action as knowrob instance
    :return: A dictionary which maps the object that are part of the given action to a TF cursor
    """
    intervals = get_event_intervals()[action]
    objects = get_objects_for_action(action)
    result = {}
    for obj in objects:
        link_name = get_link_name_for_object(obj)
        if link_name:
            # -3600 is to compensate for difference in intervals of knowrob and TFs, this is a problem between different
            # formats the MongoDB uses datetime and Knowrob uses unix time stamps. While datetime takes the timezone and
            # daylight saving time into account, Knowrob does not which creates a difference of 3600 or
            # 7200 seconds (1 or 2 h) depending on summer or winter time.
            result[obj] = tf.find({"header.stamp": {"$gt": datetime.fromtimestamp(intervals["start"] - 3600),
                                                    "$lt": datetime.fromtimestamp(intervals["end"] - 3600)},
                                   "child_frame_id": link_name})
        else:
            continue

    return result


@init_prolog
def map_sequence_to_action(object: str) -> Dict[int, str]:
    """
    Maps the sequences of the trajectory of the given object to the action that occurred during that sequence. The object
    has to given as a knowrob object. The returned action is readable and not the specific instance.

    :param object: The knowrob object for which the trajectory should be mapped
    :return: A dictionary with the sequence as key and the Action as value.
    """
    all_actions = get_all_actions_in_neem()
    o_t_a = get_actions_for_object()
    link_names = map(get_link_name_for_object, o_t_a.keys())
    obj_to_link = dict(zip(o_t_a.keys(), link_names))
    action_intervals = get_event_intervals()
    seqs_to_action = {}

    for act in o_t_a[object]:
        intervals = action_intervals[act]
        tfs = tf.find({"header.stamp": {"$gt": datetime.fromtimestamp(intervals["start"] - 3600),
                                        "$lt": datetime.fromtimestamp(intervals["end"] - 3600)},
                       "child_frame_id": obj_to_link[object]})
        seqs = map(lambda x: x["header"]["seq"], tfs)
        for s in seqs:
            seqs_to_action[s] = all_actions[act]
    return seqs_to_action


@init_prolog
def get_object_for_link(link_name: str) -> str:
    """
    Returns the knowrob object name for a link/TF name

    :param link_name: The name of the link
    :return: The corresponding name in knowrob
    """
    all_objects = list(get_actions_for_object().keys())
    link_to_obj = {}
    for obj in all_objects:
        link_to_obj[get_link_name_for_object(obj)] = obj
    return link_to_obj[link_name]
