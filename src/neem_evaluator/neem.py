# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from .knowrob import *
from .mongo import *
from .helper import time_ordered_actions
from typing import Dict, List, Iterable, Union
from bson.objectid import ObjectId
from datetime import datetime
from pytz import UTC

import json
import copy
import rospy


class NeemObject:

    def __init__(self, name: str, instance: str, link_name: str = None, tf_list: List = None, action: Action = None):
        """
        Representing an object that is part of a NEEM

        :param name: A human-readable name
        :param instance: The unique instance of the ontology
        """
        self.name: str = name
        self.instance: str = instance
        self._tf = None
        self._tf_list: List = []
        # Special case since NaturalPerson has no link name and restoring from json would otherwise try to use
        # get_link_name_for_object which would fail
        if name == instance:
            self.link_name = link_name
        else:
            self.link_name: str = link_name if link_name else get_link_name_for_object(self.instance)

        if tf_list:
            self._tf_list = tf_list
        else:
            self._load_tf()
        self.action = action
        self._calculate_start()

    def __repr__(self) -> str:
        """
        String representation of this Object, used when printing this object.

        :return: A string with the most important attributes of this object
        """
        skip_attributes = ["_tf", "_tf_list", "action"]
        return self.__class__.__qualname__ + f"(" + ', '.join(
            [f"{key}={value}" if key not in skip_attributes else f"{key}=..." for key, value in
             self.__dict__.items()]) + ")" + "\n"

    def _load_tf(self) -> None:
        """
        Load TF cursor for this object from the Mongo DB.
        """
        if self.link_name:
            self._tf = get_tf_for_object(self.link_name)
        else:
            rospy.loginfo(f"NEEM Object: {self.name} has no tf_link_name therefore no tf pointer could be loaded")

    def get_tfs(self) -> Iterable:
        """
        Method to get the TF generator for this NEEMObject, for a NEEM that is currently in knowrob this is a MongoDB
        cursor. For a NEEM from a json file this is a Python Generator. Both of these behave like a generator object.

        :return: A generator for the TF frames of this NEEMObject
        """
        if self._tf:
            return copy.copy(self._tf)
        else:
            # return lambda x: for t in self._tf_list: yield t
            return (t for t in self._tf_list)
        # else:
        #     return []

    def __iter__(self):
        pass

    def _json_to_mongo(self) -> None:
        """
        Converts the datatypes which are used in a json file to the ones used in Mongo for consistency.
        """
        for tf in self._tf_list:
            tf["_id"] = ObjectId(tf["_id"])
            tf["header"]["stamp"] = datetime.fromisoformat(tf["header"]["stamp"])
            tf["__recorded"] = datetime.fromisoformat(tf["__recorded"])

    def _clean_tf_for_json(self) -> Iterable:
        """
        Converts the datatypes used in MongoDB to ones used in json to save this Object as a json.

        :yield: A generator for the cleaned TFs
        """
        for tf in self.get_tfs():
            tf["_id"] = str(tf["_id"])
            tf["header"]["stamp"] = str(tf["header"]["stamp"])
            tf["__recorded"] = str(tf["__recorded"])
            yield tf

    def to_json(self) -> Dict:
        """
        Converts this Object to a dictionary for serialization in json

        :return: A dictionary with all information in this Object
        """
        obj_json = {"name": self.name,
                    "instance": self.instance,
                    "link_name": self.link_name,
                    "tf_list": self._tf_list if self._tf_list else list(self._clean_tf_for_json())}

        return obj_json

    def get_tfs_during_action(self, action) -> Iterable:
        """
        Returns the TFs for this Object during an action. This method also takes the offset between TFs and actions into
        account.

        :param action: Action during which the TF should be returned
        :return: An iterable for the TFs during the action
        """
        if self._tf:
            return list(tf.find({"header.stamp": {"$gt": datetime.fromtimestamp(action.start, UTC),
                                                  "$lt": datetime.fromtimestamp(action.end, UTC)},
                        "child_frame_id": self.link_name}))
        else:
            start_index = 0
            end_index = 0
            i = 0
            if not self._tf_list:
                return []
            for transform in self._tf_list:
                if not start_index and datetime.fromisoformat(transform["header"]["stamp"]).timestamp() >= action.start:
                    start_index = i
                if action.end <= datetime.fromisoformat(transform["header"]["stamp"]).timestamp():
                    end_index = i

                i += 1
            return self._tf_list[start_index: end_index + 1]

    def _calculate_start(self):
        """
        Calculates the start time for this NEEM. This is the time of the first TF.
        """
        if first_tf := next(self.get_tfs(), False):
            # first_tf = next(self.get_tfs())
            stamp = first_tf["header"]["stamp"]
            if isinstance(stamp, str):
                stamp = datetime.fromisoformat(stamp)
            self.start = stamp.timestamp()
        else:
            self.start = 0


class Action:
    """
    Represents an action during a NEEM. An action consists of a number of informations, these are:

        * A human-readable name
        * The knowrob instance name
        * A start time (as timestamp)
        * An end time (as timestamp)
        * A relative start time, relative to start of NEEM
        * A relative end time, relative to start of NEEM
        * A list of participants
        * A list of Objects
        * A reference to the NEEM
    """

    def __init__(self, name: str, instance: str, start: int = None, end: int = None, objects: List[NeemObject] = None,
                 neem: Neem = None):
        self.name: str = name
        self.instance: str = instance
        self.start: int = 0
        self.end: int = 0
        # Relative attributes are set by parent NEEM
        self.relative_start = 0
        self.relative_end = 0
        self.participants: List = []
        self.objects: List[NeemObject] = []

        if start and end:
            self.start = start
            self.end = end
        else:
            self._load_intervals()

        if objects:
            self.objects = objects
        else:
            self._load_objects()

        self.neem = neem
        for obj in self.objects:
            obj.action = self

    def __repr__(self) -> str:
        """
        String representation of this Action, used when printing this object.

        :return: A string of the most important attributes of this Actiont
        """
        return f"Action(name={self.name}, start={self.start}, end={self.end})"
        skip_attributes = ["participants", "objects", ]
        return self.__class__.__qualname__ + f"(" + ', '.join(
            [f"{key}={value}" if key not in skip_attributes else f"{key}=..." for key, value in
             self.__dict__.items()]) + ")"

    def _load_intervals(self) -> None:
        """
        Loads the start and end time for this action.
        """
        intervals = get_event_intervals()
        self.start = intervals[self.instance]['start']
        self.end = intervals[self.instance]['end']

    def _load_objects(self) -> None:
        """
        Loads all objects that are participating in this action.
        """
        objects = get_objects_for_action(self.instance)
        for obj in objects:
            if '#' in obj:
                name = obj.split('#')[1]
            else:
                name = obj
            self.objects.append(NeemObject(name, obj, action=self))

    def get_all_objects_for_action(self) -> List[NeemObject]:
        """
        Returns all objects that are participating in this action
        """
        return self.objects

    def to_json(self) -> Dict:
        """
        Creates a dictionary for this Action for the serialization as json

        :return: A dictionary with all information in this Action
        """
        action_json = {"name": self.name,
                       "instance": self.instance,
                       "start": self.start,
                       "end": self.end,
                       "participants": self.participants,
                       "objects": [obj.to_json() for obj in self.objects]}
        return action_json


class Neem:
    """
    Representation of a whole NEEM. This class contains all relevant information of a NEEM, these are:

        * A name
        * A list of all actions
        * A start time
        * An end time
        * An action tf time offset.
    """

    def __init__(self, json_path: str = None):
        if json_path:
            with open(json_path, "r") as file:
                self.from_json(file.read())
        else:
            self.name: str = None
            self.actions: Dict[str, List[Action]] = {}
            self.action_list: List[Action] = []
            rospy.logdebug("Loading from Knowrob")
            self._load_actions()
        self.action_list: List[Action] = time_ordered_actions(self.action_list)
        self.start: int = self.action_list[0].start
        self.end: int = self.action_list[-1].end

        self._set_relative_times()
        self._try_correcting_tf_links()

        #self._calculate_action_tf_offset()

    def _try_correcting_tf_links(self):
        """
        Tries to find matches for missing or not correctly mateched tf_links.
        This is done by checking if there is a TF in mongo which is a subset of the
        name of the NeemObject, this is ussully the case if unreal appended a _1.
        """
        no_link = list(filter(lambda x: not x.link_name, self.get_all_objects_in_neem()))
        for collection_link_name in get_all_values_for_key("child_frame_id"):
            for obj in no_link:
                if collection_link_name in obj.name:
                    obj.link_name = collection_link_name
                    obj._load_tf()

    def _calculate_action_tf_offset(self):
        """
        Calculates the time difference between the first action and the first TF in this neem. The offset is used to get
        trajectories for objects during actions.
        """
        print(self.action_list[1].objects[1])
        first_tf = list(self.action_list[0].objects[1].get_tfs())[0]
        if type(first_tf["header"]["stamp"]) == datetime:
            first_tf_timestamp = first_tf["header"]["stamp"].timestamp()
        else:
            first_tf_timestamp = datetime.fromisoformat(first_tf["header"]["stamp"]).timestamp()
        self.action_tf_offset = abs(self.start - first_tf_timestamp)

    def _load_actions(self) -> None:
        """
        Gets the actions for this NEEM from knowrob and creates instances of Action classes for them
        """
        actions = get_all_actions_in_neem()
        for name, action_instances in actions.items():
            self.actions[name] = []
            for act_ins in action_instances:
                self.actions[name].append(Action(name, act_ins, neem=self))
                self.action_list.append(Action(name, act_ins, neem=self))

    def _set_relative_times(self):
        """
        Calculates the relative time for each Action. The realative time is the start and end time for an Action
        relative to the NEEM start.
        """
        for act in self.action_list:
            act.relative_start = act.start - self.start
            act.relative_end = act.end - self.start

    def get_all_objects_in_neem(self) -> List[NeemObject]:
        """
        Returns a list with all objects in this NEEM

        :return: A list with the unique objects in this neem
        """
        objects = set()
        instances = []
        for act in self.action_list:
            for obj in act.objects:
                if obj.instance not in instances:
                    objects.add(obj)
                    instances.append(obj.instance)
        return list(objects)

    def get_objects_by_name(self, name: str) -> Union[NeemObject, List[NeemObject]]:
        """
        Finds all objects that fit the given name and returns it. If there is only one object with the name the single
        object will be returned, if more than one object has this name a list of all objects will be returned.

        :param name: Name of the object that should be found
        :return: The object with the name if there is only one and a list of all objects with the name if there are more
        """
        fitting_object = list(filter(lambda obj: obj.name == name, self.get_all_objects_in_neem()))
        if len(fitting_object) == 1:
            return fitting_object[0]
        else:
            return fitting_object

    def get_all_actions_in_neem(self) -> List[Action]:
        """
        Returns a list with all actions in this NEEM

        :return: All actions in this neem
        """
        return self.action_list

    def to_json(self) -> Dict:
        """
        Creates a dictionary of this NEEM for serialization in json.

        :return: A dictionary containing all information of this neem
        """
        neem_json = {"name": self.name,
                     "actions": [act.to_json() for act in self.action_list]}

        return neem_json

    def from_json(self, neem_json: str) -> None:
        """
        Creates an instance of the NEEM class from a json file

        :param neem_json: Path to the json containing the neem infos
        """
        d = json.loads(neem_json)
        self.name = d["name"]
        self.action_list = []
        self.actions = {}
        for action in d["actions"]:
            objects = [NeemObject(obj["name"], obj["instance"], obj["link_name"], obj["tf_list"]) for obj in
                       action["objects"]]
            self.action_list.append(
                Action(action["name"], action["instance"], action["start"], action["end"], objects, neem=self))
            if action["name"] not in self.actions.keys():
                self.actions[action["name"]] = [Action(action["name"], action["instance"], action["start"], action["end"], objects, neem=self)]
            else:
                self.actions[action["name"]].append(Action(action["name"], action["instance"], action["start"], action["end"], objects, neem=self))

    def save(self, path: str) -> None:
        """
        Saves this NEEM as a json file.

        :param path: Path to where the neem should be saved.
        """
        neem_json = json.dumps(self.to_json(), indent=4, separators=(", ", ": "))
        with open(path, "w") as file:
            file.write(neem_json)
