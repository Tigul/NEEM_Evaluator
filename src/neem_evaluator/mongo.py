import pymongo
import os
from typing import List
import bson

client = pymongo.MongoClient('mongodb://localhost:27017/')
db = client["roslog"]
tf = db["tf"]


def get_object_names():
    """
    Returns all link names occuring in this NEEM

    :return: A cursor for all link names
    """
    return tf.distinct("child_frame_id")


def get_tf_for_object(name):
    """
    Returns a TF cursor for all TF that fit the given link name.

    :param name: The link name of the object
    :return: A cursor that points to all TFs fitting the link name
    """
    return tf.find({"child_frame_id": name})

def get_all_values_for_key(key: str) -> List:
    """
    Returns a list of values the given key can have in the TF collection.
    :param key: Key for which all possible values should be found.
    :return: A list of all values
    """
    return tf.distinct(key)

def restore(path: str):
    """
    MongoDB Restore
    >>> DB_BACKUP_DIR = '/path/backups/'
    >>> conn = MongoClient("mongodb://admin:admin@127.0.0.1:27017", authSource="admin")
    >>> db_name = 'my_db'
    >>> restore(DB_BACKUP_DIR, conn, db_name)

    :param path: Database dumped path
    :return:
    """

    #db = conn[db_name]
    if set(['triples', 'tf', 'annotations']) <= set(os.listdir(path)):
        coll_paths = [path + "/" + x + "/roslog/" for x in ['triples', 'tf', 'annotations']]
    else:
        print("The given path is no valid NEEM")
        return
    for coll in coll_paths:
        for file in os.listdir(coll):
            if file.endswith('.bson'):
                with open(os.path.join(coll, file), 'rb+') as f:
                    db[file.split('.')[0]].insert_many(bson.decode_all(f.read()))
    print("Successfully loaded NEEM")


def remove_neem():
    """
    Removes the NEEM from MongoDB.

    :return:
    """
    return client.drop_database("roslog")
