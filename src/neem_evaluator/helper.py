import numpy as np


def transform_to_list(transform):
    """
    Extracts the xyz and xyzw values from a transform and returns them as two lists

    :param transform: The transform which should be unpacked
    :return: Two lists for position and orientation
    """
    translation = [transform["translation"]["x"],
                    transform["translation"]["y"],
                    transform["translation"]["z"]]
    rotation = [transform["rotation"]["x"],
                transform["rotation"]["y"],
                transform["rotation"]["z"],
                transform["rotation"]["w"]]
    return [translation, rotation]


def transform_to_translation(transform):
    """
    Returns only the translation of the given transform as a list

    :param transform: The transform which should be unpacked
    :return: A list representing the translation
    """
    return transform_to_list(transform)[0]


def transform_to_rotation(transform):
    """
    Extracts the orientation of the given transformation and returns it as a list

    :param transform: The transform which should be unpacked
    :return: A list representing the orientation of the given transform
    """
    return transform_to_list(transform)[1]


def next_n(coll, n: int):
    """
    Returns the next n elements from the collection coll
    """
    result = []
    i = 0
    while i < n and (s := next(coll, False)):
        result.append(s)
        i += 1
    return result


def chunks(coll, n):
    """
    Generates chunks of size n from the given collection.

    :param coll: Cursor to a mongo collection
    :param n: The size of chunks
    :yield: Chunks from the collection
    """
    while chunk := next_n(coll, n):
        yield chunk


def docs_in_cursor(cursor):
    """
    Counts the amount of elements in a cursor.

    :param cursor: The cursor which should be counted
    :return: The number of elements
    """
    coll = cursor.collection
    return coll.count_documents(cursor.explain()["executionStats"]["executionStages"]["filter"])


def tfs_to_velocity(coll, chunks=1):
    """
    Creates the velocities from a collection of TFs, the number of calulated velocities is number_of_tfs - 1

    :param coll: The collection from which to calculate the velocities
    :param chunks: Number velocity vectors that should be returned
    :return: A generator that returns velocities with chunk size
    """
    velocities = []
    tfs = []
    for i in range(chunks+1):
        tfs.append(coll.next())

    while coll.alive:
        for i in range(len(tfs) - 1):
            first_vector = np.array(transform_to_translation(tfs[i]["transform"]))
            second_vector = np.array(transform_to_translation(tfs[i + 1]["transform"]))
            velocities.append((second_vector - first_vector, tfs[i]["header"]["seq"]))
        yield velocities
        velocities = []
        tfs.remove(tfs[0])
        tfs.append(coll.next())


def cluster_sequences(sequences):
    """
    Creates clusters of sequences, a cluster is any number of sequences with a distance with 1.

    :param sequences: A list of sequences
    :return: A list of clusters
    """
    cluster = (np.diff(sequences)>1).nonzero()[0] + 1
    result = []
    for i in range(len(cluster) - 1):
        result.append([sequences[cluster[i]: cluster[i+1]]][0])
    return result


def cluster_to_actions(cluster, seq_to_actions):
    """
    Matches clusters of sequences to actions that happened during the sequence.

    :param cluster: List with cluster of sequences
    :param seq_to_actions: Mapping of sequences to actions
    :return: A dictionary mapping individual sequences to action
    """
    result = {}
    i = 0
    for cls in cluster:
        try:
            mapping = [seq_to_actions[x] for x in cls]
            result[i] = mapping
        except KeyError:
            pass
        i+=1
    return result
