import os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import pandas as pd
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3

# Define the current working directory
CURRENT_FILE = os.getcwd()

# List of relative file paths to be processed (without the .bag extension)
# Example: file_paths = ['file1', 'file2', 'file3']
file_paths = []

# List of topics to extract from the ROS bag file
# Example: topics = ['/objectposes', '/person_state_estimation/person_states', '/qualisys/pedestrian/pose', '/qualisys/pedestrian/velocity', '/qualisys/tinman/pose', '/qualisys/tinman/velocity']
topics = []

def create_dir(directory):
    """
    Create a directory if it doesn't exist.
    
    Args:
        directory (str): The directory path to create.

    Returns:
        int: 0 if the directory was created successfully. Otherwise, exits with -1.
    """
    try:
        if not os.path.exists(directory):
            os.makedirs(directory)
        return 0
    except Exception as err:
        print(f"Error creating directory: {err}")
        exit(-1)

def flatten_dict(d, parent_key='', sep='.'):
    """
    Flatten a nested dictionary.

    Args:
        d (dict): The dictionary to flatten.
        parent_key (str, optional): The base key to prepend. Defaults to ''.
        sep (str, optional): Separator to use between keys. Defaults to '.'.

    Returns:
        dict: The flattened dictionary.
    """
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        elif isinstance(v, list):
            for i, item in enumerate(v):
                if isinstance(item, dict):
                    items.extend(flatten_dict(item, f"{new_key}{sep}{i}", sep=sep).items())
                else:
                    items.append((f"{new_key}{sep}{i}", item))
        else:
            items.append((new_key, v))
    return dict(items)

def process_data(data):
    """
    Process the data to handle lists of dictionaries by flattening the structure.

    Args:
        data (dict): The dictionary data to process.

    Returns:
        list[dict]: A list of processed dictionaries.
    """
    list_keys = [key for key, value in data.items() if (len(value) >0 and isinstance(value, list)) and all(isinstance(item, dict) for item in value)]
    if len(list_keys) == 0:
        return [flatten_dict(data)] # Return original data if no suitable list is found

    res = []
    for key in list_keys:
        for item in data[key]:
            new_row = data.copy()
            new_row.update({key: item})
            res.append(flatten_dict(new_row))
    return res

def save_data(results_path, data_path, topics):
    """
    Save data from a ROS bag to CSV files.

    Args:
        results_path (str): The directory path where CSV files will be saved.
        data_path (str): The file path of the ROS bag directory.
    """
    reader = SequentialReader()
    storage_options = StorageOptions(uri=data_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    
    create_dir(results_path)
    datasets = {}

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic in topics:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            data_dict = message_to_ordereddict(msg)
            processed_data = process_data(data_dict)

            if topic not in datasets:
                datasets[topic] = pd.DataFrame(processed_data)
            else:
                datasets[topic] = pd.concat([datasets[topic], pd.DataFrame(processed_data)], ignore_index=True)

    reader.close()

    for topic, df in datasets.items():
        data_file_path = os.path.join(results_path, topic.replace('/', '_') + '.csv')
        df.to_csv(data_file_path, index=False)


if __name__ == "__main__":
    for file_name in file_paths:
        results_path = os.path.join(CURRENT_FILE, 'results', file_name)
        data_path = os.path.join(CURRENT_FILE, 'data', file_name)
        save_data(results_path, data_path, topics)
