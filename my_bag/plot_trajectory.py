import sqlite3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

DB_PATH = "my_bag.db3"

conn = sqlite3.connect(DB_PATH)

topics = pd.read_sql("SELECT id, name FROM topics", conn)

def get_topic_data(topic_name):
    topic_id = topics[topics["name"] == topic_name]["id"].values[0]
    query = f"""
    SELECT timestamp, data
    FROM messages
    WHERE topic_id = {topic_id}
    """
    return pd.read_sql(query, conn)

odom = get_topic_data("/odom")
odom_noisy = get_topic_data("/odom_noisy")
odom_kf = get_topic_data("/odom_kf")

print("Data loaded successfully.")