#!/usr/bin/env python
# coding: utf-8
import bagpy
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math
import statistics

#reading the rosbag files
bag_1 = bagreader(r'C:\Users\AVISH\OneDrive\Desktop\Data_Lab3\LocationB.bag')
bag_2 = bagreader(r'C:\Users\AVISH\OneDrive\Desktop\Data_Lab3\Individual.bag')
bag_3 = bagreader(r'C:\Users\AVISH\OneDrive\Desktop\Data_Lab3\Group.bag')
bag_4 = bagreader(r'C:\Users\AVISH\OneDrive\Desktop\Data_Lab3\LocationA.bag')
bag_5 = bagreader(r'C:\Users\AVISH\OneDrive\Desktop\Data_Lab3\LocationC.bag')
bag_6 = bagreader(r'C:\Users\AVISH\OneDrive\Desktop\Data_Lab3\LocationD.bag')

#Reading the messages by topic name
message_1 = bag_1.message_by_topic('/vectornav')
message_2 = bag_2.message_by_topic('/imu')
message_3 = bag_3.message_by_topic('/imu')
message_4 = bag_4.message_by_topic('/vectornav')
message_5 = bag_5.message_by_topic('/vectornav')
message_6 = bag_6.message_by_topic('/vectornav')

#Convert to CSV file
data_1 = pd.read_csv(message_1)
data_2 = pd.read_csv(message_2)
data_3 = pd.read_csv(message_3)
data_4 = pd.read_csv(message_4)
data_5 = pd.read_csv(message_5)
data_6 = pd.read_csv(message_6)
