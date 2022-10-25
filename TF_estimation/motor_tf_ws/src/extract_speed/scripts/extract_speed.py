#!/usr/bin/env python3

import rospy
import csv
from std_msgs.msg import String
from time import time
import os

folderName = "motor_data"
folderPath = f"{os.path.expanduser('~')}/{folderName}/"
if folderName not in os.listdir():
    os.mkdir(folderPath) # create folder in ~/motor_data/ named after current time

def writeData(msg: String, fileName: str):
    with open(f"{folderPath + fileName}.csv", 'a') as csvFile:
        csvWriter = csv.writer(csvFile)
        csvWriter.writerow(msg.data.split(','))

def main():
    rospy.init_node("my_master_node", anonymous=True)

    rospy.Subscriber("/motor_data", String, writeData, f"m{time()}")
    # rospy.Subscriber("/speeds/A", String, writeData, f"A+{time()}")
    # rospy.Subscriber("/speeds/B", String, writeData, f"B+{time()}")
    # rospy.Subscriber("/speeds/C", String, writeData, f"C+{time()}")
    # rospy.Subscriber("/speeds/D", String, writeData, f"D+{time()}")

    rospy.spin()

if __name__ == '__main__':
    main()
