from swarm_utils import get_rover_status
from ezrassor_swarm_control.srv import GetRoverStatus
import matplotlib.pyplot as plt
import numpy as np
import time
import math
import rospy


rows, cols = (50, 50)
totalBat = [[0 for i in range(cols)] for j in range(rows)]
avgBat = [[0 for i in range(cols)] for j in range(rows)]
totalTime = [[0 for i in range(cols)] for j in range(rows)]
avgTime = [[0 for i in range(cols)] for j in range(rows)]

rovCount = [[0 for i in range(cols)] for j in range(rows)]
#print(arr)



def updateMap(roverCount):
    num_rovers = roverCount + 1
    #print(costMap)
    currBattery = [0 for x in range(num_rovers)]
    #timePerRover = [0 for x in range(num_rovers)]
    currXPos = [0 for x in range(num_rovers)]
    currYPos = [0 for x in range(num_rovers)]
    timePerChange = [0 for x in range(num_rovers)]

    for i in range(0, num_rovers):
        response = get_rover_status(i+1)
        currBattery[i] = response.battery
        #timePerRover[i] = rospy.get_time()
        currXPos[i] = response.pose.position.x + 25
        currYPos[i] = response.pose.position.y + 25
        timePerChange[i] = time.time()
    

    t_end = time.time() + 15
    while time.time() < t_end:
        for i in range(0, num_rovers):
            response = get_rover_status(i+1)
            temp = response.battery
            task = response.activity
            #timePerChange[i] = rospy.get_time()
            
            #print(task)
            #Don't care about how much it cost to be on the dig sites
            #When they reach the digsite they will dig until they hit the target
            #Need to change it so that the follow if block also does not keep track
            #when the rovers are sitting still trying to find a new path, becasue
            #that also greatly affects the data because if the rover is sitting trying to find a new path
            #it will take up battery, and make the cost map thing that spot takes up a lot of battery
            if task != "digging":
                #so the negative positions, will fit onto the array
                
                x = response.pose.position.x + 25
                y = response.pose.position.y +  25
                if(temp > currBattery[i]):
                    currBattery[i] = temp

                if(temp < currBattery[i]):
                    #currBattery[i] = temp
                    #update the cost map based on how much battery they spent in one location
                    change = currBattery[i] - temp
                    totalBat[x][y] += change
                    #print("change")

                    currBattery[i] = temp  
                # print("X pos")
                # print(x)
                # print(currXPos[i])
                # print(y)
                # print(currYPos[i])
                #print((timePerChange[i] - time.time()))

                if (x != currXPos[i] or y != currYPos[i]) and (( time.time() - timePerChange[i]) > 1):
                    preX = currXPos[i]
                    preY = currYPos[i]
                    timePerChange[i] = time.time()
                    rovCount[preX][preY] += 1
                    #This is just a simple version to get an average battery
                    #Would like to change it so that the time spent on the rover gives it a weight
                    #So that incase a rover turns a corner in a spot it won't bring down the average by a lot
                    #Current issues is that if the rover is inbetween 2 spots it will increment the total rovers
                    #That have visited that spot by a lot which will drive down the average
                    #need to make it so there is a slight delay between incrementing the total rovers
                    #And also find a way to get the rover status if it is checking for a new path to take
                    #Because while it is sitting on that tile it will spin causing it to drive the cost up, and the total rovers
                    #on that title becasue it will slightly move inbetween 2 spots.
                    avgBat[preX][preY] = totalBat[preX][preY] / rovCount[preX][preY]
                    print(rovCount[preX][preY])
                    #timeChange = rospy.get_time() - timePerRover[i]
                    #avgTime[preX][preY] = (totalTime[preX][preY] + timeChange) / rovCount[preX][preY]
                    #totalTime[preX][preY] += timeChange
                    #need to make a function to find out avg battery use, with time being a weight on the average 




    #printing the cost map
    #These should be avgBat, but totalBat is the only thing that currently works
    # for i in range(rows):
    #     for j in range(cols):
    #         if(totalBat[i][j] != 0):
    #             print("[{},{}] = {}".format(i - 25,j - 25,totalBat[i][j]))
    
    # This prints out the graph so you can get a visualization of the cost map_path
    # Currently it is only showing the total cost, the avg cost does not work

    # arr = np.array(totalBat)
    # plt.imshow(arr, cmap='hot', interpolation = 'nearest')
    # plt.draw()
    # plt.show(block=False)
    # plt.pause(5)
    # plt.close()

                


def costMap(roverCount):
    while True:
        updateMap(roverCount)
        #time.sleep(35)


while True:
    updateMap(2)
    #time.sleep(35)

