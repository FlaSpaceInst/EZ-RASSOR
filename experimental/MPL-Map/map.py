import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import numpy as np
from matplotlib.colors import Normalize
import sys
sys.path.insert(0, "../../packages/autonomy/ezrassor_swarm_control/source/ezrassor_swarm_control")
from swarm_utils import get_rover_status

#Smaller scale MPL Frontend, currently only works with the "swarm_simulation.launch" file and 3 rovers.
#Additional features can be added such as additional stats, and on-click for a better frontend-interface.

#Polling rate in milliseconds
UPDATE_RATE = 50
#TODO: Grab number of rovers from launch files
NUM_ROVERS = 3

rovers = np.array([[0,0], [0,0], [0,0]])
rovers_battery = [0,0,0]
rovers_status = ["","",""]

fig = plt.figure()
grid = gridspec.GridSpec(3, 6, figure=fig, wspace = 5)
axGraph = plt.subplot(grid[:, 2:])
roverList = plt.subplot(grid[:, :2])
fig.patch.set_facecolor('xkcd:dark grey')

#Redraws the 2d plane map
def updateMap():
    axGraph.clear()
    axGraph.set_title("Rover Location")
    axGraph.set_xlabel("X-Coordinate")
    axGraph.set_ylabel("Y-Coordinate")
    axGraph.patch.set_facecolor('xkcd:grey')
    axGraph.axhline(y=0, color='k', lw = 0.5)
    axGraph.axvline(x=0, color='k', lw = 0.5)
    axGraph.set_xlim([-25,25])
    axGraph.set_ylim([-25,25])
    for i in range(0, NUM_ROVERS):
        axGraph.annotate("EZ_RASSOR_"+str(i+1)+"\n"+rovers_status[i], xy=rovers[i])
    axGraph.scatter(rovers[:,0], rovers[:,1])

#Redraws the battery chart
def updateRoverList():
    roverList.clear()
    roverList.set_title("Rover Battery")
    cm = plt.cm.RdYlGn
    norm = Normalize(vmin=0, vmax=100)
    colors = [cm(norm(rovers_battery[0])), cm(norm(rovers_battery[1])), cm(norm(rovers_battery[2])), ]
    roverList.patch.set_facecolor('xkcd:grey')
    roverList.set_yticks(range(1,4))
    roverList.set_yticklabels(["EZ_RASSOR_3", "EZ_RASSOR_2", "EZ_RASSOR_1"])
    roverList.set_xlabel("Battery Percentage")
    roverList.barh(range(1,4), rovers_battery[::-1], color=colors)

#Calls get_rover_status and updates everything
def update(h):
    for i in range(0, NUM_ROVERS):
        response = get_rover_status(i+1)
        rovers[i][0] = response.pose.position.x
        rovers[i][1] = response.pose.position.y
        rovers_battery[i] = response.battery
        rovers_status[i] = response.activity
    updateMap()
    updateRoverList()

#Calls update function every UPDATE_RATE ms
graph = animation.FuncAnimation(fig, update, interval=UPDATE_RATE)
plt.show()
