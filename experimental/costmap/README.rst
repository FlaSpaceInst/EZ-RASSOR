Cost Maps
==========


Cost map was an idea to have it so when the rover wants to go from
Point A to Point B the rover will build the predicted path, and then 
look at the cost map to see if it can even make it there.

Currently this has only been tested inside the simulation.
To run what is there run 

python cost_map.py

The main part of the code is a 50,50 array that is the current size of the 
running simulations. With each piece of the array representing a 1x1 meter square.
When the rover is driving around the cost map should auto build its self, by 
keeping track of the battery usage over that one tile and then update the array
by telling it how much battery was used.

The main issue the cost map is currenly having is trying to get an average usage for the battery.
What I originally used, which is causing issues, so there must be a better way.
Is to try and track when the rover gets onto a new tile and then leave it when the rover leaves
the tile it will ingrement by 1 for the total number of new rovers that have been on that tile.
What the issues is when a rover is driving inbetween 2 tiles, the response from the simulation
will cause it to flip back and forth between these 2 tiles causing the incrementation processes to happen
very rappidly. Driving the average down a lot. A thought was to try and make it so there has to be
at least a second between when a tile could increment its number to try and delay that processes.
But another issues came up when a rover would get stuck be either seeing a wall or another rover
it would try and find a new path by spinning in place and calculating new paths.
This would cause either that tile it is currently on to go up a lot in terms of "cost"
or it would spin between multiple tiles causing the average to go down because of the incrementing processes.

Need to set up a way so that if a rover is finding a new path it will skip keeping track of battery for the cost map.
And find a way to have it so that when a rover is driving between 2 tiles it won't cause a spike of new rovers seen on those tiles.

Most of the current code has been commented out, what it is doing if it is run.
Is it will show the total amount of battery that has been used on each of those tiles.

