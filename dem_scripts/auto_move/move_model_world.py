import os
import sys

def main():

	world_ext = ""
	choice = "queue_packs/"

	l = os.listdir("./" + choice)

	for i in l:

		print("Processing {}".format(i))

		folder_name = ""
		cool = i.split("_")
		num = len(cool)
		new_str = ""
		for p in range(num - 1):
			if p < num - 2:
				new_str += cool[p] + "_"
			else:
				new_str += cool[p]

		world_file = ""
		folder_name = new_str
		lister = os.listdir("./" + choice + i)
		for file in lister:
			if ".world" in file:
				world_file = file

		print("Copying model and world to appropriate folders")
		os.system(
			"cp -r ./" + choice + i + "/" + folder_name
			+ " $HOME/.gazebo/models/"
		)
		os.system(
			"cp ./" + choice + i + "/" + world_file
			+ " ../../packages/simulation/ezrassor_sim_gazebo/worlds"
		)

if __name__ == '__main__':
	main()
