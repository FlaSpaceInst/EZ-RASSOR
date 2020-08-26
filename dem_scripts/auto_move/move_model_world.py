#!/usr/bin/env python
import os


def main():

    choice = "queue_packs/"

    packs = os.listdir("./" + choice)

    for i in packs:

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
        tester = os.listdir(
            "./" + choice + i + "/" + folder_name + "/materials/textures/"
        )
        cache = ""
        for file in tester:
            if file != "AS16-110-18026HR-512x512.jpg":
                temp = file.split(".")
                cache = temp[0]
        print("Remove cache for {}".format(cache))
        os.system("rm -rf $HOME/.gazebo/paging/" + cache)

        print("Copying model and world to appropriate folders")
        os.system(
            "cp -r ./"
            + choice
            + i
            + "/"
            + folder_name
            + " $HOME/.gazebo/models/"
        )
        os.system(
            "cp ./"
            + choice
            + i
            + "/"
            + world_file
            + " ../../packages/simulation/ezrassor_sim_gazebo/worlds"
        )


if __name__ == "__main__":
    main()
