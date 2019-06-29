#######################################################################################
#                                                                                     # 
#   This script is intended to track CPU usage of all of the ez_rassors processes and #
#   give the total cpu usage in a way that can be handeled by the gui software. Its   #
#   main function is to separate the CPU consumption of gazebo from the processes     #
#   being run in ros, to allow researchers to better understand the power drain and   #
#   cpu demands of the AI systems they are testing.                                   #
#                                                                                     #  
#######################################################################################

#!/usr/bin/env python
import subprocess
import rospy
from std_msgs.msg import Float64
import numpy as np


# This function will read in a list of process names to find the cpu usage of and return an array of the names
# A text file is used to make adding processes in the future easy to do
def read_in_process_names():
    
    # Initialize the variables
    f = open("process_names.txt", "r")
    process_names_array = []
    place = 0
    line = f.readline()

    # While the text is not empty, read in the line and add it to the names array. Each line should be a different process names\
    while line:
        
        if line != '' and line != '/n':
            
            process_names_array.insert(place, remove_split_line(line))
            place += 1
            line = f.readline()
    
    # Close the file and return the process name array
    f.close()
    return process_names_array





# This function will remove the split line character from a list
def remove_split_line(line):

    new_line = ''
    place = 0
    
    for i in range(len(line)-1):
        
        new_line = new_line + line[i]

    return new_line




# This function will change the array of process names and change it into PIDs
def names_to_pids(names_array):

    # Initialize an empty array to store all of the pids in 
    pids_array = []

    # Go through each of the names and find the pids correlating to that name
    for i in range(len(names_array)-1):

        # The names can be related to an entire package running multiple processes
        # This means multiple PIDs can be returned from one name. These will need to be separated before being added onto the main array
        incoming_pids = subprocess.check_output("pgrep -f " + names_array[i], shell =True)

        # The names can be related to an entire package running multiple processes
        # This means multiple PIDs can be returned from one name. These will need to be separated before being added onto the main array
        # The following function will seperate them for us
        if incoming_pids != None :
            seperated_pids = parse_pids(incoming_pids)

        # We now have an array of PIDs. So we will have to add them to the main array one by one.
            for j in range(len(seperated_pids)-1):

                pids_array.insert(i, seperated_pids[j])
    
    # Return the array full of the PID numbers of all of the EZ_RASSOR's running processes

    return pids_array


    


# This function will take in a string of process identification numbers (PIDS) and convert them into an array of PIDS for easy use later in the program.
def parse_pids(pid_string):
    
    # Initialize the array and helper variables
    pid_array=[]
    place_in_array=0
    single_parsed_pid = ''
    input_length = len(pid_string)

    # Begin the loop through the entire pid_string
    for i in range(input_length) : 
        
        # Check to see if the current value of the pid_string is an integer. If it isnt then it is a space between PIDs and should start a new PID
        int_check = pid_string[i].isdigit()

        # If it is the last value of pid_string then this if statement will be called and will add the PID to the array in its current state
        if (i == input_length-1) :
            if (int_check == True):

                # Add the current value to PID currently being formed
                single_parsed_pid = single_parsed_pid + pid_string[i]

                # Add the PID currently being formed to the array
                pid_array.insert(place_in_array, single_parsed_pid)
                place_in_array += 1
                single_parsed_pid = ''

            else :

                pid_array.insert(place_in_array, single_parsed_pid)
                place_in_array += 1
                single_parsed_pid = ''
        
        # If it is not the last value then the PID will only be added to the array when a line break is found
        else:

            if (int_check == True):

                # Add the current value to PID currently being formed
                single_parsed_pid = single_parsed_pid + pid_string[i]

            else :

                # Add the PID currently being formed to the array
                pid_array.insert(place_in_array, single_parsed_pid)
                # Increment the array place counter
                place_in_array += 1
                # Reset the 
                single_parsed_pid = ''

    # Return the usable PIDs
    return pid_array





# This function will use the array given by parse_pids and run a command line command to get the CPU usage of that command. The last PID in the array doesnt ever correlate to a real process so it will be stripped here. The output will be an unparsed array of the command line arguments.
def use_pids(pid_array):
    
    # This array will store
    unparsed_cpu_array = []

    # This will get the CPU data of all of the PIDs listed in the pid_array and add them each into a new spot in the unparsed_cpu_array
    for i in range(len(pid_array)-1):
        unparsed_cpu_array.insert(i,subprocess.check_output("top -b -n 1 -p " + pid_array[i], shell =True))
    
    # This will return the unparsed CPU data to be sorted through later
    return unparsed_cpu_array





# This function will parse through the cpu data of multiple pids and return an array of only the parsed numbers from the input data
# Combining the numbers into individual strings will make their relationship with one another consistent and more easily parsed
def parse_cpu_stats(unparsed_cpu_array):
    
    # Declare all of the variables
    length_of_unparsed_array = len(unparsed_cpu_array)
    parsed_numbers_array=[]
    cpu_array = []
    place_in_array=0
    temp_num = ''
    

    # Begin going through the unparsed array
    for i in range(length_of_unparsed_array):
        
        length_of_array_data = len(unparsed_cpu_array[i])

        # The unparsed array is a 2D array where each spot is the cpu information of another PID. So here we will begin searching through the CPU info one PID at a time
        for j in range(length_of_array_data):
            
            # Temp is a single value representing the current position of [i][j]
            temp = unparsed_cpu_array[i][j]

            # Temp_num is an entire number concatenated into a string.    
            # The CPU usage number is a floating point, so we are interested in adding the decimal sign and any integers
            if(temp.isdigit() or temp == '.'):

                temp_num = temp_num + temp

            # If temp is neither of those then thats the end of that particular number. Add it to the array and start a new number
            else:

                    # If temp is blank we dont want to add it to the array. Only if it has a number in it
                    if(temp_num != ''):
                        
                        parsed_numbers_array.insert(place_in_array, temp_num)
                        place_in_array += 1
                        temp_num = ''


    return parsed_numbers_array





# This function will take in the array of only numbers and the PIDS array and parse out the cpu usage. It will return the total CPU usage of all the PIDS.
def return_total_cpu_usage(pids_array, cleaned_stats_array):
    
    # Declare the necessary variables
    place_holder = 0
    cpu_array = []
    total = 0

    # It starts at 37 because that is always the position of the cpu usage in the array. 
    total_length_into_array=37
    
    # Now we will begin 
    for x in range(len(pids_array)-1):

        # We are looking for total CPU usage so anything discovered can just be added to the total
        total = total + float(cleaned_stats_array[total_length_into_array])
        cpu_array.insert(x,cleaned_stats_array[total_length_into_array])

        # We add 41 because the cpu data occurs every 41 spots from the original cpu data
        total_length_into_array = total_length_into_array + 41
        
    return total





# This function will read in the amount of CPUS on the computer
def num_cpus():

    # Read in all the raw data from the command
    mumbo_jumbo = subprocess.check_output("lscpu", shell =True)

    # Now we need to parse out that information
    cleaned_info = parse_cpu_stats(mumbo_jumbo)
    
    return cleaned_info[4]





# This function will publish the cpu usage data
def publisher():

    # Create the publisher instance
    pub = rospy.Publisher('/ezrassor/cpuUsage', Float64, queue_size=10)
    rospy.init_node('publisher', anonymous=True)

    # Begin publishing CPU data
    while not rospy.is_shutdown():

        # Get the number of cores on the machine
        num_cores = num_cpus()

        # Get the names of the processes youd like to check the CPU usage of
        names = read_in_process_names()

        # Change the process names into PIDs
        pids = names_to_pids(names)

        # Change the pids into raw cpu data
        unparsed_cpu_array = use_pids(pids)

        # Make the raw cpu data more manageable
        cpu_stats = parse_cpu_stats(unparsed_cpu_array)

        # Calculate the total percentage of your CPUs being used
        cpu = return_total_cpu_usage(pids, cpu_stats)

        # Divide cpu usage by number of cores on the machine
        cpu = float(cpu)/  float(num_cores)

        # Publish the cpu data to the topic
        pub.publish(cpu)




# This function will be called on startup
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass