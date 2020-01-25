#!/bin/bash

dot="$(cd "$(dirname "$0")"; pwd)"
#StringArray=("reset" "populate" "run" "clean" "queue_reset")

if [ -z $1 ]
then
    echo "Please Rerun with one of the following command words, see readme for more info:"
    for val in reset populate run clean queue_reset results_reset; do
        echo $val
    done
else
    case $1 in
        reset)
            echo "Resetting mk_gaz_wrld"
            rm -rf "$dot/mk_gaz_wrld/queued_dems"/*
            rm -rf "$dot/mk_gaz_wrld/downsized_dems"/*
            rm -rf "$dot/mk_gaz_wrld/converted_dems"/*
            echo "Resetting get_elev"
            rm -rf "$dot/get_elev/queued_dems"/*
            rm -rf "$dot/get_elev/dem_results"/*
            ;;
        populate)
            echo "Copying files from ./queue"
            cp -a "$dot/queue/." "$dot/mk_gaz_wrld/queued_dems/"
            #cp -a "$dot/queue/." "$dot/get_elev/queued_dems/"
            ;;
        queue_reset)
            echo "Removing items from queue"
            rm -rf "$dot/queue"/*
            ;;
        results_reset)
            echo "Removing items from results"
            rm -rf "$dot/results"/*
            ;;
        run)
            request=0
            while [ $request -eq 0 ]
            do
                read -p "Would you like to run get_elev, mk_gaz_wrld, or both? [1, 2, 3]" ans
                case $ans in
                    1)
                        request=1
                        ;;
                    2)
                        request=2
                        ;;
                    3)
                        request=3
                        ;;
                    *)
                        echo "I didn't understand, please enter 1, 2, or 3"
                        ;;
                esac
            done
            
            echo "Building"
            
            if [ $request -eq 1 ]
            then
                cp -a "$dot/queue/." "$dot/get_elev/queued_dems/"
                
                docker image build --tag mybase -f "$dot/get_elev/Dockerfile.base" .
                docker image build --tag elev_img:1.0 -f "$dot/get_elev/Dockerfile.child" .
                
                echo "Running get_elev"
                
                docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/get_elev",target=/tmp,type=bind --name elev_cont elev_img:1.0
                
                cp -a "$dot/get_elev/dem_results/." "$dot/results/"
                
            elif [ $request -eq 2 ]
            then
                docker image build --tag mybase -f "$dot/mk_gaz_wrld/Dockerfile.base" .
                docker image build --tag mk_gaz_img:1.0 -f "$dot/mk_gaz_wrld/Dockerfile.child" .
                
                echo "Running mk_gaz_wrld"
                
                docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/mk_gaz_wrld",target=/tmp,type=bind --name gaz_cont mk_gaz_img:1.0
                
                cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/results/"
                cp -a "$dot/mk_gaz_wrld/converted_dems/." "$dot/results/"
                
            elif [ $request -eq 3 ]
            then
                docker image build --tag mybase -f "$dot/mk_gaz_wrld/Dockerfile.base" .
                docker image build --tag mk_gaz_img:1.0 -f "$dot/mk_gaz_wrld/Dockerfile.child" .
                
                echo "Running mk_gaz_wrld"
                
                docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/mk_gaz_wrld",target=/tmp,type=bind --name gaz_cont mk_gaz_img:1.0
                
                echo "Copying files from mk_gaz_wrld to get_elev"
                
                cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/get_elev/queued_dems/"
                
                docker image build --tag mybase -f "$dot/get_elev/Dockerfile.base" .
                docker image build --tag elev_img:1.0 -f "$dot/get_elev/Dockerfile.child" .
                
                echo "Running get_elev"
                
                docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/get_elev",target=/tmp,type=bind --name elev_cont elev_img:1.0
                
                cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/results/"
                cp -a "$dot/mk_gaz_wrld/converted_dems/." "$dot/results/"
                cp -a "$dot/get_elev/dem_results/." "$dot/results/"
                
            else
                echo "Something went wrong"
            fi    
            ;;
        clean)
            read -p "Would you like to purge your computer of all Docker stuff (won't uninstall docker tho) [y/n] " pur
            if [ $pur = "y" ]
            then
                echo "Cleaning up memory from Docker"
                docker system prune -a
            else
                echo "Please remove manaully the Docker images"
            fi    
            ;;
        *)
            echo "I don't understand"
            ;;
    esac
fi
