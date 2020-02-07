#!/bin/bash

dot="$(cd "$(dirname "$0")"; pwd)"
#StringArray=("reset" "populate" "run" "clean" "queue_reset")
choose_prog() 
{
    request=$1
    #previous=$2
    echo "Building"
    
    if [ $request -eq 1 ]
    then
        #cp -a "$dot/queue/." "$dot/get_elev/queued_dems/"
        
        docker image build --tag mybase -f "$dot/get_elev/Dockerfile.base" .
        docker image build --tag elev_img:1.0 -f "$dot/get_elev/Dockerfile.child" .
        
        echo "Running get_elev"
        
        docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/get_elev",target=/tmp,type=bind --name elev_cont elev_img:1.0
        
        cp -a "$dot/get_elev/dem_results/." "$dot/results/"
        
    elif [ $request -eq 2 ]
    then
        #cp -a "$dot/queue/." "$dot/mk_gaz_wrld/queued_dems/"

        docker image build --tag mybase -f "$dot/mk_gaz_wrld/Dockerfile.base" .
        docker image build --tag mk_gaz_img:1.0 -f "$dot/mk_gaz_wrld/Dockerfile.child" .
        
        echo "Running mk_gaz_wrld"
        
        docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/mk_gaz_wrld",target=/tmp,type=bind --name gaz_cont mk_gaz_img:1.0
        
        cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/results/"
        cp -a "$dot/mk_gaz_wrld/converted_dems/." "$dot/results/"

    elif [ $request -eq 3 ] 
    then
        #cp -a "$dot/queue/." "$dot/extract_tile/queued_dems/"

        docker image build --tag mybase -f "$dot/extract_tile/Dockerfile.base" .
        docker image build --tag extr_tile:1.0 -f "$dot/extract_tile/Dockerfile.child" .
        
        echo "Running extract_tile"
        
        docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/extract_tile",target=/tmp,type=bind --name extr_tile_cont extr_tile:1.0
        
        cp -a "$dot/extract_tile/results/." "$dot/results/"                
    else
        echo "Something went wrong"
    fi    
}

if [ -z $1 ]
then
    echo "Please Rerun with one of the following command words, see readme for more info:"
    for val in reset run clean queue_reset results_reset; do
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
            echo "Resetting extract_tile"
            rm -rf "$dot/extract_tile/queued_dems"/*
            rm -rf "$dot/extract_tile/results"/*
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
            num_prog=0
            read -p "How many programs to run? [0-3]" num_query
            #order_prog[0]="hi"
            while [ $request -eq 0 ] && [ $num_prog -ne $num_query ]
            do
                read -p "Would you like to run get_elev, mk_gaz_wrld, or extract_tile? [1, 2, 3]" ans
                case $ans in
                    1)
                        request=1
                        # run
                        if [[ $num_prog -ne 0 ]]
                        then
                            temp=${order_prog[num_prog-1]}
                            #echo "$temp"
                            if [[ $temp -eq 2 ]]
                            then
                                echo "copy from mk_gaz_wrld"
                                cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/get_elev/queued_dems/"
                            elif [[ $temp -eq 3 ]]
                            then
                                #read -p "Full path: " tile
                                #echo "$tile"
                                #COULD CONVERT ALL TILES
                                cp -a "$dot/extract_tile/results/." "$dot/get_elev/queued_dems/"
                                
                            else
                                echo "invalid previous"
                            fi
                        else
                            echo "copy from queue"
                            cp -a "$dot/queue/." "$dot/get_elev/queued_dems/"
                        fi
                        choose_prog $request
                        order_prog[num_prog]=$request
                        #echo "${order_prog[num_prog]}"

                        num_prog=$((num_prog+1))
                        request=0
                        ;;
                    2)
                        request=2
                        # run
                        if [[ $num_prog -ne 0 ]]
                        then
                            temp=${order_prog[num_prog-1]}
                            #echo "$temp"
                            if [[ $temp -eq 1 ]]
                            then
                                # counter intuitive
                                echo "copy from get_elev"
                                cp -a "$dot/get_elev/queued_dems/." "$dot/mk_gaz_wrld/queued_dems/"
                            elif [[ $temp -eq 3 ]]
                            then
                                cp -a "$dot/extract_tile/results/." "$dot/mk_gaz_wrld/queued_dems/"
                                #echo "$tile"
                                #COULD JUST CONVERT ALL TILES
                                #cp "$tile" "$dot/mk_gaz_wrld/queued_dems/"
                            else
                                echo "invalid previous"
                            fi
                        else
                            echo "copy from queue"
                            cp -a "$dot/queue/." "$dot/mk_gaz_wrld/queued_dems/"
                        fi
                        choose_prog $request
                        order_prog[num_prog]=$request
                        #echo "${order_prog[num_prog]}"
                        num_prog=$((num_prog+1))
                        request=0
                        ;;
                    3)
                        request=3
                        # run
                        if [[ $num_prog -ne 0 ]]
                        then
                            temp=${order_prog[num_prog-1]}
                            echo "$temp"
                            if [[ $temp -eq 1 ]]
                            then
                                # counter intuitive
                                echo "copy from get_elev"
                                cp -a "$dot/get_elev/queued_dems/." "$dot/mk_gaz_wrld/queued_dems/"
                            elif [[ $temp -eq 2 ]]
                            then
                                echo "copy from mk_gaz_wrld"
                                cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/extract_tile/queued_dems/"
                            else
                                echo "invalid previous"
                            fi
                        else
                            echo "copy from queue"
                            cp -a "$dot/queue/." "$dot/extract_tile/queued_dems/"
                        fi
                        choose_prog $request
                        order_prog[num_prog]=$request
                        #echo "${order_prog[num_prog]}"
                        num_prog=$((num_prog+1))
                        request=0
                        ;;
                    *)
                        echo "I didn't understand, please enter 1, 2, or 3"
                        ;;
                esac
            done   
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
