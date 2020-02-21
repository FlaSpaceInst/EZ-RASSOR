#!/bin/bash

dot="$(cd "$(dirname "$0")"; pwd)"

# Toggles between the different programs
choose_prog()
{
    request=$1

    echo "Building"

    # get_elev
    if [ $request -eq 1 ]
    then

        docker image build --tag mybase -f "$dot/get_elev/Dockerfile.base" .
        docker image build --tag elev_img:1.0 -f "$dot/get_elev/Dockerfile.child" .

        echo "Running get_elev"

        docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/get_elev",target=/tmp,type=bind --name elev_cont elev_img:1.0

        cp -a "$dot/get_elev/dem_results/." "$dot/results/"

    # mk_gaz_wrld
    elif [ $request -eq 2 ]
    then

        docker image build --tag mybase -f "$dot/mk_gaz_wrld/Dockerfile.base" .
        docker image build --tag mk_gaz_img:1.0 -f "$dot/mk_gaz_wrld/Dockerfile.child" .

        echo "Running mk_gaz_wrld"

        docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/mk_gaz_wrld",target=/tmp,type=bind --name gaz_cont mk_gaz_img:1.0

        cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/results/"
        cp -a "$dot/mk_gaz_wrld/converted_dems/." "$dot/results/"

    # extract_tile
    elif [ $request -eq 3 ]
    then

        docker image build --tag mybase -f "$dot/extract_tile/Dockerfile.base" .
        docker image build --tag extr_tile:1.0 -f "$dot/extract_tile/Dockerfile.child" .

        echo "Running extract_tile"

        docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/extract_tile",target=/tmp,type=bind --name extr_tile_cont extr_tile:1.0

        cp -a "$dot/extract_tile/results/." "$dot/results/"

    # convert2tif
    elif [ $request -eq 4 ]
    then
        docker image build --tag mybase -f "$dot/convert2tif/Dockerfile.base" .
        docker image build --tag convert2tif:1.0 -f "$dot/convert2tif/Dockerfile.child" .

        echo "Running convert2tif"

        docker run --rm -it -e LOCAL_USER_ID=`id -u $USER` --mount src="$(pwd)/convert2tif",target=/tmp,type=bind --name conv_tif convert2tif:1.0

        cp -a "$dot/convert2tif/results/." "$dot/results/"
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

            echo "Resetting convert2tif"
            rm -rf "$dot/convert2tif/queued_dems"/*
            rm -rf "$dot/convert2tif/results"/*
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
            read -p "How many programs to run? [0-4]" num_query

            while [ $request -eq 0 ] && [ $num_prog -ne $num_query ]
            do
                read -p "Would you like to run get_elev, mk_gaz_wrld, extract_tile, or convert2tif? [1, 2, 3, or 4]" ans

                case $ans in
                    1) # get_elev
                        request=1

                        # get input from previous program
                        if [[ $num_prog -ne 0 ]]
                        then
                            temp=${order_prog[num_prog-1]}

                            # mk_gaz_wrld
                            if [[ $temp -eq 2 ]]
                            then
                                echo "copy from mk_gaz_wrld"

                                # checks if there was any downsizing or not
                                if [ -z "$(ls -A "$dot/mk_gaz_wrld/downsized_dems/.")" ]
                                then
                                    cp -a "$dot/mk_gaz_wrld/queued_dems/." "$dot/get_elev/queued_dems/"
                                else
                                    cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/get_elev/queued_dems/"
                                fi

                            # extract_tile
                            elif [[ $temp -eq 3 ]]
                            then
                                echo "copy from extract_tile"
                                cp -a "$dot/extract_tile/results/." "$dot/get_elev/queued_dems/"

                            # convert2tif
                            elif [[ $temp -eq 4 ]]
                            then
                                echo "copy from convert2tif"
                                cp -a "$dot/convert2tif/results/." "$dot/get_elev/queued_dems/"
                            else
                                echo "invalid previous"
                            fi

                        # this is the first program to run
                        else
                            echo "copy from queue"
                            cp -a "$dot/queue/." "$dot/get_elev/queued_dems/"
                        fi

                        # run the program and then add to past programs array
                        choose_prog $request
                        order_prog[num_prog]=$request

                        # increment number programs that have ran and reset
                        # current request
                        num_prog=$((num_prog+1))
                        request=0
                        ;;
                    2) # mk_gaz_wrld
                        request=2

                        # get input from previous program
                        if [[ $num_prog -ne 0 ]]
                        then
                            temp=${order_prog[num_prog-1]}

                            # get_elev
                            if [[ $temp -eq 1 ]]
                            then
                                echo "copy from get_elev"
                                cp -a "$dot/get_elev/queued_dems/." "$dot/mk_gaz_wrld/queued_dems/"

                            # extract_tile
                            elif [[ $temp -eq 3 ]]
                            then
                                echo "copy from extract_tile"
                                cp -a "$dot/extract_tile/results/." "$dot/mk_gaz_wrld/queued_dems/"

                            # convert2tif
                            elif [[ $temp -eq 4 ]]
                            then
                                echo "copy from convert2tif"
                                cp -a "$dot/convert2tif/results/." "$dot/mk_gaz_wrld/queued_dems/"
                            else
                                echo "invalid previous"
                            fi

                        # this is the first program to run
                        else
                            echo "copy from queue"
                            cp -a "$dot/queue/." "$dot/mk_gaz_wrld/queued_dems/"
                        fi

                        # run the program and then add to past programs array
                        choose_prog $request
                        order_prog[num_prog]=$request

                        # increment number programs that have ran and reset
                        # current request
                        num_prog=$((num_prog+1))
                        request=0
                        ;;
                    3) # extract_tile
                        request=3

                        # get input from previous program
                        if [[ $num_prog -ne 0 ]]
                        then
                            temp=${order_prog[num_prog-1]}

                            # get_elev
                            if [[ $temp -eq 1 ]]
                            then
                                echo "copy from get_elev"
                                cp -a "$dot/get_elev/queued_dems/." "$dot/extract_tile/queued_dems/"

                            # mk_gaz_wrld
                            elif [[ $temp -eq 2 ]]
                            then
                                echo "copy from mk_gaz_wrld"

                                # checks if there was any downsizing or not
                                if [ -z "$(ls -A "$dot/mk_gaz_wrld/downsized_dems/.")" ]
                                then
                                    cp -a "$dot/mk_gaz_wrld/queued_dems/." "$dot/extract_tile/queued_dems/"
                                else
                                    cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/extract_tile/queued_dems/"
                                fi

                            # convert2tif
                            elif [[ $temp -eq 4 ]]
                            then
                                echo "copy from convert2tif"
                                cp -a "$dot/convert2tif/results/." "$dot/extract_tile/queued_dems/"
                            else
                                echo "invalid previous"
                            fi

                        # this is the first program to run
                        else
                            echo "copy from queue"
                            cp -a "$dot/queue/." "$dot/extract_tile/queued_dems/"
                        fi

                        # run the program and then add to past programs array
                        choose_prog $request
                        order_prog[num_prog]=$request

                        # increment number programs that have ran and reset
                        # current request
                        num_prog=$((num_prog+1))
                        request=0
                        ;;
                    4) # convert2tif
                        request=4

                        # get input from previous program
                        if [[ $num_prog -ne 0 ]]
                        then
                            temp=${order_prog[num_prog-1]}

                            # get_elev
                            if [[ $temp -eq 1 ]]
                                then
                                echo "copy from get_elev"
                                cp -a "$dot/get_elev/queued_dems/." "$dot/convert2tif/queued_dems/"

                            # mk_gaz_wrld
                            elif [[ $temp -eq 2 ]]
                            then
                                echo "copy from mk_gaz_wrld"

                                # checks if there was any downsizing or not
                                if [ -z "$(ls -A "$dot/mk_gaz_wrld/downsized_dems/.")" ]
                                then
                                    cp -a "$dot/mk_gaz_wrld/queued_dems/." "$dot/convert2tif/queued_dems/"
                                else
                                    cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/convert2tif/queued_dems/"
                                fi

                            # extract_tile
                            elif [[ $temp -eq 3 ]]
                            then
                                echo "copy from extract_tile"
                                cp -a "$dot/extract_tile/results/." "$dot/convert2tif/queued_dems/"
                            else
                                echo "invalid previous"
                            fi

                        # this is the first program to run
                        else
                            echo "copy from queue"
                            cp -a "$dot/queue/." "$dot/convert2tif/queued_dems/"
                        fi

                        # run the program and then add to past programs array
                        choose_prog $request
                        order_prog[num_prog]=$request

                        # increment number programs that have ran and reset
                        # current request
                        num_prog=$((num_prog+1))
                        request=0
                        ;;
                    *)
                        echo "I didn't understand, please enter 1, 2, 3, or 4"
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
            echo "Please Rerun with one of the following command words, see readme for more info:"

            for val in reset run clean queue_reset results_reset; do
                echo $val
            done
            ;;
    esac
fi
