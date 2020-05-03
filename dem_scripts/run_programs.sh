#!/bin/sh

dot=$(pwd)

# Toggles between the different programs
choose_prog()
{
    request=$1

    echo "Building"

    # get_elev
    if [ "$request" -eq 1 ]
    then

        docker image build --tag mybase -f "$dot/get_elev/Dockerfile.base" .
        docker image build --tag elev_img:1.0 -f "$dot/get_elev/Dockerfile.child" .

        echo "Running get_elev"

        docker run --rm -it -e LOCAL_USER_ID="$(id -u "$USER")" --mount src="$(pwd)/get_elev",target=/tmp,type=bind --name elev_cont elev_img:1.0

        cp -a "$dot/get_elev/dem_results/." "$dot/results/"

    # mk_gaz_wrld
    elif [ "$request" -eq 2 ]
    then

        docker image build --tag mybase -f "$dot/mk_gaz_wrld/Dockerfile.base" .
        docker image build --tag mk_gaz_img:1.0 -f "$dot/mk_gaz_wrld/Dockerfile.child" .

        echo "Running mk_gaz_wrld"

        docker run --rm -it -e LOCAL_USER_ID="$(id -u "$USER")" --mount src="$(pwd)/mk_gaz_wrld",target=/tmp,type=bind --name gaz_cont mk_gaz_img:1.0

        cp -a "$dot/mk_gaz_wrld/downsized_dems/." "$dot/results/"
        cp -a "$dot/mk_gaz_wrld/converted_dems/." "$dot/results/"

    # extract_tile
    elif [ "$request" -eq 3 ]
    then

        docker image build --tag mybase -f "$dot/extract_tile/Dockerfile.base" .
        docker image build --tag extr_tile:1.0 -f "$dot/extract_tile/Dockerfile.child" .

        echo "Running extract_tile"

        docker run --rm -it -e LOCAL_USER_ID="$(id -u "$USER")" --mount src="$(pwd)/extract_tile",target=/tmp,type=bind --name extr_tile_cont extr_tile:1.0

        cp -a "$dot/extract_tile/results/." "$dot/results/"

    # convert2tif
    elif [ "$request" -eq 4 ]
    then
        docker image build --tag mybase -f "$dot/convert2tif/Dockerfile.base" .
        docker image build --tag convert2tif:1.0 -f "$dot/convert2tif/Dockerfile.child" .

        echo "Running convert2tif"

        docker run --rm -it -e LOCAL_USER_ID="$(id -u "$USER")" --mount src="$(pwd)/convert2tif",target=/tmp,type=bind --name conv_tif convert2tif:1.0

        cp -a "$dot/convert2tif/results/." "$dot/results/"
    else
        echo "Something went wrong"
    fi
}

delete_files() {
    if [ "$1" = "y" ]
    then
        mkdir empty
        rsync -a --delete empty/ "$2"
        rmdir empty
    elif [ "$1" = "n" ]
    then
        rm -rf "$2"/*
    else
        echo "Error couldn't understand $1"
    fi
}

delete_folders_n_files() {
    any_exists=0

    echo "Resetting get_elev"

    if [ -d "$dot/get_elev/queued_dems" ]
    then
        delete_files "$1" "$dot/get_elev/queued_dems"
        rmdir "$dot/get_elev/queued_dems"
        any_exists=1
    fi

    if [ -d "$dot/get_elev/dem_results" ]
    then
        delete_files "$1" "$dot/get_elev/dem_results"
        rmdir "$dot/get_elev/dem_results"
        any_exists=1
    fi

    echo "Resetting mk_gaz_wrld"

    if [ -d "$dot/mk_gaz_wrld/queued_dems" ]
    then
        delete_files "$1" "$dot/mk_gaz_wrld/queued_dems"
        rmdir "$dot/mk_gaz_wrld/queued_dems"
        any_exists=1
    fi

    if [ -d "$dot/mk_gaz_wrld/converted_dems" ]
    then
        delete_files "$1" "$dot/mk_gaz_wrld/converted_dems"
        rmdir "$dot/mk_gaz_wrld/converted_dems"
        any_exists=1
    fi

    if [ -d "$dot/mk_gaz_wrld/downsized_dems" ]
    then
        delete_files "$1" "$dot/mk_gaz_wrld/downsized_dems"
        rmdir "$dot/mk_gaz_wrld/downsized_dems"
        any_exists=1
    fi

    echo "Resetting extract_tile"

    if [ -d "$dot/extract_tile/queued_dems" ]
    then
        delete_files "$1" "$dot/extract_tile/queued_dems"
        rmdir "$dot/extract_tile/queued_dems"
        any_exists=1
    fi

    if [ -d "$dot/extract_tile/results" ]
    then
        delete_files "$1" "$dot/extract_tile/results"
        rmdir "$dot/extract_tile/results"
        any_exists=1
    fi

    echo "Resetting convert2tif"

    if [ -d "$dot/convert2tif/queued_dems" ]
    then
        delete_files "$1" "$dot/convert2tif/queued_dems"
        rmdir "$dot/convert2tif/queued_dems"
        any_exists=1
    fi

    if [ -d "$dot/convert2tif/results" ]
    then
        delete_files "$1" "$dot/convert2tif/results"
        rmdir "$dot/convert2tif/results"
        any_exists=1
    fi

    if [ "$any_exists" -eq 0 ]
    then
        echo "No files to delete"
    fi
}

if [ -z "$1" ]
then
    echo "Please Rerun with one of the following command words, see readme for more info:"
    for val in reset run clean queue_reset results_reset; do
        echo $val
    done
else
    case $1 in
        reset)
            printf "Too many files to delete? [y or n]"
            read -r too_many

            delete_folders_n_files "$too_many"

            ;;
        queue_reset)
            printf "Too many files to delete? [y or n]"
            read -r too_many
            if [ "$too_many" = "n" ]
            then
              echo "Removing items from queue"
              rm -rf "$dot/queue"/*
            else
              mkdir empty
              echo "Removing items from results"
              rsync -a --delete empty/ "$dot/queue"
              rmdir empty
            fi
            ;;
        results_reset)
            if [ -d "$dot/results" ]
            then
                printf "Too many files to delete? [y or n]\n"
                read -r too_many
                if [ "$too_many" = "n" ]
                then
                  echo "Removing items from results"
                  rm -rf "$dot/results"/*
                else
                  mkdir empty
                  echo "Removing items from results"
                  rsync -a --delete empty/ "$dot/results"
                  rmdir empty
                fi
                rmdir "$dot/results"
            else
                echo "No files to be deleted"
            fi
            ;;
        run)
            request=0
            num_prog=0
            prev_prog=0
            printf "How many programs to run? [0-4]\n"
            read -r num_query

            if [ "$num_query" -ge 1 ] && [ ! -d "$dot/results" ]
            then
                echo "Making results/"
                mkdir "$dot/results"
            fi

            while [ "$request" -eq 0 ] && [ "$num_prog" -ne "$num_query" ]
            do
                printf "Would you like to run get_elev, mk_gaz_wrld, extract_tile, or convert2tif? [1, 2, 3, or 4]\n"
                read -r ans

                case $ans in
                    1) # get_elev
                        request=1

                        if [ ! -d "$dot/get_elev/queued_dems" ]
                        then
                            mkdir "$dot/get_elev/queued_dems"
                        fi

                        if [ ! -d "$dot/get_elev/dem_results" ]
                        then
                            mkdir "$dot/get_elev/dem_results"
                        fi

                        # get input from previous program
                        if [ "$num_prog" -ne 0 ]
                        then
                            temp=$prev_prog

                            # mk_gaz_wrld
                            if [ "$temp" -eq 2 ]
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
                            elif [ "$temp" -eq 3 ]
                            then
                                echo "copy from extract_tile"
                                cp -a "$dot/extract_tile/results/." "$dot/get_elev/queued_dems/"

                            # convert2tif
                            elif [ "$temp" -eq 4 ]
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
                        prev_prog=$request

                        # increment number programs that have ran and reset
                        # current request
                        num_prog=$((num_prog+1))
                        request=0
                        ;;
                    2) # mk_gaz_wrld
                        request=2

                        if [ ! -d "$dot/mk_gaz_wrld/queued_dems" ]
                        then
                            mkdir "$dot/mk_gaz_wrld/queued_dems"
                        fi

                        if [ ! -d "$dot/mk_gaz_wrld/converted_dems" ]
                        then
                            mkdir "$dot/mk_gaz_wrld/converted_dems"
                        fi

                        if [ ! -d "$dot/mk_gaz_wrld/downsized_dems" ]
                        then
                            mkdir "$dot/mk_gaz_wrld/downsized_dems"
                        fi

                        # get input from previous program
                        if [ "$num_prog" -ne 0 ]
                        then
                            temp=$prev_prog

                            # get_elev
                            if [ "$temp" -eq 1 ]
                            then
                                echo "copy from get_elev"
                                cp -a "$dot/get_elev/queued_dems/." "$dot/mk_gaz_wrld/queued_dems/"

                            # extract_tile
                            elif [ "$temp" -eq 3 ]
                            then
                                echo "copy from extract_tile"
                                cp -a "$dot/extract_tile/results/." "$dot/mk_gaz_wrld/queued_dems/"

                            # convert2tif
                            elif [ "$temp" -eq 4 ]
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
                        prev_prog=$request

                        # increment number programs that have ran and reset
                        # current request
                        num_prog=$((num_prog+1))
                        request=0
                        ;;
                    3) # extract_tile
                        request=3

                        if [ ! -d "$dot/extract_tile/queued_dems" ]
                        then
                            mkdir "$dot/extract_tile/queued_dems"
                        fi

                        if [ ! -d "$dot/extract_tile/results" ]
                        then
                            mkdir "$dot/extract_tile/results"
                        fi

                        # get input from previous program
                        if [ "$num_prog" -ne 0 ]
                        then
                            temp=$prev_prog

                            # get_elev
                            if [ "$temp" -eq 1 ]
                            then
                                echo "copy from get_elev"
                                cp -a "$dot/get_elev/queued_dems/." "$dot/extract_tile/queued_dems/"

                            # mk_gaz_wrld
                            elif [ "$temp" -eq 2 ]
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
                            elif [ "$temp" -eq 4 ]
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
                        prev_prog=$request

                        # increment number programs that have ran and reset
                        # current request
                        num_prog=$((num_prog+1))
                        request=0
                        ;;
                    4) # convert2tif
                        request=4

                        if [ ! -d "$dot/convert2tif/queued_dems" ]
                        then
                            mkdir "$dot/convert2tif/queued_dems"
                        fi

                        if [ ! -d "$dot/convert2tif/results" ]
                        then
                            mkdir "$dot/convert2tif/results"
                        fi

                        # get input from previous program
                        if [ "$num_prog" -ne 0 ]
                        then
                            temp=$prev_prog

                            # get_elev
                            if [ "$temp" -eq 1 ]
                                then
                                echo "copy from get_elev"
                                cp -a "$dot/get_elev/queued_dems/." "$dot/convert2tif/queued_dems/"

                            # mk_gaz_wrld
                            elif [ "$temp" -eq 2 ]
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
                            elif [ "$temp" -eq 3 ]
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
                        prev_prog=$request

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
            printf "Would you like to purge your computer of all Docker stuff (won't uninstall docker tho) [y or n]\n"
            read -r pur

            if [ "$pur" = "y" ]
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
