#!/bin/bash
#version: 0.1
#Author: Anthony Suen
_GREEN='\e[32m'
_NORMAL='\e[0m'
_BOLD='\e[33m'
_RED='\e[31m'
clear

CHOOSE=1

function PRINT_MENU()
{
    echo -e "${_BOLD}--------------------------${_NORMAL}"
    echo -e "\e[1;10H Menu${_NORMAL}"
    echo -e "${_GREEN} 1.Auto start (Recommend)${_NORMAL}"
    echo -e "${_GREEN} 2.Build image${_NORMAL}"
    echo -e "${_GREEN} 3.Start Container${_NORMAL}"
    echo -e "${_GREEN} 4.Delete Container${_NORMAL}"
    echo -e "${_GREEN} 5.Backup environment${_NORMAL}"
    echo -e "${_GREEN} 6.Restore environment${_NORMAL}"
    echo -e "${_BOLD}--------------------------${_NORMAL}"
    echo -n "Your chose(1-6):"
}

function prepare()
{
    (mv .devcontainer .. &> /dev/null) | echo -n ""
    (mv setup.sh .. &> /dev/null) | echo -n ""
}

function BUILD_IMAGE() {
    Docker_file=../.devcontainer/Dockerfile
    image_tag=limo_ros2:dev
    if [ $# -gt 2 ]
    then
        Docker_file=$1
        image_tag=$2
    elif [ $# -gt 1 ] 
    then
        Docker_file=$1
    fi
    docker build --file $Docker_file --tag image_tag ..
}

function start_image()
{
    image_tag = limo_ros2:dev
    if [ $# -gt 1 ] 
    then
        image_tag=$1
    fi

    # give docker root user X11 permissions
    sudo xhost +si:localuser:root

    # enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
    XAUTH=/tmp/.docker.xauth
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
    chmod 777 $XAUTH
    
    docker run --network=host \
        -d
        -v=/dev:/dev \
        --privileged \
        --device-cgroup-rule="a *:* rmw" \
        --volume=/tmp/.X11-unix:/tmp/.X11-unix \
        -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \
        --runtime nvidia
        --gpus=all \
        -v=${PWD}/..:/workspace \
        -w=/workspace \
        --name limo_dev \
        -e LIBGL_ALWAYS_SOFTWARE="1" \
        -e DISPLAY=${DISPLAY} \
        --restart=always \
        image_tag \
        ./setup.sh
    echo -e "${_GREEN} Container start success!${_NORMAL}"
    echo -e "${_GREEN} Now you can now connect to the container via SSH by using 'ssh -p 10022 root@ip' the password is 'agx'${_NORMAL}"

}

function backup_container()
{
    docker commit limo_dev limo_dev:backup | (echo -e "${_RED} A backup is already exist. Use 'docker rmi limo_dev:backup' and try again.${_NORMAL}" && exit 1)
    echo -e "${_GREEN} Do you want to save the image locally (save as a .tar file)? (Y/N):${_NORMAL}"
    read input
    case $input in
        [yY][eE][sS]|[yY])
            docker save -o limo_dev_backup.tar limo_dev:backup
            ;;

        [nN][oO]|[nN])
            ;;

        *)
            echo "Invalid input..."
            exit 1
            ;;
    esac
    echo -e "${_GREEN} Container backup success!${_NORMAL}"

}

function restore_image()
{
    echo -e "${_RED}This operation will overwrite your current backup image and default image. Continue?(y/n):${_NORMAL}"
    read input
    case $input in
        [yY][eE][sS]|[yY])
            docker rmi -f limo_ros2:dev
            docker rmi -f limo_dev:backup
            docker load < limo_dev_backup.tar
            ;;

        [nN][oO]|[nN])
            ;;

        *)
            echo "Invalid input..."
            exit 1
            ;;
    esac
    echo -e "${_GREEN} Container restore success!${_NORMAL}"

}

PRINT_MENU

prepare

read CHOOSE

case "${CHOOSE}" in
    1)
    BUILD_IMAGE
    start_image
    ;;
    2)
    BUILD_IMAGE
    ;;
    3)
    start_image
    ;;
    4)
    docker rm -f limo_dev
    ;;
    5)
    backup_container
    ;;
    6)
    restore_image
    ;;

esac

