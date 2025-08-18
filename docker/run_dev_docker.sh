
isRunning=`docker ps -f name=astroviz | grep -c "astroviz"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm astroviz
    docker run  \
        --gpus all \
        --device /dev/dri \
        --name astroviz  \
        --env DISPLAY=$DISPLAY \
        --env NVIDIA_DRIVER_CAPABILITIES=all \
        --env QTWEBENGINE_DISABLE_SANDBOX=1 \
        --env QT_X11_NO_MITSHM=1 \
        --net host \
        --ipc host \
        --pid host \
        --privileged \
        -it \
        -v /dev:/dev \
        -v `pwd`/../:/ros2_ws/src/astroviz \
        -v `pwd`/../../g1pilot:/ros2_ws/src/g1pilot \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /run/dbus:/run/dbus \
        -w /ros2_ws \
        astroviz:latest

else
    echo "Docker already running."
    docker exec -it astroviz /bin/bash
fi