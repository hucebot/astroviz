
isRunning=`docker ps -f name=ros2_teleoperation | grep -c "ros2_teleoperation"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm ros2_teleoperation
    docker run  \
        --gpus all \
        --device /dev/dri \
        --name ros2_teleoperation  \
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
        -v `pwd`/../:/ros2_ws/src/ros2_teleoperation \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /run/dbus:/run/dbus \
        -w /ros2_ws \
        ros2_teleoperation:latest

else
    echo "Docker already running."
    docker exec -it ros2_teleoperation /bin/bash
fi