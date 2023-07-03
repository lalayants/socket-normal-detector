export PATH_SCRIPT="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
# IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
# echo $IP
xhost +

ROS1_DISTRO="noetic"
docker run -it \
-e DISPLAY=$DISPLAY \
-e "QT_X11_NO_MITSHM=1" \
-e XAUTHORITY \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
-v /etc/localtime:/etc/localtime:ro \
-v .:/workspace/ \
--privileged \
--net=host \
--name ROS_$ROS1_DISTRO \
--rm lalayants/raskat_test