# Midterm_ws

## Environment Setup

```bash
xhost +local:
sudo docker run \
-it \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
-p 2233:22 \
--rm \
--name ros \
--user root \
-e GRANT_SUDO=yes \
-v ~/midterm_ws_v2:/root/catkin_ws \
softmac/sdc-course-docker:latest \
bash
```

```bash
sudo docker exec -it ros bash
```

## 

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch localization localization.launch
```

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch localization visualization.launch
```

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch localization map_modified.launch
```