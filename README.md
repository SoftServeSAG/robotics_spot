# robotics_spot
Reinforced Spot Mini


## Install
Install docker:
Install Docker-CE using these [instructions](https://docs.docker.com/engine/install/ubuntu/)

Install nvidia-docker 
```bash
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install nvidia-docker2
sudo systemctl restart docker
```

Now we should build Spot container:

### Spot:

Init workspace

```bash
mkdir -p spot_ws/src
cd spot_ws/src
git clone --recursive https://github.com/chvmp/champ
git clone https://github.com/chvmp/champ_teleop
git clone -b gazebo https://github.com/chvmp/spot_ros
git clone https://github.com/chvmp/robots.git
git clone https://github.com/SoftServeSAG/robotics_spot.git
```

Build docker images

```bash
cd robotics_spot/docker/ros_melodic
chmod a+x build.bash
sudo ./build.bash 
```
Run docker container

```bash
chmod a+x run.bash
sudo ./run.bash
```

Inside docker container

```bash
cd /root/ws
rosdep check --from-paths . --ignore-src --rosdistro melodic
catkin config --init --extend /opt/ros/melodic   
catkin build
```

## Use state machine 
All following commands should be executed in specified containers:

```bash
cd ~/ws/src/robotics_spot/scripts
chmod a+x start.sh
chmod a+x session.yml
./start.sh
```
Now tmux session will start with all required tabs. All required commandsmay be added to the history, in order to simplify their usage.

