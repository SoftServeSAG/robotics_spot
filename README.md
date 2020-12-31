# robotics_spot
Spot in AWS Robomaker

## Setup 

Create Source and Output Amazon S3 Buckets according to [guideline](https://docs.aws.amazon.com/robomaker/latest/dg/application-create-simjob.html) 

Create a Simulation Job Role according to [guideline](https://docs.aws.amazon.com/robomaker/latest/dg/application-create-simjob.html) 

Create a development environment according to [guideline](https://docs.aws.amazon.com/robomaker/latest/dg/gs-build.html)

In the terminal section of your development environment run:
```bash
cd ~/environment
git clone -b temp_robomaker https://github.com/SoftServeSAG/robotics_spot.git

cd robotics_spot
```
In files robot_application.sh, simulation_application.sh, simulation_job.sh specify the names of buckets and job role that were created before.

## Build and bundle
```bash
./build_robot.sh
./build_simulation
```

## Create robot and simulation application
```bash
./robot_application.sh
./simulation_application.sh
```

## Run simulatiom  job
```bash
./simulation_job.sh
```

## Run teleop to control
In robot application terminal run:

```bash
roslaunch rs_teleop teleop_spot.launch
```
