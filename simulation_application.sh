echo "###############################################################################"
echo " Create simulation application "
echo "###############################################################################"

echo "Copy the robot application source bundle to your Amazon S3 bucket"

aws s3 cp simulation_ws/bundle/output.tar s3://borospotsource/spot1_sim.tar


aws robomaker create-simulation-application --name Spot1_sim --sources s3Bucket=borospotsource,s3Key=spot1_sim.tar,architecture=X86_64 --robot-software-suite name=ROS,version=Melodic --simulation-software-suite name=Gazebo,version=9 --rendering-engine name=OGRE,version=1.x