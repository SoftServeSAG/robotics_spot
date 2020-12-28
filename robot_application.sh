echo "###############################################################################"
echo " Create robot application "
echo "###############################################################################"

echo "Copy the robot application source bundle to your Amazon S3 bucket"

aws s3 cp robot_ws/bundle/output.tar s3://borospotsource/spot1.tar

aws robomaker create-robot-application --name Spot1 --sources s3Bucket=borospotsource,s3Key=spot1.tar,architecture=X86_64 --robot-software-suite name=ROS,version=Melodic