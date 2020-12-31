echo "###############################################################################"
echo " Create simulation job "
echo "###############################################################################"


JOB_ROLE=arn:aws:iam::448733523991:role/tborospotrole
robot_application=arn:aws:robomaker:us-west-2:448733523991:robot-application/Spot1/1608564193018
simulation_application=arn:aws:robomaker:us-west-2:448733523991:simulation-application/Spot1_sim/1608130519861 

aws robomaker create-simulation-job --max-job-duration-in-seconds 3600 --iam-role $JOB_ROLE --output-location s3Bucket=borospotoutput,s3Prefix=job --robot-applications application=$robot_application,launchConfig='{packageName=rs_inverse,launchFile=inverse.launch}' --simulation-applications application=$simulation_application,launchConfig='{packageName=rs_gazebo,launchFile=HQ.launch}'