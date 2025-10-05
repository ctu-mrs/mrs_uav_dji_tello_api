echo "arming"

ros2 service call /uav1/hw_api/arming std_srvs/srv/SetBool '{"data": true}' 
