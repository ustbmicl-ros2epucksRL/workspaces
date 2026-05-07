#!/bin/sh
# Created by Indraneel on 26/01/21

CODE_PATH=$1
KHEPERA_IP_ADDRESS=$2
SERVER_IP_ADDRESS=$3
FEEDBACK_PORT=$4
CONTROL_PORT=$5
FEEDBACK_FREQUENCY=$6
CONTROL_TIMEOUT=$7
CAMERA_ENABLED=$8
PERCEPTION_CODE_PATH=$9


# Kill any old running code
ssh root@${KHEPERA_IP_ADDRESS} pkill template

# Transfer latest khepera code
scp ${CODE_PATH} root@${KHEPERA_IP_ADDRESS}:/home/root

# Execute latest khepera code with the right arguments
ssh root@${KHEPERA_IP_ADDRESS} ./template ${SERVER_IP_ADDRESS} ${CONTROL_PORT} ${FEEDBACK_PORT} ${FEEDBACK_FREQUENCY} ${CONTROL_TIMEOUT}  > /dev/null 2>&1 &

# Set up camera if enabled
if [ ${CAMERA_ENABLED} = "1" ]
then
    scp ${PERCEPTION_CODE_PATH} root@${KHEPERA_IP_ADDRESS}:/home/root

    # Run the script 
    ssh root@${KHEPERA_IP_ADDRESS} ./template_perception > /dev/null 2>&1 &  
fi