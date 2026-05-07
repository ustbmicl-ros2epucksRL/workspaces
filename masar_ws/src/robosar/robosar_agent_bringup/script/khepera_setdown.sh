#!/bin/sh
# Created by Indraneel on 14/02/21

KHEPERA_IP_ADDRESS=$1

# Kill any old running code
ssh root@${KHEPERA_IP_ADDRESS} pkill -9 template
ssh root@${KHEPERA_IP_ADDRESS} pkill -9 template_perception