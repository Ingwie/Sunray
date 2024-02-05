#!/bin/bash

echo "EUID=$EUID"
echo "PWD=$PWD"

CMD=""

if [ "$EUID" -ne 0 ]
  then echo "Please run as root (sudo)"
  exit
fi

if [[ `pidof sunray` != "" ]]; then
  echo "Sunray linux app already running! Exiting..."
  #exit
fi

# start sunray
#DIR=`echo $PWD`
#echo "current directory:" $DIR
# -------------------------------------------
# throttle down max cpu freq
echo "throttle down max cpu freq..."
cpufreq-set -f 300MHz

echo "----waiting for TCP connections to be closed from previous sessions----"
echo "Waiting TCP port 80 to be closed..."
for _ in `seq 1 30`; do 
  RES=$(netstat -ant | grep -w 80)
  #RES=$(lsofs -i:80)
  #RES=$(fuser 80/tcp) 
  if [ -z "$RES" ]; then
    break
  fi
  echo $RES
  # echo -n .  
  sleep 2.0     
done; 


echo "----starting sunray----"
echo "CMD=$CMD"
    
cd /home/pi/RAMDisk

./sunray

