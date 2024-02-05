#!/bin/bash

# start/stop sunray service
# start/stop cassandra service

echo "EUID=$EUID"
echo "PWD=$PWD"

CMD=""


# start USB camera streaming web server 
function start_cassandra_service() {
  if [[ `pidof python3.11` != "" ]]; then
        echo "Cassandra already running!"
  else
  echo "starting Cassandra service..."
  systemctl daemon-reload
  systemctl enable cassandra
  systemctl start cassandra
  systemctl --no-pager status cassandra
  echo "Cassandra service started!"
  fi
}

function stop_cassandra_service() {
  echo "stopping Cassandra service..."
  systemctl stop cassandra
  systemctl disable cassandra
  echo "Cassandra service stopped!"
}

function start_sunray_service() {
  if [[ `pidof sunray` != "" ]]; then
        echo "Sunray linux app already running!"
  else
  # enable sunray service
  echo "starting sunray service..."
  systemctl daemon-reload
  systemctl enable sunray
  systemctl start sunray
  systemctl --no-pager status sunray
  echo "sunray service started!"
  fi
}

function stop_sunray_service() {
  # disable sunray service
  echo "stopping sunray service..."
  systemctl stop sunray
  systemctl disable sunray
  echo "sunray service stopped!"
}


if [ "$EUID" -ne 0 ]
  then echo "Please run as root (sudo)"
  exit
fi

#systemctl status sunray
#systemctl status cassandra


# show menu
PS3='Please enter your choice: '
options=("Start Sunray service" 
  "Stop Sunray service" 
  "Start Cassandra service"  
  "Stop Cassandra service"   
  "Quit")
select opt in "${options[@]}"
do
    case $opt in
        "Start Sunray service")
            start_sunray_service
            ;;
        "Stop Sunray service")
            stop_sunray_service
            ;;            
        "Start Cassandra service")
            start_cassandra_service
            ;;
        "Stop Cassandra service")
            stop_cassandra_service
            ;;            
        
        "Quit")
            break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done


