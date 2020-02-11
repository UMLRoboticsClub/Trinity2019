#!/bin/sh
sudo docker-compose -f docker-compose-pi.yaml up --force-recreate roscore
#sudo docker-compose -f docker-compose-pi.yaml build roscore
