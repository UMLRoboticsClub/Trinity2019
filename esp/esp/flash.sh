#!/bin/sh

SRC_FILE=$1
DST_DIR="/home/pi/esp/build"
DST_FILE=$(basename ${SRC_FILE})

DST_USER="pi"
DST_HOST="192.168.1.5"
PUB_KEY="/home/jackson/.ssh/id_rsa_robotpi"

scp -i $PUB_KEY $SRC_FILE $DST_USER@$DST_HOST:$DST_DIR
