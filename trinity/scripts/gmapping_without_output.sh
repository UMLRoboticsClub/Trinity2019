#!/bin/bash
roslaunch trinity gmapping_trinity.launch 2>&1 | grep "INFO|DEBUG|ERROR"
