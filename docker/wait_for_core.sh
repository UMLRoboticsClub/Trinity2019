#!/usr/bin/env bash
until rostopic list > /dev/null 2>&1; do sleep 0.1; done;
