#!/usr/bin/env bash
until rostopic list; do sleep 0.1; done;
