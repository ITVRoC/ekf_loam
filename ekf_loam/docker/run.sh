#!/bin/bash
docker run --net=host -it --rm \
            -v $(realpath ..):/root/catkin_ws/src/ekf_loam/ \
            -w /root/catkin_ws/src/ekf_loam \
            $@ \
            ekf_loam
