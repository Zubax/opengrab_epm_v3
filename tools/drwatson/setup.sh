#!/bin/bash

if [[ $EUID -ne 0 ]]; then
    echo "You are not root. Why $USER, why?!" 1>&2
    exit 1
fi

set -o xtrace

apt-get install -y python3 python3-pip can-utils

pip3 install colorama 'uavcan>=0.0.dev0'
