#!/bin/bash
echo 0000 | sudo -S cpupower -c all frequency-set -g performance
