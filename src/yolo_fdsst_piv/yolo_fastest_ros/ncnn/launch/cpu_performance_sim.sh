#!/bin/bash
#echo 0000 | sudo -S cpufreq-set -u 3400000
echo ' ' | sudo -S cpupower -c all frequency-set -g performance
