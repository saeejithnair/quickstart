#!/bin/bash


if [[ $# -lt 1 ]];then
  c=1
else
  c=$1
fi

addr=$(ipconfig getifaddr en0)

echo "adding $c cameras on $addr"
for ((i = 1 ; i <= c ; i++));
do

  currentcam="cam$i"
  echo "setting up $currentcam"
  ffplay -f live_flv -fast -x 640 -y 360 \
    -fflags nobuffer -flags low_delay \
    -strict experimental -vf "setpts=N/60/TB" \
    -af "asetpts=N/60/TB" -noframedrop \
    -i "rtmp://$addr/live/$currentcam" &

done
