#!/bin/bash
cd "$(dirname "$0")"
while ! grep -q REALDONE stress/real_chain.log; do sleep 15; done
for m in track ambush; do
 ./run_real.sh x_${m}_noise5_matched "struct(\"sig_th\",0.1*pi/180,\"ovr\",struct(\"mode\",\"$m\",\"sig_th\",0.1*pi/180))" 1:8:160
done
echo EXTRADONE
