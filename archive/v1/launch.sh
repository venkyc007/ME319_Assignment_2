#!/bin/bash
# usage: launch.sh tag mode "struct-literal" [first last]
tag=$1; mode=$2; ovr=$3; a=${4:-1}; b=${5:-100}
cd /home/claude/g
octave-cli --eval "sweep_batch('$mode', $a:2:$b, $ovr, 'sweep/${tag}_o.mat')" > sweep/${tag}_o.log 2>&1 &
octave-cli --eval "sweep_batch('$mode', $((a+1)):2:$b, $ovr, 'sweep/${tag}_e.mat')" > sweep/${tag}_e.log 2>&1 &
wait
echo done $tag
