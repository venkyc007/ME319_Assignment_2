#!/bin/bash
# run_real.sh tag "options-struct-literal" ids-expr [casefile]
tag=$1; o=$2; ids=${3:-1:160}; cf=${4:-stress/cases.mat}
cd "$(dirname "$0")"
octave-cli -q --eval "ids=$ids; batch_real(ids(1:2:end), $o, 'stress/${tag}_a.mat', '$cf')" > stress/${tag}_a.log 2>&1 &
octave-cli -q --eval "ids=$ids; batch_real(ids(2:2:end), $o, 'stress/${tag}_b.mat', '$cf')" > stress/${tag}_b.log 2>&1 &
wait; echo done $tag
