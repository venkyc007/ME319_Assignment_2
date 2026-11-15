#!/bin/bash
# run2.sh tag "options-struct-literal" [ids-expr]
tag=$1; o=$2; ids=${3:-1:160}
cd "$(dirname "$0")"
octave-cli -q --eval "ids=$ids; batch_stress(ids(1:2:end), $o, 'stress/${tag}_a.mat')" > stress/${tag}_a.log 2>&1 &
octave-cli -q --eval "ids=$ids; batch_stress(ids(2:2:end), $o, 'stress/${tag}_b.mat')" > stress/${tag}_b.log 2>&1 &
wait; echo done $tag
