#!/bin/bash
cd "$(dirname "$0")"
AM='"ovr",struct("mode","ambush")'
bash run2.sh am_full "struct($AM)" "1:160"
for m in track ambush; do
  O="\"ovr\",struct(\"mode\",\"$m\")"
  bash run2.sh ${m}_dt20 "struct(\"dt\",0.02,$O)" "1:4:160"
  bash run2.sh ${m}_dt50 "struct(\"dt\",0.05,$O)" "1:4:160"
  bash run2.sh ${m}_hold "struct(\"meas\",\"hold\",$O)" "1:4:160"
  bash run2.sh ${m}_mmhi "struct(\"scale\",[1.2 1.2 1 0.8],$O)" "1:4:160"
  bash run2.sh ${m}_mmlo "struct(\"scale\",[0.8 0.8 1 1.2],$O)" "1:4:160"
done
echo ALLDONE
