#!/bin/bash
cd "$(dirname "$0")"
while ! grep -q ALLDONE stress/audit_chain.log; do sleep 15; done
octave-cli -q --eval "ids=1:8:160; CS=cell(1,160); for k=ids, CS{k}=real_case(k,2); end; save('-v7','stress/cases_a2.mat','CS')" > stress/build_a2.log 2>&1
D=2*pi/180/100   # placeholder
SUB="1:8:160"
for m in track ambush; do
 M="\"ovr\",struct(\"mode\",\"$m\")"
 ./run_real.sh r_${m}_base   "struct($M)" $SUB
 ./run_real.sh r_${m}_noise5 "struct(\"sig_th\",0.1*pi/180,$M)" $SUB
 ./run_real.sh r_${m}_noise25 "struct(\"sig_th\",0.5*pi/180,$M)" $SUB
 ./run_real.sh r_${m}_glint  "struct(\"glint_p\",0.05,$M)" $SUB
 ./run_real.sh r_${m}_rate10 "struct(\"Ts_r\",0.1,$M)" $SUB
 ./run_real.sh r_${m}_lat50  "struct(\"lat\",0.05,$M)" $SUB
 ./run_real.sh r_${m}_lat100 "struct(\"lat\",0.1,$M)" $SUB
 ./run_real.sh r_${m}_v400   "struct(\"Vmax\",400,$M)" $SUB
 ./run_real.sh r_${m}_fric   "struct(\"tau_c\",3000,$M)" $SUB
 ./run_real.sh r_${m}_grav   "struct(\"tau_g\",1e4,$M)" $SUB
 ./run_real.sh r_${m}_backl  "struct(\"backlash\",5e-4,$M)" $SUB
 ./run_real.sh r_${m}_enc    "struct(\"enc_q\",2*pi/2^16,\"rate_sig\",0.005,$M)" $SUB
 ./run_real.sh r_${m}_jit    "struct(\"jit\",2e-4,$M)" $SUB
 ./run_real.sh r_${m}_bo2x   "struct(\"bo_scale\",2,$M)" $SUB
 ./run_real.sh r_${m}_a2x    "struct($M)" $SUB stress/cases_a2.mat
done
REAL='"sig_th",0.1*pi/180,"glint_p",0.02,"lat",0.05,"Vmax",400,"tau_c",3000,"tau_g",1e4,"backlash",3e-4,"enc_q",2*pi/2^16,"rate_sig",0.005,"scale",[1.1 1.1 1 0.95],"jit",1e-4'
HARSH='"sig_th",0.25*pi/180,"glint_p",0.05,"Ts_r",0.1,"lat",0.1,"Vmax",300,"tau_c",5000,"tau_g",2e4,"backlash",1e-3,"enc_q",2*pi/2^14,"rate_sig",0.01,"scale",[1.2 1.2 1 0.85],"jit",2e-4'
for m in track ambush; do
 M="\"ovr\",struct(\"mode\",\"$m\")"
 ./run_real.sh c_${m}_realistic "struct($REAL,$M)" "1:2:160"
 ./run_real.sh c_${m}_harsh "struct($HARSH,$M)" "1:2:160"
done
echo REALDONE
