#!/usr/bin/env bash

# Build all files
clear
# Uncomment below code if you want to rebuild all programs
# cd init_solver
# sudo bazel build --cxxopt="-std=c++2a" //main:all
# echo "Build OR-Tools done."
# cd ../opt_solver
# mpicxx -std=c++2a *.cpp -o cpp_bha
# echo "Build BHA done."
# cd ../online_ROMP
# mpicxx -std=c++2a *.cpp -o cpp_romp
# echo "Build Online ROMP done."
# cd ../

echo "Check workspace directory layout"
inp_dir="inp/demo/"
if [ ! -d $inp_dir ]; then
    mkdir $inp_dir
fi
out_dir="out/demo/"
if [ ! -d $out_dir ]; then
    mkdir $out_dir
fi
fig_dir="fig/demo/"
if [ ! -d $fig_dir ]; then
    mkdir $fig_dir
fi
echo "Check layout done."
echo "Initialization Solver starts."
sudo init_solver/bazel-bin/main/cpp_ortools "$inp_dir" "$out_dir" "iter0.csv" "init_sol0.txt" 0 0 0 0 1
echo "Initialization Solver done."
echo "Optimization Solver starts."
mpiexec -n 4 opt_solver/cpp_bha "$inp_dir" "$out_dir" "iter0.csv" "init_sol0.txt" 0 50 100 100 75 10 100 50
echo "Optimization Solver done."
python3 visual_data.py --inp "$inp_dir" --out_dir "$out_dir" --fig "$fig_dir" --case "iter0.csv" --or "init_sol0.txt" --bha "plan.txt" --out "iter_fig0.png"
echo "Offline mission planning done."

energy=40
echo "After charged 5 nodes, the UAV battery dropped to $energy Wh."
python3 online_romp/readjust.py --inp $inp_dir --out $out_dir --fig $fig_dir --offl_case_inp "iter0.csv" --offl_simu_inp "simu_res.txt" --onl_case_inp "re_iter0.csv" --onl_simu_inp "re_simu_res0.txt" --onl_case_out "re_iter1.csv" --onl_simu_out "re_simu_res1.txt" --new_eng $energy --num_pass 5 -v

echo "Online ROMP starts"
mpiexec -n 4 online_ROMP/cpp_romp "$inp_dir" "$out_dir" "re_iter1.csv" "re_simu_res1.txt" "re_simu_res2.txt" 50 80 80 5 0.75
python3 visual_readjust.py --inp "$inp_dir" --out "$out_dir" --fig "$fig_dir" --case_fname "re_iter1.csv" --res_fname "re_simu_res2.txt" --out_fname "iter_fig2.png" 
echo "Calculation done. Please check results in $out_dir and figures in $fig_dir".
