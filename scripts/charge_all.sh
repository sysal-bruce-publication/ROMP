#!/usr/bin/env bash

# Build needed files
clear
# Uncomment below code if you want to rebuild all programs
# cd init_solver
# sudo bazel build --cxxopt="-std=c++2a" //main:all
# echo "Build OR-Tools done."
# cd ../opt_solver
# mpicxx -std=c++2a *.cpp -o cpp_bha
# echo "Build BHA done."
# cd ../

echo "Simulation start."
# 40 50 60 70 80 90 100 110 
for case_cnt in 110 
do
    echo "Case $case_cnt starts:"
    temp_cnt=1
    while [ $temp_cnt -le 20 ]
    do
        # echo "Attemp $temp_cnt starts:"
        # temp_in_dir="inp/iter/case$case_cnt/temp$temp_cnt/"
        # if [ ! -d $temp_in_dir ]; then
        #     mkdir $temp_in_dir

            # prev_cnt=$(($temp_cnt-1))
            # mv inp/iter/case$case_cnt/temp$prev_cnt/iter0.csv $temp_in_dir
            # mv inp/iter/case$case_cnt/temp$prev_cnt/grid_info.txt $temp_in_dir
            # mv inp/iter/case$case_cnt/temp$prev_cnt/wind_vector.csv $temp_in_dir
        # fi
        # temp_out_dir="out/iter/case$case_cnt/temp$temp_cnt/"
        # if [ ! -d $temp_out_dir ]; then
        #     mkdir $temp_out_dir
        # fi
        # temp_fig_dir="fig/iter/case$case_cnt/temp$temp_cnt/"
        # if [ ! -d $temp_fig_dir ]; then
        #     mkdir $temp_fig_dir
        # fi
        # # Initial calculation (before take-off)
        # sudo init_solver/bazel-bin/main/cpp_ortools "$temp_in_dir" "$temp_out_dir" "iter0.csv" "init_sol0.txt" 0 0 0 1
        # mpiexec -n 4 opt_solver/cpp_bha "$temp_in_dir" "$temp_out_dir" "iter0.csv" "init_sol0.txt" 0 50 80 80 75 10 100 50
        # bha_exit_status=$?        
        iter_cnt=0
        python3 visual_data.py --inp "$temp_in_dir" --out_dir "$temp_out_dir" --fig "$temp_fig_dir" --case "iter0.csv" --or "init_sol0.txt" --bha "plan.txt" --out "iter_fig0.png" 
        while [[ $iter_cnt -le 10 && $bha_exit_status -ne 10 ]] 
        do
            # # First change
            # python3 edit_uav_pos.py --inp "$temp_in_dir" --out "$temp_out_dir" --case "iter$iter_cnt.csv" --fname "iter$(($iter_cnt+1)).csv"
            # exit_status=$?
            # if [ $exit_status -eq 8 ]; then
            #     # echo "$(($iter_cnt+1)) UAVs needed."
            #     break
            # elif [ $exit_status -eq 9 ]; then
            #     # echo "$(($iter_cnt+2)) UAVs needed."
            #     break
            # # else
            #     # echo "Update charging list."
            # fi
            # echo "Iter $(($iter_cnt+1))"
            # sudo init_solver/bazel-bin/main/cpp_ortools "$temp_in_dir" "$temp_out_dir" "iter$(($iter_cnt+1)).csv" "init_sol$(($iter_cnt+1)).txt" 0 0 0 1
            # mpiexec -n 4 opt_solver/cpp_bha "$temp_in_dir" "$temp_out_dir" "iter$(($iter_cnt+1)).csv" "init_sol$(($iter_cnt+1)).txt" 0 50 80 80 75 10 100 50
            # python3 visual_data.py --inp "$temp_in_dir" --out_dir "$temp_out_dir" --fig "$temp_fig_dir" --case "iter$(($iter_cnt+1)).csv" --or "init_sol$(($iter_cnt+1)).txt" --bha "plan.txt" --out "iter_fig$(($iter_cnt+1)).png"
            ((iter_cnt++))
        done
        echo "Attempt $temp_cnt done."
        ((temp_cnt++))
    done
done