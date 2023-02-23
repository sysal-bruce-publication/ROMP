import pandas as pd
import numpy as np
import argparse
import sys
import os

def modify_pos(case_fname, sol_fname, out_fname, sum_fname, num_visit):
    inp = pd.read_csv(case_fname)
        
    bha_sol = []
    with open(sol_fname, 'r') as f:
        lines = f.read().splitlines()
        bha_sol = np.array(lines[-1].split(','), dtype=np.int16)[1:]
    if num_visit >= len(bha_sol):
        print("When modifing UAV position, visited number is larger than solution length value.\n")
        sys.exit(-1)
    if num_visit == 0:
        col_name = ['time', 'or_m', 'bha_m', 'or_de', 'bha_de', 'all_de', 
            'or_re', 'bha_re', 'all_re', 'eff']
        sum_inp = pd.read_csv(sum_fname, names=col_name)
        if sum_inp['bha_de'].to_numpy()[-1] <= sum_inp['all_de'].to_numpy()[-1]:
            for idx in bha_sol:
                inp = inp.drop(inp[inp['id'] == idx].index)
            inp = inp.reset_index(drop=True)
            for i in range(1, len(inp['id']) - 1):
                inp.at[i, 'id'] = int(i - 1)
        else:
            for idx in bha_sol[:-1]:
                inp = inp.drop(inp[inp['id'] == idx].index)
            inp = inp.reset_index(drop=True)
            for i in range(1, len(inp['id']) - 1):
                inp.at[i, 'id'] = int(i - 1)
        inp.to_csv(out_fname, index=False)
        if len(inp['id']) <= 2:
            sys.exit(0x08)
        # elif len(inp['id']) <= 6:
        #     sys.exit(0x09)
        else:
            sys.exit(0x00)
    else:   
        inp.at[0, 'x_pos'] = inp.loc[inp['id'] == bha_sol[num_visit - 1], 'x_pos'].values[0]
        inp.at[0, 'y_pos'] = inp.loc[inp['id'] == bha_sol[num_visit - 1], 'y_pos'].values[0]
        for idx in bha_sol[:num_visit]:
            inp = inp.drop(inp[inp['id'] == idx].index)
        inp = inp.reset_index(drop=True)
        for i in range(1, len(inp['id']) - 1):
            inp.at[i, 'id'] = int(i - 1)
        inp.to_csv(out_fname, index=False)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Modify UAV pos based on BHA solution." + 
        " It will remove visited nodes from case file.")
    parser.add_argument("--inp", dest="inp_dir", type=str)
    parser.add_argument("--out", dest="out_dir", type=str)
    parser.add_argument("--case", dest="case_fname", type=str,
        default="", help="Initial node file name.")
    parser.add_argument("--sol", dest="sol_fname", type=str,
        default="plan.txt", help="BHA solution file name.")
    parser.add_argument("--sum", dest="sum_fname", type=str,
        default="summary.csv", help="Simulation summary file name.")
    parser.add_argument("--fname", dest="out_fname", type=str,
        default="", help="Modified node file name.")    
    parser.add_argument("--num", dest="num_visit", type=int,
                        default=0, help="The nodes UAV has visited.")
    args = parser.parse_args()

    modify_pos(case_fname=os.path.join(args.inp_dir, args.case_fname), 
        sol_fname=os.path.join(args.out_dir, args.sol_fname),
	    out_fname=os.path.join(args.inp_dir, args.out_fname), 
        sum_fname=os.path.join(args.out_dir, args.sum_fname), 
        num_visit=args.num_visit)
