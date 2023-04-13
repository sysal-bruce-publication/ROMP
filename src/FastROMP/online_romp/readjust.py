"""Modify first iteration output (from offline ROMP) and update UAV energy."""
import numpy as np
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap

cMap = ListedColormap(['grey', 'seagreen', 'steelblue', 'coral', 'red'])


def update_inp(inp_dir: str, out_dir: str, fig_dir: str, case_fname: str,
               simu_fname: str, visual_check=True):
    # Read simulation results
    bh_sol, bh_eng, bh_time, bh_prize = [-2], [99.9], [0], [0]
    with open(out_dir + simu_fname, 'r') as rf:
        lines = rf.read().splitlines()
        time = lines[0]
        sol_line = lines[1].split(',')
        for i in range(len(sol_line)):
            bh_sol.append(int(sol_line[i]))
        bh_sol.append(-1)
        eng_line = lines[2].split(',')
        for i in range(len(eng_line)):
            bh_eng.append(float(eng_line[i]))
        time_line = lines[3].split(',')
        for i in range(len(time_line)):
            bh_time.append(float(time_line[i]))
        prize_line = lines[4].split(',')
        for i in range(len(prize_line)):
            bh_prize.append(float(prize_line[i]))
    bh_sol = np.array(bh_sol) + 2
    assert (len(bh_sol) == len(bh_eng) == len(bh_time) == len(bh_prize))
    # Read sensor node inputs
    inps, xs, ys, cs = [], [], [], []
    with open(inp_dir + case_fname, 'r') as rf:
        lines = rf.read().splitlines()
        line_data = []
        for line in lines[1:]:
            line = line.split(",")
            for i in range(len(line)):
                line_data.append(float(line[i]))
            xs.append(float(line[1]) / 100.)
            ys.append(float(line[2]) / 100.)
            cs.append(float(line[-1]))
            inps.append(np.array(line, dtype=np.float64))
    inps[0][0] = 0
    inps[-1][0] = 1
    inps.insert(1, inps[-1])
    inps.pop(len(inps) - 1)
    for i in range(2, len(inps)):
        inps[i][0] += 2
    # Write to new sensor node input files
    with open(inp_dir + "re_iter0.csv", 'w') as wf:
        wf.write("id,x_pos,y_pos,z_pos,p_flag,volts,weights\n")
        for i in range(len(inps)):
            for j in range(len(inps[i])):
                if j == len(inps[i]) - 1:
                    wf.write(str(int(inps[i][j])) + "\n")
                    break
                if j == len(inps[i]) - 2:
                    wf.write(str(inps[i][j]) + ",")
                else:
                    wf.write(str(int(inps[i][j])) + ",")
    # Write to new simulation result files
    with open(inp_dir + "re_simu_res0.txt", 'w') as wf:
        wf.write(time + "\n")
        for i in range(len(bh_sol)):  # black hole solution
            if i == len(bh_sol) - 1:
                wf.write(str(bh_sol[i]) + "\n")
                break
            wf.write(str(bh_sol[i]) + ",")
        for item in [bh_eng, bh_time]:  # energy and time
            for i in range(len(item)):
                if i == len(item) - 1:
                    wf.write(str(item[i]) + "\n")
                    break
                wf.write(str(item[i]) + ",")

    if visual_check:
        fig, axes = plt.subplots(1, 1, figsize=(12, 18), dpi=200)
        im1 = axes.scatter(xs, ys, c=cs, s=120, cmap=cMap)
        axes.set_xlim(-10, np.max(xs) + 50)
        axes.set_ylim(-10, np.max(ys) + 50)
        axes.set_xlabel('X coordinate (m)', fontsize=25)
        axes.set_ylabel('Y coordinate (m)', fontsize=25)
        axes.set_title('OR-Tools solution', fontsize=32)
        axes.tick_params(axis='both', which='major', labelsize=18)
        plt.subplots_adjust(right=0.85)
        cax = plt.axes([0.87, 0.143, 0.01, 0.7])
        cbar = plt.colorbar(im1, cax=cax)
        im1.set_clim(vmin=6, vmax=10)
        cbar.set_label('prize', fontsize=25)
        cbar.ax.tick_params(labelsize=30)
        cbar.ax.locator_params(nbins=5)
        sol_pts, weights = [], []
        for idx in bh_sol:
            sol_pts.append([inps[idx][1] / 100., inps[idx][2] / 100.])
            weights.append(inps[idx][-1])
        sol_xs = np.array([item[0] for item in sol_pts])
        sol_ys = np.array([item[1] for item in sol_pts])
        for id_x in range(len(sol_xs)):
            if id_x == len(sol_xs) - 1:
                break
            arrow = patches.FancyArrowPatch((sol_xs[id_x], sol_ys[id_x]),
                                            (sol_xs[id_x + 1], sol_ys[id_x + 1]),
                                            arrowstyle='->',
                                            mutation_scale=45)
            axes.add_artist(arrow)

        plt.savefig(fig_dir + "offline0.png")
        plt.close(fig)


def modify_pos(inp_dir: str, out_dir: str, fig_dir: str,
               case_fname: str, simu_fname: str, case_out_fname: str = None,
               res_out_fname: str = None, num_pass: int = 4,
               updated_eng: float = 50, visual_check: bool = True):
    # Read simulation results file
    bh_sol, bh_eng, bh_time = [], [], []
    with open(inp_dir + simu_fname, 'r') as rf:
        lines = rf.read().splitlines()
        time = lines[0]
        sol_line = lines[1].split(',')
        for i in range(len(sol_line)):
            bh_sol.append(int(sol_line[i]))
        eng_line = lines[2].split(',')
        for i in range(len(eng_line)):
            bh_eng.append(float(eng_line[i]))
        time_line = lines[3].split(',')
        for i in range(len(time_line)):
            bh_time.append(float(time_line[i]))
    # Read sensor node input file
    inps, xs, ys, cs = [], [], [], []
    with open(inp_dir + case_fname, 'r') as rf:
        lines = rf.read().splitlines()
        line_data = []
        for line in lines[1:]:
            line = line.split(",")
            for i in range(len(line)):
                line_data.append(float(line[i]))
            xs.append(float(line[1]) / 100.)
            ys.append(float(line[2]) / 100.)
            cs.append(float(line[-1]))
            inps.append(np.array(line, np.float64))
    # Modify the nodes
    for i in range(1, num_pass + 1):  # +1 because the first index is always 0
        idx = bh_sol[1]
        for j in range(2, len(bh_sol)):  # modify index in sol
            if bh_sol[j] > idx:
                bh_sol[j] = bh_sol[j] - 1
        for j in range(len(inps)):  # modify inp
            if int(inps[j][0]) > idx:
                inps[j][0] = inps[j][0] - 1
        for j in range(len(inps)):
            if int(inps[j][0]) == idx:
                inps[0][1] = inps[j][1]
                inps[0][2] = inps[j][2]
                del inps[j]
                break
        del bh_sol[1]
        del bh_eng[0]
        del bh_time[0]

        if visual_check and i == num_pass:
            fig, axes = plt.subplots(1, 1, figsize=(12, 18), dpi=200)
            im1 = axes.scatter(xs, ys, c=cs, s=120, cmap=cMap)
            axes.set_xlim(-10, np.max(xs) + 50)
            axes.set_ylim(-10, np.max(ys) + 50)
            axes.set_xlabel('X coordinate (m)', fontsize=25)
            axes.set_ylabel('Y coordinate (m)', fontsize=25)
            axes.set_title('OR-Tools solution', fontsize=32)
            axes.tick_params(axis='both', which='major', labelsize=18)
            plt.subplots_adjust(right=0.85)
            cax = plt.axes([0.87, 0.143, 0.01, 0.7])
            cbar = plt.colorbar(im1, cax=cax)
            im1.set_clim(vmin=6, vmax=10)
            cbar.set_label('prize', fontsize=25)
            cbar.ax.tick_params(labelsize=30)
            cbar.ax.locator_params(nbins=5)
            sol_pts, weights = [], []
            for idx in bh_sol:
                sol_pts.append([inps[idx][1] / 100., inps[idx][2] / 100.])
                weights.append(inps[idx][-1])
            sol_xs = np.array([item[0] for item in sol_pts])
            sol_ys = np.array([item[1] for item in sol_pts])
            for id_x in range(len(sol_xs)):
                if id_x == len(sol_xs) - 1:
                    break
                arrow = patches.FancyArrowPatch((sol_xs[id_x], sol_ys[id_x]),
                                                (sol_xs[id_x + 1], sol_ys[id_x + 1]),
                                                arrowstyle='->',
                                                mutation_scale=45)
                axes.add_artist(arrow)

            plt.savefig(fig_dir + "offline" + str(i) + ".png")
            plt.close(fig)

    inps[0][5] = bh_eng[0] if updated_eng is None else updated_eng
    with open(inp_dir + case_out_fname, 'w') as wf:
        wf.write("id,x_pos,y_pos,z_pos,p_flag,volts,weights\n")
        for i in range(len(inps)):
            for j in range(len(inps[i])):
                if j == len(inps[i]) - 1:
                    wf.write(str(int(inps[i][j])) + "\n")
                    break
                if j == len(inps[i]) - 2:
                    wf.write(str(inps[i][j]) + ",")
                else:
                    wf.write(str(int(inps[i][j])) + ",")

    with open(inp_dir + res_out_fname, 'w') as wf:
        wf.write(time + "\n")
        for i in range(len(bh_sol)):
            if i == len(bh_sol) - 1:
                wf.write(str(int(bh_sol[i])) + "\n")
                break
            wf.write(str(int(bh_sol[i])) + ",")
        for item in [bh_eng, bh_time]:
            for i in range(len(item)):
                if i == len(item) - 1:
                    wf.write(str(item[i]) + "\n")
                    break
                wf.write(str(item[i]) + ",")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--inp', type=str)
    parser.add_argument('--out', type=str)
    parser.add_argument('--fig', type=str, default=None)
    parser.add_argument('--offl_case_inp', type=str, default=None)
    parser.add_argument('--offl_simu_inp', type=str, default=None)
    parser.add_argument('--onl_case_inp', type=str, default=None)
    parser.add_argument('--onl_simu_inp', type=str, default=None)
    parser.add_argument('--onl_case_out', type=str, default=None)
    parser.add_argument('--onl_simu_out', type=str, default=None)
    parser.add_argument('--new_eng', type=float, default=None)
    parser.add_argument('--num_pass', type=int, default=None)
    parser.add_argument('-v', action="store_true")
    args = parser.parse_args()

    update_inp(inp_dir=args.inp, out_dir=args.out, fig_dir=args.fig,
               case_fname=args.offl_case_inp, simu_fname=args.offl_simu_inp,
               visual_check=args.v)
    modify_pos(inp_dir=args.inp, out_dir=args.out, fig_dir=args.fig,
               case_fname=args.onl_case_inp, simu_fname=args.onl_simu_inp,
               case_out_fname=args.onl_case_out, res_out_fname=args.onl_simu_out,
               updated_eng=args.new_eng, num_pass=args.num_pass, 
               visual_check=args.v)
    # update_inp(case_fname="inp/iter0.csv", simu_fname="out/simu_res0.txt")
    # modify_pos(case_fname="inp/re_iter0.csv", simu_fname="out/re_simu_res0.txt",
    #            case_out_fname="inp/re_iter1.csv", res_out_fname="out/re_simu_res1.txt",
    #            num_pass=5)
