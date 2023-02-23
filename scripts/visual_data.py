import pandas as pd
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap
from mpl_toolkits.axes_grid1 import make_axes_locatable

cMap = ListedColormap(['grey', 'seagreen', 'steelblue', 'coral','red'])

def plot_init_final_sol(case_fname, or_fname, bha_fname, out_fname, inc_obs):
    inp = pd.read_csv(case_fname)
    xs = inp['x_pos'].to_numpy()[1:-1] / 100.
    ys = inp['y_pos'].to_numpy()[1:-1] / 100.
    cs = inp['weights'].to_numpy()[1:-1]
    x_max = np.max(xs)
    y_max = np.max(ys)
    
    fig, axes = plt.subplots(1, 2, 
                            gridspec_kw={'width_ratios':[1.07, 1]}, 
                            figsize=(28, 18))
    im1 = axes[0].scatter(xs, ys, c=cs, s=120, cmap=cMap)
    axes[0].set_xlim(-10, x_max + 50)
    axes[0].set_ylim(-10, y_max + 50)
    axes[0].set_xlabel('X coordinate (m)', fontsize=25)
    axes[0].set_ylabel('Y coordinate (m)', fontsize=25)
    axes[0].set_title('OR-Tools solution', fontsize=32)
    axes[0].tick_params(axis='both', which='major', labelsize=18)
    im2 = axes[1].scatter(xs, ys, c=cs, s=120, cmap=cMap)
    axes[1].set_xlim(-10, x_max + 50)
    axes[1].set_ylim(-10, y_max + 50)
    axes[1].set_xlabel('X coordinate (m)', fontsize=25)
    axes[1].set_ylabel('Y coordinate (m)', fontsize=25)
    axes[1].set_title('BHA solution', fontsize=32)
    axes[1].tick_params(axis='both', which='major', labelsize=18)
    for i in range(1, len(inp['x_pos']) - 1):
        axes[0].text(inp['x_pos'][i] / 100. + 20,
                inp['y_pos'][i] / 100. + 20,
                s="{}".format(inp['weights'][i]),
                fontdict=dict(color='black', alpha=0.6, size=30))
        axes[1].text(inp['x_pos'][i] / 100. + 20,
                inp['y_pos'][i] / 100. + 20,
                s="{}".format(inp['weights'][i]),
                fontdict=dict(color='black', alpha=0.6, size=30))
    
    plt.subplots_adjust(right=0.85)
    cax = plt.axes([0.87, 0.143, 0.01, 0.7])
    cbar = plt.colorbar(im2, cax=cax)
    im1.set_clim(vmin=6, vmax=10)
    im2.set_clim(vmin=6, vmax=10)
    cbar.set_label('prize', fontsize=25)
    cbar.ax.tick_params(labelsize=30)
    cbar.ax.locator_params(nbins=5)

    ################ Plot initial solution ################
    sol = []
    with open(or_fname, 'r') as f:
        line = f.readline()
        sol = np.array(line.split(','), dtype=np.int16)

    sol_pts=[[inp['x_pos'][0]/100., inp['y_pos'][0]/100.]]
    weights = [0]
    for idx in sol:
        sol_pts.append([inp['x_pos'][idx + 1]/100.,
                       inp['y_pos'][idx + 1]/100.])
        weights.append(inp['weights'][idx + 1])
    sol_pts.append([inp['x_pos'].to_numpy()[-1]/100.,
                    inp['y_pos'].to_numpy()[-1]/100.])
    weights.append(0)    
    xs = np.array([item[0] for item in sol_pts])
    ys = np.array([item[1] for item in sol_pts])
    for id_x in range(len(xs)):
        if id_x == len(xs) - 1:
            break
        arrow = patches.FancyArrowPatch((xs[id_x], ys[id_x]),
                                        (xs[id_x + 1], ys[id_x + 1]),
                                        arrowstyle='->', 
                                        mutation_scale=45)
        axes[0].add_artist(arrow)
    
    ################ Plot BHA solution ################
    final_sol = []
    with open(bha_fname, 'r') as f:
        lines = f.read().splitlines()
        final_sol = np.array(lines[-1].split(','), dtype=np.int16)[1:]

    sol_pts=[[inp['x_pos'][0]/100., inp['y_pos'][0]/100.]]
    weights = [0]
    for idx in final_sol:
        sol_pts.append([inp['x_pos'][idx + 1]/100.,
                       inp['y_pos'][idx + 1]/100.])
        weights.append(inp['weights'][idx + 1])
    sol_pts.append([inp['x_pos'].to_numpy()[-1]/100.,
                    inp['y_pos'].to_numpy()[-1]/100.])
    weights.append(0)    
    xs = np.array([item[0] for item in sol_pts])
    ys = np.array([item[1] for item in sol_pts])
    for id_x in range(len(xs)):
        if id_x == len(xs) - 1:
            break
        arrow = patches.FancyArrowPatch((xs[id_x], ys[id_x]),
                                        (xs[id_x + 1], ys[id_x + 1]),
                                        arrowstyle='->', 
                                        mutation_scale=45)
        axes[1].add_artist(arrow)
    if inc_obs:
        obs = []
        with open("./inp/obs_info.txt", 'r') as f:
            line = f.readline()
            obs = np.array(line.split(','), dtype=np.int32)/100.

        rect0 = patches.Rectangle((obs[0], obs[1]), obs[2]-obs[0], 
                                 obs[3]-obs[1], linewidth=8,
                                 edgecolor='black', facecolor='none',
                                 label="Obstacle")
        rect1 = patches.Rectangle((obs[0], obs[1]), obs[2]-obs[0], 
                                 obs[3]-obs[1], linewidth=8,
                                 edgecolor='black', facecolor='none', 
                                  label="Obstacle")
        axes[0].add_artist(rect0)
        axes[1].add_artist(rect1)
        
    plt.savefig(out_fname)
    plt.close(fig)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--inp", dest="inp_dir", type=str)
    parser.add_argument("--out_dir", dest="out_dir", type=str)
    parser.add_argument("--fig", dest="fig_dir", type=str)
    parser.add_argument("--case", dest="case_fname", type=str, 
                        help="Initial node file name")
    parser.add_argument("--or", dest="or_fname", type=str,
                        help="OR solution file name")
    parser.add_argument("--bha", dest="bha_fname", type=str,
                        default="plan.txt", help="BHA solution file name")
    parser.add_argument("--out", dest="out_fname", type=str,
                        help="Figure file name")
    parser.add_argument("--obs", dest="inc_obs", type=int,
                        default=0, help="0: not include obstacle")
    args = parser.parse_args()
    
    plot_init_final_sol(os.path.join(args.inp_dir, args.case_fname), 
                        os.path.join(args.inp_dir, args.or_fname),
                        os.path.join(args.out_dir, args.bha_fname),
                        os.path.join(args.fig_dir, args.out_fname),
                        args.inc_obs)
