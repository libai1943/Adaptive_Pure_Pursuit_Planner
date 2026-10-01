"""Render verified APP trajectories with distance-colored footprints and bend details."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import numpy as np


def render(scene_file, result_dir, output, parameters):
    config = {k: float(v) for k, v in (
        line.split() for line in parameters.read_text().splitlines()
        if line.strip() and not line.startswith('#'))}
    scene = np.fromstring(scene_file.read_text(), sep=' ')
    obstacles = scene[11:].reshape(-1, 3, 2)
    report = json.loads((result_dir/'summary.json').read_text())
    if not report['success']:
        raise ValueError('Cannot present an unsuccessful planning run as successful.')
    path = np.loadtxt(result_dir/'dense_path.csv', delimiter=',', skiprows=1)
    route = np.loadtxt(result_dir/'initial_route.csv', delimiter=',', skiprows=1)
    history = np.loadtxt(result_dir/'history.csv', delimiter=',', skiprows=1, ndmin=2)
    distance = path[:, 0]*config['speed']
    length = distance[-1]
    cmap = plt.get_cmap('turbo')
    norm = Normalize(0, length)
    samples = np.unique(np.r_[np.arange(0, len(path), 10), len(path)-1])
    points = path[samples, 1:3]
    segments = np.stack([points[:-1], points[1:]], axis=1)
    segment_distance = distance[samples[:-1]]
    rear, front, half = config['rear_overhang'], config['length']-config['rear_overhang'], config['width']/2
    local = np.array([[-rear,-half],[front,-half],[front,half],[-rear,half]])
    spacing = config['simulation_dt']/config['integration_steps']*config['speed']
    body_indices = np.arange(0, len(path), max(1,round(2.8/spacing)))
    bodies = []
    for row in path[body_indices]:
        c, s = np.cos(row[3]), np.sin(row[3])
        bodies.append(local@np.array([[c,s],[-s,c]])+row[1:3])
    colors = cmap(norm(distance[body_indices]))
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,
        'axes.spines.top':False,'axes.spines.right':False,
        'axes.labelcolor':'#596574','xtick.color':'#72808b','ytick.color':'#72808b'})
    fig = plt.figure(figsize=(14,9),facecolor='white')
    ax = fig.add_axes([.055,.17,.64,.69])
    top = fig.add_axes([.755,.53,.215,.29])
    bottom = fig.add_axes([.755,.195,.215,.29])

    def draw(axis, limits, linewidth):
        axis.set_facecolor('#fbfcfd')
        axis.add_collection(PolyCollection(obstacles,facecolors='#e2e7eb',edgecolors='none',
            antialiaseds=False,rasterized=True,zorder=1))
        axis.plot(route[:,0],route[:,1],color='#7a8996',lw=1.05,ls=(0,(3,3)),alpha=.8,zorder=3)
        fill = colors.copy(); fill[:,3] = .06
        edge = colors.copy(); edge[:,3] = .62
        axis.add_collection(PolyCollection(bodies,facecolors=fill,edgecolors=edge,linewidths=.65,zorder=4))
        axis.plot(points[:,0],points[:,1],color='white',lw=linewidth+1.6,zorder=5)
        lines = LineCollection(segments,cmap=cmap,norm=norm,linewidths=linewidth,zorder=6)
        lines.set_array(segment_distance); lines.set_capstyle('round'); axis.add_collection(lines)
        axis.set(xlim=limits[:2],ylim=limits[2:],aspect='equal')
        for side in ['left','bottom']:
            axis.spines[side].set_color('#d9e0e5')
        axis.tick_params(length=3,labelsize=9)
        return lines

    lines = draw(ax,[-10,79,-45,19],2.9)
    ax.set(xlabel='x (m)',ylabel='y (m)')
    ax.scatter(*path[0,1:3],s=74,c=[cmap(0.)],edgecolors='white',linewidths=1.8,zorder=8)
    ax.scatter(*scene[7:9],s=135,c=[cmap(1.)],marker='*',edgecolors='white',linewidths=.8,zorder=8)
    ax.text(-6,3.1,'START',fontsize=9,fontweight='bold',color='#314762')
    ax.text(65,-41.3,'GOAL',fontsize=9,fontweight='bold',color='#9d292b')
    for travelled in [28,67,105,142,173]:
        row = path[np.argmin(abs(distance-travelled))]
        direction = np.array([np.cos(row[3]),np.sin(row[3])])
        ax.annotate('',xy=row[1:3]+direction*1.3,xytext=row[1:3]-direction*1.3,
            arrowprops={'arrowstyle':'-|>','color':cmap(norm(travelled)),'lw':1.5},zorder=9)
    for bounds, label in [([47,71,1,19],'A'),([27,54,-42,-24],'B')]:
        ax.add_patch(Rectangle((bounds[0],bounds[2]),bounds[1]-bounds[0],bounds[3]-bounds[2],
            fill=False,edgecolor='#7f8fa0',ls=(0,(3,3)),lw=.75,zorder=2))
        ax.text(bounds[0]+.7,bounds[3]-2,label,color='#42586b',fontsize=10,fontweight='bold')
    draw(top,[48,70,1,18],3.4)
    draw(bottom,[29,52,-42,-25],3.4)
    top.set_title('A  /  Tight upper bend',loc='left',fontsize=11,pad=12,fontweight='bold',color='#294357')
    bottom.set_title('B  /  Lower hairpin',loc='left',fontsize=11,pad=12,fontweight='bold',color='#294357')
    fig.text(.055,.948,'ADAPTIVE PURE PURSUIT',fontsize=24,fontweight='bold',color='#183548')
    fig.text(.055,.91,'Chapter 8  /  Planning through narrow, curved corridors',fontsize=12,color='#637485')
    fig.text(.968,.943,'ACTUAL RUN\nBENCHMARK 005',ha='right',va='top',fontsize=10,color='#637485',linespacing=1.7)
    legend = [Line2D([0],[0],color='#7a8996',ls='--',lw=1.2,label='A* initial guide'),
              Rectangle((0,0),1,1,fill=False,edgecolor='#329396',lw=1,label='Vehicle footprints')]
    ax.legend(handles=legend,loc='lower left',frameon=True,facecolor='white',edgecolor='none',fontsize=9)
    colorbar = fig.colorbar(lines,cax=fig.add_axes([.075,.108,.60,.015]),orientation='horizontal')
    colorbar.outline.set_visible(False)
    colorbar.set_label('Travelled distance along the APP path (m)',fontsize=10,labelpad=6)
    colorbar.set_ticks([0,40,80,120,160,length]); colorbar.ax.tick_params(length=2,labelsize=9)
    fig.text(.758,.124,f'{length:.1f} m',fontsize=20,fontweight='bold',color='#183548')
    fig.text(.885,.124,f'{len(history)} iterations',fontsize=15,fontweight='bold',color='#183548')
    fig.text(.758,.095,f'{len(path):,} integrated poses checked',fontsize=10,color='#637485')
    fig.text(.055,.028,'Rainbow encodes path progress. Footprints and bends come from the computed trajectory; obstacle geometry is unchanged.',fontsize=9,color='#78848e')
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=190,facecolor='white')
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene',type=Path); parser.add_argument('result',type=Path); parser.add_argument('output',type=Path)
    parser.add_argument('--parameters',type=Path,default=Path(__file__).resolve().parents[1]/'data/paper_parameters.txt')
    args = parser.parse_args(); render(args.scene,args.result,args.output,args.parameters)
