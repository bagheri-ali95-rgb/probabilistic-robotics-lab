#!/usr/bin/env python3
"""
Offline evaluation and plotting tool.

Usage
-----
  # Compare two motion model runs:
  python3 plot_results.py ~/eval_simple_<ts>.csv ~/eval_diff_<ts>.csv \
      --labels "Simple" "Diff-Drive" --out results/

  # Single run:
  python3 plot_results.py ~/eval_run_<ts>.csv

Output
------
  trajectory_comparison.png  – 2-D paths for GT, KF, EKF, PF
  rmse_bar.png               – RMSE bar chart per filter
  error_over_time.png        – RMSE over time for x and y
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')           # no display needed
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# ── helpers ────────────────────────────────────────────────────────────────

def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df['time'] -= df['time'].iloc[0]   # relative time
    return df


def rmse_col(df: pd.DataFrame, est: str, gt: str) -> float:
    return float(np.sqrt(((df[est] - df[gt]) ** 2).mean()))


def rmse_table(df: pd.DataFrame) -> dict:
    return {
        'KF':  {'x': rmse_col(df, 'kf_x',  'gt_x'),
                'y': rmse_col(df, 'kf_y',  'gt_y'),
                'th': rmse_col(df, 'kf_th', 'gt_th')},
        'EKF': {'x': rmse_col(df, 'ekf_x', 'gt_x'),
                'y': rmse_col(df, 'ekf_y', 'gt_y'),
                'th': rmse_col(df, 'ekf_th','gt_th')},
        'PF':  {'x': rmse_col(df, 'pf_x',  'gt_x'),
                'y': rmse_col(df, 'pf_y',  'gt_y'),
                'th': rmse_col(df, 'pf_th', 'gt_th')},
    }


# ── plot functions ──────────────────────────────────────────────────────────

COLORS = {'GT': 'black', 'KF': 'blue', 'EKF': 'orange', 'PF': 'green'}


def plot_trajectories(dfs: list, labels: list, out_dir: str):
    n = len(dfs)
    fig, axes = plt.subplots(1, n, figsize=(7 * n, 6), squeeze=False)

    for ax, df, label in zip(axes[0], dfs, labels):
        ax.plot(df['gt_x'],  df['gt_y'],  color=COLORS['GT'],  lw=2,   label='Ground Truth')
        ax.plot(df['kf_x'],  df['kf_y'],  color=COLORS['KF'],  lw=1.5, label='KF')
        ax.plot(df['ekf_x'], df['ekf_y'], color=COLORS['EKF'], lw=1.5, label='EKF')
        ax.plot(df['pf_x'],  df['pf_y'],  color=COLORS['PF'],  lw=1.5, label='PF')
        ax.set_title(label, fontsize=13)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.legend()
        ax.grid(True)
        ax.set_aspect('equal')

    fig.suptitle('Trajectory Comparison', fontsize=15)
    fig.tight_layout()
    path = os.path.join(out_dir, 'trajectory_comparison.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'Saved {path}')


def plot_rmse_bar(dfs: list, labels: list, out_dir: str):
    filters = ['KF', 'EKF', 'PF']
    dims    = ['x', 'y', 'th']
    dim_labels = ['RMSE x [m]', 'RMSE y [m]', 'RMSE θ [rad]']

    fig, axes = plt.subplots(1, 3, figsize=(14, 5))

    for ax, dim, dlabel in zip(axes, dims, dim_labels):
        x = np.arange(len(filters))
        width = 0.8 / len(dfs)
        for i, (df, run_label) in enumerate(zip(dfs, labels)):
            rmse = rmse_table(df)
            vals = [rmse[f][dim] for f in filters]
            ax.bar(x + i * width, vals, width, label=run_label)
        ax.set_xticks(x + width * (len(dfs) - 1) / 2)
        ax.set_xticklabels(filters)
        ax.set_ylabel(dlabel)
        ax.set_title(dlabel)
        ax.legend()
        ax.grid(axis='y')

    fig.suptitle('RMSE per Filter', fontsize=15)
    fig.tight_layout()
    path = os.path.join(out_dir, 'rmse_bar.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'Saved {path}')


def plot_error_over_time(dfs: list, labels: list, out_dir: str):
    fig, axes = plt.subplots(2, len(dfs), figsize=(7 * len(dfs), 8), squeeze=False)

    for col, (df, label) in enumerate(zip(dfs, labels)):
        for ax_row, (dim, gt) in enumerate([('x', 'gt_x'), ('y', 'gt_y')]):
            ax = axes[ax_row][col]
            ax.plot(df['time'], np.abs(df[f'kf_{dim}']  - df[gt]), color=COLORS['KF'],  label='KF')
            ax.plot(df['time'], np.abs(df[f'ekf_{dim}'] - df[gt]), color=COLORS['EKF'], label='EKF')
            ax.plot(df['time'], np.abs(df[f'pf_{dim}']  - df[gt]), color=COLORS['PF'],  label='PF')
            ax.set_title(f'{label} – |error {dim}|')
            ax.set_xlabel('time [s]')
            ax.set_ylabel(f'|e_{dim}| [m]')
            ax.legend()
            ax.grid(True)

    fig.suptitle('Absolute Error Over Time', fontsize=15)
    fig.tight_layout()
    path = os.path.join(out_dir, 'error_over_time.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f'Saved {path}')


def print_rmse_table(dfs: list, labels: list):
    print('\n' + '=' * 62)
    print(f'{"":12s}  {"KF x":>8s}  {"KF y":>8s}  {"EKF x":>8s}  {"EKF y":>8s}  {"PF x":>8s}  {"PF y":>8s}')
    print('-' * 62)
    for df, label in zip(dfs, labels):
        r = rmse_table(df)
        print(f'{label[:12]:12s}  '
              f'{r["KF"]["x"]:8.4f}  {r["KF"]["y"]:8.4f}  '
              f'{r["EKF"]["x"]:8.4f}  {r["EKF"]["y"]:8.4f}  '
              f'{r["PF"]["x"]:8.4f}  {r["PF"]["y"]:8.4f}')
    print('=' * 62 + '\n')


# ── main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Plot filter evaluation results')
    parser.add_argument('csv_files', nargs='+', help='CSV files from evaluator_node')
    parser.add_argument('--labels', nargs='+', help='Legend labels (one per CSV)')
    parser.add_argument('--out', default='.', help='Output directory for plots')
    args = parser.parse_args()

    if args.labels and len(args.labels) != len(args.csv_files):
        print('ERROR: number of --labels must match number of CSV files')
        sys.exit(1)

    labels = args.labels if args.labels else [os.path.basename(f) for f in args.csv_files]
    os.makedirs(args.out, exist_ok=True)

    dfs = [load(f) for f in args.csv_files]

    print_rmse_table(dfs, labels)
    plot_trajectories(dfs, labels, args.out)
    plot_rmse_bar(dfs, labels, args.out)
    plot_error_over_time(dfs, labels, args.out)

    print(f'\nAll plots saved to {args.out}/')


if __name__ == '__main__':
    main()
