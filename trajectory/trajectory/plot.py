from trajectory import *
import matplotlib.pyplot as plt
from IPython.display import display

import trajectory as tj
import pandas as pd


def sel_axis(df, axis):
    t = df[df.axis == axis].copy()

    t = pd.concat([t.iloc[0:1], t]).reset_index()
    t.at[0, 't'] = 0
    t.at[0, 'v_f'] = 0
    t = t.set_index('t')
    t = t[['v_f', 'ss', 'del_t']]
    return t


def plot_axis(df, axis, ax=None):
    t = sel_axis(df, axis)

    ax = t[['v_f']].plot(ax=ax, figsize=(20, 3))

    return ax


def plot_segment_list(df):
    ax = None

    for axn in df.axis.unique():
        ax = plot_axis(df, axn, ax=ax)

    # Map sub-segment names ( a,c,d) to matplotlib colors.
    # The color here is actually he one that edns the segment, but it is
    # interpreted as the one that starts it.
    cm = {'a': 'b',
          'c': 'r',
          'd': 'g'}

    for idx, row in sel_axis(df, 0).iterrows():
        #rectangle = plt.Rectangle((idx,0), row.del_t, row.v_f, fc=cm[row.ss], alpha=0.05)
        #plt.gca().add_patch(rectangle)

        ax.axvline(x=idx, color=cm[row.ss], alpha=.5, lw= 3 if row.ss == 'd' else 1)