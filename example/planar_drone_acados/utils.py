#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import numpy as np
from acados_template import latexify_plot


def plot_planar(t, U, X_true, X_est=None, Y_measured=None, latexify=False, plt_show=True, X_true_label=None):
    """
    Params:
        t: time values of the discretization
        u_max: maximum absolute value of u
        U: arrray with shape (N_sim-1, nu) or (N_sim, nu)
        X_true: arrray with shape (N_sim, nx)
        X_est: arrray with shape (N_sim-N_mhe, nx)
        Y_measured: array with shape (N_sim, ny)
        latexify: latex style plots
    """

    if latexify:
        latexify_plot()

    WITH_ESTIMATION = X_est is not None and Y_measured is not None

    N_sim = X_true.shape[0]
    nx = X_true.shape[1]
    nu = U.shape[1]

    Tf = t[N_sim-1]
    Ts = t[1] - t[0]

    if WITH_ESTIMATION:
        N_mhe = N_sim - X_est.shape[0]
        t_mhe = np.linspace(N_mhe * Ts, Tf, N_sim-N_mhe)

    plt.figure()
    plt.subplot(nu, 1, 1)
    line, = plt.step(t, np.append([U[0,0]], U[:,0]))
    if X_true_label is not None:
        line.set_label(X_true_label)
    else:
        line.set_color('r')

    plt.ylabel('$u1$')
    plt.xlabel('$t$')
    plt.xlim(t[0], t[-1])
    plt.grid()


    plt.subplot(nu, 1, 2)
    line, = plt.step(t, np.append([U[0,1]], U[:,1]))
    if X_true_label is not None:
        line.set_label(X_true_label)
    else:
        line.set_color('r')

    plt.ylabel('$u2$')
    plt.xlabel('$t$')
    plt.xlim(t[0], t[-1])
    plt.grid()


    plt.figure()
    states_lables = ['$x$', '$y$', r'$\theta$', '$vx$', '$vy$',r'$\dot{\theta}$']

    for i in range(int(nx/2)):
        plt.subplot(int(nx/2), 1, i+1)
        line, = plt.plot(t, X_true[:, i])
        if X_true_label is not None:
            line.set_label(X_true_label)

        if WITH_ESTIMATION:
            plt.plot(t_mhe, X_est[:, i], '--', label='estimated')
            plt.plot(t, Y_measured[:, i], 'x', label='measured')

        plt.ylabel(states_lables[i])
        plt.xlabel('$t$')
        plt.grid()
        plt.xlim(t[0], t[-1])

    plt.figure()
    for i in range(int(nx/2), nx):
        plt.subplot(int(nx/2), 1, i-int(nx/2)+1)
        line, = plt.plot(t, X_true[:, i])
        if X_true_label is not None:
            line.set_label(X_true_label)

        if WITH_ESTIMATION:
            plt.plot(t_mhe, X_est[:, i], '--', label='estimated')
            plt.plot(t, Y_measured[:, i], 'x', label='measured')

        plt.ylabel(states_lables[i])
        plt.xlabel('$t$')
        plt.grid()
        plt.xlim(t[0], t[-1])

    plt.subplots_adjust(left=None, bottom=None, right=None, top=None, hspace=0.4)


    plt.figure()
    plt.plot(X_true[:,0],X_true[:,1])

    # Create circle object
    circle1 = plt.Circle((1, -1), 0.5, color='r', fill=False)
    circle2 = plt.Circle((4, -2), 0.5, color='r', fill=False)

    for i in range(0,N_sim,int(N_sim/30)):
        p = [X_true[i,0],X_true[i,1]]
        p1 = [p[0]+np.cos(X_true[i,2]),p[0]-np.cos(X_true[i,2])]
        p2 = [p[1]+np.sin(X_true[i,2]),p[1]-np.sin(X_true[i,2])]

        l = mlines.Line2D(p1,p2, color='g', linestyle='--')
        plt.plot(p[0],p[1],'go')
        plt.gca().add_line(l)


    # Add circle to plot
    plt.gca().add_patch(circle1)
    plt.gca().add_patch(circle2)
    
    plt.xlabel('x')
    plt.ylabel('y')

    if plt_show:
        plt.show()
