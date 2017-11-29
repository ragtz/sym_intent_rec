#!/usr/bin/env python

import numpy as np
from scipy.optimize import approx_fprime

def get_line_traj(s, g, n):
    return np.c_[[np.linspace(s[i], g[i], n) for i in range(len(s))]].T 

def off_diag(M, d, v):
    D = np.diag(M, d)
    D.setflags(write=True)
    D.fill(v)
    return M

def get_M(n):
    M = 2*np.diag(np.ones(n))
    off_diag(M, 1, -1)
    off_diag(M, -1, -1)
    M[0,1] = 0
    M[-1,-2] = 0
    return np.matrix(M)

def get_C(traj):
    if len(traj) > 0:
        return 0.5*np.square(np.linalg.norm(np.append(traj[1:], traj[-1][np.newaxis], axis=0)  - traj, axis=1)).sum()
    else:
        return 0

def get_V(s, g, n):
    traj = get_line_traj(s, g, n)
    return get_C(traj)

def get_Vs(traj, g):
    n = len(traj)
    return np.array([get_V(traj[i], g, n-i) for i in range(len(traj))])

def get_Vp(s, g, n):
    if n > 1:
        traj = get_line_traj(s, g, n)
        return traj[1] - traj[0]
    else:
        return np.zeros(len(s))

def get_Vps(traj, g):
    n = len(traj)
    return np.array([get_Vp(traj[i], g, n-i) for i in range(len(traj))])

def grad_legibility(traj, objs, g, probs, f):
    V = np.array([get_Vs(traj, objs[g_i]) for g_i in range(len(objs))])
    Vp = np.array([get_Vps(traj, objs[g_i]) for g_i in range(len(objs))])

    grad = []
    for t in range(traj.shape[0]):
        if t > 0 and t < traj.shape[0]-1:
            exp_V_diffs = np.array([np.exp(V[g_i][0] - V[g_i][t]) for g_i in range(len(objs))])
            exp_neg_V = np.array([np.exp(-V[g_i][t]) for g_i in range(len(objs))])
            exp_neg_V_s = np.array([np.exp(-V[g_i][0]) for g_i in range(len(objs))])
            Vp_diffs = np.array([Vp[g_i][t] - Vp[g][t] for g_i in range(len(objs))])

            a = (exp_V_diffs[g]*probs[g]) / np.sum(exp_V_diffs*probs)**2
            b = (np.tile((exp_neg_V*probs) / exp_neg_V_s, (traj.shape[1],1)).T*Vp_diffs).sum(0)

            grad_t = a*b*f[t]
        else:
            grad_t = np.zeros(traj.shape[1])

        grad.append(grad_t)

    return np.matrix(grad)

def legibility_funct(objs, g, probs, f):
    def L(traj, n_wp):
        traj = traj.reshape(n_wp, len(traj)/n_wp)

        T = traj.shape[0]
        N = len(objs)

        C = np.array([get_C(traj[:t]) for t in range(T)])
        V = np.array([get_Vs(traj, objs[g_i]) for g_i in range(N)])

        P = np.array([[(np.exp(-C[t] - V[g_i][t])*probs[g_i]) / np.exp(-V[g_i][0]) for g_i in range(N)] for t in range(T)])
        Z = np.array([P[t].sum() for t in range(T)])
        P = P[:,g]/Z

        return (P*f).sum() / f.sum() 
    return L

def approx_grad_legibility(traj, objs, g, probs, f):
    L = legibility_funct(objs, g, probs, f)
    n_wp = traj.shape[0]
    traj = traj.flatten()
    eps = np.sqrt(np.finfo(float).eps)
    return approx_fprime(traj, L, eps, n_wp).reshape(n_wp, len(traj)/n_wp)

def plan(s, objs, g, eta, n_wp, n_iter):
    M_inv = np.linalg.inv(get_M(n_wp))
    P_g = np.ones(len(objs))/len(objs)

    traj = get_line_traj(s, objs[g], n_wp)
    f = np.array(range(len(traj)-1,-1,-1))

    for i in range(n_iter):
        #print '---------- traj ----------'
        #print traj

        grad = grad_legibility(traj, objs, g, P_g, f)
        #grad = approx_grad_legibility(traj, objs, g, P_g, f)

        #print '---------- grad ----------'
        #print grad
        #print
        
        traj -= (1./eta)*M_inv*grad
        #traj += (1./eta)*M_inv*grad_legibility(traj, objs, g, P_g, f)

    return traj

