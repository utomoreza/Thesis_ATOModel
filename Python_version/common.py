import os
import numpy as np
import scipy.io as sio

# predefined variables
gravity = 9.81;
mass = 286000;
max_trac = 310000;
max_accel = max_trac/mass;
max_speed = 22.2;
max_brake = 260000;
max_decel = -(max_brake/mass);
Davis = [0.005 0.23 2965];

# get location of parent directory
above_cwd = os.path.dirname(os.getcwd())

# load mat files
s = sio.loadmat(above_cwd + '/s.mat')['s']
v_ref = sio.loadmat(above_cwd + '/v_ref.mat')['v_ref']
v_ref_ori = sio.loadmat(above_cwd + '/v_ref_ori.mat')['v_ref_ori']
v_lim =  sio.loadmat(above_cwd + '/v_lim.mat')['v_lim']
gradient = sio.loadmat(above_cwd + '/gradient')['gradient']

# set important variables
S_max = len(v_ref)-1
del_S = S_max/(len(v_ref)-1)
s = np.arange(start=0, stop=S_max+1, step=del_S)
t = np.arange(start=0, stop=(S_max+1)/10, step=0.1)