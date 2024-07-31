from numpy import cos, sin
import numpy as np
import pandas as pd

def Rx(theta):
    return np.array([
    [ 1,           0,          0],
    [ 0, cos(theta),-sin(theta)],
    [ 0, sin(theta), cos(theta)],
    ])

def Ry(theta):
    return np.array([
    [ cos(theta), 0, sin(theta)],
    [          0, 1,          0],
    [-sin(theta), 0, cos(theta)],
    ])

def Rz(theta):
    return np.array([
    [ cos(theta),-sin(theta), 0],
    [ sin(theta), cos(theta), 0],
    [          0,          0, 1]
    ])

def R_end_effector(roll, pitch, yaw):
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)

def R_sensor(theta):
    return Rz(theta)

def rotate_forces(force_file, position_file):
    ft_df = pd.read_csv(force_file)
    pos_df = pd.read_csv(position_file)
    F = ft_df[['Fx', 'Fy', 'Fz']].to_numpy().T
    roll, pitch, yaw = pos_df[['theta_x', 'theta_y', 'theta_z']].mean().to_numpy() * np.pi / 180
    theta = (np.pi / 180) * 0.0
    F_base = R_end_effector(roll, pitch, yaw) @ R_sensor(theta) @ F
    ds = pd.DataFrame(F_base.T, columns=['Fx', 'Fy', 'Fz'])
    ds['t'] = ft_df['t']
    return ds[['t', 'Fx', 'Fy', 'Fz']]