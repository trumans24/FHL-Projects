import time
import numpy as np
from xarm.wrapper import XArmAPI

def triangle_motion(arm: XArmAPI, amplitude, frequency, duration):
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.1)
    velocity = 4 * amplitude * frequency
    x0,y0,z0,roll0,pitch0,yaw0=arm.last_used_position
    for _ in range(int(frequency*duration)+1):
        arm.set_position(z=z0+amplitude,speed=velocity,wait=False,mvacc=800,radius=0)
        arm.set_position(z=z0-amplitude,speed=velocity,wait=False,mvacc=800,radius=0)
        arm.set_position(z=z0,speed=velocity,wait=False,mvacc=800,radius=0)
        

def next_cmd(t, path_function, time_step=0.01):
    new_z = path_function(t+time_step)
    return new_z


def sinusoidal_motion(arm:XArmAPI, yaw_amplitude, yaw_frequency, amplitude, frequency, phase_shift, duration):
    arm.set_mode(1)
    arm.set_state(0)
    time.sleep(0.1)
    _, (start_x, start_y, start_z, start_roll, start_pitch, start_yaw) = arm.get_position()
    _, (current_x, current_y, current_z, current_roll, current_pitch, current_yaw) = arm.get_position()
    path = lambda t: amplitude * np.sin(2 * np.pi * frequency * t)
    yaw = lambda t: yaw_amplitude * np.sin(2 * np.pi * yaw_frequency * t + phase_shift)
    steps = []
    time_step = 0
    t0 = t = time.perf_counter()
    while time.perf_counter()-t0 < duration:
        t = time.perf_counter()-t0
        i = next_cmd(t, path_function=path)
        j = next_cmd(t, yaw_function=yaw)
        arm.set_servo_cartesian([start_x, start_y+i, start_z, start_roll, start_pitch, start_yaw+j])
        steps.append([t, i])

    _, (current_x, current_y, current_z, current_roll, current_pitch, current_yaw) = arm.get_position()
    while abs(current_y - start_y) > 0.1:
        _, (current_x, current_y, current_z, current_roll, current_pitch, current_yaw) = arm.get_position()
        t = time.perf_counter()-t0
        i = next_cmd(t, path_function=path)
        j = next_cmd(t, yaw_function=yaw)
        arm.set_servo_cartesian([start_x, start_y+i, start_z, start_roll, start_pitch, start_yaw+j])
        steps.append([t, i])
    _, (current_x, current_y, current_z, current_roll, current_pitch, current_yaw) = arm.get_position()
    
    arm.set_servo_cartesian([start_x, start_y, start_z, start_roll, start_pitch, start_yaw])
    
    return steps

if __name__ == '__main__':
    arm = XArmAPI('192.168.1.217')
    arm.connect()
    
    frequency = 1
    amplitude = 5
    yaw_amplitude = 5
    yaw_frequency = frequency
    phase_shift = 0
    
    duration = 10

    # triangle_motion(arm, amplitude, frequency, duration)

    steps = sinusoidal_motion(arm, yaw_amplitude, yaw_frequency, amplitude, frequency, phase_shift, duration)

    arm.disconnect()
    