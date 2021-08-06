#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np

# Lateral control

# TODO: write the PID controller using what you've learned in the previous activities

# Note: y_hat will be calculated based on your DeltaPhi() and poseEstimate() functions written previously 

def PIDController(
    v_0, # assume given (by the scenario)
    y_ref, # assume given (by the scenario)
    y_hat, # assume given (by the odometry)
    prev_e_y, # assume given (by the previous iteration of this function)
    prev_int_y, # assume given (by the previous iteration of this function)
    delta_t): # assume given (by the simulator)
    """
    Args:
        v_0 (:double:) linear Duckiebot speed.
        y_ref (:double:) reference lateral pose
        y_hat (:double:) the current estiamted pose along y.
        prev_e_y (:double:) tracking error at previous iteration.
        prev_int_y (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e_y (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int_y (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    # Compute errors
    e_y = y_ref - y_hat
    e_int_y = prev_int_y + e_y * delta_t
    e_der_y = (e_y - prev_e_y) / delta_t
    
    # anti-windup - preventing the integral error from growing too much
    e_int_y = max(min(e_int_y,2),-2)
    
    # Compute PID coefficients
    KP = 1.0
    KD = KP * 50.0
    KI = 0.03

    P = KP * e_y
    D = KD * e_der_y
    I = KI * e_int_y
            
    # Compute controller
    omega = P + D + I
    # v_0 = 0.1 if abs(e_y)>0.2 else 0.2
    # saturating control
    # max_omega = np.deg2rad(20) # 10°/s
    # omega = max(min(omega,max_omega),-max_omega)
    #omega = 2.0
    print(f"\n\n E : {e_y} \n U : {omega} \n P : {P} \n D : {D} \n  I : {I}\n")
            
    return [v_0, omega], e_y, e_int_y

