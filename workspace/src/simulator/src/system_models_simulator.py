#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

from numpy import sin, cos, tan, arctan, array, dot
from numpy import sign, argmin, sqrt, abs, pi
import rospy

def bikeFE(x, y, psi, v, a, d_f, a0, Ff, theta, ts):
    """
    process model
    """
    # external parameters
    L_a                    = 1.738
    L_b                    = 1.738
    g                      = 9.8

    # compute slip angle
    beta         = arctan(L_a*tan(d_f)/(L_a+L_b))

    # compute next state
    x_next      = x + ts*v*cos(psi+beta) 
    y_next      = y + ts*v*sin(psi+beta)
    psi_next    = psi + ts*(v/L_b)*sin(beta)
    v_next      = v + ts*a

    return array([x_next, y_next, psi_next, v_next])

