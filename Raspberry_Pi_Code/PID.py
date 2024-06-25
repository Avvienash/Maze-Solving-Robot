import spidev
from picamera2 import Picamera2
from datetime import datetime
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import random
import math
from rrt import *
from scipy.ndimage import binary_dilation


class PID:
    
	"""
	Discrete PID control
	"""

	def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=10000, Integrator_min=-10000):

		self.Kp=P
		self.Ki=I
		self.Kd=D
		self.Derivator=Derivator
		self.Integrator=Integrator
		self.Integrator_max=Integrator_max
		self.Integrator_min=Integrator_min

		self.set_point=0.0
		self.error=0.0

	def update(self,current_value):
		"""
		Calculate PID output value for given reference input and feedback
		"""

		self.error = self.set_point - current_value

		self.P_value = self.Kp * self.error
		self.D_value = self.Kd * ( self.error - self.Derivator)
		self.Derivator = self.error

		self.Integrator = self.Integrator + self.error

		if self.Integrator > self.Integrator_max:
			self.Integrator = self.Integrator_max

		elif self.Integrator < self.Integrator_min:
			self.Integrator = self.Integrator_min


		self.I_value = self.Integrator * self.Ki

		PID = self.P_value + self.I_value + self.D_value

		return PID

	def setPoint(self,set_point):
		"""
		Initilize the setpoint of PID
		"""
		self.set_point = set_point
		self.Integrator=0
		self.Derivator=0

	def setIntegrator(self, Integrator):
		self.Integrator = Integrator

	def setDerivator(self, Derivator):
		self.Derivator = Derivator

	def setKp(self,P):
		self.Kp=P

	def setKi(self,I):
		self.Ki=I

	def setKd(self,D):
		self.Kd=D

	def getPoint(self):
		return self.set_point

	def getError(self):
		return self.error

	def getIntegrator(self):
		return self.Integrator

	def getDerivator(self):
		return self.Derivator


def plot_pid_control_signals(outputs_x, outputs_y, y_pos, targets_y, x_pos, targets_x, target_x, target_y, dt):
    
    # Create subplots
    fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1, figsize=(10, 60))

    # Plot on the first subplot
    ax1.plot(outputs_x, label='PID Control for X-axis')
    ax1.set_ylabel('X PID Output')
    ax1.set_title('PID Control Signals - X-axis')
    ax1.grid(True)

    # Plot on the second subplot
    ax2.plot(outputs_y, label='PID Control for Y-axis')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Y PID Output')
    ax2.set_title('PID Control Signals - Y-axis')
    ax2.grid(True)
    
    # Plot on the second subplot
    ax3.plot(y_pos, label='Y-Pos')
    ax3.plot(targets_y, label='Target Y')
    ax3.set_xlabel('Time')
    ax3.set_ylabel('Y Postion')
    ax3.grid(True)
    
    # Plot on the second subplot
    ax4.plot(x_pos, label='X-Pos')
    ax4.plot(targets_x, label='Target X')
    ax4.set_xlabel('Time')
    ax4.set_ylabel('x Postion')
    ax4.grid(True)
    
        # Plot on the second subplot
    ax5.plot(x_pos, y_pos)
    ax5.plot(x_pos[-1], y_pos[-1], 'bo')
    ax5.plot(target_x, target_y, 'ro')
    
    ax5.set_xlabel('X')
    ax5.set_ylabel('Y')
    ax5.set_xlim(0, 200)  # Set x-axis limits
    ax5.set_ylim(0, 200)  # Set y-axis limits
    ax5.grid(True)
    
    # Plot on the second subplot
    ax6.plot(dt, label='time Interval')
    ax6.set_xlabel('Time')
    ax6.set_ylabel('dt')
    ax6.set_title('time Interval')
    ax6.grid(True)
    
    # Save the figure
    plt.savefig('images/pid_control_signals.png')
    plt.close()