import numpy as np
import control
from scipy.integrate import odeint


class Inverted_Pendulum_Cart_Walk_COM_control():
  '''
  m - mass of the pendulum cargo
  M - mass of cart 
  L - length of pendulum
  g - 9.81, gravity. 
  x0 - initial state of system
  This model provides an ability to simulate trajectories of COM for walking on a smooth surface. You can apply your parameters in this model to achieve a better perfomance
  of healthy walking or stable action of dynamic system.

  '''
  def __init__(self, M = 1, m = 70, L = 0.8, start = np.array([0, 0, np.pi/10, 0])):
    self.M = M
    self.m = m
    self.L = L
    self.g = 9.81
    self.x0 = start
    self.A = np.array([[0, 1, 0, 0],
              [0, 0, -self.m*self.g/self.M, 0],
              [0, 0, 0, 1],
              [0, 0, self.g*(self.m+self.M)/(self.L*self.M), 0]])
    self.B = np.array([[0], [1/self.M], [0], [-1/(self.L*self.M)]])
    self.C = np.eye(4)
    self.D = np.zeros((4, 1))
    # Design the LQR controller
    self.Q = np.matrix([
					[100,0,0,0],
					[0,100,0,0],
					[0,0,100,0],
					[0,0,0,100]
					])
    self.R = np.array([[0.1]])
    self.K, self.S, self.E = control.lqr(self.A, self.B, self.Q, self.R)
    self.T = 10
    # Simulate the response of the inverted pendulum system
    self.t = np.linspace(0, self.T, 1000)  # Time vector
    self.dt = self.T/1000

  #Create control input
  def control_input(self, x):
    return -np.dot(self.K, x)

  #Inverted pendulum control apply
  def inverted_pendulum(self, x, t, x0):
    x0 = x0[0]
    u = self.control_input(x)
    u = u[0] + 80*np.sign(0.001*(x[0]-x0))
    dxdt = np.dot(self.A, x) + np.dot(self.B, [u])
    return dxdt

  #Make model of inverted pendulum
  def make_model(self):
    res = odeint(self.inverted_pendulum, self.x0, self.t, args = (self.x0, ) )
    return res