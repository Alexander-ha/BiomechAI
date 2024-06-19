import numpy as np

class Classic_Pendulum_EK_control():
  '''
  Сlassic pendulum control model for controlling the classic oscillating biomechanical movements
  M - mass of the pendulum
  m - mass of the cargo
  l - length of the pendulum
  setpoint - is the target angle of your movement, that you want to achieve
  '''
  def __init__(self, M, m, l, start = [np.pi / 4, 0], setpoint = np.pi/2):
   self.m = m
   self.M = M
   self.g = 9.81
   self.L = l
   self.start = start
   self.t = np.linspace(0, 10, 1000)
   self.dt = self.t[1] - self.t[0]
   self.states = np.zeros((len(self.t), 2))
   self.states[0] = self.start
   self.setpoint = setpoint
   self.Kp = 100.0
   self.Ki = 1.0
   self.Kd = 10.0
   self.prev_error = self.setpoint - self.start[0]
   self.integral = 0
   self.eps = 10e-2
   self.flag = 0

  def generalized_force(self, theta):
    if theta < self.eps or theta > np.pi/2:
        return -2000.0 * theta
    else:
        return 0.0

  def equations_of_motion(self, t, y, setpoint):
    th, w = y
    error = setpoint - th
    derivative = (error - self.prev_error) / self.dt
    self.integral = self.integral + error * self.dt
    force = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
    if abs(error) < self.eps:
      self.flag = 1
    if self.flag == 1:
      force = 0
    dth_dt = w
    dw_dt = (force + self.generalized_force(th) - self.m*self.L*self.g*np.sin(th)) / (self.M + self.m)
    self.prev_error = error
    return np.array([dth_dt, dw_dt])

  def adams_bashforth_moulton(self, init_state):
    f = self.equations_of_motion
    n = len(self.t)
    y = np.zeros((n, 2))
    y[0] = init_state
    for i in range(1, n):
        # Adams-Bashforth predictor
        y_pred = y[i-1] + self.dt/2 * (3*f(self.t[i-1], y[i-1], self.setpoint) - f(self.t[i-2], y[i-2], self.setpoint))
        # Adams-Moulton corrector
        y[i] = y[i-1] + self.dt/2 * (f(self.t[i], y_pred, self.setpoint) + f(self.t[i-1], y[i-1], self.setpoint))
    return y

  def make_model(self):
    init_state = self.states[0]
    self.states[0:] = self.adams_bashforth_moulton(init_state)
    return self.states