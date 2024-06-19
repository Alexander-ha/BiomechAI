import numpy as np
import mediapipe as mp
mp_pose = mp.solutions.pose

class REAM:
  '''
  This class provides usable interface for Inverted Pendulum Model, requires mp.mediapipe.pose_estimation landmarks as an input data.
  Barycenter_Get method - returns mass-centre for human walking model
  get_Tmax method - retur max period of oscillations
  Update_Data method - helpful when you use motion capture and need to update your data in order to determine errors between your optimal model and real results
  get_Leg_Len method - returns leg length
  get_Init_Data - you can use it in order to obtain initial conditions for Inverted Pendulum Model from your mocap Data (angular velocity, angle, velocity, points)
  '''
  def __init__(self, mass, data, iter, dt):
    self.temp = 0.5
    self.dt = dt
    self.mass = mass
    self.data = data
    self.init_point1 = np.array([data.landmark[mp_pose.PoseLandmark(24).value].x, data.landmark[mp_pose.PoseLandmark(24).value].y, data.landmark[mp_pose.PoseLandmark(24).value].z])
    self.init_point2 = np.array([data.landmark[mp_pose.PoseLandmark(26).value].x, data.landmark[mp_pose.PoseLandmark(26).value].y, data.landmark[mp_pose.PoseLandmark(26).value].z])
    self.init_point3 = np.array([data.landmark[mp_pose.PoseLandmark(28).value].x, data.landmark[mp_pose.PoseLandmark(28).value].y, data.landmark[mp_pose.PoseLandmark(28).value].z])
    self.p_start = np.arccos(np.clip(np.dot((np.array(self.init_point3) - np.array(self.init_point2))/np.linalg.norm(np.array(self.init_point3) - np.array(self.init_point2)), [0, 1, 0]), -1.0, 1.0))
    self.phi_start = np.arccos(np.clip(np.dot([0, 1, 0], (self.init_point2 - self.init_point1)/np.linalg.norm(self.init_point2 - self.init_point1)), -1.0, 1.0))
    self.tau_start = np.arccos(np.clip(np.dot([0, 1, 0], (self.init_point3 - self.init_point1)/np.linalg.norm(self.init_point3 - self.init_point1)), -1.0, 1.0))
    self.init_len1 = np.sqrt(np.dot(self.init_point2 - self.init_point1, self.init_point2 - self.init_point1))
    self.init_len2 = np.sqrt(np.dot(self.init_point3 - self.init_point2, self.init_point3 - self.init_point2))
    self.approx_len_start = np.sqrt(np.dot(self.init_point3[:2] - self.init_point1[:2], self.init_point3[:2] - self.init_point1[:2]))
    self.nu = 0.16*self.mass
    self.wsp = 9.8/(self.approx_len_start)
    self.t = 0
    self.x = self.Barycenter_Get()
    if iter == 1:
     self.foot_point = np.array([data.landmark[mp_pose.PoseLandmark(28).value].x, data.landmark[mp_pose.PoseLandmark(28).value].y, data.landmark[mp_pose.PoseLandmark(28).value].z])
     self.v = (self.foot_point-self.init_point3)/self.dt
     self.x = self.Barycenter_Get()
     self.swing = 1
    self.Tmax = 0

  def Barycenter_Get(self):
    self.coord_data = []
    for i in range(0, 32):
      self.coord_data.append([self.data.landmark[mp_pose.PoseLandmark(24).value].x, self.data.landmark[mp_pose.PoseLandmark(24).value].y, self.data.landmark[mp_pose.PoseLandmark(24).value].z])
    self.bct = (np.add.reduce(self.coord_data[:23]) + np.add.reduce(self.coord_data[24:]))/32
    return self.bct

  def get_Tmax(self):
    return self.Tmax

  def get_Leg_Len(self):
    return self.approx_len_start

  def get_Init_Data(self):
    return self.wtau, self.tau, self.init_point3, self.init_point3, self.v

  def Torso_And_Foot_Traj_Opt(self):
    #if self.swing == 1:
     self.C3 = ((self.wsp**4)/(self.wst*(self.wsp**2-self.wst**2)**2))*self.v
     self.C2 = self.C3*0.5*(self.wst**3)*(self.wsp**2 - self.wst**2)
     self.C1 = -(self.wst**3/self.wsp**5)*(self.wsp**2 + 1.5*(self.wsp**2 - self.wst**2))

     self.k3 = ((12*self.temp)/((12 + (self.wsw*self.temp)**2)*np.sinh(self.wsw*self.temp/2) - 6*(self.wsw*self.temp)*np.cosh(self.wsw*self.temp)))
     self.k1 = 2*self.v+((((self.wsw*self.temp)**2) - 24)/(12*self.temp))*np.sinh(self.wsw*self.temp/2)*self.k3
     self.k2 = -((self.wsw*self.temp)**2/(3*self.temp**3))*np.sinh(self.wsw*self.temp/2)*self.k3

     self.x_torso = self.C1*np.sinh(self.wsp*self.t) + self.C2*np.cosh(self.wsp*self.t)*self.C1*np.sinh(self.wst*self.t)
     self.x_foot = self.v*self.t + self.k1*self.t + self.k2*self.t**3 + self.C3*np.sinh(self.wsw*self.t)
   #  print(self.x_torso)
     return self.x_torso, self.x_foot


  def Update_Data(self, data, iter):

    self.t = (self.t + self.dt)%(self.temp)
    #print(self.t)

    #if iter == 1:
    # self.prev_point1 = self.init_point1
    # self.prev_point2 = self.init_point2
    # self.prev_point3 = self.init_point3

   # else:
   #  self.prev_point1 = self.point1
   #  self.prev_point2 = self.point2
   #  self.prev_point3 = self.point3
    self.iter = iter
    self.data = data
    self.point1 = np.array([data.landmark[mp_pose.PoseLandmark(24).value].x, data.landmark[mp_pose.PoseLandmark(24).value].y, data.landmark[mp_pose.PoseLandmark(24).value].z])
    self.point2 = np.array([data.landmark[mp_pose.PoseLandmark(26).value].x, data.landmark[mp_pose.PoseLandmark(26).value].y, data.landmark[mp_pose.PoseLandmark(26).value].z])
    self.point3 = np.array([data.landmark[mp_pose.PoseLandmark(28).value].x, data.landmark[mp_pose.PoseLandmark(28).value].y, data.landmark[mp_pose.PoseLandmark(28).value].z])
    self.ppoint3 = np.array([data.landmark[mp_pose.PoseLandmark(28).value].x, data.landmark[mp_pose.PoseLandmark(28).value].y, data.landmark[mp_pose.PoseLandmark(28).value].z])

    if self.iter > 3 and self.Tmax == 0:
     if abs(self.point3[0] - self.init_point3[0]) < abs(self.check_point3[0] - self.init_point3[0]):
      if abs(self.point3[0] - self.ppoint3[0])<0.5:
        #print(self.iter)
        self.Tmax = self.iter*self.dt*2
        print(self.Tmax)

    self.check_point3 = np.copy(self.point3)


    #if np.linalg.norm(self.point3-self.prev_point3) < 0.01:
    #  self.swing = 1
    #else:
    #  self.swing = 0

    if self.iter == 1:
     self.v = (self.point3-self.init_point3)/self.dt
     self.p = np.arccos(np.clip(np.dot((self.point3 - self.point2)/np.linalg.norm(self.point3 - self.point2), [0, 1, 0]), -1.0, 1.0))
     self.phi = np.arccos(np.clip(np.dot([0, 1, 0], (self.point2 - self.point1)/np.linalg.norm(self.point2 - self.point1)), -1.0, 1.0))
     self.tau = np.arccos(np.clip(np.dot([0, 1, 0], (self.init_point3 - self.init_point1)/np.linalg.norm(self.init_point3 - self.init_point1)), -1.0, 1.0))
     self.wsw = (self.phi - self.phi_start)/self.dt
     self.wst = (self.p - self.p_start)/self.dt
     self.wtau = (self.tau - self.tau_start)/self.dt

    else:
     #if np.linalg.norm(self.point3-self.init_point3)<0.1:
     # self.Tmax = iter*dt*2
     self.p_start = self.p
     self.phi_start = self.phi
     self.p = np.arccos(np.clip(np.dot((self.point3 - self.point2)/np.linalg.norm(self.point3 - self.point2), [0, 1, 0]), -1.0, 1.0))
     self.phi = np.arccos(np.clip(np.dot([0, 1, 0], (self.point2 - self.point1)/np.linalg.norm(self.point2 - self.point1)), -1.0, 1.0))
     self.wsw = np.abs(self.phi - self.phi_start)/self.dt
     self.wst = np.abs(self.p - self.p_start)/self.dt
     self.x_prev = self.x
     self.x = self.Barycenter_Get()
     self.v = (self.x - self.x_prev)/self.dt
     self.res = self.Maximinimization_L2_norm()
     #print(self.res)


  def Maximinimization_L2_norm(self):
    self.torso, self.foot = self.Torso_And_Foot_Traj_Opt()
    self.L2norm_torso = np.linalg.norm(self.x - self.torso)
    self.L2norm_foot = np.linalg.norm(self.point3 - self.foot)
    #print(self.L2norm_torso, self.L2norm_foot)
    return self.L2norm_torso, self.L2norm_foot
