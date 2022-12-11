# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

# Sample from a normal distribution using 12 uniform samples.
def sample_normal_distribution(mu, sigma):
  #return sample from normal distribution with mean = a and standard deviation b
  sum_val = 0
  for i in range(0,12):
    #r_val = np.random.normal(mu,sigma,1)
    r_val = np.random.uniform(mu-sigma, mu+sigma)
    sum_val += r_val
  return sum_val/2 # replace


  """ Sample odometry motion model.
  
  Arguments:
  x -- pose of the robot before moving [x, y, theta]
  u -- odometry reading obtained from the robot [rot1, rot2, trans]
  a -- noise parameters of the motion model [a1, a2, a3, a4]
  
  """
def sample_motion_model(x, u, a):
  # replace with your computation of x_prime, y_prime, and theta_prime
  rot1 = u[0]
  rot2 = u[1]
  trans = u[2]

  del_rot1_ = rot1 + sample_normal_distribution(0, (a[0]*abs(rot1)) + (a[1]*trans))
  del_rot2_ = rot2 + sample_normal_distribution(0, (a[0]*abs(rot2)) + (a[1]*trans))
  del_trans_ = trans + sample_normal_distribution(0, a[2]*abs(trans) + a[3]*(abs(rot1)+abs(rot2)))

  x_prime = x[0] + del_trans_*math.cos(x[2] + del_rot1_)
  y_prime = x[1] + del_trans_*math.sin(x[2] + del_rot1_)
  theta_prime = x[2] + del_rot1_ + del_rot2_
  return np.array([x_prime, y_prime, theta_prime])


""" Evaluate motion model """

def main():
  # start pose
  x = [0.0, 0.0, 0.0]
  # odometry
  u = [[0.0, 0.0, 1.0],
       [0.0, 0.0, 1.0],
       [np.pi/2, 0.0, 1.0],
       [0.0, 0.0, 1.0],
       [0.0, 0.0, 1.0],
       [np.pi/2, 0.0, 1.0],
       [0.0, 0.0, 1.0],
       [0.0, 0.0, 1.0],
       [0.0, 0.0, 1.0],
       [0.0, 0.0, 1.0]]
  # noise parameters
  a = [0.02, 0.02, 0.01, 0.01]

  num_samples = 1000
  x_prime = np.zeros([num_samples, 3])
  # 1000 samples with initial pose
  for i in range(0, num_samples):
      x_prime[i,:] = x

  plt.axes().set_aspect('equal')
  plt.xlim([-5, 5])
  plt.ylim([-3, 7])
  plt.plot(x[0], x[1], "bo")
  
  # incrementally apply motion model
  for i in range(0,10):
      for j in range(0, num_samples):
        x_prime[j,:] = sample_motion_model(x_prime[j,:],u[i],a).ravel()        
      plt.plot(x_prime[:,0], x_prime[:,1], "r,")

  
  plt.xlabel("x-position [m]")
  plt.ylabel("y-position [m]")

  plt.show()

if __name__ == "__main__":
  main()


