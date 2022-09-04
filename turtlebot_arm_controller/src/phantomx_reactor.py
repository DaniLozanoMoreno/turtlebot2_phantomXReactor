#!/usr/bin/env python
  
# phantomx_reactor.py
# 
# Author: Daniel Lozano Moreno
# File created for the final degree project in robotics engineering at
# the University of Alicante (UA), promotion of 2021-2022. Proyect Name:
# Control y simulaciOn en ROS de un PhantomX Reactor Arm en cooperaciOn 
# con un Turtlebot2
# 
# Execution command:
#   rosrun turtlebot_arm_controller phantomx_reactor.py


"""
  @import rospy: ROS-Python driver
  @import xml.dom.minidom: xml interaction
  @import JointTrajectory: trajectory type
  @import JointTrajectoryPoint: waypoint trajectory type
  @import JointTrajectoryControllerState: controller state trajectory type
  @import FollowJointTrajectoryGoal: goal trajectory type
  @import FollowJointTrajectoryActionResult: action result trajectory type
  @import FollowJointTrajectoryActionGoal: action goal trajectory result type
  @import JointState: joint state type
  @import matplotlib: graph plotter
  @import numpy: math operations
  @import operator: utils operators
"""
import rospy
import xml.dom.minidom
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from operator import add

"""
  PhantomX Reactor utils poses
"""
ARM_HOME = [0,0,0,0,0]
ARM_PICK1 = [0,1.2,-0.5,-0.7,0]
ARM_PICK2 = [-1,1.2,-0.5,-0.7,0]
GRIP_CLOSE = [0.00]
GRIP_OPEN = [0.03]

"""
  Waypoints in 1 second in plannig
"""
plan_rate = 100.0

"""*********************************************************************
  TRAJECTORY PLANNINGS
  
  tpoly3: Cubic interpolation point to point.
  tpolyN: N degree interpolation point to point.
  jtraj3: Plan multiples trajectories for multiples joints (N joints)
          usig the cubic interpolation point to point tpoly3.
  jtrajN: Plan multiples trajectories for multiples joints (J joints)
          using the N interpolation point to point tpolyN.
  mstrajSplines: Plan multiples trajectories of multiples points (K points) 
          for multiples joints (N joints) using splines and the cubic 
          interpolation point to point tpolyto.
   
*********************************************************************"""

"""
  @fn    tpoly3
  
  @brief Cubic interpolation point to point
  
  @param q0        Initial trajectory point scalar
  @param qf        Final trajectory point scalar
  @param T         Time scalar of the trajectory
  @param qd0       Initial velocity trajectory scalar 
  @param qdf       Final velocity trajectory scalar
  @param show      True for plot pos, veloc and accel profiles
  @param filename  Directory path to save the pos, veloc and accel 
                   profiles .png. None to not save it.
"""
def tpoly3 (q0, qf, T, qd0 = 0, qdf = 0, show = False, filename = None):
    
  N = 3    # Polynomial order
  M = N+1  # Number of unknowns to solve
  
  # Time normalzation
  sampling = T*plan_rate
  T = np.linspace(0, T, sampling)
  h = max(T)-min(T)
  tau = T/h
  
  # N Polynomial order, M unknowns
  # q_tau = a3*tau^3 + a2*tau^2 + a1*tau + a0
  # q_tau = (3*a3*tau^2 + 2*a2*tau + a1)*(1/h)
  # q_tau = (6*a3*tau + 2*a2)*(1/h)^2
  q_tau = np.ones(M);
  qd_tau = q_tau[0:M-1] * np.arange(M-1, 0, -1)
  qdd_tau = qd_tau[0:M-2] * np.arange(M-2,0,-1);
  qd_tau = qd_tau * (1/h)
  qdd_tau = qdd_tau * ((1/h)**2);
  
  # Matrix X
  ##########
  # X*A = B
  # Num condiciones: 2 de paso + 2 de velocidad = M
  
  # Pass conditions:
  q_0 = q_tau * np.concatenate( (np.zeros(M-1), [1]) ) # 1. q(0)  [1 1 1 1] .* [0 0 0 1]
  q_1 = q_tau;                                         # 2. q(1)  [1 1 1 1]
  
  # Velocity conditions:
  qd_0 = np.concatenate(((qd_tau * np.concatenate( (np.zeros(M-2), [1]) )), [0])) # 3. qd(0)  [[3 2 1] .* [0 0 1], 0]
  qd_1 = np.concatenate((qd_tau, [0]))                                            # 4. qd(1)  [[3 2 1], 0]
  
  X = [list(q_0), list(q_1), list(qd_0), list(qd_1)];
  
  # Vector B
  ##########
  #   q(0)  = q0
  #   q(1)  = qf
  #   qd(0) = qd0
  #   qd(1) = qdf
  B = [[q0], [qf], [qd0], [qdf]];
  
  A = np.linalg.inv(X).dot(B)
  A = A.flatten()
  
  # Poynomials
  q_tau = A;
  qd_tau = qd_tau * A[0:M-1]
  qdd_tau = qdd_tau * A[0:M-2]
  
  # Evaluate polynomials
  p = np.polyval(q_tau, tau)
  pd = np.polyval(qd_tau, tau);
  pdd = np.polyval(qdd_tau, tau);  
  
  # Plot
  if show == True or filename != None:
  
    plt.subplot(1, 3, 1)
    plt.grid()
    plt.plot(T, p)
    plt.title("Position profile")
    plt.legend()
    plt.subplot(1, 3, 2)
    plt.grid()
    plt.plot(T, pd)
    plt.title("Velocity profile")
    plt.legend()
    plt.subplot(1, 3, 3)
    plt.grid()
    plt.plot(T, pdd)
    plt.title("Acceleration profile")
    plt.legend()
    
    if show == True:  
      plt.show()
    
    if filename != None:
      plt.savefig(filename)
  
  plan = Planning()
  plan.positions = p
  plan.velocities = pd
  plan.accelerations = pdd
  plan.times = T
  return plan

"""
  @fn    tpolyN
  
  @brief N degree interpolation point to point
  
  @param N         Degree of the interpolation
  @param q0        Initial trajectory point scalar
  @param qf        Final trajectory point scalar
  @param T         Time scalar of the trajectory
  @param qd0       Initial velocity trajectory scalar 
  @param qdf       Final velocity trajectory scalar
  @param show      True for plot pos, veloc and accel profiles
  @param filename  Directory path to save the pos, veloc and accel 
                   profiles .png. None to not save it.
"""
def tpolyN(N, q0, qf, T, qd0 = 0, qdf = 0, qdd0 = 0, qddf = 0, qddd0 = 0, qdddf = 0, show = False, filename = None):
  
  M = N+1     # Number of unknowns to solve
  D = M/2-1   # Number of necessary derivatives
  
  # Time normalzation
  sampling = T*plan_rate
  T = np.linspace(0, T, sampling)
  h = max(T)-min(T)
  tau = T/h
  
  # Matrix X
  ##########
  # X*A = B -> A = polynomials vector; B = results vector; X = tau vector
  # Number of conditions: M
  q_tau = np.ones(M);                            # Nomralized polynomial
  qd_tau = q_tau[0:M-1] * np.arange(M-1, 0, -1) *(1/h) # Polynomial derivative 1 
  qdd_tau = qd_tau[0:M-2] * np.arange(M-2,0,-1) *(1/h) # Polynomial derivative 2 
  qi_tau = q_tau                                 # Polynomial derivative i
  X = []; # Matrix X
  
  # For each derivative. i is the derivative index
  for i in range(D+1):
    
    a = M-i # Number of unknowns in qi_tau
    b = a-1 # Number of unknowns in qi_tau dependent on tau
    
    # Derivative q_t calculation = (derivative of qi_tau with respect to tau) * 
    #                              (derivative of tau with respect to time)
    if i != 0:
      qi_tau = qi_tau[0:a]*np.arange(a, 0, -1) # (derivative of qi_tau with respect to tau)
      qi_tau = qi_tau * (1/h)                  # (derivative of tau with respect to time)
    
    # X[i:] Calculation for tau = 0
    qi_0 = np.concatenate(( (qi_tau*np.concatenate((np.zeros(b),[1]))), np.zeros(i)))
    
    if X == []:
      X = qi_0
    else:
      X = np.vstack( ( X, qi_0 ))
    
    # X[i+1:] calculation for tau = 1
    qi_1 = np.concatenate((qi_tau, np.zeros(i)));
    X = np.vstack( ( X, qi_1 ))
    
  # Vector B
  ##########
  B = np.concatenate(([q0],[qf]))
  if D >= 1:
    B = np.concatenate((B,[qd0]))
    B = np.concatenate((B,[qdf]))
  if D >= 2:
    B = np.concatenate((B,[qdd0]))
    B = np.concatenate((B,[qddf]))
  if D >= 3:
    B = np.concatenate((B,[qddd0]))
    B = np.concatenate((B,[qdddf]))
  if D > 3:
    B = np.concatenate((B,np.zeros((D-3)*2)))
  
  # Vector A
  ##########
  A = np.linalg.inv(X).dot(B)
  A = A.flatten()
  
  # Poynomials
  q_tau = A;
  qd_tau = qd_tau * A[0:M-1]
  qdd_tau = qdd_tau * A[0:M-2]
  
  # Evaluate polynomials
  p = np.polyval(q_tau, tau)
  pd = np.polyval(qd_tau, tau);
  pdd = np.polyval(qdd_tau, tau); 
  
  # Plot
  if show == True or filename != None:
  
    plt.subplot(1, 3, 1)
    plt.grid()
    plt.plot(T, p)
    plt.title("Position profile")
    plt.legend()
    plt.subplot(1, 3, 2)
    plt.grid()
    plt.plot(T, pd)
    plt.title("Velocity profile")
    plt.legend()
    plt.subplot(1, 3, 3)
    plt.grid()
    plt.plot(T, pdd)
    plt.title("Acceleration profile")
    plt.legend()
    
    if show == True:  
      plt.show()
    
    if filename != None:
      plt.savefig(filename)
  
  plan = Planning()
  plan.positions = p
  plan.velocities = pd
  plan.accelerations = pdd
  plan.times = T
  return plan

"""
  @fn    jtraj3
  
  @brief Plan multiples trajectories for multiples joints (N joints)
         usig the cubic interpolation point to point tpoly3
  
  @param q0        1xN Initial trajectory point vector
  @param qf        1xN Final trajectory point vector
  @param T         Time scalar of the trajectory
  @param qd0       1xN Initial velocity trajectory vector 
  @param qdf       1xN Final velocity trajectory vector
  @param show      True for plot pos, veloc and accel profiles
  @param filename  Directory path to save the pos, veloc and accel 
                   profiles .png. None to not save it.
"""
def jtraj3 (q0, qf, T, qd0 = None, qdf = None, show = False, filename = None):
  
  num_joints = len(q0)
  plan = list()
  
  if qd0 == None:
    qd0 = np.zeros(num_joints)
  if qdf == None:
    qdf = np.zeros(num_joints)
  
  # Plan
  for i in range(num_joints):
    
    plan_aux = tpoly3(q0[i] ,qf[i], T, qd0[i], qdf[i])
    plan.append(plan_aux)
  
  sampling = T*plan_rate
  T = np.linspace(0, T, sampling)
  
  # Plot
  if show == True or filename != None:
    
    plt.subplot(1, 3, 1)
    plt.grid()
    for i in range(num_joints):
      plt.plot(T, plan[i].positions)
    plt.title("Position profile")
    plt.legend()
    
    plt.subplot(1, 3, 2)
    plt.grid()
    for i in range(num_joints):
      plt.plot(T, plan[i].velocities)
    plt.title("Velocity profile")
    plt.legend()
    
    plt.subplot(1, 3, 3)
    plt.grid()
    for i in range(num_joints):
      plt.plot(T, plan[i].accelerations)
    plt.title("Acceleration profile")
    plt.legend()    
    
    if show == True:  
      plt.show()
      
    if filename != None:
      plt.savefig(filename)
      
  return plan

"""
  @fn    jtrajN
  
  @brief Plan multiples trajectories for multiples joints (J joints)
         using the N interpolation point to point tpolyN
  
  @param N         Degree of the interpolation
  @param q0        1xJ Initial trajectory point vector
  @param qf        1xJ Final trajectory point vector
  @param T         Time scalar of the trajectory
  @param qd0       1xJ Initial velocity trajectory vector 
  @param qdf       1xJ Final velocity trajectory vector
  @param show      True for plot pos, veloc and accel profiles
  @param filename  Directory path to save the pos, veloc and accel 
                   profiles .png. None to not save it.
"""
def jtrajN (N, q0, qf, T, qd0 = None, qdf = None, qdd0 = None, qddf = None, qddd0 = None, qdddf = None, show = False, filename = None):
  
  num_joints = len(q0)
  plan = list()
  
  if qd0 == None:
    qd0 = np.zeros(num_joints)
  if qdf == None:
    qdf = np.zeros(num_joints)
  if qdd0 == None:
    qdd0 = np.zeros(num_joints)
  if qddf == None:
    qddf = np.zeros(num_joints)
  if qddd0 == None:
    qddd0 = np.zeros(num_joints)
  if qdddf == None:
    qdddf = np.zeros(num_joints)
      
  # Plan
  for i in range(num_joints):
    
    plan_aux = tpolyN(N, q0[i] ,qf[i], T, qd0[i], qdf[i], qdd0[i], qddf[i],qddd0[i], qdddf[i])
    plan.append(plan_aux)
  
  sampling = T*plan_rate
  T = np.linspace(0, T, sampling)
  
  # Plot
  if show == True or filename != None:
    
    plt.subplot(1, 3, 1)
    plt.grid()
    for i in range(num_joints):
      plt.plot(T, plan[i].positions)
    plt.title("Position profile")
    plt.legend()
    
    plt.subplot(1, 3, 2)
    plt.grid()
    for i in range(num_joints):
      plt.plot(T, plan[i].velocities)
    plt.title("Velocity profile")
    plt.legend()
    
    plt.subplot(1, 3, 3)
    plt.grid()
    for i in range(num_joints):
      plt.plot(T, plan[i].accelerations)
    plt.title("Acceleration profile")
    plt.legend()    
    
    if show == True:  
      plt.show()
      
    if filename != None:
      plt.savefig(filename)
      
  return plan

"""
  tpolyto
  
  @brief tpoly3 special function for mstrajSplines planning. Shouln't be
         used
"""
def tpolyto(q0, qf, T, qd0, qdf):
  
  sampling = T*plan_rate
  T = np.linspace(0, T, sampling)
  
  tf = max(T)
  tau = T/tf;
  
  X = [
        [0      ,0      ,0      ,1],    # 1. q(0)
        [1      ,1      ,1      ,1],    # 2. q(1)
        [0      ,0      ,1/tf   ,0],    # 3. qd(0)
        [3/tf   ,2/tf   ,1/tf   ,0]     # 4. qd(1)
      ]
  
  B = [[q0], [qf], [qd0], [qdf]];
  # A = (X \ B')'
  A = np.linalg.inv(X).dot(B)
  A = A.flatten()
  
  # Poynomials
  coeffs = A;
  coeffs_d = coeffs[0:3] * np.arange(3, 0, -1)
  coeffs_dd = coeffs_d[0:2] * np.arange(2,0,-1)

  # Evaluate polynomials
  p = np.polyval(coeffs, tau)
  pd = np.polyval(coeffs_d, tau)*(1/tf)
  pdd = np.polyval(coeffs_dd, tau)*(1/tf)*(1/tf)
  
  plan = Planning()
  plan.positions = p
  plan.velocities = pd
  plan.accelerations = pdd
  return plan

"""
  @fn    mstrajSplines
  
  @brief Plan multiples trajectories of multiples points (K points) for
         multiples joints (N joints) using splines and the cubic 
         interpolation point to point tpolyto
  
  @param Q         KxN Trajectory points vector
  @param h         K time scalars vector
  @param T         Time scalar of the trajectory
  @param QD        2xN Velocity matrix where the first row is the initial
                   velocities and the second the final velocities 
  @param show      True for plot pos, veloc and accel profiles
  @param filename  Directory path to save the pos, veloc and accel 
                   profiles .png. None to not save it.
"""
def mstrajSplines(Q, h, QD=[[0,0,0,0,0],[0,0,0,0,0]], show = False, filename = None):
  
  if len(Q) == 3:
    Q.append(Q[2])
    h.append(0.01)
  
  N = len(Q[0])               # Num joints
  Pts = len(Q)                # Num of waypoints
  plan_v = list()
  
  # t creation
  T = []
  t_before = 0
  for i in range(len(h)):
    sampling = h[i]*plan_rate
    if T == []:
      T = np.linspace(t_before, t_before+h[i], sampling)
    else:
      T = np.concatenate( ( T,np.linspace(t_before+0.01, t_before+h[i], sampling) ) )
    t_before = h[i]+t_before;
  
  # For each joint
  ################
  for j in range(N):
    
    # A matrix creation
    ###################
    A = []; # A = [pts-2;Pts-2]
    # For each row in A
    for k in range(Pts-2):
      row = []                  # Row in A
      c = 0                     # Element in row
      # "For each element in row"
      for i in range(Pts-2):
        # If row isn't completed
        if c < Pts-2:
        
          # First row of A
          if (k==0) and (c==0):
            row = np.concatenate( ( row,  [2*(h[k]+h[k+1]),h[k]] ) ) # Copy this formula
            c = 1
          # Last row of A
          elif ((k+1)==(Pts-2)) and ((c+1)==(Pts-2-1)):
            row = np.concatenate( ( row,  [h[k+1], 2*(h[k]+h[k+1])] ) ) # Copy this formula
            c = c + 2
          # Diagonal of A
          elif k == (i+1):
            row = np.concatenate( ( row, [h[k+1],2*(h[k]+h[k+1]),h[k]] ) ) # Copy this formula
            c = c + 2;
          #Ceros
          else:
            row = np.concatenate( ( row, [0] ) )   # Copy a 0
          c = c + 1 # Increase counter
      
      # Stack row in A matrix
      if A == []:
        A = row
      else:
        A = np.vstack( ( A, row ))
      
    
    # B matrix creation
    ###################
    B = []
    for k in range(Pts-2):
      if k == 0: # If first row
        B = [(3.0/(h[k]*h[k+1])) * ((h[k]**2)*(Q[k+2][j]-Q[k+1][j]) + (h[k+1]**2)*(Q[k+1][j]-Q[k][j]) ) - h[k+1]*QD[1][j]]
      elif (k+1) == (Pts-2): # If last row
        B = np.vstack( (B, (3.0/(h[k]*h[k+1])) * ((h[k]**2)*(Q[k+2][j]-Q[k+1][j]) + (h[k+1]**2)*(Q[k+1][j]-Q[k][j]) ) - h[k+1]*QD[1][j]) )
      else:
        B = np.vstack( (B, (3.0/(h[k]*h[k+1])) * ((h[k]**2)*(Q[k+2][j]-Q[k+1][j]) + (h[k+1]**2)*(Q[k+1][j]-Q[k][j])) ) )
    
    # (inv(A)*B)'
    if A == []:
      v = [0,0]
    else:
      aux = np.linalg.inv(A)
      aux = np.matmul(aux,B)
      aux = aux.transpose()
      aux = reduce(add ,aux)
    
      v = np.concatenate( ( [QD[0][j]], aux  ) ) # Velocity vector
      v = np.concatenate( ( v,  [QD[1][j]]) )
    
    # Get joint plan
    plan = Planning()
    # For each stretch
    for k in range(len(h)):
      if (h[k] != 0.01):
        plan_aux = tpolyto(Q[k][j], Q[k+1][j], h[k], v[k], v[k+1])
        if plan.positions == []:
          plan.positions = plan_aux.positions
          plan.velocities = plan_aux.velocities
          plan.accelerations = plan_aux.accelerations
        else:
          plan.positions = np.concatenate( ( plan.positions, plan_aux.positions ) )
          plan.velocities = np.concatenate( ( plan.velocities, plan_aux.velocities ) )
          plan.accelerations = np.concatenate( ( plan.accelerations, plan_aux.accelerations ) )
      else:
        plan.positions = np.concatenate( ( plan.positions, [Q[Pts-1][j]] ) )
        plan.velocities = np.concatenate( ( plan.velocities, [QD[1][j]] ) )
        plan.accelerations = np.concatenate( ( plan.accelerations, [0] ) )
      
    plan.times = T
    plan_v.append(plan)
      

  
  # Plot
  if show == True or filename != None:
    
    plt.subplot(3, 1, 1)
    plt.grid()
    for i in range(N):
      plt.plot(T, plan_v[i].positions)
    plt.title("Position profile")
    plt.legend()
    
    plt.subplot(3, 1, 2)
    plt.grid()
    for i in range(N):
      plt.plot(T, plan_v[i].velocities)
    plt.title("Velocity profile")
    plt.legend()
    
    plt.subplot(3, 1, 3)
    plt.grid()
    for i in range(N):
      plt.plot(T, plan_v[i].accelerations)
    plt.title("Acceleration profile")
    plt.legend()    
    
    if show == True:  
      plt.show()
      
    if filename != None:
      plt.savefig(filename)
  
  return plan_v


"""
  @fn get_joints_from_URDF
  
  @brief From a robot model, create a dictionary of joints including the
         kinematic parameters. It has the follow structure:
         dict = {
           joint_name: {
             limit_lower: value
             limit_upper: value
             max_velocity: value
             max_effort: value
           }
           ...
         }   
  @params robot_model: robot model parsed in string
  @params joint_names: joints names to include in dictionary
  
  @return joints_dict: joints dictionary
"""
def get_joints_from_URDF(robot_model, joint_names):
  
  joints_dict = dict()

  # Get joints from URDF
  for child in robot_model.childNodes:
  
    if child.nodeType is child.TEXT_NODE:
      continue
  
    if child.localName == 'joint':
      
      if child.getAttribute('name') in joint_names:
        
        limit = child.getElementsByTagName('limit')[0]
        name = child.getAttribute('name')
        joints_dict[name] = {}
        joints_dict[name]['limit_lower'] = float(limit.getAttribute('lower'))
        joints_dict[name]['limit_upper'] = float(limit.getAttribute('upper'))
        joints_dict[name]['max_velocity'] = float(limit.getAttribute('velocity'))
        joints_dict[name]['max_effort'] = float(limit.getAttribute('effort'))
        
  return joints_dict

"""
  @fn error_control
  
  @brief Error message with the respective solution
  
  @param error_code: code of the error produced
  @param data: aditional information 
"""
def error_control(error_code, data = ''):
  
  if error_code == 0:
    rospy.logerr('Error in PhantomxReactor.__init__(): robot_description is not defined or robot model is not loaded. \nSolution: Load with robot_description tag one of this robots models: \n - phantomx_reactor \n - turtlebot_arm')
  elif error_code == 3:
    rospy.logerr('Error in PhantomxReactor.creteJointTrajectory(): parameter vectors haven\'t the same lengths')
  elif error_code == 4:
    rospy.logerr('Error in PhantomxReactor.creteJointTrajectory(): values in \''+data+'\' trajectory out of limits.')

"""*********************************************************************
  @cn Planning
  
  @briec  Represents a list of waypoints of a trajectory
  
*********************************************************************"""
class Planning():
  def __init__(self):
    self.positions = list()
    self.velocities = list()
    self.accelerations = list()
    self.times = list()

"""*********************************************************************
  @cn ActualJointStates
  
  @briec  Represents a list of state waypoints of a joint
*********************************************************************"""
class ActualJointStates():
  def __init__(self):
    self.names = list()
    self.positions = list()
    self.velocities = list()
    self.efforts = list()
  
"""*********************************************************************
  @cn PhantomXReactor
  
  @brief Represents a PhantomX Reactor URDF with his controllers and
         allows planning possible trajectories with direct kinematic.
*********************************************************************"""
class PhantomXReactor():
   
  """
    @fn __init__
    
    @brief Initialize class
    
    @param arm_controller_name: name of the arm controller
    @param grip_controller_name: name of the gripper controller
    @param ns: robot namespace
    @param ns: robot namespace
    @param arm_joint_names: joints names of the arm group
    @param grip_joint_names: joints names of the grip group
    @param mimic_real_joints: Nx2 matrix of joints that has a mimic joint.
           N is the nuumber of joints, the first colum the name of the 
           principal joint and the second colum the name of his mimic joint.
  """
  def __init__(self, 
               arm_controller_name = 'arm_controller', 
               grip_controller_name = 'grip_controller', 
               ns = '',
               arm_joint_names = [
                                  "shoulder_yaw_joint", 
                                  "shoulder_pitch_joint", 
                                  "elbow_pitch_joint", 
                                  "wrist_pitch_joint", 
                                  "wrist_roll_joint"
                                  ],
               grip_joint_names = [
                                  "gripper_right_joint"
                                  ],
               mimic_real_joints = [
                           ["shoulder_pitch_joint", "shoulder_pitch_mimic_joint"],
                           ["elbow_pitch_joint", "elbow_pitch_mimic_joint"]
                          ],
               ):
    
    # Open robot_description
    try:
      self.robot_description = rospy.get_param("robot_description")
      self.robot_model = xml.dom.minidom.parseString(self.robot_description).getElementsByTagName('robot')[0]
    except:
      error_control(0)
      return None  
    
    # Load dictionary joints from URDF
    self.arm_joint_names = arm_joint_names
    self.grip_joint_names = grip_joint_names
    self.joint_names = np.concatenate((arm_joint_names,grip_joint_names))
    self.joints_info = get_joints_from_URDF(self.robot_model, self.joint_names)
    self.mimic_real_joints = mimic_real_joints
    #rospy.loginfo(self.joints_info)
    
    # Set controllers namespaces
    self.ns = ns
    self.arm_controller_ns  = ns+'/'+arm_controller_name
    self.grip_controller_ns = ns+'/'+grip_controller_name
    
    # Control trajectory variables
    self.arm_traj_started = False
    self.arm_traj_terminated = True
    self.grip_traj_started = False
    self.grip_traj_terminated = True
    self.last_arm_traj_controller_states = list()
    self.last_grip_traj_controller_states = list()
    
    self.arm_traj_started_sub = rospy.Subscriber(self.arm_controller_ns + '/follow_joint_trajectory/result',
                                                 FollowJointTrajectoryActionResult,
                                                 self.arm_FollowJointTrajectoryActionResult_cb,
                                                 queue_size=1)
    
    self.arm_traj_pub = rospy.Publisher(self.arm_controller_ns + '/follow_joint_trajectory/goal',
                                        FollowJointTrajectoryActionGoal,
                                        queue_size=1)
    
    self.arm_traj_save_sub = rospy.Subscriber(self.arm_controller_ns + '/state',
                                              JointTrajectoryControllerState,
                                              self.arm_controller_state_cb,
                                              queue_size=1)
    
    self.grip_traj_started_sub = rospy.Subscriber(self.grip_controller_ns + '/follow_joint_trajectory/result',
                                                  FollowJointTrajectoryActionResult,
                                                  self.grip_FollowJointTrajectoryActionResult_cb,
                                                  queue_size=1)
    
    self.grip_traj_pub = rospy.Publisher(self.grip_controller_ns + '/follow_joint_trajectory/goal',
                                        FollowJointTrajectoryActionGoal,
                                        queue_size=1)
    
    self.grip_traj_save_sub = rospy.Subscriber(self.grip_controller_ns + '/state',
                                              JointTrajectoryControllerState,
                                              self.grip_controller_state_cb,
                                              queue_size=1)
  
    # Converter angle-distance for gripper information
    self.open_gripper_angle = 0.0
    self.open_gripper_distance = 0.03
    self.close_gripper_angle = -2.5
    self.close_gripper_distance = 0.0
    self.angle_to_distance_param_a = (self.open_gripper_angle - self.close_gripper_angle) / (self.open_gripper_distance - self.close_gripper_distance)
    self.angle_to_distance_param_b = self.close_gripper_angle - self.angle_to_distance_param_a*self.close_gripper_distance
  
  # ********** Callbacks functions
  """
    @fn    controller_name_state_cb
    @brief controller_name/state callbacks functions. If a trejectory is
           been executed, save the state in a ActualJointStates() handler
  """
  def arm_controller_state_cb (self, msg):    
    if self.arm_traj_started:
      self.last_arm_traj_controller_states.append(msg)
  def grip_controller_state_cb (self, msg):    
    if self.grip_traj_started:
      self.last_grip_traj_controller_states.append(msg)      
  
  """
    @fn    joints_group_name_FollowJointTrajectoryActionResult_cb
    @brief controller_name/follow_joint_trajectory/result callbacks 
           functions. Determine if a trajectory has started or terminated
  """
  def arm_FollowJointTrajectoryActionResult_cb(self, msg):
    trajStatus = msg.status.status
    if trajStatus == 3 or trajStatus == 4:
      self.arm_traj_terminated = True
      self.arm_traj_started = False
      rospy.loginfo('Arm goal achieved')
  def grip_FollowJointTrajectoryActionResult_cb(self, msg):
    trajStatus = msg.status.status
    if trajStatus == 3 or trajStatus == 4:
      self.grip_traj_terminated = True
      self.grip_traj_started = False
      rospy.loginfo('Gripper goal achieved')
  
  # ********** Getters functions
  """
    @fn    get_joints_group_name_joints_names
    @brief Return the names of a joint group
    @param None
  """
  def get_joint_names(self):
    return self.joint_names
  def get_arm_joint_names(self):
    return self.arm_joint_names
  def get_grip_joint_names(self):
    return self.grip_joint_names
  
  """
    @fn    get_traj_state
    @brief Return the state of a trajectory that is executing in the moment
  """
  def get_traj_started(self):
    if self.grip_traj_started and self.arm_traj_started:
      return True
    else:
      return False
  def get_traj_terminated(self):
    if self.grip_traj_terminated and self.arm_traj_terminated:
      return True
    else:
      return False
  
  """
    @fn    get_joints_group_name_traj_controller_states
    @brief Return the last controller/states saved in a trajectory to view
           how the trajectory has been executed.
    @param None
  """
  def get_last_arm_traj_controller_states(self):
    return self.last_arm_traj_controller_states
  def get_last_grip_traj_controller_states(self):
    return self.last_grip_traj_controller_states
  
  """
    @fn    get_actual_state
    @brief Return a vector of joints_states
    @param joint_names: joints to get position state
  """
  def get_actual_state(self, joint_names, real = False):
    
    joint_state = JointState()
    joint_state.header.frame_id = 'empty'
    actual_joint_states = ActualJointStates()
    actual_joint_states.names = joint_names
    getted = False
    
    # Get state 
   # Can be few groups in robot_description, get only the Phantom group
    while getted == False:
      joint_state = rospy.wait_for_message(self.ns+'/joint_states', JointState)
      try:        
        i = joint_state.name.index(joint_names[0])
        getted = True
      except:
        getted = False
        
    # Return desired joints states
    for name in joint_names:
      i = joint_state.name.index(name)
      if real == True and name == "gripper_right_joint":
        actual_joint_states.positions.append(self.angleToDistance(joint_state.position[i]))
      else:
        actual_joint_states.positions.append(joint_state.position[i])
      actual_joint_states.velocities.append(joint_state.velocity[i])
      if joint_state.effort != ():
        actual_joint_states.efforts.append(joint_state.effort[i])
          
    return actual_joint_states
      
  # ********** Trajectory functions
  
  """
    @fn    Converters angle-distance for gripper joint
    @breif Convert an angle to distance or distance to angle for the
           gripper joint. It's beacause the real gripper joint works with
           angle commands.
    @param angle: to convert in distance
    @param distance: to convert in angle
  """
  def angleToDistance(self, angle):
    return (angle - self.angle_to_distance_param_b) / self.angle_to_distance_param_a
  def distanceToAngle(self, distance):
    return self.angle_to_distance_param_a*distance + self.angle_to_distance_param_b
  
  """
    @fn    creteJointTrajectory
    @brief From a vector of joints Planning(), create a trajectory.
    @param joint_names: trajectory of joints integrated in the trajectory.
           It has to be in the same ordenr and length that pla param.
    @param plan: Planning vector with the waypoints of the joints 
           trajectories
    @param real: True if the trajectory will be executed in the real
           PhantomX Reactor.
  """
  def creteJointTrajectory (self, joint_names ,plan ,real = False):
    
    traj = JointTrajectory()
    traj.joint_names = joint_names
    
    for i in range(len(plan[0].positions)):
      
      point = JointTrajectoryPoint()
      
      for joint in range(len(joint_names)):
        
        position = plan[joint].positions[i]
        velocity = plan[joint].velocities[i]
        
        # Check limits
        if position < self.joints_info[joint_names[joint]]['limit_lower'] or  position > self.joints_info[joint_names[joint]]['limit_upper'] or velocity > self.joints_info[joint_names[joint]]['max_velocity']:
          error_control(4,joint_names[joint])
          return None
        
        point.positions.append(position)
        point.velocities.append(plan[joint].velocities[i])
        point.accelerations.append(plan[joint].accelerations[i])
        point.time_from_start = rospy.Duration(plan[joint].times[i])
        
      traj.points.append(point)
      
    # Add real joints
    if real == True:
      traj = self.createRealJointTrajectory(traj)
              
    return traj
  
  """
    @fn    createRealJointTrajectory
    @brief Add in a traj the mimic joints waypoints
    @param traj: trajectory to be updated
  """
  def createRealJointTrajectory(self, traj):
    # Add mimic joints
    for row in self.mimic_real_joints:
      
      if row[0] in traj.joint_names:
        i = traj.joint_names.index(row[0])
        traj.joint_names.append(row[1])
        for j in range(len(traj.points)):
          traj.points[j].positions.append(-traj.points[j].positions[i])
          traj.points[j].velocities.append(-traj.points[j].velocities[i])
          traj.points[j].accelerations.append(-traj.points[j].accelerations[i])
     # Distance grip to angle
    for joint in traj.joint_names:
      if joint == self.grip_joint_names[0]:
        i = traj.joint_names.index(joint)
        for j in range(len(traj.points)):
          traj.points[j].positions[i] = self.distanceToAngle(traj.points[j].positions[i])
    
    return traj
    
  """
    @fn    executeTrajectory
    @brief Execute a trajectory in the correct action controller. Start 
           the the controller_state saves to save the trajectory executed.
    @param arm_traj: trajectory arm
    @param grip_traj: trayectory grip
  """
  def executeTrajectory (self, arm_traj = None, grip_traj = None):
    
    goal = FollowJointTrajectoryGoal()
    arm_action_goal = None
    grip_action_goal = None
    
    if arm_traj != None:
      goal.trajectory = arm_traj
      arm_action_goal = FollowJointTrajectoryActionGoal()
      arm_action_goal.goal = goal      
      self.last_arm_traj_controller_states = list()
      self.arm_traj_terminated = False
      self.arm_traj_pub.publish(arm_action_goal)
      self.arm_traj_started = True
      
    if grip_traj != None:
      goal.trajectory = grip_traj
      grip_action_goal = FollowJointTrajectoryActionGoal()
      grip_action_goal.goal = goal 
      self.last_grip_traj_controller_states = list()
      self.grip_traj_terminated = False
      self.grip_traj_pub.publish(grip_action_goal)
      self.grip_traj_started = True
  
  """*******************************************************************
  *** Finish PhantomXReactor class *************************************
  *******************************************************************"""

"""
  @fn    compare
  @brief Compare a trajectory planned with a trajectory executed of a 
         joint plotting the planned, executed and error trajectory.
  @param plan: a Planning variable
  @param controller_states: an ActualJointStates variable
  @param joint_i: joint i of the vector joints
"""
def compare (plan, controller_states, joint_i):
  
  # Get positions executed
  real_positions = list()
  error = list()
  for i in range(len(controller_states)):
    real_positions.append(controller_states[i].actual.positions[joint_i])
    error.append(controller_states[i].error.positions[joint_i])
  
  t = np.linspace(0, 1, len(real_positions))
  
  plt.subplot(1, 3, 1)
  plt.grid()
  plt.plot(plan[joint_i].times, plan[joint_i].positions)
  plt.title("Planned trajectory")
  plt.legend()
  plt.subplot(1, 3, 2)
  plt.grid()
  plt.plot(t, real_positions)
  plt.title("Executed trajectory")
  plt.legend()
  plt.subplot(1, 3, 3)
  plt.grid()
  plt.plot(t, error)
  plt.title("Trajectory error")
  plt.legend()
  plt.show()

"""
  @fn    get_arm_positions
  @brief Inifinite loop that allow to get the positions of the PhantomX 
         Reactor joints. To this, the robot desciption has to be loaded.
         Executed the function, a vector joint position will be printed 
         when the intro key is pressed.
         
"""
def get_arm_positions():
  
  phantomx_reactor = PhantomXReactor()
  arm_joint_names = phantomx_reactor.get_arm_joint_names()
  enter = ''
  while not rospy.is_shutdown():
    print "Press enter to print actual arm pose "
    something = raw_input()
    arm_actual_state = phantomx_reactor.get_actual_state(arm_joint_names)
    rospy.loginfo(arm_actual_state.positions)
  
"""
  @fn main
"""
if __name__=='__main__':
  
  # Exemple of a trajectory executed in the simulation robot (7 degree PTP).
  rospy.init_node('phantomx_reactor_node')
  
  phantomx_reactor = PhantomXReactor()
  joint_names = phantomx_reactor.get_arm_joint_names()
  actual_state = phantomx_reactor.get_actual_state(joint_names, real = False)
  q0 = actual_state.positions
  qf = ARM_PICK1
  T = 1
  N = 7
  plan = jtrajN(N, q0, qf, T)
  traj = phantomx_reactor.creteJointTrajectory(joint_names, plan, real = False)
  phantomx_reactor.executeTrajectory(arm_traj = traj)
  
  while not phantomx_reactor.get_traj_terminated():
    g = 0
  
  compare(plan, phantomx_reactor.get_last_arm_traj_controller_states(), 0)
  
