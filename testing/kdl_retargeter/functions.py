# functions.py
# Tarik Tosun

from PyKDL import *
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from scipy.optimize import fmin_tnc

import pdb

SAMPLE_POINTS = 20

def forwardKinematics(chain, angles):
    ''' returns the joint positions correspoinding to the given angles for the
    given chain. '''
    angles_JntArray= JntArray(len(angles))
    for i,a in enumerate(angles):
        angles_JntArray[i] = angles[i]

    fk = ChainFkSolverPos_recursive(chain)
    frame = Frame()
    N = chain.getNrOfSegments() + 1
    positions = [0]*N
    for i in range(0,N):
        fk.JntToCart(angles_JntArray,frame,i)
        pvect = frame.p
        positions[i] = [pvect.x(), pvect.y(), pvect.z()] 
    return positions


def draw(chain, angles, ax, plt_opts="", label=""):
   ''' Draws the KDL chain passed in. Angles are the specified chain angles.  ax
   is an instance of p3.Axes3D.  plt_opts are plot options passed directly in.'''
   ep = forwardKinematics(chain,angles)
   ep = np.array(ep)
   #ax = p3.Axes3D(figure)
   x = ep[:,0]
   y = ep[:,1]
   z = ep[:,2]
   ax.plot(x,y,z,plt_opts, label=label)
   ax.legend()

def retarget(source, source_angles, target, target_initial_angles, target_bounds, EE_ratio=1, mode='links'):
    ''' 
    uses source_angles to set position of source chain, then returns
    retargeted angles for target chain.
    source: a KDL chain object
    source_angles: list of angles: [theta0, theta1,...thetaN]
    target & target_initial_angles similar
    target_bounds: list of bounds in the form: [(l1,u1),(l2,u2),...(lN,uN)]
    '''

    eps = forwardKinematics(source,source_angles)
    objective = lambda angles: cost_joints_ee(angles, target, eps, source, EE_ratio, mode)
    res = fmin_tnc(objective, target_initial_angles, approx_grad=1, maxfun=100000, bounds=target_bounds, disp=0)
    return res

def cost_joints_ee(angles, target, eps, source, EE_ratio, mode='links'):
    ''' joint-total-distance cost function, implemented in python,
    mode
    -original: by Tarik
    -joints_only: comparing only joint positions
    -links: comparing link by link
    -scale_by_length: comparing points distance on the chain based on dividing the full length equally
    -scale_by_unit_length: comparing point distances as scale_by_length but also as both chain as the same length
    '''
    Nt = target.getNrOfSegments()
    Ns = len(eps)-1
    ept = forwardKinematics(target,angles)
    eps = np.array(eps)
    ept = np.array(ept)
    # end effector term:
    s_ee = eps[-1,:]
    t_ee = ept[-1,:]
    rEE = np.abs(np.linalg.norm(s_ee - t_ee))
    #print "rEE: {0}".format(rEE)

    # interior joint term:
    rJoints = 0

    if mode == 'original':
        for i in range(1,Nt):
            t = ept[i,:]
            for j in range(1,Ns):
                s = eps[j,:]
                rJoints += np.abs(np.linalg.norm(s-t))

    elif mode == 'joints_only':
        ## !! only works if Nt = Ns
        for i in range(1,Nt):
            t = ept[i,:]
            s = eps[i,:]
            rJoints += np.abs(np.linalg.norm(s-t))

    elif mode == 'links':
        eps_increment, ept_increment = float(Ns)/float(SAMPLE_POINTS), \
                                       float(Nt)/float(SAMPLE_POINTS)

        for idx in range(0,SAMPLE_POINTS+1):
            s = find_pt_on_chain(eps, idx*eps_increment)
            t = find_pt_on_chain(ept, idx*ept_increment)
            #print s,t
            rJoints += np.abs(np.linalg.norm(s-t))

    elif mode == 'scale_by_length':
        rJoints = calculate_rJoints_with_scaled_chain(ept, target, eps, source)

    elif mode == 'scale_by_unit_length':
        rJoints = calculate_rJoints_with_scaled_chain(ept, target, eps, source, as_unit_length=True)

    else:
        print "Not in any mode!"

    #print "rJoints: {0}".format(rJoints)

    # normalize for number of source interior joints, multiply by ratio:
    m = Ns-1 # if EEratio = 1, EE counts as much as any other joint
    cost = rEE*m*EE_ratio + rJoints
    return cost

    # TODO: need to do it not be joint but by distance !!!!


def interpolate(start_pt, end_pt, unit_distance_from_start_pt):
    """
    find a pt on the line of start_pt to end_pt with a
    unit_distance_from_start_pt.
    start_pt: N dimension - 1D array
    end_pt: N dimension - 1D array
    unit_distance_from_start_pt: from 0 to 1. 0 is the start pt, 1 is the end pt
    """
    new_pt = []
    for idx in range(len(start_pt)):
        new_pt.append(start_pt[idx] + \
                      (end_pt[idx]-start_pt[idx])*unit_distance_from_start_pt)
    return np.array(new_pt)


def find_pt_on_chain(ep_matrix, distance_from_origin):
    """
    find pt on the kinematic chain based on joint coorindates
    ep_matrix: kinematic chain coordinates. Each row is a joint
    distance_from_base: distance from the origin. Max is the number of joints
    """
    start_pt = ep_matrix[int(np.floor(round(distance_from_origin,2))),:]
    end_pt = ep_matrix[int(np.ceil(round(distance_from_origin,2))),:]
    unit_distance_from_start_pt = distance_from_origin-np.floor(distance_from_origin)
    #print "start_pt: {0}, end_pt:{1}, unit_distance_from_start_pt:{2}".format(\
    #    start_pt, end_pt, unit_distance_from_start_pt)
    return interpolate(start_pt, end_pt, unit_distance_from_start_pt)


def calculate_rJoints_with_scaled_chain(ept, target, eps, source, as_unit_length=False):

    #ept = forwardKinematics(target,angles)
    #eps = np.array(eps)
    #ept = np.array(ept)

    source_chain_length_list = [0.0]
    for idx in range(source.getNrOfSegments()):
        if not source_chain_length_list:
            source_chain_length_list.append(source.getSegment(idx).getFrameToTip().p.Norm())
        else:
            source_chain_length_list.append(source_chain_length_list[-1]+\
                                        source.getSegment(idx).getFrameToTip().p.Norm())
    #print 'source_chain_list: {0}, length: {1}'.format(source_chain_length_list, \
    #                                                   source_chain_length_list[-1])
    #print '-------'
    target_chain_length_list = [0.0]
    for idx in range(target.getNrOfSegments()):
        if not target_chain_length_list:
            target_chain_length_list.append(target.getSegment(idx).getFrameToTip().p.Norm())
        else:
            target_chain_length_list.append(target_chain_length_list[-1]+\
                                        target.getSegment(idx).getFrameToTip().p.Norm())

    #print 'target_chain_list: {0}, length: {1}'.format(target_chain_length_list, \
    #                                                       target_chain_length_list[-1])

    eps_increment, ept_increment = source_chain_length_list[-1]/float(SAMPLE_POINTS-1), \
                                   target_chain_length_list[-1]/float(SAMPLE_POINTS-1)

    rJoints = 0
    for idx in range(0,SAMPLE_POINTS):
        dist_s, dist_t = idx*eps_increment, idx*ept_increment
        lower_s, upper_s, lower_s_idx, upper_s_idx = \
                    find_nearest_neighbour(source_chain_length_list, dist_s)
        lower_t, upper_t, lower_t_idx, upper_t_idx = \
                    find_nearest_neighbour(target_chain_length_list, dist_t)

        if lower_s_idx != upper_s_idx:
            #print "dist_s: {0}".format(dist_s)
            #print eps[lower_s_idx,:], eps[upper_s_idx,:], (dist_s-lower_s)/(upper_s-lower_s)
            s = interpolate(eps[lower_s_idx,:], eps[upper_s_idx,:], (dist_s-lower_s)/(upper_s-lower_s))
        else:
            s = eps[lower_s_idx,:]

        if lower_t_idx != upper_t_idx:
            #print "dist_t: {0}".format(dist_t)
            #print ept[lower_t_idx,:], ept[upper_t_idx,:], (dist_t-lower_t)/(upper_t-lower_t)
            t = interpolate(ept[lower_t_idx,:], ept[upper_t_idx,:], (dist_t-lower_t)/(upper_t-lower_t))
        else:
            t = ept[lower_t_idx,:]

        # look at it as unit vector
        if as_unit_length:
            rJoints += np.abs(np.linalg.norm(s/source_chain_length_list[-1]-\
                                             t/target_chain_length_list[-1]))
        # just simply compare distance
        rJoints += np.abs(np.linalg.norm(s-t))

    return rJoints

def find_nearest_neighbour(array_orig, value):
    """
    Find the neareast neighour to the value in the array
    in terms of value
    """
    array = np.array(array_orig)
    upper_array = array[array >= value]
    lower_array = array[array <= value]

    #print lower_array, upper_array
    # find max value
    if len(upper_array):
        max_value =  upper_array.min()
    else:
        max_value =  array[0]

    # find min value
    if len(lower_array):
        min_value =  lower_array.max()
    else:
        min_value = array[-1]

    # find index
    min_idx = array_orig.index(min_value)
    max_idx = array_orig.index(max_value)

    return min_value, max_value, min_idx, max_idx


if __name__ == "__main__":
    start_pt = [0,0,0]
    end_pt = [1,1,1]
    print interpolate(start_pt, end_pt, 0)
    print interpolate(start_pt, end_pt, 1)
    print interpolate(start_pt, end_pt, 0.5)

    ep_matrix = np.array([[0.0, 0.0, 0.0],
     [1.0, 0.0, 0.0],
     [1.0, 1.0, 0.0],
     [1.0, 1.0, 1.0],
     [1.0, 1.0, 2.0],
     [1.0, 0.0, 2.0],
     [1.0, 0.0, 1.0],
     [1.0, 0.0, 0.0]])
    cur_ep_idx = 4
    distance_from_origin = 5.1
    print find_pt_on_chain(ep_matrix, distance_from_origin)


