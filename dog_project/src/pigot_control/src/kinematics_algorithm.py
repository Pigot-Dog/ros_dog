import numpy as np
import math
import tarj_data as td
import rospy
import pos_pub as pp

l1 = 0.15
l2 = 0.35
l3 = 0.35
radio = 40
rate = 2

class OMNI_CONTROL(object):
    def __init__(self, h, v_E1, v_E2, steps_num):
            
        self.h = h
        self.v_E1 = v_E1
        self.v_E2 = v_E2
        self.step_num = steps_num
        
        self.gait_data = np.zeros((radio, 12))  
        self.x_line = np.zeros(steps_num / 2)
        self.y_line = np.zeros(steps_num / 2)
        self.z_line = np.zeros(steps_num / 2)
        self.rate = 0
        self.turn_angle = 0
        self.rpy_angle = [0, 0, 0]
        
        self.xyz_control = td.XYZ_CONTROL(self.h, self.v_E1, self.v_E2, self.step_num)
        
        self.origin_foot_y, self.old_foot_y = np.array([self.v_E2, self.v_E2, self.v_E2, self.v_E2]), np.array([self.v_E2, self.v_E2, self.v_E2, self.v_E2])

        self.origin_foot_z, self.old_foot_z = np.array([ self.v_E1 * 3.0 / 4, self.v_E1 * 3.0 / 4, self.v_E1 * 3.0 / 4, self.v_E1 * 3.0 / 4]), np.array([ self.v_E1 * 3.0 / 4, self.v_E1 * 3.0 / 4, self.v_E1 * 3.0 / 4, self.v_E1 * 3.0 / 4])
        self.r = (self.v_E2**2 + (self.v_E1 * 3.0 / 4)**2) ** 0.5
        
        self.default_angle = [0, 0, 0]
        
    def diagonal_tranlate_control(self, forward, back, left, right, control_h, control_v_E1, control_v_E2, rpy_angle=[0, 0, 0]): 
        
        self.h = control_h
        self.v_E1 = control_v_E1
        self.v_E2 = control_v_E2
        self.x_line = self.xyz_control.x_dict(self.h)
        self.y_line = self.xyz_control.y_dict(self.v_E2)
        self.z_line = self.xyz_control.z_dict(self.v_E1)
        self.rate = 2
            
        RPY_B_traj = self.rpy_gait()
        for t in range(self.gait_data.shape[0]):
            
            R = RPY_B_traj[t, 0]
            P = RPY_B_traj[t, 1]
            Y = RPY_B_traj[t, 2]
            self.pos_data = self.rpy_control(R, P, Y)
                 
            if (t < 20):
                x1, x3 = self.x_line[t], self.x_line[t]
                x2, x4 = self.x_line[t + 20], self.x_line[t + 20]
            else:
               x1, x3 = self.x_line[t], self.x_line[t]
               x2, x4 = self.x_line[t - 20], self.x_line[t - 20] 
               
            if forward - back == 1:
                if (t < 20):
                    z1, z3 = self.z_line[t], self.z_line[t]
                    z2, z4 = self.z_line[t + 20], self.z_line[t + 20]
                else:
                    z1, z3 = self.z_line[t], self.z_line[t]
                    z2, z4 = self.z_line[t - 20], self.z_line[t - 20]
            elif forward - back == 0:
                z1, z2, z3, z4 = 0, 0, 0, 0
            elif forward - back == -1:
                if (t < 20):
                    z1, z3 = -self.z_line[t], -self.z_line[t]
                    z2, z4 = -self.z_line[t + 20], -self.z_line[t + 20]
                else:
                    z1, z3 = -self.z_line[t], -self.z_line[t]
                    z2, z4 = -self.z_line[t - 20], -self.z_line[t - 20]
                
            if left - right == 1:
                if (t < 20):
                    y1, y3 = -self.y_line[t], self.y_line[t]
                    y2, y4 = self.y_line[t + 20], -self.y_line[t + 20]
                else:
                    y1, y3 = -self.y_line[t], self.y_line[t]
                    y2, y4 = self.y_line[t - 20], -self.y_line[t - 20]
            
            elif left - right == 0:
                y1, y2, y3, y4 = 0, 0, 0, 0
            elif left - right == -1:
                if (t < 20):
                    y1, y3 = self.y_line[t], -self.y_line[t]
                    y2, y4 = -self.y_line[t + 20]+self.v_E2, self.y_line[t + 20]
                else:
                    y1, y3 = self.y_line[t], -self.y_line[t]
                    y2, y4 = -self.y_line[t - 20], self.y_line[t - 20]
                    
            self.rpy_angle = [rpy_angle[0] * math.pi / 180, rpy_angle[1] * math.pi / 180, rpy_angle[2] * math.pi / 180]
            alpha, beta = 50, 50
            z1, z2, z3, z4 = z1 - alpha * math.tan(self.rpy_angle[1]) * x1, z2 - alpha * math.tan(self.rpy_angle[1]) * x2, z3 - alpha * math.tan(self.rpy_angle[1]) * x3, z4 - alpha * math.tan(self.rpy_angle[1]) * x4
#            y1, y2, y3, y4 = y1 - beta * math.tan(self.rpy_angle[0]) * x1, y2 + beta * math.tan(self.rpy_angle[0]) * x2, y3 + beta * math.tan(self.rpy_angle[0]) * x3, y4 - beta * math.tan(self.rpy_angle[0]) * x4
    
            self.gait_data[t, 0], self.gait_data[t, 1 ], self.gait_data[t, 2 ] = leg_ikine(x1 + self.pos_data[0, 0], y1 + self.pos_data[0, 1], z1 + self.pos_data[0, 2])
            self.gait_data[t, 3], self.gait_data[t, 4 ], self.gait_data[t, 5 ] = leg_ikine(x2 + self.pos_data[1, 0], y2 + self.pos_data[1, 1], z2 + self.pos_data[1, 2])
            self.gait_data[t, 6], self.gait_data[t, 7 ], self.gait_data[t, 8 ] = leg_ikine(x4 + self.pos_data[2, 0], y4 + self.pos_data[2, 1], z4 + self.pos_data[2, 2])
            self.gait_data[t, 9], self.gait_data[t, 10], self.gait_data[t, 11] = leg_ikine(x3 + self.pos_data[3, 0], y3 + self.pos_data[3, 1], z3 + self.pos_data[3, 2])
        
        return self.rate, self.gait_data

    def diagonal_turn_control(self, direction, control_h, theta, rpy_angle=[0, 0, 0]):  
        self.turn_angle = theta * math.pi / 180
        
        new_foot_y, new_foot_z = np.zeros(4), np.zeros(4)
        
        for i in range(4):
            theta = math.atan2(self.origin_foot_z[i], self.origin_foot_y[i]) - self.turn_angle
            new_foot_y[i] = self.r * math.cos(theta) 
            new_foot_z[i] = self.r * math.sin(theta)
        
        self.h = control_h
        self.v_E1 = new_foot_z[0]
        self.v_E2 = new_foot_y[0]
        self.x_line = self.xyz_control.x_dict(self.h)
        self.y_line = self.xyz_control.y_dict(self.v_E1)
        self.z_line = self.xyz_control.z_dict(self.v_E2)
        self.rate = 2
        
        RPY_B_traj = self.rpy_gait()
        for t in range(self.gait_data.shape[0]):
            
            R = RPY_B_traj[t, 0]
            P = RPY_B_traj[t, 1]
            Y = RPY_B_traj[t, 2]
            self.pos_data = self.rpy_control(R, P, Y)
            
            if (t < 20):
                x1, x3 = self.x_line[t], self.x_line[t]
                x2, x4 = self.x_line[t + 20], self.x_line[t + 20]
            else:
                x1, x3 = self.x_line[t], self.x_line[t]
                x2, x4 = self.x_line[t - 20], self.x_line[t - 20] 
        
            if direction == 1:
                if (t < 20):
                    y1, y3 = -self.y_line[t], -self.y_line[t]
                    y2, y4 = self.y_line[t + 20], self.y_line[t + 20]
                    z1, z3 = -self.z_line[t], self.z_line[t]
                    z2, z4 = -self.z_line[t + 20], self.z_line[t + 20]
                else:
                    y1, y3 = -self.y_line[t], -self.y_line[t]
                    y2, y4 = self.y_line[t - 20], self.y_line[t - 20]
                    z1, z3 = -self.z_line[t], self.z_line[t]
                    z2, z4 = -self.z_line[t - 20], self.z_line[t - 20]
                    
            if direction == -1:
                if (t < 20):
                    y1, y3 = self.y_line[t], self.y_line[t]
                    y2, y4 = -self.y_line[t + 20], -self.y_line[t + 20]
                    z1, z3 = -self.z_line[t], self.z_line[t]
                    z2, z4 = -self.z_line[t + 20], self.z_line[t + 20]
                else:
                    y1, y3 = self.y_line[t], self.y_line[t]
                    y2, y4 = -self.y_line[t - 20], -self.y_line[t - 20]
                    z1, z3 = -self.z_line[t], self.z_line[t]
                    z2, z4 = -self.z_line[t - 20], self.z_line[t - 20]
                    
            self.rpy_angle = [rpy_angle[0] * math.pi / 180, rpy_angle[1] * math.pi / 180, rpy_angle[2] * math.pi / 180]
            alpha, beta = 50, 50
            z1, z2, z3, z4 = z1 - alpha * math.tan(self.rpy_angle[1]) * x1, z2 - alpha * math.tan(self.rpy_angle[1]) * x2, z3 - alpha * math.tan(self.rpy_angle[1]) * x3, z4 - alpha * math.tan(self.rpy_angle[1]) * x4
#            y1, y2, y3, y4 = y1 - beta * math.tan(self.rpy_angle[0]) * x1, y2 + beta * math.tan(self.rpy_angle[0]) * x2, y3 + beta * math.tan(self.rpy_angle[0]) * x3, y4 - beta * math.tan(self.rpy_angle[0]) * x4
        
            self.gait_data[t, 0], self.gait_data[t, 1 ], self.gait_data[t, 2 ] = leg_ikine(x1 + self.pos_data[0, 0], y1 + self.pos_data[0, 1], z1 + self.pos_data[0, 2])
            self.gait_data[t, 3], self.gait_data[t, 4 ], self.gait_data[t, 5 ] = leg_ikine(x2 + self.pos_data[1, 0], y2 + self.pos_data[1, 1], z2 + self.pos_data[1, 2])
            self.gait_data[t, 6], self.gait_data[t, 7 ], self.gait_data[t, 8 ] = leg_ikine(x4 + self.pos_data[2, 0], y4 + self.pos_data[2, 1], z4 + self.pos_data[2, 2])
            self.gait_data[t, 9], self.gait_data[t, 10], self.gait_data[t, 11] = leg_ikine(x3 + self.pos_data[3, 0], y3 + self.pos_data[3, 1], z3 + self.pos_data[3, 2])
        
        return self.rate, self.gait_data

    def rpy_control(self, R, P, Y):
        # Define the structure paramesters of robot
        l = 0.8 # The length of platform.
        w = 0.8 # The width of platform.
        # Rebuild the Euler-angle.
        R = R
        P = P
        Y = Y
        
        # Foot position vector
        det = np.array([[ l/2, -0.60, -0.55],
                        [ l/2, -0.60,  0.55],
                        [-l/2, -0.60, -0.55],
                        [-l/2, -0.60,  0.55],])
        
        pos_data = np.zeros((4, 3))
        for i in range(4):
            detx = det[i, 0]
            dety = det[i, 1]
            detz = det[i, 2]
            rotx = np.mat([[ 1,       0,       0     ],
                           [ 0,       math.cos(R), -math.sin(R)],
                           [ 0,       math.sin(R),  math.cos(R)]])
                          
            rotz = np.mat([[ math.cos(P), -math.sin(P),  0     ],
                           [ math.sin(P),  math.cos(P),  0     ],
                           [ 0,       0,       1     ]])
                           
            roty = np.mat([[ math.cos(Y),  0,      -math.sin(Y)],
                           [ 0,       1,       0     ],
                           [ math.sin(Y),  0,       math.cos(Y)]]) 
            det_vec = np.mat([[detx], [dety], [detz]])
            off_vec = np.mat([[ l/2, l/2, -l/2, -l/2], 
                              [ 0,   0,    0,    0  ], 
                              [-w/2, w/2, -w/2, w/2 ]])
            pos_vec = rotx * roty * rotz * det_vec - off_vec[:, i]
            # Converte the global coordinate into the kinematics coordinates of each leg.  
            x = -pos_vec[1]
            z = pos_vec[0]
            if (i%2)==0 :
                y = -pos_vec[2]
            else:
                y = pos_vec[2]
            pos_data[i, :] = np.array([x, y, z]).T
        return pos_data
    
    def rpy_gait(self):
        
        RPY_B_traj = np.zeros((radio,3)) 
#        RPY_angle_data_old = rospy.get_param('/pigot/RPY_angle_old', self.default_angle)
        RPY_angle_data_new = rospy.get_param('/pigot/RPY_angle_new', self.default_angle)
        
        error = np.linalg.norm(np.array(RPY_angle_data_new))
        if (error>0.05):
            RPY_angle_new_traj = np.array(RPY_angle_data_new)
            traj = np.vstack(([0, 0, 0], RPY_angle_new_traj))
            for i in range (3):
                RPY_B_traj[:, i] = td.interpolate_method(traj[:, i].tolist())
            
#        rospy.set_param('/pigot/RPY_angle_old', RPY_angle_data_new)
        rospy.set_param('/pigot/in_wait', True)
        
        return RPY_B_traj         
    
def forward_gait():
    gait_data = np.zeros((radio, 12))
    x_line, y_line, z_line = gait_line()
    rate = 2
    for t in range(gait_data.shape[0]):
        if (t < 20):
            xf = x_line[t]
            xb = x_line[t + 20]
            yf = y_line[t]
            yb = y_line[t + 20]
            zf = z_line[t]
            zb = z_line[t + 20]
        else:
            xf = x_line[t]
            xb = x_line[t - 20]
            yf = y_line[t]
            yb = y_line[t - 20]
            zf = z_line[t]
            zb = z_line[t - 20]

        gait_data[t, 0], gait_data[t, 1 ], gait_data[t, 2 ] = leg_ikine(xf, yf + 0.15, zf)
        gait_data[t, 3], gait_data[t, 4 ], gait_data[t, 5 ] = leg_ikine(xb, yb + 0.15, zb)
        gait_data[t, 6], gait_data[t, 7 ], gait_data[t, 8 ] = leg_ikine(xb, yb + 0.15, zb)
        gait_data[t, 9], gait_data[t, 10], gait_data[t, 11] = leg_ikine(xf, yf + 0.15, zf)
        
    return rate, gait_data


def backward_gait():
    gait_data = np.zeros((radio, 12))
    x_line, y_line, z_line = gait_line()
    for t in range(gait_data.shape[0]):
        if (t < 20):
            xf = x_line[t]
            xb = x_line[t + 20]
            yf = y_line[t]
            yb = y_line[t + 20]
            zf = -z_line[t]
            zb = -z_line[t + 20]
        else:
            xf = x_line[t]
            xb = x_line[t - 20]
            yf = y_line[t]
            yb = y_line[t - 20]
            zf = -z_line[t]
            zb = -z_line[t - 20]

        gait_data[t, 0], gait_data[t, 1 ], gait_data[t, 2 ] = leg_ikine(xf, yf, zf)
        gait_data[t, 3], gait_data[t, 4 ], gait_data[t, 5 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 6], gait_data[t, 7 ], gait_data[t, 8 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 9], gait_data[t, 10], gait_data[t, 11] = leg_ikine(xf, yf, zf)
    return rate, gait_data

def turnleft_gait():
    gait_data = np.zeros((radio, 12))
    data = turn_line(1)
    for t in range(gait_data.shape[0]):
        gait_data[t, 0], gait_data[t, 1 ], gait_data[t, 2 ] = leg_ikine(data[0, t], data[4, t], data[8, t])
        gait_data[t, 3], gait_data[t, 4 ], gait_data[t, 5 ] = leg_ikine(data[1, t], data[5, t], data[9, t])
        gait_data[t, 6], gait_data[t, 7 ], gait_data[t, 8 ] = leg_ikine(data[2, t], data[6, t], data[10, t])
        gait_data[t, 9], gait_data[t, 10], gait_data[t, 11] = leg_ikine(data[3, t], data[7, t], data[11, t])
    return rate, gait_data

def turnright_gait():
    gait_data = np.zeros((radio, 12))
    data = turn_line(-1)
    for t in range(gait_data.shape[0]):
        gait_data[t, 0], gait_data[t, 1 ], gait_data[t, 2 ] = leg_ikine(data[0, t], data[4, t], data[8, t])
        gait_data[t, 3], gait_data[t, 4 ], gait_data[t, 5 ] = leg_ikine(data[1, t], data[5, t], data[9, t])
        gait_data[t, 6], gait_data[t, 7 ], gait_data[t, 8 ] = leg_ikine(data[2, t], data[6, t], data[10, t])
        gait_data[t, 9], gait_data[t, 10], gait_data[t, 11] = leg_ikine(data[3, t], data[7, t], data[11, t])
    return rate, gait_data

def slantleft_gait():
    gait_data = np.zeros((radio, 12))
    data = td.slantleft_gait(1)
    for t in range(gait_data.shape[0]):
        gait_data[t, 0], gait_data[t, 1 ], gait_data[t, 2 ] = leg_ikine(data[0, t], data[2, t], data[4, t])
        gait_data[t, 3], gait_data[t, 4 ], gait_data[t, 5 ] = leg_ikine(data[1, t], data[3, t], data[5, t])
        gait_data[t, 6], gait_data[t, 7 ], gait_data[t, 8 ] = leg_ikine(data[1, t], data[3, t], data[5, t])
        gait_data[t, 9], gait_data[t, 10], gait_data[t, 11] = leg_ikine(data[0, t], data[2, t], data[4, t])
    return rate, gait_data

def slantright_gait():
    gait_data = np.zeros((radio, 12))
    data = td.slantleft_gait(-1)
    for t in range(gait_data.shape[0]):
        gait_data[t, 0], gait_data[t, 1 ], gait_data[t, 2 ] = leg_ikine(data[0, t], data[2, t], data[4, t])
        gait_data[t, 3], gait_data[t, 4 ], gait_data[t, 5 ] = leg_ikine(data[1, t], data[3, t], data[5, t])
        gait_data[t, 6], gait_data[t, 7 ], gait_data[t, 8 ] = leg_ikine(data[1, t], data[3, t], data[5, t])
        gait_data[t, 9], gait_data[t, 10], gait_data[t, 11] = leg_ikine(data[0, t], data[2, t], data[4, t])
    return rate, gait_data


def jump_gait():

    return 

def keep_gait():
    gait_data = np.zeros((radio, 12))
    rate = 2
    x_line, y_line, z_line = keep_line()
    for t in range(gait_data.shape[0]):
        if (t < 20):
            xf = x_line[t]
            xb = x_line[t + 20]
            yf = y_line[t]
            yb = y_line[t + 20]
            zf = z_line[t]
            zb = z_line[t + 20]
        else:
            xf = x_line[t]
            xb = x_line[t - 20]
            yf = y_line[t]
            yb = y_line[t - 20]
            zf = z_line[t]
            zb = z_line[t - 20]

        gait_data[t, 0], gait_data[t, 1 ], gait_data[t, 2 ] = leg_ikine(xf, yf, zf)
        gait_data[t, 3], gait_data[t, 4 ], gait_data[t, 5 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 6], gait_data[t, 7 ], gait_data[t, 8 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 9], gait_data[t, 10], gait_data[t, 11] = leg_ikine(xf, yf, zf)
    return rate, gait_data

def clam_gait():
    gait_data = np.zeros((radio, 12))
    rate = 0.5
#    x_line, y_line, z_line = keep_line()
    for t in range(gait_data.shape[0]):
        if (t < 20):
            xf = 0.4
            xb = 0.4
            yf = 0.15
            yb = 0.15
            zf = 0.1
            zb = 0.1
        else:
            xf = 0.4
            xb = 0.4
            yf = 0.15
            yb = 0.15
            zf = 0.1
            zb = 0.1

        gait_data[t, 0], gait_data[t, 1 ], gait_data[t, 2 ] = leg_ikine(xf, yf, zf)
        gait_data[t, 3], gait_data[t, 4 ], gait_data[t, 5 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 6], gait_data[t, 7 ], gait_data[t, 8 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 9], gait_data[t, 10], gait_data[t, 11] = leg_ikine(xf, yf, zf)
    return rate, gait_data
    

def leg_ikine(x, y, z):     
    theta1 = math.atan2(y, x) + math.atan2(l1, -(x**2 + y**2 - l1**2)**0.5)
    c1 = math.cos(theta1)    
    s1 = math.sin(theta1) 
    c3 = (x**2 + y**2 + z**2 - l1**2 - l2**2 - l3**2) / (2 * l2 * l3)
    s3 = (1 - c3**2)**0.5
    theta3 = math.atan2(s3, c3)   
    s2p = (l3 * s3) / ((y * s1 + x * c1)**2 + z**2)**0.5
    c2p = (1 - s2p**2)**0.5
    theta2p = -math.atan2(s2p, c2p)
    thetap = -math.atan2(z, -(y * s1 + x * c1))
    theta2 = theta2p - thetap
    return theta1 - math.pi, theta2, theta3


def gait_line():
    data = td.forward_gait()
    x_line = data[0, :]
    y_line = data[1, :]
    z_line = data[2, :]
    return x_line, y_line, z_line

def keep_line():
    data = td.keep_gait()
    x_line = data[0, :]
    y_line = data[1, :]
    z_line = data[2, :]
    return x_line, y_line, z_line

def turn_line(direction):
    data = td.turn_gait(direction)
    return data

def jump_line():
    xf_line = np.zeros((20))

    return


