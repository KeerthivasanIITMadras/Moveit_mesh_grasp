import open3d as o3d
import numpy as np
import math
import tf

class Robotiq:
    def __init__(self):
        self.t1 =0
        self.t2 =0
        self.t3 =0
        self.dist =0
        self.n1 =0
        self.n2 =0
        self.normal =0
        

    def isclose(self,a,b):
        if abs(a-b)<0.1:
            return True
        else:
            return False

    def are_parallel(self,vec1, vec2):
        theta = np.arctan2(np.linalg.norm(np.cross(
                vec1, vec2)), np.dot(vec1, vec2))
        if theta*180/np.pi<=2 and theta*180/np.pi>=0:
            return True
        else:
            return False

    def is_antiparallel(self,vec1, vec2):
        theta = np.arctan2(np.linalg.norm(np.cross(
               vec1, vec2)), np.dot(vec1, vec2))
        if theta*180/np.pi<=180 and theta*180/np.pi>=178:
            return True
        else:
            return False
    
    def EEFpose(self,s1,s2):
        altitude = (s1**2 - (s2**2)/4)**0.5
        base = altitude/2 - 4.4704
        hpt = 9.4488
        palm_dist = 9.2075 + (hpt**2 - base**2)**0.5
        return palm_dist

    def EEf(self):
        centr = (self.t1 +(self.t2 + self.t3)/2)/2
        # Note: The incentre part doesnt work
        '''taking incentre'''
        # a = np.linalg.norm(self.t2-self.t3)
        # b = np.linalg.norm(self.t3-self.t1)
        # c = np.linalg.norm(self.t2-self.t1)
        # centr = (a*self.t1+b*self.t2+c*self.t3)/(a+b+c)
        # centr = (self.t1+self.t2+self.t3)/3   #not the centroid
        '''taking circumcenter'''
        # A = np.arctan2(np.linalg.norm(np.cross(
        #         self.t3-self.t1, self.t2-self.t1)), np.dot(self.t3-self.t1, self.t2-self.t1))
        # B = np.arctan2(np.linalg.norm(np.cross(
        #         self.t3-self.t2, self.t1-self.t2)), np.dot(self.t3-self.t2, self.t1-self.t2))
        # C= np.arctan2(np.linalg.norm(np.cross(
        #         self.t2-self.t3, self.t1-self.t3)), np.dot(self.t2-self.t3, self.t1-self.t3))
        # centr = (self.t1 * math.sin(2 * A) + self.t2 * math.sin(2 * B) + self.t3 * math.sin(2 * C)) / (math.sin(2 * A) + math.sin(2 * B) + math.sin(2 * C))

        self.normal = np.cross((self.t1 -self.t2),(self.t1 -self.t3))
        pose = centr - self.dist*self.normal/np.linalg.norm(self.normal)
        return np.array(pose)

    def orientation(self):
        # normal will be the z axis of the end effector
        # roll = np.arctan2(np.linalg.norm(np.cross(
        #         self.normal, np.asarray([1, 0, 0]))), np.dot(self.normal, np.asarray([1, 0, 0])))
        # pitch = np.arctan2(np.linalg.norm(np.cross(
        #        self.normal, np.asarray([0, 1, 0]))), np.dot(self.normal, np.asarray([0, 1, 0])))
        # yaw = np.arctan2(np.linalg.norm(np.cross(
        #        self.normal, np.asarray([0, 0, 1]))), np.dot(self.normal, np.asarray([0, 0, 1])))
        
        '''
        R=[tx ty tz]
        where each entry is 3x1 vector
        tool vector is [a,b,c]
        is tz
        then tx is cross of tz and [0,1,0]
        and ty is cross between tx and tz
        soln: https://answers.ros.org/question/253361/define-end-effector-pose-by-point-and-vector/
        '''
        R = np.eye(3,3)
        R[:3,2] = -self.normal/np.linalg.norm(-self.normal.T)
        tx = np.cross(-self.normal,np.array([0,-1,0]))
        tx = tx/np.linalg.norm(tx)
        ty = np.cross(-self.normal,tx)
        ty = ty/np.linalg.norm(ty)
        R[:3,1]=ty.T
        R[:3,0]=tx.T
        
        rotation_z_90 = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])

        # Perform the rotation of the Y-axis by multiplying R with the rotation matrix
        new_R = np.dot(R, rotation_z_90)
        # theta = np.pi/2
        # R_90 =  np.array(tf.transformations.rotation_matrix(theta, tx))
        # roll, pitch, yaw = tf.transformations.euler_from_matrix(R*R_90[:3,:3])
        roll, pitch, yaw = tf.transformations.euler_from_matrix(new_R)
        return [roll,pitch,yaw]

    def choose(self,triplet,normals):
        side1 = np.linalg.norm(triplet[0]-triplet[1])
        side2 = np.linalg.norm(triplet[1]-triplet[2])
        side3 = np.linalg.norm(triplet[2]-triplet[0])
        # the first condition has many cases but checking only this
        if self.isclose(side1,side2) and self.isclose(side1,side3):
            if self.isclose(side1,7.302):
                if self.are_parallel(normals[0],normals[1]) and self.is_antiparallel(normals[0],normals[2]):
                    self.dist = self.EEFpose(side1,side3)
                    self.n1 = normals[1]
                    self.n2 = normals[2]
                    self.t1 = triplet[1]
                    self.t2 = triplet[2]
                    self.t3 = triplet[0]
                    return True
        elif self.isclose(side1,side2):
            if side1<18.8695779 and side1>0.1 and self.isclose(side3,7.302):
               if self.are_parallel(normals[0],normals[2]) and self.is_antiparallel(normals[0],normals[1]):
                    self.dist = self.EEFpose(side1,side3)
                    self.n1 = normals[1]
                    self.n2 = normals[2]
                    self.t1 = triplet[1]
                    self.t2 = triplet[2]
                    self.t3 = triplet[0]
                    return True
        elif self.isclose(side1,side3):
            if side1<18.8695779 and side1>0.1 and self.isclose(side2,7.302):
                if self.are_parallel(normals[2],normals[1]) and self.is_antiparallel(normals[0],normals[2]):
                    self.dist = self.EEFpose(side1,side2)
                    self.n1 = normals[0]
                    self.n2 = normals[2]
                    self.t1 = triplet[0]
                    self.t2 = triplet[1]
                    self.t3 = triplet[2]
                    return True
        elif self.isclose(side3,side2):
            if side2<18.8695779 and side2>0.1 and self.isclose(side1,7.302):
                if self.are_parallel(normals[0],normals[1]) and self.is_antiparallel(normals[0],normals[2]):
                    self.dist = self.EEFpose(side2,side1)
                    self.n1 = normals[2]
                    self.n2 = normals[1]
                    self.t1 = triplet[2]
                    self.t2 = triplet[1]
                    self.t3 = triplet[0]
                    return True

        return False
