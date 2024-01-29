#!/usr/bin/env python3

import open3d as o3d
import rospy
import tf
import tf2_ros
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from itertools import combinations
from scipy.optimize import minimize
import time
import Gripper
import math


def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()

class ros_interface():
    def __init__(self,ctr,eef,rpy):
        rospy.init_node('MBGP')
        self.rate = rospy.Rate(10)
        self.center_point = ctr/100
        self.eef = eef/100
        self.rpy = rpy
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    def EEFtf(self):
        # isolated finger corresponds to the orientation of the y axis of the end effector
        br = tf.TransformBroadcaster()
        br.sendTransform((self.eef[0], self.eef[1], self.eef[2]),
                     tf.transformations.quaternion_from_euler(self.rpy[0], self.rpy[1], self.rpy[2]),
                     rospy.Time.now(),
                     "eef_frame",
                     "object_frame")
       

    def TFpub(self):
         
        #  self.sendworldTF()
         self.EEFtf()
         #self.EEf_obj()
         rospy.loginfo("Publishing Transforms")

    

class logger():
    def __init__(self,):
      
        self.candidate1 = []
        self.candidate2 = []
        self.candidate3 = []
        self.err = []
        self.handle_force = []
        self.sec_force = []
        self.ter_force = []
        self.temp = 70
        self.min = []
        self.eefa = []
        self.min_eef = []
    
    

    def log(self,sol,id,err,pcd,eefs,rpy):

        ns = np.asarray([pcd.normals[id[0]],pcd.normals[id[1]],pcd.normals[id[2]]])

        if err < self.temp:
            self.temp = err
            self.min = id
            self.min_eef= eefs
            self.rpy = rpy
        self.candidate1.append(id[0])
        self.candidate2.append(id[1])
        self.candidate3.append(id[2])
        self.handle_force.append([sol[2]])
        self.sec_force.append([sol[5]]) 
        self.ter_force.append([sol[8]])
        self.err.append(err)
        self.eefa.append(eefs)
        
          

    def save_file(self,filename):
       df = pd.DataFrame({"Pt1" : self.candidate1, "Pt2" : self.candidate2,"Pt3" : self.candidate3, "F1" : self.handle_force, "F2": self.sec_force, "F3": self.ter_force, "Error" :self.err, "EEF":self.eefa})
       df.to_csv(f"{filename}.csv", index = False)
       print(self.min_eef)
       return [self.min, self.min_eef, self.rpy]
       

    def cost_visualizer(self):
        k = []
        plt.figure(num='MBGP Solver Results')
        plt.style.use('seaborn-darkgrid')
        for i in range(len(self.err)):
            k.append(i)
        plt.scatter( k ,self.err,label=' Cost comparison')
        plt.xlabel('Candidate Number')
        plt.ylabel('Final cost')
        plt.title('cost analysis')
        plt.show()
    

class Optimization():

    def __init__(self, pcd):
        self.iter = 0
        self.isochoose = Gripper.Robotiq()
        self.pcd = pcd
        self.max_force = 70
        self.f_ext_1 = np.array([0, 0, 20, 0, 0, 0])
        # For point contact with friction
        self.Bci = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1], [
            0, 0, 0], [0, 0, 0], [0, 0, 0]])
        self.G = None
        self.mew = 0.7  
        self.fc = np.array([0, 0, 70, 0, 0, 70, 0, 0, 70])
        self.min = 70  # Taking 15 since max error will be 10
        self.solved_combination = None
        self.solution = None
        self.idt = 0
        self.valid_points = 0

   
    def select(self):
        points = np.array(self.pcd.points)
        normals_all = np.array(self.pcd.normals)
        point_indices = list(range(len(points)))
        point_combinations = combinations(point_indices, 3)
        for comb in point_combinations:
            indices = list(comb)
            self.idt = indices
            triplets = np.array(points)[indices]
            normals = normals_all[indices]
            self.iter = self.iter +1
            printProgressBar(self.iter, math.comb(len(points),3), prefix = 'Solver progress:', suffix = 'Complete', length = 50)
            if self.isochoose.choose(triplets,normals):
                self.transformation(triplets,normals)
                self.valid_points+=1
                
            else:
                continue
        print("Found",self.valid_points,"solutions")
            
    def transformation(self,triplets,normals):
        self.G = None
        for i in range(len(triplets)):
            normal = normals[i]
            point = triplets[i]
            # This gives us orientation of normal vector with x,y and z axis
            normal = -normal
            x_axis_angle = np.arctan2(np.linalg.norm(np.cross(
                normal, np.asarray([1, 0, 0]))), np.dot(normal, np.asarray([1, 0, 0])))
            y_axis_angle = np.arctan2(np.linalg.norm(np.cross(
                normal, np.asarray([0, 1, 0]))), np.dot(normal, np.asarray([0, 1, 0])))
            z_axis_angle = np.arctan2(np.linalg.norm(np.cross(
                normal, np.asarray([0, 0, 1]))), np.dot(normal, np.asarray([0, 0, 1])))
            R_alpha = np.array([[np.cos(z_axis_angle), -np.sin(z_axis_angle), 0],
                                [np.sin(z_axis_angle), np.cos(
                                    z_axis_angle), 0],
                                [0, 0, 1]])
            R_beta = np.array([[np.cos(y_axis_angle), 0, np.sin(y_axis_angle)],
                               [0, 1, 0],
                               [-np.sin(y_axis_angle), 0, np.cos(y_axis_angle)]])
            R_gamma = np.array([[1, 0, 0],
                                [0, np.cos(x_axis_angle), -
                                 np.sin(x_axis_angle)],
                                [0, np.sin(x_axis_angle), np.cos(x_axis_angle)]])
            R = np.dot(R_alpha, np.dot(R_beta, R_gamma))
            H = np.matrix(
                [[0, -point[2], point[1]], [point[2], 0, -point[0]], [-point[1], point[0], 0]])
            cross_G = np.dot(H, R)
            zeros_matrix = np.zeros((3, 3))
            G_i = np.vstack((np.hstack((R, zeros_matrix)),
                            np.hstack((np.cross(cross_G, R), R))))
            F_oi = np.dot(G_i, self.Bci)
            if self.G is None:
                self.G = F_oi
            else:
                self.G = np.hstack((self.G, F_oi))     
        self.solve()



    def objective_function(self, fc):
        return np.linalg.norm(np.dot(self.G, fc)+self.f_ext_1)
    
    def constraint_1(self, fc):
        return self.mew*fc[2]-np.sqrt(fc[0]**2+fc[1]**2)

    def constraint_2(self, fc):
        return self.mew*fc[5]-np.sqrt(fc[3]**2+fc[4]**2)

    def constraint_3(self, fc):
        return self.mew*fc[8]-np.sqrt(fc[6]**2+fc[7]**2)
        
    # def constraint_4(self,fc):
    #     return fc[2]-fc[5]
    
    # def constraint_5(self,fc):
    #     return fc[5]-fc[8]

    def solve(self):
        con1 = {'type': 'ineq', 'fun': self.constraint_1}
        con2 = {'type': 'ineq', 'fun': self.constraint_2}
        con3 = {'type': 'ineq', 'fun': self.constraint_3}
        # con4 = {'type': 'eq', 'fun': self.constraint_4}
        # con5 = {'type': 'eq', 'fun': self.constraint_5}
        b = (0, 70)
        bnds = [b, b, b, b, b, b, b, b, b]

        cons = [con1, con2, con3]
        sol = minimize(self.objective_function, self.fc,
                       method='SLSQP', bounds=bnds, constraints=cons)
        err = self.objective_function(sol.x)
        if err < 10:
            print(
                f"Normal forces to be applied at the contacts {sol.x[2]} {sol.x[5]} {sol.x[8]} and corresponding error = {self.objective_function(sol.x)}")
            print(
                f"Friction forces in these points are {sol.x[0]} {sol.x[1]} {sol.x[3]} {sol.x[4]} {sol.x[6]} {sol.x[7]}")
        solution = list(sol.x)
        eef = np.asarray(self.isochoose.EEf())
        RPY = self.isochoose.orientation()
        
        
        log1.log(solution,self.idt,err,self.pcd,eef,RPY)

       

def mesh2PointCloud(mesh,n_pts):
    pcd = mesh.sample_points_uniformly(n_pts,seed=32)
    return pcd


def force_visualizer(mesh, points, normals,center_point,eef):
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window()
    visualizer.add_geometry(mesh)


    print(eef)
    point3 = np.array([eef])
    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(point3)
    visualizer.add_geometry(pcd1)

    point4 = np.array([0,0,0])
    pcd4 = o3d.geometry.PointCloud()
    pcd4.points = o3d.utility.Vector3dVector(point3)
    visualizer.add_geometry(pcd4)

    points5 = np.array([eef,points[0]])
    pcd5 = o3d.geometry.PointCloud()
    pcd5.points = o3d.utility.Vector3dVector(points5)
    lines5 = [[0, 1]]
    colors5 = [[0, 0, 1]]  # Red color
    line_set5 = o3d.geometry.LineSet()
    line_set5.points = o3d.utility.Vector3dVector(points5)
    line_set5.lines = o3d.utility.Vector2iVector(lines5)
    line_set5.colors = o3d.utility.Vector3dVector(colors5)
    visualizer.add_geometry(line_set5)

    points6 = np.array([eef,points[1]])
    pcd6 = o3d.geometry.PointCloud()
    pcd6.points = o3d.utility.Vector3dVector(points6)
    lines6 = [[0, 1]]
    line_set6 = o3d.geometry.LineSet()
    line_set6.points = o3d.utility.Vector3dVector(points6)
    line_set6.lines = o3d.utility.Vector2iVector(lines6)
    line_set6.colors = o3d.utility.Vector3dVector(colors5)
    visualizer.add_geometry(line_set6)

    points7 = np.array([eef,points[2]])
    pcd7 = o3d.geometry.PointCloud()
    pcd7.points = o3d.utility.Vector3dVector(points7)
    lines7 = [[0, 1]]
    colors8 = [[0, 1, 0]] 
    line_set7 = o3d.geometry.LineSet()
    line_set7.points = o3d.utility.Vector3dVector(points7)
    line_set7.lines = o3d.utility.Vector2iVector(lines7)
    line_set7.colors = o3d.utility.Vector3dVector(colors5)
    visualizer.add_geometry(line_set7)

# Visualize the point cloud
    

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.normals = o3d.utility.Vector3dVector(normals)
    visualizer.add_geometry(pcd)

    scene = o3d.geometry.PointCloud()
    points2 = np.array([[100, 200, 300], [400, 500, 600], [700, 800, 900]])
    scene.points = o3d.utility.Vector3dVector(points2)
    axes_line_set = o3d.geometry.LineSet()
    axes_line_set.points = o3d.utility.Vector3dVector([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
    axes_line_set.lines = o3d.utility.Vector2iVector([[0, 1], [0, 2], [0, 3]])
    colors1 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    axes_line_set.colors = o3d.utility.Vector3dVector(colors1)
    visualizer.add_geometry(axes_line_set)


    # the axis in open3d is in rhs frame where the x axis is towards to the right
    end_points_z_axis = np.array([[center_point[0],center_point[1],0], [center_point[0],center_point[1],75]])
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(end_points_z_axis)

    end_points_x_axis = np.array([[0,center_point[1],center_point[2]], [75,center_point[1],center_point[2]]])
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(end_points_x_axis)


    end_points_y_axis = np.array([[center_point[0],0,center_point[2]], [center_point[0],75,center_point[2]]])
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(end_points_y_axis)

    eef_center= np.array([eef, center_point])
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(end_points_y_axis)

    t1  = points[0]
    t2 = points[1]
    t3 = points[2]

    normal = np.cross((t1 -t2),(t1 -t3))

    eef_normal_axis_point = eef-normal

    eef_normal= np.array([eef, eef_normal_axis_point/4])
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(eef_normal)


    lines = [[0, 1]]
    colors = [[0, 0, 1]]  # Blue color
    line_set_z = o3d.geometry.LineSet()
    line_set_z.points = o3d.utility.Vector3dVector(end_points_z_axis)
    line_set_z.lines = o3d.utility.Vector2iVector(lines)
    line_set_z.colors = o3d.utility.Vector3dVector(colors)


    lines = [[0, 1]]
    colors = [[1, 0, 0]]  # Red color
    line_set_x = o3d.geometry.LineSet()
    line_set_x.points = o3d.utility.Vector3dVector(end_points_x_axis)
    line_set_x.lines = o3d.utility.Vector2iVector(lines)
    line_set_x.colors = o3d.utility.Vector3dVector(colors)

    lines = [[0, 1]]
    colors = [[0, 1, 0]]  # Green color
    line_set_y = o3d.geometry.LineSet()
    line_set_y.points = o3d.utility.Vector3dVector(end_points_y_axis)
    line_set_y.lines = o3d.utility.Vector2iVector(lines)
    line_set_y.colors = o3d.utility.Vector3dVector(colors)


    lines = [[0, 1]]
    colors = [[0, 0, 0]]  # Black color
    line_set_eef_center = o3d.geometry.LineSet()
    line_set_eef_center.points = o3d.utility.Vector3dVector(eef_center)
    line_set_eef_center.lines = o3d.utility.Vector2iVector(lines)
    line_set_eef_center.colors = o3d.utility.Vector3dVector(colors)
    

    lines = [[0, 1]]
    colors = [[1, 0, 0]]  # Red color
    line_set_normal_eef = o3d.geometry.LineSet()
    line_set_normal_eef.points = o3d.utility.Vector3dVector(eef_normal)
    line_set_normal_eef.lines = o3d.utility.Vector2iVector(lines)
    line_set_normal_eef.colors = o3d.utility.Vector3dVector(colors)

    visualizer.add_geometry(line_set_x)
    visualizer.add_geometry(line_set_z)
    visualizer.add_geometry(line_set_y)
    visualizer.add_geometry(line_set_eef_center)
    visualizer.add_geometry(line_set_normal_eef)
    
    visualizer.run()
    visualizer.destroy_window()


def main():
    eefy=[]
    start = time.time()
    name = "custom_planner/cad_files/Cylinder"
    mesh_path = f"{name}.stl"
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.compute_vertex_normals()
    pcd = mesh2PointCloud(mesh,1500)
    # for cube
    scaling_factor = 0.5
    # for cuboid
    scaling_factor = 0.3
    # for cylinder
    scaling_factor = 0.7
    # stl file works like this
    # It will have some x axis points starting from x to y
    # but that x to y may not contain the 0
    # So the mesh needs to be normalised inorder for axis transformations to work

    # step1 : scaling down the mesh according ot a scaling factor depending on the stl file

    pcd.points = o3d.utility.Vector3dVector(np.array(pcd.points)*scaling_factor)
    center_point = np.mean(np.asarray(pcd.points), axis=0)
    
    # shifting the whole mesh such that the center of the mesh is the center of the open3d gui
    pcd.points = o3d.utility.Vector3dVector(np.array(pcd.points)-center_point)
    
    # reconstructed mesh
    mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices)-np.array([center_point/scaling_factor]))
    center_point = np.mean(np.asarray(pcd.points), axis=0)
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    print(f"min value of z coordinate = {np.min(points[:,2])}")
    print(f"max value of z coordinate = {np.max(points[:,2])}")

    # removing the non accesible regions``
    # these are set by the user according to the visibity and accesibility
    # filtered_normals = normals[points[:, 1] <= 2]
    # filtered_points = points[points[:, 1] <= 2]

    filtered_normals = normals
    filtered_points = points
    filtered_points2 = filtered_points[filtered_points[:,2]>=(np.min(points[:,2])+0.1)]
    filtered_normals2 = filtered_normals[filtered_points[:,2]>=(np.min(points[:,2])+0.1)]
    filtered_point_cloud = o3d.geometry.PointCloud()
    filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points2)
    filtered_point_cloud.normals = o3d.utility.Vector3dVector(filtered_normals2)
    # Reconstruct a mesh from the filtered point cloud
    # for cube
    distance_threshold =0.4 # Adjust as needed
    # for cuboid
    distance_threshold =0.8
    mesh2 = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        filtered_point_cloud,
        o3d.utility.DoubleVector([distance_threshold, distance_threshold * 2])
    )

    # Visualize the resulting mesh
    # for cube and cuboid 200 is enough
    # for cylinder the basic solution obtainer is 250, increasing this will increase the solution but increases computational time
    pcd = mesh2PointCloud(mesh2,300)
    o3d.visualization.draw_geometries([mesh2])
    # this line is for visualization, the vertices are scaled appropriately for visualization
    mesh2.vertices = o3d.utility.Vector3dVector(np.asarray(mesh2.vertices)/scaling_factor)
    optimser = Optimization(pcd)
    optimser.select()
    path_to_save = "custom_planner/csv_files/Cylinder"
    [min, eefy, RPY] = log1.save_file(path_to_save)
    log1.cost_visualizer()


    ns = np.asarray([pcd.normals[min[0]],pcd.normals[min[1]],pcd.normals[min[2]]])
    
    pts = np.asarray([pcd.points[min[0]],pcd.points[min[1]],pcd.points[min[2]]])

    force_visualizer(mesh2,pts/scaling_factor,ns,center_point/scaling_factor,eefy/scaling_factor)


    return [center_point, eefy, RPY]


log1 = logger()

if __name__ == "__main__":
    [ctr,eef,rpy] = main()
    df = pd.DataFrame({"ctr" : ctr, "eef" :eef,"rpy" : rpy})
    df.to_csv("buffer.csv", index = False)
    df2 = pd.read_csv("buffer.csv")
    ctr_array = df2['ctr'].to_numpy()  # Assuming 'ctr' column contains NumPy arrays
    eef_array = df2['eef'].to_numpy()  # Assuming 'eef' column contains NumPy arrays
    rpy_array = df2['rpy'].to_numpy()  # Assuming 'rpy' column contains NumPy arrays
    MBGP = ros_interface(ctr_array,eef_array,rpy_array)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        MBGP.TFpub()
        try:
           # Perform the transform lookup from 'your_frame' to 'base_link'
           transform = tf_buffer.lookup_transform('world', 'eef_frame', rospy.Time(0))
           # 'your_frame' should be replaced with the frame you want to look up
           # Print the transform details
        #    rospy.loginfo("Transform from 'base_link' to 'your_frame':")
        #    rospy.loginfo("Translation: x={}, y={}, z={}".format(
        #        transform.transform.translation.x,
        #        transform.transform.translation.y,
        #        transform.transform.translation.z
        #    ))
        #    rospy.loginfo("Rotation: x={}, y={}, z={}, w={}".format(
        #        transform.transform.rotation.x,
        #        transform.transform.rotation.y,
        #        transform.transform.rotation.z,
        #        transform.transform.rotation.w
        #    ))
           tf_broadcaster.sendTransform(transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Exception occurred: {}".format(e))
        rate.sleep()
        


    
