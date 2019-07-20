#!/usr/bin/env python

import rospy
import rospkg

import cv2

from orb_slam2_ros.srv import getMap
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np


lower_limmit=0
upper_limit=0

def filterPoints(point):
    global upper_limit,lower_limmit
    if(point[2]<lower_limmit):
        return False
    if(point[2]>upper_limit):
        return False
    if(point[2]==0.0):
        return False
    else:
        return True


class generator:

    def __init__(self):
        #self.image_pub = rospy.Publisher("image_topic_2",Image)
       
        self.sub = None
        self.image= None
        rospack = rospkg.RosPack()
        self.height=0.6
        self.path=rospack.get_path('orb_slam2_ros')

        print(self.path)
    
    def generator(self,req):
        global upper_limit,lower_limmit
        self.sub = rospy.Subscriber("/orb_slam2_stereo/map_points", PointCloud2, self.callback)
        self.name=req.FileName
        self.radius = req.radius
        self.dx=req.dx
        self.dy=req.dy

        lower_limit=req.lower_level
        upper_limit=req.upper_level
        while(self.image is None):
            i=1
        return []


    def callback(self,msg):
        global lower_limmit,upper_limit
        print(msg.height)
        print(msg.width)
        self.image=12
        self.sub.unregister()

        
        cloud_points=self.get_list(msg)
        cloud_points=self.filter_floor_roof(cloud_points)
        cloud_points=np.array(cloud_points)
        self.get_margins(cloud_points)
        self.get_image(cloud_points)
    
        print(cloud_points.shape)
        return

    def get_image(self,point_cloud):
        width=int((self.x_max-self.x_min)/self.dx)
        height=int((self.y_max-self.y_min)/self.dy)

        image=np.ones((width,height,3), np.uint8)*255
        cv2.imwrite("test.jpg",image.astype('uint8'))
        for i in range(point_cloud.shape[0]):
            ix=int((point_cloud[i,0]-self.x_min)/self.dx)-1
            iy=int((point_cloud[i,1]-self.y_min)/self.dy)-1
            #self.draw_circle(ix,iy,image)
            cv2.circle(image,(iy,ix), int(self.radius/self.dx), (0,255,191), -1)

        for i in range(point_cloud.shape[0]):
            ix=int((point_cloud[i,0]-self.x_min)/self.dx)-1
            iy=int((point_cloud[i,1]-self.y_min)/self.dy)-1
            #self.draw_circle(ix,iy,image)
            cv2.circle(image,(iy,ix), 2, (0,0,0), -1)

           

        cv2.imwrite("{}/maps/{}.jpg".format(self.path,self.name),image.astype('uint8'))


    def draw_circle(self,ix,iy,image):
        rad=int((self.radius)/self.dx)
        print(rad)
        for i in range(ix-rad+1,ix+rad):
            if (i>=0) and (i<image.shape[0]):
                for j in range(iy-rad+1,iy+rad):
                    if (j>=0) and (j<image.shape[1]):
                        if (i-ix)**2+(j-iy)**2<rad**2:
                            image[i,j]=[0,191,255]
        return

    def get_margins(self,point_cloud):
        minimo=point_cloud.min(axis=0)
        maximo=point_cloud.max(axis=0)
        
        self.x_min=minimo[0]
        self.x_max=maximo[0]

        self.y_min=minimo[1]
        self.y_max=maximo[1]
        
        return
    
    def get_list(self,data):
        cloud_points = list(pc2.read_points(data, skip_nans=True, field_names = ("x", "y", "z")))
        return cloud_points

    
    
    def filter_floor_roof(self,cloud_points):
        print(len(cloud_points))
        cloud_points=filter(filterPoints,cloud_points)
        print(len(cloud_points))
        return cloud_points

def map_generator(generator):
        
        rospy.Service('orb_slam2/map_generator', getMap, generator.generator)
        rospy.loginfo("Service ready!")
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('map_generator_server')
    gen = generator()
    map_generator(gen)