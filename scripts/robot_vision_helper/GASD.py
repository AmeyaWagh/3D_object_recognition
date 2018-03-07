import pcl
import numpy as np
from numpy import linalg as LA
import math
# import tensorflow as tf

class GASD:
	def __init__(self):
		self.p=np.array([])
		self.new_p=np.array([])

	def compute_centroid(self):
		self.centroid = np.mean(self.p,axis=0)
	
	def compute_covariance(self):
		C = []
		self.cov = np.cov(self.p.transpose())

	def compute_eigen_values(self):
		self.compute_covariance()
		w,v = LA.eig(self.cov)
		
		self.Vz = v[:,0]
		self.Vx = v[:,2]
		self.Vy = np.cross(self.Vz, self.Vx)

		
		self.rotation_mat = np.array([self.Vx,self.Vy,self.Vz])
		self.rotation_mat = self.rotation_mat.transpose()
		
		self.translation_mat = np.array([[self.centroid[0],
										self.centroid[1],
										self.centroid[2]]])
		
		
		self.transform = np.concatenate((self.rotation_mat, self.translation_mat.transpose()), axis=1)
		self.transform = np.concatenate((self.transform, np.array([[0,0,0,1]])), axis=0)
		self.inv_trans = np.linalg.inv(self.transform)
		

	def compute_transform(self):
		p_vec = np.concatenate((self.p.T, np.ones((1,self.p.shape[0]))), axis=0)
		self.new_p = np.matmul(self.inv_trans,p_vec)
		self.new_p = self.new_p[0:3,:].T


	def compute_gasd(self,p_array):
		self.p = p_array
		self.compute_centroid()
		self.compute_eigen_values()
		self.compute_transform()
		return self.centroid,[list(self.Vx),list(self.Vy),list(self.Vz)],self.new_p

	
	def compute_quaternion(self):
		w = math.sqrt(1.0+self.rotation_mat[0,0]+self.rotation_mat[1,1]+self.rotation_mat[2,2])
		x = (self.rotation_mat[2,1]-self.rotation_mat[1,2])/(4*w)
		y = (self.rotation_mat[0,2]-self.rotation_mat[2,0])/(4*w)
		z = (self.rotation_mat[1,0]-self.rotation_mat[0,1])/(4*w)
		self.quaternion = (x,y,z,w)

	def get_volumetric_data(self,p_array,size=(0.3,0.3,0.3),grid=40):
		self.p = p_array
		self.compute_centroid()
		self.compute_eigen_values()
		self.compute_transform()
		self.compute_quaternion()

		step = lambda x: (x/abs(x))*math.ceil(abs(x)) if abs(abs(x)-math.floor(abs(x)))>0.5 else (x/abs(x))*math.floor(abs(x))
		idx = lambda x,offset: int(step(x)+offset)
		volume_data = np.zeros((grid,grid,grid))

		for pt in self.new_p:
			_x = idx(pt[0]*grid/size[0],grid/2)
			_y = idx(pt[1]*grid/size[1],grid/2)
			_z = idx(pt[2]*grid/size[2],grid/2)
			try:
				volume_data[_x][_y][_z] += 1.0
			except:
				# print("[ warn] x,y,z out of bound, skipped point")
				pass
		return volume_data,self.quaternion	



