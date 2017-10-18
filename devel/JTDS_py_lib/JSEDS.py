import numpy as np
from scipy.stats import multivariate_normal
import sys
sys.path.append('/home/yonadav/Research/joint-pbd/utils/robotics-toolbox-python/')
from robot import *

#JSEDS.MotionGenerator.CreateMotionGeneratorFromFile('/home/yonadav/Research/joint-pbd/kuka_demos/results/backhand_params.txt')
def arrayFromNextLine(f):
	return np.fromstring(f.readline(), sep=' ')

class MotionGenerator:
	def __init__(self, robotPlant, AGenerator, bound_type=None):
		self.robotPlant = robotPlant
		self.AGenerator = AGenerator
		if bound_type != None and bound_type != 'discrete' and bound_type != 'continuous':
			raise Exception('Motion generator bound type must be either "continuous" or "discrete"')
		self.bound_type = bound_type

		qmin = []
		qmax = []
		qdim = self.robotPlant.robot.n
		for i in xrange(qdim):
			qmin.append(self.robotPlant.robot.links[i].qlim[0,0])
			qmax.append(self.robotPlant.robot.links[i].qlim[0,1])
		self.qmin = np.asarray(qmin).reshape((qdim, 1))
		self.qmax = np.asarray(qmax).reshape((qdim, 1))

	@staticmethod
	def CreateMotionGeneratorFromFile(filename, bound_type=None):
		f = open(filename, 'r')
		dimq, dimz, K = map(int, arrayFromNextLine(f)) # First line provides basic GMM parameters
		Priors = arrayFromNextLine(f) # Next line is the priors
		Mu = np.reshape(arrayFromNextLine(f), (dimz, K), order='F') # Next line is the mus
		for i in xrange(K): # Next K lines are the sigmas, reshaped fortran style
			sigma = np.reshape(arrayFromNextLine(f), (dimz, dimz), order='F')
			if i == 0:
				Sigma = sigma
			else:
				Sigma = np.dstack((Sigma, sigma))
		for i in xrange(K): # Next K lines are the As, reshaped fortran style
			A = np.reshape(arrayFromNextLine(f), (dimq, dimq), order='F')
			if i == 0:
				As = A
			else:
				As = np.dstack((As, A))
		M = np.reshape(arrayFromNextLine(f), (dimz, dimq), order='F')
		# Then gather all the robot parameters
		D = arrayFromNextLine(f)
		A = arrayFromNextLine(f)
		Alpha = arrayFromNextLine(f)
		Qmin = arrayFromNextLine(f)
		Qmax = arrayFromNextLine(f)

		f.close()

		robotPlant = RobotPlant(A, D, Alpha, Qmin, Qmax)
		aGenerator = AGenerator(Priors, Mu, Sigma, As, M)
		return MotionGenerator(robotPlant, aGenerator, bound_type)




	def get_next_motion(self, q, x_t, dt=None):
		qd = np.dot(self.AGenerator.get_A(q),self.robotPlant.q_basis(q, x_t))
		q_new = q.reshape(q.size, 1) + qd*dt

		if self.bound_type == None:
			pass
		elif self.bound_type == 'discrete':
			if dt == None:
				raise Exception('Must specify a timestep for discretely-bounded motions')
			for i in xrange(len(q)):
				if q_new[i] < self.qmin[i] or q_new[i] > self.qmax[i]:
					qd[i] = 0
		elif self.bound_type == 'continuous':
			K = (self.qmax - self.qmin)/2
			dqdv = np.multiply(K,(1 - np.power(np.divide(q_new - self.qmin, K) - 1, 2))).transpose()
			qd = np.dot(np.multiply(np.multiply(dqdv, self.AGenerator.get_A(q)), dqdv), qd)
			#qd = np.multiply(np.dot(K.transpose()*K,np.power(1 - np.power(np.divide(q_new - self.qmin, K) - 1, 2), 2)), qd)
			raise Exception('Continuous bounds are not yet fully functional... they don"t match MATLAB, who"s wrong? Probably MATLAB')
		return qd


class RobotPlant:
	def __init__(self, A, D, Alpha, Qmin, Qmax):
		n = len(A)
		links = []
		for i in xrange(n):
			link = Link(alpha=Alpha[i], A=A[i], D=D[i])
			link.qlim = [Qmin[i], Qmax[i]]
			links.append(link)
		self.robot = Robot(links)

	def q_basis(self, q, x_e):
		# As of now only works for 3d x_t
		end_effector_transform = fkine(self.robot, q)
		x_error = np.subtract(end_effector_transform[0:3,3].reshape((3,1)), x_e.reshape(3,1))
		return -1*np.dot(jacob0(self.robot, q)[0:3, :].transpose(), x_error)


class AGenerator:
	def __init__(self, Priors, Mu, Sigma, As, M):
		# Priors is a k-sized of priors
		# Mu is a k-sized list of n-d column matrices of means
		# Sigma is a k-sized list of nxn-d covariance matrices
		# As is a k-sized list of dimz x dimz-d augmentation matrices
		self.behaviorRegions = []
		self.M = M

		for i in xrange(len(Priors)):
			self.behaviorRegions.append(BehaviorRegion(Priors[i], Mu[:,i], Sigma[:,:,i], As[:,:,i]))

	def get_A(self, q):
		A_cumulative = 0
		pdf_cumulative = 0
		z = np.dot(self.M,q)
		for i in xrange(len(self.behaviorRegions)):
			A_cumulative += self.behaviorRegions[i].get_A(z)
			pdf_cumulative += self.behaviorRegions[i].get_pdf(z)
		return A_cumulative/pdf_cumulative

class BehaviorRegion:
	def __init__(self, prior, mu, sigma, A):
		self.prior = prior
		self.mu = mu
		self.sigma = sigma
		self.A = A

	def get_pdf(self, z):
		return self.prior*multivariate_normal.pdf(z, self.mu, self.sigma)

	def get_A(self, z):
		return self.A*self.get_pdf(z)

mg = MotionGenerator.CreateMotionGeneratorFromFile('/home/yonadav/Research/joint-pbd/kuka_demos/results/backhand_params.txt', bound_type = 'discrete')
q_initial = np.asarray([-0.728371429443359,-1.3605418920517,2.69252680319093,0.620675325393677,0.955035626888275,0.141930669546127,-0.17979271709919]).transpose()
x_end = np.asarray([-.5, -.5, 0.3])
print mg.get_next_motion(q_initial, x_end, .1)