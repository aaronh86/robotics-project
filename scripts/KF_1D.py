#!/usr/bin/python
# -*- coding: utf-8 -*-

#Simulation for 1D-Kalman Filter
#Include a Simple Gaussian Calculator


#import rospy
import numpy as np
from matplotlib import pyplot as plt
import math


def Gaussian (mu, sigma2, x):

    '''
	A simple Guassian Calculator
	Input: mean, variance, value of variable
	Output: probability of the distribution
    '''

    return 1/math.sqrt(2.*math.pi*sigma2) * math.exp(-0.5*(x-mu)**2 / sigma2)


def update (mean1, var1, mean2, var2):

    ''''
	Update of Gaussian Distribution after Measurement, Posterior, Increase belief
	Input: mean and variance of two distribution
	Output: mean and variance of new distribution
    '''
    Kalman_Gain = var1*1/(var1+var2)
    posterior_mean = mean1+Kalman_Gain*(mean2-mean1)
    posterior_var = (1-Kalman_Gain)*var1
    #posterior_mean = 1/(var1+var2)*(var2*mean1+var1*mean2)
    #posterior_var = 1/((1/var1)+(1/var2))

    return [posterior_mean, posterior_var]

def predict (mean1, var1, mean2, var2):

    '''
	Prediction Distribution after movement, Prior, Loss information
	Input: mean and variance of two distribution
	Output: mean and variance of new distribution
    '''

    prior_mean = mean1 + mean2
    prior_var = var1 + var2

    return [prior_mean, prior_var]

if __name__ == "__main__":

    #For self-testing only

    #Gaussian Calculator

    #print Gaussian(10., 4., 10.)

    #Kalman Filter

    measurements = [5., 6., 7., 9., 10.]
    motion = [1., 1., 2., 1., 1.]
    measurement_sig = 4.
    motion_sig = 2.
    mu = 0.
    sig = 10000.

    for i in range(len(measurements)):
        [mu,sig] = update(mu,sig,measurements[i],measurement_sig)
        print('update:    ', [mu, sig])
        [mu,sig] = predict(mu,sig,motion[i],motion_sig)
        print('predict:   ', [mu, sig])
        
    print([mu,sig])
    
    xmin=mu-3*sig
    xmax=mu+3*sig
    x=np.arange(xmin,xmax,abs(xmax-xmin)/100)
    y1=np.empty_like(x)
    y2=np.empty_like(x)
    y3=np.empty_like(x)

    #for j in range(len(x)):
        #y1[j]=Gaussian (mu, sig, x[j])
	#y
    #print x
    #plt.plot(x,y1,'r')
    #plt.plot(x,y2,'g')
    #plt.plot(x,y3,'b')
    #plt.show()








