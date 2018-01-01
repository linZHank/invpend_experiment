import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
%matplotlib inline

tf.reset_default_graph()
inputs1 = tf.placeholder(shape=[1,16],dtype=tf.float32)
W = tf.Variable(tf.random_uniform([16,4,0,0,0.01]))

