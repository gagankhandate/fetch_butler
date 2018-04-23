print('start of person identification file')
import tensorflow as tf
import numpy as np
import os
import scipy
from scipy.misc import imresize
from nets import inception
from tensorflow.contrib import slim

import matplotlib.pyplot as plt

print('done loading model params')

class Person_Classifier(object):
    def __init__(self):
        self.tf = tf
        self.slim = slim
        self.inception = inception
        self.class_mapping = {0: 'John_Ashcroft',
         1: 'Jean_Chretien',
         2: 'Junichiro_Koizumi',
         3: 'Hugo_Chavez',
         4: 'Ariel_Sharon',
         5: 'Gerhard_Schroeder',
         6: 'Donald_Rumsfeld',
         7: 'Tony_Blair',
         8: 'Colin_Powell',
         9: 'George_W_Bush',
         10: 'Will_Pascucci',
         11: 'Gagan_Khandate'}

        # restart_augmented_training_path = "/home/gagan/humanoid_robot/butler_ws2/src/fetch_butler/person_locator/scripts/face_detector/models/cnn/inception_v3_faces_restart_augmented.ckpt"
        self.restart_augmented_training_path = os.getcwd()+"/src/fetch_butler/person_locator/scripts/models/cnn/inception_v3_faces_restart_augmented.ckpt"
        #self.restart_augmented_training_path = os.getcwd() + "/models/cnn/inception_v3_faces_restart_augmented.ckpt"
        print('loading model params')
        self.tf.reset_default_graph()
        self.X = self.tf.placeholder(tf.float32, [None, 299, 299, 3], name='X')

        self.is_training = self.tf.placeholder_with_default(False, [])

        # Run inception function to determine endpoints
        with self.slim.arg_scope(self.inception.inception_v3_arg_scope()):
            self.logits, self.end_points = self.inception.inception_v3(self.X, num_classes=1001, is_training=self.is_training)

        # Create saver of network before alterations
        #inception_saver = tf.train.Saver()

        self.prelogits = self.tf.squeeze(self.end_points['PreLogits'], axis=[1,2])

        # Define the training layer and the new output layer
        self.n_outputs = len(self.class_mapping)
        with self.tf.name_scope("new_output_layer"):
            self.people_logits = self.tf.layers.dense(self.prelogits, self.n_outputs, name="people_logits")
            self.probability = self.tf.nn.softmax(self.people_logits, name='probability')
        self.y = self.tf.placeholder(tf.int32, None)

        # Loss function and training operation
        # The training operation is passed the variables to train which includes only the single layer
        with self.tf.name_scope("train"):
            self.xentropy = self.tf.nn.sparse_softmax_cross_entropy_with_logits(logits=self.people_logits, labels=self.y)
            self.loss = self.tf.reduce_mean(self.xentropy)
            self.optimizer = self.tf.train.AdamOptimizer(learning_rate=0.01)
            # Single layer to be trained
            self.train_vars = self.tf.get_collection(self.tf.GraphKeys.TRAINABLE_VARIABLES, scope="people_logits")
            # The variables to train are passed to the training operation
            self.training_op = self.optimizer.minimize(self.loss, var_list=self.train_vars)

        # Accuracy for network evaluation
        with self.tf.name_scope("eval"):
            self.correct = self.tf.nn.in_top_k(predictions=self.people_logits, targets=self.y, k=1)
            self.accuracy = self.tf.reduce_mean(self.tf.cast(self.correct, self.tf.float32))
        with self.tf.name_scope("init_and_saver"):
            self.init = self.tf.global_variables_initializer()
            self.saver = self.tf.train.Saver()
        '''
        print('init image')
	prepared_image = np.reshape(imresize(plt.imread('face_0.jpg'),(299,299)).astype(np.float32)/255, newshape=(-1,299,299,3))
        print('init run')
        with self.tf.Session() as sess:
            self.saver.restore(sess, self.restart_augmented_training_path)
            predictions = sess.run(self.probability, {self.X: prepared_image})
        print('init predictions ' + str(predictions))
        #self.saver = saver
        #self.class_mapping = class_mapping
        #self.probability = probability
        #self.X = X
        '''
        self.sess = self.tf.Session()
        self.saver.restore(self.sess, self.restart_augmented_training_path)

    # Function takes in an image array and returns the resized and normalized array
    def prepare_image(self, image, target_height=299, target_width=299):
        image = imresize(image, (target_width, target_height))
        return image.astype(np.float32) / 255

    def classify_image(self, image_array):
	print('classify image called')
        #image_array = images[index]
        #label = class_mapping[labels[index]]

        prepared_image = self.prepare_image(image_array)
        prepared_image = np.reshape(prepared_image, newshape=(-1, 299, 299, 3))

        #with self.tf.Session() as sess:
        #self.saver.restore(self.sess, self.restart_augmented_training_path)
        predictions = self.sess.run(self.probability, {self.X: prepared_image})

        predictions = [(i, prediction) for i, prediction in enumerate(predictions[0])]
        predictions = sorted(predictions, key=lambda x: x[1], reverse=True)
        print(predictions)
        #print('\nCorrect Answer: {}'.format(label))
        '''
        print('\nPredictions:')
        for prediction in predictions:
            class_label = prediction[0]
            probability_value = prediction[1]
            #print(class_label)
            #print(class_mapping[class_label])
            #label = class_mapping[class_label]
            #print("{:26}: {:.2f}%".format(label, probability_value * 100))
        '''

        best_prediction = predictions[0][0]
        return self.class_mapping[best_prediction], predictions[0][1]
