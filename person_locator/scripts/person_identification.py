print('start of person identification file')
import tensorflow as tf
import numpy as np
import os
import scipy
from scipy.misc import imresize
from nets import inception
from tensorflow.contrib import slim

class_mapping = {0: 'John_Ashcroft',
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
restart_augmented_training_path = os.getcwd()+"/models/cnn/inception_v3_faces_restart_augmented.ckpt"
print('loading model params')
tf.reset_default_graph()
X = tf.placeholder(tf.float32, [None, 299, 299, 3], name='X')

is_training = tf.placeholder_with_default(False, [])

# Run inception function to determine endpoints
with slim.arg_scope(inception.inception_v3_arg_scope()):
    logits, end_points = inception.inception_v3(X, num_classes=1001, is_training=is_training)

# Create saver of network before alterations
#inception_saver = tf.train.Saver()

prelogits = tf.squeeze(end_points['PreLogits'], axis=[1,2])

# Define the training layer and the new output layer
n_outputs = len(class_mapping)
with tf.name_scope("new_output_layer"):
    people_logits = tf.layers.dense(prelogits, n_outputs, name="people_logits")
    probability = tf.nn.softmax(people_logits, name='probability')
y = tf.placeholder(tf.int32, None)

# Loss function and training operation
# The training operation is passed the variables to train which includes only the single layer
with tf.name_scope("train"):
    xentropy = tf.nn.sparse_softmax_cross_entropy_with_logits(logits=people_logits, labels=y)
    loss = tf.reduce_mean(xentropy)
    optimizer = tf.train.AdamOptimizer(learning_rate=0.01)
    # Single layer to be trained
    train_vars = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope="people_logits")
    # The variables to train are passed to the training operation
    training_op = optimizer.minimize(loss, var_list=train_vars)

# Accuracy for network evaluation
with tf.name_scope("eval"):
    correct = tf.nn.in_top_k(predictions=people_logits, targets=y, k=1)
    accuracy = tf.reduce_mean(tf.cast(correct, tf.float32))
with tf.name_scope("init_and_saver"):
    init = tf.global_variables_initializer()
    saver = tf.train.Saver()

print('done loading model params')

class Person_Classifier(object):
    # Function takes in an image array and returns the resized and normalized array
    def prepare_image(self, image, target_height=299, target_width=299):
        image = imresize(image, (target_width, target_height))
        return image.astype(np.float32) / 255

    def classify_image(self, image_array):
        #image_array = images[index]
        #label = class_mapping[labels[index]]

        prepared_image = self.prepare_image(image_array)
        prepared_image = np.reshape(prepared_image, newshape=(-1, 299, 299, 3))

        with tf.Session() as sess:
            saver.restore(sess, restart_augmented_training_path)
            predictions = sess.run(probability, {X: prepared_image})

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
        return class_mapping[best_prediction]
