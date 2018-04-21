import sys
import dlib
from skimage import io
from PIL import Image
from person_identification import Person_Classifier
import numpy as np
import matplotlib.pyplot as plt

dlib_face_detector = dlib.get_frontal_face_detector()
file_name = 'camera_img.jpg'
LOCATION_FILENAME = 'location.txt'

class face_detector(object):
    def __init__(self):
        self.classifier = Person_Classifier()

    def get_photo(self):
    	faces_found = False
    	image = io.imread(file_name)
    	i = 0
    	while(not faces_found and i < 5):
    		#file_name = 'test_imgs/Gagan_Khandate_330.jpg'
    		detected_faces = dlib_face_detector(image, 1)
    		faces_found = len(detected_faces) >= 1
    		i+=1
    		
	print(str(i) + ' faces found')
    	return detected_faces, image

    def run(self):
    	main_image = Image.open(file_name)
    	detected_faces, image = self.get_photo()
    	faces = []
    	for i, face_rect in enumerate(detected_faces):
    		new_img = main_image.crop((face_rect.left(), face_rect.top(), face_rect.right(), face_rect.bottom()))
    		bounding_box = (face_rect.left(), face_rect.top(), face_rect.right(), face_rect.bottom())
    		new_img = new_img.resize((250, 250))
    		new_img.save("face_" + str(i) + ".jpg")
    		faces.append(("face_" + str(i) + ".jpg", new_img, np.array(new_img), bounding_box))

    	result = []

    	for face in faces:
    		img = plt.imread(face[0])
    		print(self.classifier.classify_image(img))
    		result.append((self.classifier.classify_image(img), face[3]))

    	with open(LOCATION_FILENAME, 'w') as out_f:
    		out_f.write(str(result[0][1]))

    	return result[0][1]

#if __name__ == '__main__':
#	run()
