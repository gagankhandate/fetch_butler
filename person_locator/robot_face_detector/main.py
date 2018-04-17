import sys
import dlib
from skimage import io
from PIL import Image
from person_identification import classify_image
import numpy as np
import matplotlib.pyplot as plt

face_detector = dlib.get_frontal_face_detector()

def get_photo():
	faces_found = False
	while(not faces_found):
		file_name = 'test_imgs/Gagan_Khandate_330.jpg'
		image = io.imread(file_name)
		detected_faces = face_detector(image, 1)
		faces_found = len(detected_faces) >= 1

	return detected_faces

def run():
	detected_faces = get_photo()
	faces = []
	for i, face_rect in enumerate(detected_faces):
		new_img = main_img.crop((face_rect.left(), face_rect.top(), face_rect.right(), face_rect.bottom()))
		bounding_box = (face_rect.left(), face_rect.top(), face_rect.right(), face_rect.bottom())
		new_img = new_img.resize((250, 250))
		new_img.save("face_" + str(i) + ".jpg")
		faces.append(("face_" + str(i) + ".jpg", new_img, np.array(new_img), bounding_box))

	result = []

	for face in faces:
		img = plt.imread(face[0])
		print(classify_image(img))
		result.append((classify_image(img), face[3]))

	return result
