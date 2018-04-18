import sys
import dlib
from skimage import io
from PIL import Image
from person_identification import classify_image
import numpy as np
import matplotlib.pyplot as plt

face_detector = dlib.get_frontal_face_detector()
file_name = '../images/camera_img.jpg'
LOCATION_FILENAME = '../location.txt'

def get_photo():
	faces_found = False
	while(not faces_found):
		#file_name = 'test_imgs/Gagan_Khandate_330.jpg'
		image = io.imread(file_name)
		detected_faces = face_detector(image, 1)
		faces_found = len(detected_faces) >= 1

	return detected_faces, image

def run():
	main_image = Image.open(file_name)
	detected_faces, image = get_photo()
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
		print(classify_image(img))
		result.append((classify_image(img), face[3]))

	with open(LOCATION_FILENAME, 'w') as out_f:
		out_f.write(str(result[0][1]))

	return result

if __name__ == '__main__':
	run()
