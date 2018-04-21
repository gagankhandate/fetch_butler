from fetch_face_detector import face_detector

my_face_detector = face_detector()

print('run 1')
bounding_box = my_face_detector.run()
print('bbox')
print(bounding_box)
print(type(bounding_box))
print(bounding_box[0])
print(type(bounding_box[0]))

print('run 2')
bounding_box = my_face_detector.run()
print('bbox')
print(bounding_box)
