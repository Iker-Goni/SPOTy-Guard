import cv2
import sys
import os
import pathlib

cascade_file = "haarcascade_frontalface_default.xml"
haar_cascade = cv2.CascadeClassifier(cascade_file)

file_name = sys.argv[1] # take in image file name

# create directory for images if it doesn't exist
pathlib.Path('stored-faces').mkdir(parents=True, exist_ok=True) 


img = cv2.imread(file_name, 0)

gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

faces = haar_cascade.detectMultiScale(gray_img, 1.1, 4)

i = 0

for x, y, w, h in faces:

    cropped_image = img[y : y + h, x : x + w]
    target_file_name = 'stored-faces/' + file_name + '_face' + str(i) + '.jpg'
    cv2.imwrite(target_file_name, cropped_image)
    i += 1