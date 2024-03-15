import cv2
import sys
import os
import pathlib

class FaceRecognizer():
    
    def __init__(self):
        self.face_cascade_file = '/content/haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(self.face_cascade_file)
        # create directory for face images if it doesn't exist
        pathlib.Path('/content/stored-faces').mkdir(parents=True, exist_ok=True)
    
    def IdentifyFaces(self, file_name):
        print("Finding faces in " + file_name)
        img = cv2.imread(file_name, 0)
        gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        faces = self.face_cascade.detectMultiScale(gray_img, 1.1, 4)
        i = 0
        for x,y,w,h in faces:
            
            i += 1
            cropped_image = img[y : y + h, x: x + w]
            target_file_name = '/content/stored-faces' + file_name +'_face' + str(i) + '.png'
            cv2.imwrite(target_file_name, cropped_image)

        if i == 0:
            print("didn't find any faces in image " + file_name)
        else:
            print("found " + str(i) + ' faces in the image.')