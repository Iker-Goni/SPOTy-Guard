import cv2
import sys
import os
import pathlib
import psycopg2
import numpy as np

from imgbeddings import imgbeddings
from PIL import Image


class FaceRecognizer():
    
    def __init__(self):
        self.face_cascade_file = 'haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(self.face_cascade_file)
        self.db = psycopg2.connect("host='localhost' dbname='testdb' user='nick' password='qwerty'")
        # create directory for face images if it doesn't exist
        pathlib.Path('stored-faces').mkdir(parents=True, exist_ok=True)
    
    #TODO dump faces to a folder. for some reason i was getting a write permission error.
    # this should dump faces to a "new faces" folder
    def IdentifyFaces(self, file_name):
        print("Finding faces in " + file_name)
        img = cv2.imread(file_name, 0)
        gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        faces = self.face_cascade.detectMultiScale(gray_img, 1.1, 4)
        i = 0
        for x,y,w,h in faces:
            
            i += 1
            cropped_image = img[y : y + h, x: x + w]
            target_file_name = file_name +'_face' + str(i) + '.png'
            cv2.imwrite(target_file_name, cropped_image)
            print('writing ' + target_file_name)

        if i == 0:
            print("didn't find any faces in image " + file_name)
        else:
            print("found " + str(i) + ' faces in the image.')

    def SaveFaces():
        print("Saving all faces in stored-faces to database...")
        for filename in os.listdir("stored-faces"):
            img  = Image.open("stored-faces/" + filename)
            embedding = FaceRecognizer._GenerateEmbedding(img)
            FaceRecognizer._WriteToDatabase(filename, embedding)
           

    def SaveNewFaces():
        print("Saving all faces in new-faces to database...")
        for filename in os.listdir("new-faces"):
            img  = Image.open("new-faces/" + filename)
            embedding = FaceRecognizer._GenerateEmbedding(img)
            FaceRecognizer._WriteToDatabase(filename, embedding)
            
    
    def _WriteToDatabase(self, filename, data):
        cursor = self.db.cursor()
        cursor.execute("INSERT INTO pictures values (%s,%s)", (filename, data[0].tolist()))

    def RecognizeFaces(self, embedding):
        cursor = self.db.cursor()
        string_rep = "[" + ",".join(str(x) for x in embedding[0].tolist()) + "]"
        cursor.execute("SELECT * FROM pictures ORDER BY embedding <-> %s LIMIT 5", (string_rep))
        rows = cursor.fetchall()
        for row in rows:
            print(row)
    def _GenerateEmbedding(self, image):
        ibed = imgbeddings()
        return ibed.to_embeddings(image)
