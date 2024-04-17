import cv2
import sys
import os
import pathlib
import psycopg2
import numpy as np
import shutil

from imgbeddings import imgbeddings
from PIL import Image


class FaceRecognizer():
    
    def __init__(self, fuzziness=12.0):
        """
        Create a new FaceRecognizer instance. Fuzziness sets how close a given face must be to another to be considered a match, default is 15.0.
        """

        self.face_cascade_file = 'haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(self.face_cascade_file)
        self.db = psycopg2.connect("host='localhost' dbname='testdb' user='nick' password='qwerty'") #todo: real db info lmfaooo
        self.fuzziness = fuzziness

        # directory names.
        self.storedFacesDir = 'stored-faces'
        self.newFacesDir = 'new-faces'
        self.strangerFacesDir = 'strangers-faces'
        self.knownFacesDir = 'known-faces'

        # create directories for face images if they don't exist
        pathlib.Path(self.storedFacesDir).mkdir(parents=True, exist_ok=True) # will hold copies of the known faces in our database
        pathlib.Path(self.newFacesDir).mkdir(parents=True, exist_ok=True) # will hold faces that have yet to be categorized
        pathlib.Path(self.strangerFacesDir).mkdir(parents=True, exist_ok=True) # will hold faces of strangers we encounter while doing our thing
        pathlib.Path(self.knownFacesDir).mkdir(parents=True, exist_ok=True) # will hold faces of known people we encounter while doing our thing.

    

    def IdentifyFaces(self, file_name):
        "Identifies faces in an image, cuts them out, and saves them to the new faces directory."
        print("Finding faces in " + file_name)
        img = cv2.imread(file_name, 0)
        gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        faces = self.face_cascade.detectMultiScale(gray_img, 1.1, 4)
        i = 0
        for x,y,w,h in faces:
            
            i += 1
            cropped_image = img[y : y + h, x: x + w]
            target_file_name = self.newFacesDir + "/" + file_name +'_face' + str(i) + '.png'
            cv2.imwrite(target_file_name, cropped_image)
            print('writing ' + target_file_name)

        if i == 0:
            print("didn't find any faces in image " + file_name)
        else:
            print("found " + str(i) + ' faces in the image.')

    def SaveFaces(self):
        "Inserts all the images in the stored-faces directory to the database. Used for building db."
        print("Saving all faces in " + self.storedFacesDir + " to database...")
        for filename in os.listdir(self.storedFacesDir):
            img  = Image.open(self.storedFacesDir + "/" + filename)
            embedding = FaceRecognizer._GenerateEmbedding(self, img)
            FaceRecognizer._WriteToDatabase(self, filename, embedding)
    

    def SaveNewFaces(self):
        "Saves all the images in the new faces directory to the database, then moves them to the stored faces directory."
        print("Saving all faces in " + self.newFacesDir + " to database...")
        for filename in os.listdir(self.newFacesDir):
            img  = Image.open(self.newFacesDir + "/" + filename)
            embedding = FaceRecognizer._GenerateEmbedding(self, img)
            FaceRecognizer._WriteToDatabase(self, filename, embedding)
        move_files(self.newFacesDir, self.storedFacesDir)
    
    def ContainsFaces(self, image_file_name):
        "Boolean check to see if an image contains faces. Does not write anything to the disk."
        
        # Identify any faces in the image. 
        img = cv2.imread(image_file_name, 0)
        gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        faces = self.face_cascade.detectMultiScale(gray_img, 1.1, 4)
        
        # Count the faces in the image. There is definitely a better way to do this.
        i = 0
        for x,y,w,h in faces:
            i += 1
            break
        
        # Return yes/no if we found any faces.
        return i > 0
    
    def SetFuzziness(self, value):
        """Sets the fuziness of the recognizer to the given value. The default value is 15.0"""
        self.fuzziness = value

    def RecognizeFaces(self):
        """
        Compares the faces in 'new-faces' to the ones already in the database.
        Returns True if any of the faces in 'new-faces' match one in the database, and false otherwise.
        """
        known_encountered = False # flag that will be tripped if we find a face we recognize.
        for face in os.listdir(self.newFacesDir):
            img = Image.open(self.newFacesDir + "/" + face)
            embedding = FaceRecognizer._GenerateEmbedding(self, img)
            maybe_face = FaceRecognizer._RecognizeFace(self, embedding)
            for match in maybe_face:
                print(match)
                known_encountered = True
                # Move the recognized face to a different folder
                source_path = os.path.join(self.newFacesDir, face)
                dest_path = os.path.join(self.knownFacesDir, face)
                shutil.move(source_path, dest_path)
                break

        move_files(self.newFacesDir, self.strangerFacesDir)
        return known_encountered

    def _RecognizeFace(self, embedding):
        "Helper function to search the database for data close to a given embedding"
        cursor = self.db.cursor()
        string_rep = "[" + ",".join(str(x) for x in embedding[0].tolist()) + "]"
        cursor.execute("SELECT * FROM pictures WHERE embedding <-> %s <= %s ORDER BY embedding <-> %s LIMIT 5", (string_rep, self.fuzziness, string_rep))
        rows = cursor.fetchall()
        return rows

    def _GenerateEmbedding(self, image):
        "Generates embeddings for an image."
        ibed = imgbeddings()
        return ibed.to_embeddings(image)
    
        
    def _WriteToDatabase(self, filename, data):
        "Helper function that just inserts data into the database."
        cursor = self.db.cursor()
        cursor.execute("INSERT INTO pictures values (%s,%s)", (filename, data[0].tolist()))
        self.db.commit()
    
    def _printDB(self):
        "Prints out every entry in the database to the console."
        cursor = self.db.cursor()
        cursor.copy_to(sys.stdout, 'pictures', sep='\t')

def move_files(src_folder, dest_folder):
    # Get the list of files in the source folder
    files = os.listdir(src_folder)
    
    # Iterate over each file and move it to the destination folder
    for file in files:
        # Construct the full path of the source file
        src_file = os.path.join(src_folder, file)
        # Construct the full path of the destination file
        dest_file = os.path.join(dest_folder, file)
        # Move the file to the destination folder
        shutil.move(src_file, dest_file)