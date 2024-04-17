import facerecog
import cv2
import numpy as np




# tests recognition code just using your laptop's webcam.

cam = cv2.VideoCapture(0)
recognizer = facerecog.FaceRecognizer()

print ("taking photo...")
_, img = cam.read()


target_file_name = str(np.random.randint(0, 10000)) + '.png'
print ("Saving photo as " + target_file_name + ".png")
cv2.imwrite(target_file_name, img)

if (recognizer.ContainsFaces(target_file_name)):
    recognizer.IdentifyFaces(target_file_name)
 #   recognizer.SaveNewFaces()
    if (recognizer.RecognizeFaces()):
        print("we recognize this person")
    else:
        print("don't think we've seen this person before.")
else:
    print("didn't find any faces")


cam.release()