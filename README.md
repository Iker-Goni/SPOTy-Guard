# SPOTy-Guard
useful blog post https://aiven.io/developer/find-faces-with-pgvector

How to run: 
  1.) Download python files.
  2.) Set up PostgreSQL Vector database, tutorial here: https://www.postgresql.org/docs/current/tutorial-install.html
  3.) Install pgvector on database, code and tutorial here: https://github.com/pgvector/pgvector
  4.) Use pip to install required python packages: OpenCV, Imgbeddings, Numpy, and Psycopg 'pip install opencv-python imgbeddings psycopg2-binary numpy'
  5.) Identification/Registration: Place a reference image into the folder with the python scripts, and run identify-faces.py. This script will identify all of the faces in the image, and extract them to their own files. You will be able to open these files, they'll be a black and white close up of the face. Be sure to change the file_name variable in the script to point at the right file.
  6.) Move extracted faces into a directory named "stored-faces".
  7.) Run saving-faces.py. This script will vectorize all of the faces in stored-faces and save them to the database.
  8.) Take the image with the unknown faces you want to identify, and place it in the directory. Update the file_name variable of identify-faces.py to point at this image.
  9.) Run identify-faces.py again, this time making sure it is pointing at the unidentified image.
  10.) Update the file_name variable in recognize-faces.py to point at the image of the face created by identify-faces.py in step 9. The script will then print the closest matches in the database to the console.
