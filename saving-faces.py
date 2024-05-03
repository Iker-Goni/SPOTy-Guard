import numpy as np
from imgbeddings import imgbeddings
from PIL import Image
import psycopg2
import os

db = psycopg2.connect("host='localhost' dbname='facedb' user='postgres' password=''")

for filename in os.listdir("stored-faces"):
    img = Image.open("stored-faces/" + filename)

    ibed = imgbeddings()

    embedding = ibed.to_embeddings(img)

    cur = db.cursor()
    cur.execute("INSERT INTO pictures values (%s,%s)", (filename, embedding[0].tolist()))
    print(filename)
db.commit()
