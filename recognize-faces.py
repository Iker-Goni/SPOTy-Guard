# https://aiven.io/developer/find-faces-with-pgvector
import numpy as np
from imgbeddings import imgbeddings
from PIL import Image
import psycopg2
import sys
import os

db = psycopg2.connect("host='localhost' dbname='testdb' user='nick' password='qwerty'")

file_name = sys.argv[1] #take in file name to recognize

img = Image.open(file_name)

ibed = imgbeddings()
embedding = ibed.to_embeddings(img)

cur = db.cursor()
string_rep = "[" + ",".join(str(x) for x in embedding[0].tolist()) + "]"
cur.execute("SELECT * FROM pictures ORDER BY embedding <-> %s LIMIT 5", (string_rep,))
rows = cur.fetchall()
for row in rows:
    print(row)