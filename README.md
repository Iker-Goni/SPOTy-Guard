# SPOTy-Guard

# Vector Database setup stuffs (I'm using Ubuntu 22.04. Other linux versions should work, idk about Windows)

1.) Install PostgreSQL Server, I'm using PostgreSQL 16.2, but any version >= 12 should work. See here for download and install info: https://www.postgresql.org/download/

2.) You'll also need the PostgreSQL server dev tools for your postgres version. You can find them at the same link as above. 

3.) Install the pgvector extension. This allows us to use vector similarity search in postgres, which is how we match faces. You'll have to compile it yourself, but all the files and the installation tutorial are here: https://github.com/pgvector/pgvector

4.) Open a terminal, and run the ```shell psql``` command. It may complain that the database doesn't exist for your user. If that occurs, run ```shell 'sudo -i -u postgres```', then run ```shell psql```. If successful, your prompt will change to a ```shell =#'```, such as ```shell postgres=#```.

5.) Assuming you installed pgvector correctly in step 3, you should now be able to enable the pgvector extension. Run the command ```CREATE EXTENSION vector;```

6.) Create the database with ```CREATE DATABASE (database name);```

7.) Make the pictures table with ```CREATE TABLE pictures (picture text PRIMARY KEY, enbedding vector(768));```

8.) Modify the FaceRecognizer.__init__() method in facerecog.py to have the correct database information.

9.) Profit.
