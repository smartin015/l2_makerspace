import psycopg2
import os

import urlparse # for python 3+ use: from urllib.parse import urlparse
result = urlparse.urlparse(os.environ["PGRST_DB_URI"])
username = result.username
password = result.password
database = result.path[1:]
hostname = result.hostname

con = psycopg2.connect(
        database=database, 
        user=username, 
        password=password, 
        host=hostname)

Print("Database opened successfully")
