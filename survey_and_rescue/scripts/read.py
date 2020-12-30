import numpy as np
import pickle
dbfile = open('contourPickle.pickle', 'rb')      
db = pickle.load(dbfile) 
print(db)
print('no of contours',len(db))
dbfile.close() 
