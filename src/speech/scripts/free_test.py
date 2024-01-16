from environment_info import *
import fuzzywuzzy.process as p

a=p.extractOne("fridg",list(locations.keys())+list(objects.keys())+list(action.keys())+people+rooms)[0]
print(a[0])
