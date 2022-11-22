
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import sys
import ezdxf
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

#Obtain filename & path from user input from File Explorer GUI
Tk().withdraw() #keep away the root window
filename = askopenfilename()
# filename = r"D:\Documents tmp\YES\dxfExtract\3D_20221026_095430_SampleDXF.dxf"
print(filename)
if filename.endswith('.dxf'):
    print("File type correct")
else:
    print("File type incorrect")
    print("The program exits")
    sys.exit()


#Loading DXF file
print("Checking .dxf file...")
try:
    doc = ezdxf.readfile(filename)
except IOError:
    print(f"Not a .dxf file or a generic I/O error.")
    sys.exit(1)
except ezdxf.DXFStructureError:
    print(f"Invalid or corrupted DXF file.")
    sys.exit(2)

print(".dxf file ok")

def midpoint3D(points): #points is a numpy array with Nx3 shape with [[x1, y1, z1], ..., [xN, yN, zN]]
    x = np.average(points[:, 0])
    y = np.average(points[:, 1])
    z = np.average(points[:, 2])
    
    return np.array([x, y, z])

def projectPlane(vector, normvector): #Project a vector to a normal vector's plane
    return (vector - ((np.dot(vector, normvector)/(np.linalg.norm(normvector)**2))*normvector))
    
#Extract all points
msp = doc.modelspace()
pointslist = np.array([list(e.dxf.location.xyz) for e in msp if e.dxftype() == "POINT"])
#pointslist = np.array[pointslist]
print(pointslist)

#Define unit vector of dxf axis
iMatrix = np.identity(3)
unitX = iMatrix[0]
unitY = iMatrix[1]
unitZ = iMatrix[2]

#Fit a plane between 4 points that are nearly co-plane
#return centroid and normal vector of plane
points = np.array([pointslist[2:, 0], pointslist[2:, 1], pointslist[2:, 2]])
print(points)
ctr = points.mean(axis=1)
x = points - ctr[:,np.newaxis]
M = np.dot(x, x.T) # Could also use np.cov(x) here.
norm = np.linalg.svd(M)[0][:,-1]
len_offset = 220/1000 #Distance from plane centroid to End Effector centroid 220mm
# print(ctr, norm)
ctr_off = ctr + len_offset*(norm)
print("End effector centroid: ", ctr_off)
ctr_vector = ctr_off - ctr #vector from plane centroid to end effector centroid
ctr_mid = midpoint3D(pointslist[2:4])-ctr
ctr_vectorZ = projectPlane(ctr_vector, unitZ)
ctr_vectorX = projectPlane(ctr_vector, (unitX)) #Sabrina's system's XY axis equals to negative XY axis in Leica's coordinate system
ctr_vectorY = projectPlane(ctr_mid, (unitY))
# print(ctr_vectorX, ctr_vectorY, ctr_vectorZ)
radZ = np.arctan2(ctr_vectorZ[1], ctr_vectorZ[0])+(np.pi/2)
radX = np.pi - np.arctan2(ctr_vectorX[2], ctr_vectorX[1])
radY = np.arctan2(ctr_vectorY[0], ctr_vectorY[2])*(-1)
# print("xyz deg:", np.rad2deg(radX), np.rad2deg(radY), np.rad2deg(radZ))
ctr_off_treat = np.copy(ctr_off)
ctr_off_treat[0] = ctr_off_treat[0]*(-1)
ctr_off_treat[1] = ctr_off_treat[1]*(-1)
lastpos = np.append(ctr_off_treat, [radX, radY, radZ])
print("lastpos_cal:", lastpos)


#Sabrina's measurement based on the same dxf file
ref_val = [7.2535, 6.1871, -2.7236, 0.06265732, -0.02251475, -0.04380776]
# print("reference value:", ref_val[0:3], np.rad2deg(ref_val[3:]))
print("lastpos_ref:", ref_val)
#Orientation calculation



exit(0)