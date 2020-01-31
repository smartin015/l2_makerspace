FREECADPATH = '/opt/FreeCAD/lib' # path to your FreeCAD.so or FreeCAD.dll file
import sys
sys.path.append(FREECADPATH)
import FreeCAD
import Part

def import_fcstd(filename):
  doc = FreeCAD.open(filename)
  objects = doc.Objects
  for ob in objects:
    print(ob.Type)

def main():
  import_fcstd("testfile.fcstd")

if __name__=='__main__':
   main()
