FREECADPATH = '/opt/FreeCAD/lib' # path to your FreeCAD.so or FreeCAD.dll file
import sys
sys.path.append(FREECADPATH)
 
def import_fcstd(filename):
  try:
    import FreeCAD
  except ValueError:
    print('FreeCAD library not found. Please check the FREECADPATH variable in the import script is correct')
  else:
    import Part
    doc = FreeCAD.open(filename)
    objects = doc.Objects
    for ob in objects:
      if ob.Type[:4] == 'Part':
        shape = ob.Shape
          if shape.Faces:
            pass
            #mesh = Blender.Mesh.New()
            #rawdata = shape.tessellate(1)
            #for v in rawdata[0]:
            #    mesh.verts.append((v.x,v.y,v.z))
            #for f in rawdata[1]:
            #   mesh.faces.append.append(f)
            #scene.objects.new(mesh,ob.Name)

def main():
  import_fcstd("testfile.fcstd")

if __name__=='__main__':
   main()
