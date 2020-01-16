# Title

## Purpose

Run a lightweight 3d game environment that allows build and upload to
an Oculus Quest. This environment simulates a virtual makerspace, but is also connected to a physical one.

## Example Steps

Before doing this example, you'll need to build the docker image by running the following command in this directory:

```
docker build --tag l2vr:latest .
```

1. Connect the oculus quest headset to the PC. Make sure USB debugging on the host is enabled (via the headset).
1. Run `./launch.sh` to launch the editor (into test_vr project).
1. Click on the Android icon on the top right of the project window - it will build and run the app on the quest.

**Troubleshooting**

You may additionally need to populate the version templates if this is your first time running (Export Project > Android, click banner at the bottom).

## Resources

https://gitlab.com/menip/godot-multiplayer-tutorials/tree/master
https://godotengine.org/article/godot-oculus-quest-support

## Future work

Try ImmediateGeometry based streaming of triangle meshes over UDP. Can use [this paper](https://www.researchgate.net/publication/27521282_3D_Mesh_Compression) as a start when researching compression techniques. This would be useful for streaming and displaying the results of generated/optimized models.

Create a mixed reality whiteboard that can be written on in real life and VR and updates in both.
