# Title

## Objective

Run a lightweight 3d game environment that allows build and upload to
an Oculus Quest.

## Example Steps

Before doing this example, you'll need to build the docker image by running the following command in this directory:

```
docker build --tag l2vr:latest .
```

1. Connect the oculus quest headset to the PC. Make sure USB debugging on the host is enabled (via the headset).
1. Run `./launch.sh` to launch the editor (into test_vr project).
1. Click on the Android icon on the top right of the project window - it will build and run the app on the quest.
