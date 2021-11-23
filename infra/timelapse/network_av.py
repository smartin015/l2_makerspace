import gi 
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject
# Program for triggering network recording over MQTT - support for webcam, USB microphone, realsense D435 depth camera
import sys
#import subprocess, signal, threading, sys
from socket import gethostname
import paho.mqtt.client as mqtt

CMD_TOPIC = "/av/command"
STAT_TOPIC = "/av/status/" + gethostname()

Gst.init(sys.argv)

SERIAL=819112070701
pipeline_spec = (f"realsensesrc serial={SERIAL} timestamp-mode=clock_all enable-color=true "
    "! rgbddemux name=demux demux.src_color ! queue ! videoconvert ! omxh264enc ! video/x-h264, stream-format=(string)byte-stream "
    "! h264parse ! mpegtsmux ! hlssink playlist-root=http://192.168.1.8:8080 location=/tmp/ramdisk/segment_%05d.ts target-duration=5 max-files=5")
print("parsing pipeline")
pipeline = Gst.Pipeline() # parse_launch("audiotestsrc num-buffers=50 ! autoaudiosink") 
src = Gst.ElementFactory.make("realsensesrc") 
if src is None:
    raise Exception("Failed to create realsensesrc")
# src.set_property("serial", SERIAL)
#src.set_property("timestamp-mode", "clock_all")
#src.set_property("enable-color", True)
pipeline.add(src)
sink = Gst.ElementFactory.make("autovideosink")
pipeline.add(sink)
src.link(sink)
print("pipeline parsed")

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(CMD_TOPIC)

def handle_process(proc, client):
    proc.wait()
    result = proc.stdout.read()
    client.publish(STAT_TOPIC, result)
    print(result)
    print("Process terminated with code", proc.returncode)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global proc
    print(msg.topic+" "+str(msg.payload))
    msg.payload = msg.payload.decode("utf8")
    if msg.payload == "start_recording":
        if proc is not None and proc.returncode is None:
            print("Already recording")
            client.publish(STAT_TOPIC, "already recording")
            return
        print("Start recording")
        client.publish(STAT_TOPIC, "starting")
        realsense_pipeline.set_state(Gst.State.PLAYING)
        #proc = subprocess.Popen(REALSENSE_CMD, shell=True, cwd="/tmp", stdout=subprocess.PIPE)
        #threading.Thread(target=handle_process, args=[proc, client], daemon=True).start()
    elif msg.payload == "stop_recording":
        client.publish(STAT_TOPIC, "stopping")
        #if proc is not None and proc.returncode is None:
        #    print("Sending SIGINT to", proc)
        #    proc.send_signal(signal.SIGINT)
        realsense_pipeline.set_state(Gst.State.NULL)

    else:
        print("Unknown command")



def main():  
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("mqtt", 1883, 60) 
    client.loop_forever()

if __name__ == "__main__":
    main()
