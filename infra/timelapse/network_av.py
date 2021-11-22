# Program for triggering network recording over MQTT - support for webcam, USB microphone, realsense D435 depth camera
import subprocess, signal, threading
from socket import gethostname
import paho.mqtt.client as mqtt

CMD_TOPIC = "/av/command"
STAT_TOPIC = "/av/status/" + gethostname()

REALSENSE_CMD = "sleep 10 && ls -al /tmp"


proc = None

# The callback for when the client receives a CONNACK response from the server.
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
        proc = subprocess.Popen(REALSENSE_CMD, shell=True, cwd="/tmp", stdout=subprocess.PIPE)
        threading.Thread(target=handle_process, args=[proc, client], daemon=True).start()
    elif msg.payload == "stop_recording":
        print("TODO stop recording")
        client.publish(STAT_TOPIC, "stopping")
        if proc is not None and proc.returncode is None:
            print("Sending SIGTERM to", proc)
            proc.terminate()

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
