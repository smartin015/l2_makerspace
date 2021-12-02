import threading
import time
from urllib.request import urlopen
from urllib.error import URLError, HTTPError
import paho.mqtt.client as mqtt

def parseurl(url: str, outfile: str, already_parsed: list, client):
  try:
    response = [r.strip() for r in urlopen(url).read().decode("utf-8").split("\n")]
  except URLError: # No response
    return already_parsed
  except HTTPError:
    return already_parsed
  parsed = [r for r in response if not r.startswith('#') and r != '']
  for line in parsed:
    if line in already_parsed:
      continue
    with open(outfile, 'ab') as f:
      try:
        f.write(urlopen(line).read())
      except HTTPError:
        continue
    client.publish("/av/status/capture", line)
    print(line, "DONE")
  return parsed

def record(url, outfile, client):
    print("Listening on", url," --> ", outfile)
    last_parsed = set()
    t = threading.current_thread()
    while t.alive:
      last_parsed = parseurl(url, outfile, last_parsed, client)
      time.sleep(5.0)

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT with result code "+str(rc))

if __name__ == "__main__":
  print("Starting telemetry...")
  client = mqtt.Client()
  client.on_connect = on_connect
  client.connect("mqtt", 1883, 60)

  threads = [
    threading.Thread(target=record, args=['http://l2:8080/playlist.m3u8', 'l2_out.ts', client]),
    threading.Thread(target=record, args=['http://jetson1:8080/playlist.m3u8', 'jetson1_out.ts', client]),
    threading.Thread(target=record, args=['http://jetson2:8080/playlist.m3u8', 'jetson2_out.ts', client]),
  ]
  print("Starting threads...")
  for t in threads:
    t.alive = True
    t.daemon = True
    t.start()

  print("Running(Ctrl+C to stop)")
  try:
    client.loop_forever()
  except KeyboardInterrupt:
    pass
  print("Stopping threads...")
  for t in threads:
    t.alive = False
  for t in threads:
    t.join()
  print("Done")
