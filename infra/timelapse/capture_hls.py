import threading
import time
from urllib.request import urlopen
from urllib.error import URLError, HTTPError

def parseurl(url: str, outfile: str, already_parsed: list):
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
    print(line, "DONE")
  return parsed

def record(url, outfile):
    print("Listening on", url," --> ", outfile)
    last_parsed = set()
    t = threading.current_thread()
    while t.alive:
      last_parsed = parseurl(url, outfile, last_parsed)
      time.sleep(5.0)


if __name__ == "__main__":
  threads = [
    threading.Thread(target=record, args=['http://l2:8080/playlist.m3u8', 'l2_out.ts']),
    threading.Thread(target=record, args=['http://jetson1:8080/playlist.m3u8', 'jetson1_out.ts']),
    threading.Thread(target=record, args=['http://jetson2:8080/playlist.m3u8', 'jetson2_out.ts']),
  ]
  print("Starting threads...")
  for t in threads:
    t.alive = True
    t.daemon = True
    t.start()

  try:
    input()
  except KeyboardInterrupt:
    pass
  print("Stopping threads...")
  for t in threads:
    t.alive = False
  for t in threads:
    t.join()
  print("Done")
