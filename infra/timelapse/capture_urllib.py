from datetime import datetime, timedelta
def record(filepath, stream, duration):
    fd = open(filepath, 'wb')
    begin = datetime.now()
    duration = timedelta(milliseconds=duration)
    while datetime.now() - begin < duration:
        data = stream.read(10000)
        fd.write(data)
    fd.close()

from urllib.request import urlopen
record('clip.mp3', urlopen('http://192.168.1.8:8080/playlist.m3u8'), 10000)
