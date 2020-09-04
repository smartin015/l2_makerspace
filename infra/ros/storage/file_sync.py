import os
from watchdog.events import FileSystemEventHandler

class WatchHandler(FileSystemEventHandler):
    EXT_LIST = [".wbt", ".sdf"]

    def __init__(self, handler, dirpath):
        super().__init__()
        self.handler = handler
        # Push all files on init
        for dirName, subdirList, fileList in os.walk(dirpath):
            for fname in fileList:
                self._update(os.path.join(dirName, fname))

    def on_moved(self, event):
        self._update(event.dest_path)

    def on_created(self, event):
        if event.is_directory:
            return
        self._update(event.src_path)

    def on_modified(self, event):
        if event.is_directory:
            return
        self._update(event.src_path)

    def _update(self,path):
        (name, ext) = os.path.splitext(os.path.basename(path))
        if ext not in self.EXT_LIST:
            return
        with open(path, 'r') as f:
            data = f.read()
        self.handler(name, ext, data)

