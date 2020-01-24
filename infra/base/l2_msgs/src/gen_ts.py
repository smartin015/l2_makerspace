# Generate typescript from particular flavors of ros message
import os
import sys
import re

ATTR_MAP = {
  "string": "string",
  "int32": "number",
  "float32": "number",
}

INTERFACE_FMT = """
interface %s {
%s
}"""

def gen_ts(path):
    iname = os.path.splitext(os.path.basename(path))[0]
    attribs = {}
    data = ""
    with open(path, 'r') as f:
        data = f.read()

    for line in data.split('\n'):
        if line.strip().startswith('#'):
            continue
        m = re.match(r"(\w+)\s+(\w+)", line)
        if m is None:
            continue
        print(m.groups())
        attribs[m.groups()[1]] = ATTR_MAP[m.groups()[0]]
    print(INTERFACE_FMT % (iname, "\n".join(map(lambda k: "  %s: %s;" % (k, attribs[k]), attribs.keys()))))

if __name__ == "__main__":
    gen_ts(sys.argv[1])

