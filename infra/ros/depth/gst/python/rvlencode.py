"""
    export GST_PLUGIN_PATH=$GST_PLUGIN_PATH:$PWD
    gst-launch-1.0 rvlencode ! autovideosink
"""

import logging
import timeit
import traceback
import time

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject, GLib  # noqa:F401,F402


class RVLEncode(Gst.Element):

    GST_PLUGIN_NAME = 'rvlencode'

    __gstmetadata__ = ("RVLEncode",
                       "Source",
                       "Encode 16-bit depth image to RVL",
                       "Scott Martin <smartin015 at gmail dot com>")

    __gsttemplates__ = Gst.PadTemplate.new("src",
                                            Gst.PadDirection.SRC,
                                            Gst.PadPresence.ALWAYS,
                                            Gst.Caps.new_any())

    _srcpadtemplate = __gsttemplates__
    __gproperties__ = {}

    def __init__(self):
        super(RVLEncode, self).__init__()
        self.srcpad = Gst.Pad.new_from_template(self._srcpadtemplate, 'src')
        self.srcpad.set_event_function_full(self.srceventfunc, None)
        self.srcpad.set_query_function_full(self.srcqueryfunc, None)
        self.add_pad(self.srcpad)

    def do_get_property(self, prop: GObject.GParamSpec):
        raise AttributeError('unknown property %s' % prop.name)

    def do_set_property(self, prop: GObject.GParamSpec, value):
        raise AttributeError('unknown property %s' % prop.name)

    def srcqueryfunc(self, pad: Gst.Pad, parent, query: Gst.Query) -> bool:
        print(query)
        return True
    
    def srceventfunc(self, pad: Gst.Pad, parent, event: Gst.Event) -> bool:
        print(event)
        return True

# Register plugin to use it from command line
GObject.type_register(RVLEncode)
__gstelementfactory__ = (RVLEncode.GST_PLUGIN_NAME,
                         Gst.Rank.NONE, RVLEncode)
