#include <gst/gst.h>
#include <iostream>

typedef struct _CustomData {
  GstElement *pipeline;
  GstElement *source;
  GstElement *sink;
} CustomData;

int main (int argc, char *argv[])
{
  CustomData data;
  GstElement *pipeline;
  GstBus *bus;
  GstMessage *msg;

  /* Initialize GStreamer */
  gst_init (&argc, &argv);

  /*
  data.source = gst_element_factory_make("realsensesrc", "source");
  if (data.source == NULL) {
    std::cout << "Null realsensesrc" << std::endl;
    return 1;
  }
  g_object_set(data.source, "serial", "819112070701", NULL);
  g_object_set(data.source, "timestamp-mode", 2, NULL);
  g_object_set(data.source, "enable-color", true, NULL);


  data.sink = gst_element_factory_make("autovideosink", "sink");  

  data.pipeline = gst_pipeline_new("realsense-stream"); */
  data.pipeline = gst_parse_launch(
                    "realsensesrc serial=819112070701 timestamp-mode=clock_all enable-color=true "
                    "! rgbddemux name=demux demux.src_color ! queue ! videoconvert ! omxh264enc ! video/x-h264, stream-format=(string)byte-stream "
                    "! h264parse ! mpegtsmux ! hlssink playlist-root=http://192.168.1.8:8080 location=/tmp/ramdisk/segment_%05d.ts target-duration=5 max-files=5",
      NULL);
  
  /*
  gst_bin_add_many(GST_BIN(data.pipeline), data.source, data.sink, NULL);
  std::cout << "Added pipeline element" << std::endl;
  if (!gst_element_link_many (data.source, data.sink, NULL)) {
    g_printerr ("Elements could not be linked.\n");
    gst_object_unref (data.pipeline);
    return -1;
  }
  */

  /* Start playing */
  gst_element_set_state (data.pipeline, GST_STATE_PLAYING);

  /* Wait until error or EOS */
  bus = gst_element_get_bus (data.pipeline);
  msg =
      gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
      (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

  /* See next tutorial for proper error message handling/parsing */
  if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
    g_error ("An error occurred! Re-run with the GST_DEBUG=*:WARN environment "
        "variable set for more details.");
  }

  /* Free resources */
  gst_message_unref (msg);
  gst_object_unref (bus);
  gst_element_set_state (data.pipeline, GST_STATE_NULL);
  gst_object_unref (data.pipeline);
  return 0;
}
