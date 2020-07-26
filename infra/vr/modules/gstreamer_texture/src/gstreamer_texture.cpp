#include "gstreamer_texture.h"
#include <gst/app/gstappsink.h>
#include <atomic>
#include <cstdlib> // for getenv

std::atomic<char*> atomicFrame;

using namespace godot;

void GStreamerTexture::_register_methods() {
  register_method("_process", &GStreamerTexture::_process);
}

GStreamerTexture::GStreamerTexture() {
}

GStreamerTexture::~GStreamerTexture() {
  delete im;
  gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(pipeline));
}

/**
* @brief Check preroll to get a new frame using callback
*  https://gstreamer.freedesktop.org/documentation/design/preroll.html
* @return GstFlowReturn
*/
GstFlowReturn new_preroll(GstAppSink* /*appsink*/, gpointer /*data*/)
{
    return GST_FLOW_OK;
}

/**
* @brief This is a callback that get a new frame when a preroll exist
*
* @param appsink
* @return GstFlowReturn
*/
GstFlowReturn new_sample(GstAppSink *appsink, gpointer /*data*/)
{
    static int framecount = 0;

    // Get caps and frame
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    const int width = g_value_get_int(gst_structure_get_value(structure, "width"));
    const int height = g_value_get_int(gst_structure_get_value(structure, "height"));

    // Print dot every 30 frames
    if(!(framecount%30)) {
      Godot::print(".");
    }

    // Show caps on first frame
    if(!framecount) {
      Godot::print(gst_caps_to_string(caps));
    }
    framecount++;

    // Get frame data and convert
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);
    char* prevFrame = atomicFrame.exchange((char*)map.data);
    if(prevFrame) {
        delete prevFrame;
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

/**
* @brief Bus callback
*  Print important messages
*
* @param bus
* @param message
* @param data
* @return gboolean
*/
static gboolean my_bus_callback(GstBus *bus, GstMessage *message, gpointer data)
{
    // Debug message
    //g_print("Got %s message\n", GST_MESSAGE_TYPE_NAME(message));
    switch(GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR: {
            GError *err;
            gchar *debug;

            gst_message_parse_error(message, &err, &debug);
            Godot::print(err->message);
            g_error_free(err);
            g_free(debug);
            break;
        }
        case GST_MESSAGE_EOS:
            /* end-of-stream */
            break;
        default:
            /* unhandled message */
            break;
    }
    /* we want to be notified again the next time there is a message
    * on the bus, so returning TRUE (FALSE means we want to stop watching
    * for messages on the bus and our callback should not be called again)
    */
    return true;
}

void GStreamerTexture::_init() {
  im = Image::_new();
  im->create(256, 256, false, Image::FORMAT_RGBA8);
  set_storage(ImageTexture::STORAGE_RAW);
  im->fill(Color(0, 1.0, 0));
  create_from_image(Ref(im));
  // flags = Texture::FLAG_VIDEO_SURFACE;
  Godot::print("gstreamer texture init");

  Godot::print(std::getenv("GST_PLUGIN_PATH"));
  gst_init(NULL, NULL);
  // https://stackoverflow.com/questions/10403588/adding-opencv-processing-to-gstreamer-application
	gchar *descr = g_strdup(
    /*
      "udpsrc port=5600 "
      "! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 "
      "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert "
    */
      "playbin uri=https://www.freedesktop.org/software/gstreamer-sdk/data/media/sintel_trailer-480p.webm "
      "! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
  );

  // Check pipeline
  error = nullptr;
  pipeline = gst_parse_launch(descr, &error);

	if(error) {
      g_print("could not construct pipeline: %s\n", error->message);
      g_error_free(error);
      exit(-1);
  }

	/**
    * @brief Get sink signals and check for a preroll
    *  If preroll exists, we do have a new frame
    */
  sink = gst_bin_get_by_name (GST_BIN (pipeline), "sink");
  gst_app_sink_set_emit_signals((GstAppSink*)sink, true);
  gst_app_sink_set_drop((GstAppSink*)sink, true);
  gst_app_sink_set_max_buffers((GstAppSink*)sink, 1);
  GstAppSinkCallbacks callbacks = { nullptr, new_preroll, new_sample };
  gst_app_sink_set_callbacks(GST_APP_SINK(sink), &callbacks, nullptr, nullptr);
  
  // Declare bus
  bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  gst_bus_add_watch(bus, my_bus_callback, nullptr);
  gst_object_unref(bus);

  gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);

}


float d = 0;
void GStreamerTexture::_process(float delta) {
    // TODO set updated image 
    // set_data
  d += delta;
  if (d > 1.0) {
    Godot::print("hello");
    d = 0;
  }

  g_main_iteration(false);

  char* frame = atomicFrame.load();
  if(frame) {
    Godot::print("f");
    //cv::imshow("Frame", atomicFrame.load()[0]);
    //cv::waitKey(30);
  }
}

