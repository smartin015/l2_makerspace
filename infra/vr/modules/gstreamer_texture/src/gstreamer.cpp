#include "gstreamer.h"
#include <gst/app/gstappsink.h>
#include <PoolArrays.hpp>
#include <atomic>
#include <TextureRect.hpp>

using namespace godot;

// TODO make part of class
PoolByteArray* buf;
std::atomic_bool hasdata = false;

void GStreamer::_register_methods() {
  register_method("_process", &GStreamer::_process);
  register_method("_ready", &GStreamer::_ready);
}

GStreamer::GStreamer() {
}

GStreamer::~GStreamer() {
  Godot::print("Deconstructing");
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
GstFlowReturn new_sample(GstAppSink *appsink, gpointer /*data*/) {
    if(hasdata) {
      return GST_FLOW_OK;
    }
    static int framecount = 0;

    // Get caps and frame
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    GstBuffer *buffer = gst_sample_get_buffer(sample);

    // Show caps on first frame
    if(!framecount) {
      GstCaps *caps = gst_sample_get_caps(sample);
      GstStructure *structure = gst_caps_get_structure(caps, 0);
      const int width = g_value_get_int(
          gst_structure_get_value(structure, "width"));
      const int height = g_value_get_int(
          gst_structure_get_value(structure, "height"));
      Godot::print(gst_caps_to_string(caps));
    }
    framecount++;

    // Get frame data and convert
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);
    buf->resize(map.size);
    memcpy(buf->write().ptr(), map.data, map.size);
    hasdata = true;
    Godot::print(String::num(map.size));
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
            Godot::print("Error: %s", err->message);
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

void GStreamer::_init() {
  buf = new PoolByteArray();

  im = Image::_new();
  im->create(320, 120, false, Image::FORMAT_RGB8);
  im->fill(Color(0, 1.0, 0));

  it = ImageTexture::_new();
  it->set_storage(ImageTexture::STORAGE_RAW);
  it->create_from_image(Ref(im));
}

void GStreamer::_ready() {
  Node* chld = get_child(0);
  if (chld == NULL) {
    Godot::print("require single child node within GStreamer");
  }
  TextureRect* c = godot::Object::cast_to<godot::TextureRect>(chld);
  if (c == NULL) {
    Godot::print("failed to cast child to texturerect within GStreamer");
    return;
  }
  c->set_texture(it);

  // flags = Texture::FLAG_VIDEO_SURFACE;
  Godot::print("gstreamer texture init");

  gst_init(NULL, NULL);
  // https://stackoverflow.com/questions/10403588/adding-opencv-processing-to-gstreamer-application
  gchar *descr = g_strdup(
      //"playbin uri=https://www.freedesktop.org/software/gstreamer-sdk/data/media/sintel_trailer-480p.webm "
      "videotestsrc "
      "! videoconvert "
      "! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
  );

  // Check pipeline
  error = nullptr;
  pipeline = gst_parse_launch(descr, &error);
	if(error) {
      Godot::print("could not construct pipeline: %s", error->message);
      g_error_free(error);
      exit(-1);
  }

  // Get sink signals and check for a preroll. 
  // If preroll exists, we do have a new frame
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

  // Start playing
  gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);
  Godot::print("Pipeline now playing");
}


void GStreamer::_process(float delta) {
  //g_main_iteration(false);
  if (hasdata.load()) {
    hasdata = false;
    im->lock();
    Godot::print(String::num(buf->size()));
    im->create_from_data(320, 120, false, Image::FORMAT_RGB8, *buf);
    im->unlock();
    it->set_data(im);
  }
}

