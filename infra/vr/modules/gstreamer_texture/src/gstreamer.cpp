#include "gstreamer.h"
#include <TextureRect.hpp>

using namespace godot;

const char* DEFAULT_PIPELINE = (
  //"playbin uri=https://www.freedesktop.org/software/gstreamer-sdk/data/media/sintel_trailer-480p.webm "
  "videotestsrc "
  "! videoconvert ! video/x-raw, format=RGB "
  "! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
);

void GStreamer::_register_methods() {
  register_method("_process", &GStreamer::_process);
  register_method("_ready", &GStreamer::_ready);
  register_property<GStreamer, Ref<ImageTexture>>("image_texture", &GStreamer::image_texture, Ref(ImageTexture::_new()));
  register_property<GStreamer, Ref<AudioStreamGeneratorPlayback>>("asgp", &GStreamer::asgp, Ref(AudioStreamGeneratorPlayback::_new()));
  register_property<GStreamer, String>("pipeline", &GStreamer::pipeline_str, String(DEFAULT_PIPELINE));
  register_property<GStreamer, String>("videosink", &GStreamer::videosink, "videosink");
  register_property<GStreamer, String>("audiosink", &GStreamer::audiosink, "audiosink");
  register_signal<GStreamer>((char *)"new_caps", "node", GODOT_VARIANT_TYPE_OBJECT, "caps", GODOT_VARIANT_TYPE_STRING);
}

GStreamer::GStreamer() {}

GStreamer::~GStreamer() {
  gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);
  gst_object_unref(GST_OBJECT(pipeline)); // Source & sink nodes are part of pipeline
  delete buf;
}

/**
* @brief Check preroll to get a new frame using callback
*  https://gstreamer.freedesktop.org/documentation/design/preroll.html
* @return GstFlowReturn
*/
GstFlowReturn new_preroll(GstAppSink* /*appsink*/, gpointer /*data*/) {
    return GST_FLOW_OK;
}

GstFlowReturn new_audio_sample_static(GstAppSink *appsink, gpointer data) {
  return ((GStreamer*)data)->new_audio_sample(appsink);
}

GstFlowReturn new_video_sample_static(GstAppSink *appsink, gpointer data) {
  return ((GStreamer*)data)->new_video_sample(appsink);
}

GstFlowReturn GStreamer::new_audio_sample(GstAppSink *appsink) {
  if (has_audio) {
    return GST_FLOW_OK;
  }

  // Get caps and frame
  GstSample *sample = gst_app_sink_try_pull_sample(appsink, GST_SECOND/100);
  if (sample == NULL) {
    return GST_FLOW_OK;
  }
  GstBuffer *buffer = gst_sample_get_buffer(sample);

  // Show caps & record dimensions on first frame
  if(channels == 0) {
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    channels = g_value_get_int(gst_structure_get_value(structure, "channels"));
    emit_signal("new_caps", this, gst_caps_to_string(caps));
  }

  // Get frame data and convert
  GstMapInfo map;
  gst_buffer_map(buffer, &map, GST_MAP_READ);
  const int samplen = map.size / channels / 4; // 4 == sizeof(float32)
  if (abuf->size() != samplen) {
    abuf->resize(samplen);
  }
  memcpy(abuf->write().ptr(), map.data, std::min(abuf->size()*sizeof(Vector2), map.size));
  has_audio = true;
  gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

GstFlowReturn GStreamer::new_video_sample(GstAppSink *appsink) {
  if(has_video) {
    // We haven't yet consumed the last sample
    return GST_FLOW_OK;
  }

  // Get caps and frame
  GstSample *sample = gst_app_sink_pull_sample(appsink);
  GstBuffer *buffer = gst_sample_get_buffer(sample);

  // Show caps & record dimensions on first frame
  if(width == 0 || height == 0) {
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    width = g_value_get_int(gst_structure_get_value(structure, "width"));
    height = g_value_get_int(gst_structure_get_value(structure, "height"));
    emit_signal("new_caps", this, gst_caps_to_string(caps));
    //Godot::print(gst_video_format_to_string(fmt));
  }

  // Get frame data and convert
  GstMapInfo map;
  gst_buffer_map(buffer, &map, GST_MAP_READ);
  const int bitlen = width * height * 3;
  if (buf->size() != bitlen) {
    buf->resize(bitlen);
  }
  memcpy(buf->write().ptr(), map.data, std::min(buf->size(), bitlen));
  has_video = true;
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
  pipeline_str = String(DEFAULT_PIPELINE);
  videosink = String("videosink");
  audiosink = String("audiosink");
  buf = new PoolByteArray();
  abuf = new PoolVector2Array();
  has_video = false;
  width = 0;
  height = 0;
  channels = 0;
  im = Ref(Image::_new()); // Wait to initialize image
}

void GStreamer::_ready() {
  if (image_texture.is_valid()) {
    image_texture->set_storage(ImageTexture::STORAGE_RAW);
    texture_width = 0;
    texture_height = 0;
  }

  gst_init(NULL, NULL);
  // https://stackoverflow.com/questions/10403588/adding-opencv-processing-to-gstreamer-application
  // Check pipeline
  GError* error = nullptr;
  pipeline = gst_parse_launch(pipeline_str.alloc_c_string(), &error);
	if(error) {
      Godot::print("could not construct pipeline");
      Godot::print(error->message);
      g_error_free(error);
      exit(-1);
  }

  GstElement* sink = gst_bin_get_by_name(GST_BIN(pipeline), videosink.alloc_c_string()); 
  if (sink != NULL) {
    // Get sink signals and check for a preroll. 
    // If preroll exists, we do have a new frame
    gst_app_sink_set_emit_signals((GstAppSink*)sink, true);
    GstAppSinkCallbacks callbacks = {nullptr, new_preroll, new_video_sample_static};
    gst_app_sink_set_callbacks(GST_APP_SINK(sink), &callbacks, this, nullptr);
    gst_object_unref(sink);
  }
  
  // Do the same for audio sink
  asink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline), audiosink.alloc_c_string()));
  if (asink != NULL) {
    // gst_app_sink_set_emit_signals(GST_APP_SINK(asink), true);
    GstAppSinkCallbacks callbacks = {nullptr, new_preroll, new_audio_sample_static};
    gst_app_sink_set_callbacks(GST_APP_SINK(asink), &callbacks, this, nullptr);
  }
  
  // Declare bus
  GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
  gst_bus_add_watch(bus, my_bus_callback, nullptr);
  gst_object_unref(bus);

  // Start playing
  gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);
}

static float dt = 0.0;
static float at = 0.0;
static float nt = 0.0;
void GStreamer::_process(float delta) {
  if (has_video.load() && buf->size() != 0) {
    if (image_texture.is_valid()) {
      if (texture_width != width || texture_height != height) {
        im->create_from_data(width, height, false, Image::FORMAT_RGB8, *buf);
        image_texture->create_from_image(Ref(im), Texture::FLAG_VIDEO_SURFACE);
        texture_width = width;
        texture_height = height;
      } else {
        im->lock();
        im->create_from_data(width, height, false, Image::FORMAT_RGB8, *buf);
        im->unlock();
        image_texture->set_data(im);
      }
    }
    has_video = false;
  }

  if (has_audio.load()) {
    if (asgp.is_valid() && asgp->can_push_buffer(abuf->size())) {
      asgp->push_buffer(*abuf);
      has_audio = false;
      new_audio_sample(asink);
    }
  }
}

