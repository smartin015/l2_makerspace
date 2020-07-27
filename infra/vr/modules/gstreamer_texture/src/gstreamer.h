#ifndef GSTREAMERTEXTURE_H
#define GSTREAMERTEXTURE_H

#include <Godot.hpp>
#include <gst/app/gstappsink.h>
#include <ImageTexture.hpp>
#include <Image.hpp>
#include <Color.hpp>
#include <gst/gst.h>
#include <Control.hpp>
#include <PoolArrays.hpp>
#include <atomic>

namespace godot {

class GStreamer : public Control {
    GODOT_CLASS(GStreamer, Control)

private:
    ImageTexture* it;
    Ref<Image> im;
    GstElement* pipeline;
    GstElement* source;
    GstElement* sink;
    bool texture_init;
    String pipeline_str;
    int width;
    int height;
    PoolByteArray* buf;
    std::atomic_bool has_data;
public:
    static void _register_methods();

    // This function is exposed to C++ to allow
    // static, non-method gstreamer callbacks to
    // write back data. It should not be exposed
    // to GDScript.
    GstFlowReturn new_sample(GstAppSink *appsink);

    GStreamer();
    ~GStreamer();

    void _init(); // our initializer called by Godot
    void _ready();
    void _process(float delta);
};

}

#endif
