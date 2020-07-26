#ifndef GSTREAMERTEXTURE_H
#define GSTREAMERTEXTURE_H

#include <Godot.hpp>
#include <ImageTexture.hpp>
#include <Image.hpp>
#include <Color.hpp>
#include <gst/gst.h>
#include <Control.hpp>

namespace godot {

class GStreamer : public Control {
    GODOT_CLASS(GStreamer, Control)

private:
    ImageTexture* it;
    Image* im;
    GstElement* pipeline;
    GstElement* source;
    GstElement* sink;
    GstElement* bin;
    GstBus* bus;
    GError* error;
public:
    static void _register_methods();

    GStreamer();
    ~GStreamer();

    void _init(); // our initializer called by Godot
    void _ready();
    void _process(float delta);
};

}

#endif
