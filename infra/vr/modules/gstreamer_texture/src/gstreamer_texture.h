#ifndef GSTREAMERTEXTURE_H
#define GSTREAMERTEXTURE_H

#include <Godot.hpp>
#include <ImageTexture.hpp>
#include <Image.hpp>
#include <Color.hpp>
#include <gst/gst.h>

namespace godot {

class GStreamerTexture : public ImageTexture {
    GODOT_CLASS(GStreamerTexture, ImageTexture)

private:
    Image* im;
    GstElement* pipeline;
    GstElement* sink;
    GstBus* bus;
    GError* error;
public:
    static void _register_methods();

    GStreamerTexture();
    ~GStreamerTexture();

    void _init(); // our initializer called by Godot
    void _process(float delta);
};

}

#endif
