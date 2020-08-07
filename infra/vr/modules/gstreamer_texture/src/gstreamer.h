#ifndef GSTREAMERTEXTURE_H
#define GSTREAMERTEXTURE_H

#include <Godot.hpp>
#include <gst/app/gstappsink.h>
#include <ImageTexture.hpp>
#include <AudioStreamGeneratorPlayback.hpp>
#include <Image.hpp>
#include <Color.hpp>
#include <gst/gst.h>
#include <Node.hpp>
#include <PoolArrays.hpp>
#include <atomic>

namespace godot {

class GStreamer : public Node {
    GODOT_CLASS(GStreamer, Node)

private:
    Ref<ImageTexture> image_texture;
    Ref<AudioStreamGeneratorPlayback> asgp;
    Ref<Image> im;
    GstElement* pipeline;
    String pipeline_str;
    int width;
    int height;
    int channels;
    int texture_width;
    int texture_height;
    PoolByteArray* buf;

    // Under the hood, this is a Vector<Vector2>
    // so needs interleaved audio
    PoolVector2Array* abuf;
    std::atomic_bool has_data;
    std::atomic_bool has_audio;
    String videosink;
    String audiosink;
    GstAppSink* asink;
public:
    static void _register_methods();

    // This function is exposed to C++ to allow
    // static, non-method gstreamer callbacks to
    // write back data. It should not be exposed
    // to GDScript.
    GstFlowReturn new_video_sample(GstAppSink *appsink);
    GstFlowReturn new_audio_sample(GstAppSink *appsink);

    GStreamer();
    ~GStreamer();

    void _init(); // our initializer called by Godot
    void _ready();
    void _process(float delta);

};

}

#endif
