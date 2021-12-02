#include <gst/gst.h>
#include <iostream>
#include <mqtt/async_client.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// Much of MQTT work inspired by https://github.com/debihiga/mqtt_paho_cpp_subscriber/blob/master/main.cpp
const std::string ADDRESS("mqtt:1883");
std::string HOSTNAME; // Filled in main()
const std::string TOPIC("/av/command");
const std::string STATUS_TOPIC_BASE("/av/status/"); // hostname appended
const std::string START_RECORDING_CMD = "start_recording";
const std::string STOP_RECORDING_CMD = "stop_recording";
const int QOS = 1;
const int N_RETRY_ATTEMPTS = 3;

typedef enum {
 PIPELINE_TYPE_UNKNOWN = 0,
 PIPELINE_TYPE_REALSENSE_RGB,
 PIPELINE_TYPE_ALSA_HW2,
 PIPELINE_TYPE_AUTO_VIDEO,
} PipelineType;

GstElement* setupPipeline(const PipelineType& p, const std::string& serial, const std::string& outdir) {
  // NOTE: there may be further efficiencies here, but trying these breaks the pipeline in dumb ways:
  //  - use nvvidconv instead of videoconvert
  //  - use nvv4l2h264enc instead of omxh264enc
  //  - use hlssink2 instead of mpegtsmux ! hlssink
  std::string pipespec;
  switch (p) {
    case PIPELINE_TYPE_REALSENSE_RGB:
      std::cerr << "Creating Realsense RGB pipeline" << std::endl;
			pipespec = ( // "realsensesrc serial=819112070701 timestamp-mode=clock_all enable-color=true ! rgbddemux name=demux demux.src_color ! videoconvert ! omxh264enc ! hlssink2 playlist-root=http://192.168.1.8:8080 location=/tmp/ramdisk/segment_%05d.ts playlist-location=/tmp/ramdisk/playlist.m3u8 target-duration=5 max-files=5",
			    "realsensesrc serial=" + serial + " timestamp-mode=clock_all enable-color=true depth-height=480 depth-width=640 framerate=15 "
			   "! rgbddemux name=demux demux.src_color ! queue ! videoconvert ! omxh264enc ! video/x-h264, stream-format=(string)byte-stream "
			   "! h264parse ! mpegtsmux name=mux "
			   "demux.src_depth ! queue ! colorizer near-cut=300 far-cut=700 ! videoconvert ! omxh264enc ! video/x-h264, stream-format=(string)byte-stream "
			   "! h264parse ! mux. "
			   "mux. ! hlssink playlist-root=http://" + HOSTNAME + ":8080 playlist-location=" + outdir + "playlist.m3u8 location=" + outdir + "segment_%05d.ts "
	      );
      break;
    case PIPELINE_TYPE_ALSA_HW2:
      std::cerr << "Creating ALSA HW:2 pipeline" << std::endl;
      // # See https://thiblahute.github.io/GStreamer-doc/alsa-1.0/alsasrc.html?gi-language=c
      pipespec = ("alsasrc device=hw:2 ! audioconvert ! avenc_aac "
                      "! hlssink2 playlist-root=http://" + HOSTNAME + ":8080 location=" + outdir + "segment_%05d.ts "
                      "target-duration=5 max-files=5");
      break;
    default:
      std::cerr << "Pipeline type " << p << " not implemented" << std::endl;
      return NULL;
  }
  std::cerr << pipespec << std::endl << std::endl;
  return gst_parse_launch(pipespec.c_str(), NULL);
}


class action_listener : public virtual mqtt::iaction_listener
{
  std::string name_;

  void on_failure(const mqtt::token& tok) override {
    std::cerr << name_ << " failure";
    if (tok.get_message_id() != 0)
      std::cerr << " for token [" << tok.get_message_id() << "]" << std::endl;
    std::cerr << std::endl;
  }

  void on_success(const mqtt::token& tok) override {
    std::cerr << name_;
    if (tok.get_message_id() != 0)
      std::cerr << "[" << tok.get_message_id() << "]";
    auto top = tok.get_topics();
    if (top && !top->empty())
      std::cerr << " topic '" << (*top)[0] << "'";
    std::cerr << " success" << std::endl;
  }

public:
  action_listener(const std::string& name) : name_(name) {}
};

class callback : public virtual mqtt::callback,
          public virtual mqtt::iaction_listener

{
  // Counter for the number of connection retries
  int nretry_;
  // The MQTT client
  mqtt::async_client& cli_;
  // Options to use if we need to reconnect
  mqtt::connect_options& connOpts_;
  // An action listener to display the result of actions.
  action_listener subListener_;

  GstElement* pipeline_;
  PipelineType pipelineType_;
  std::string statusTopic_;
  bool running_;
  std::string serial_;
  std::string outdir_;

  void sendStatus(const std::string& str) {
    auto msg = mqtt::make_message(statusTopic_, str);
    msg->set_qos(QOS);
    cli_.publish(msg);
    std::cerr << "MQTT " << statusTopic_ << " << " << str << std::endl;
  }

  // This deomonstrates manually reconnecting to the broker by calling
  // connect() again. This is a possibility for an application that keeps
  // a copy of it's original connect_options, or if the app wants to
  // reconnect with different options.
  // Another way this can be done manually, if using the same options, is
  // to just call the async_client::reconnect() method.
  void reconnect() {
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    try {
      cli_.connect(connOpts_, nullptr, *this);
    }
    catch (const mqtt::exception& exc) {
      std::cerr << "Error: " << exc.what() << std::endl;
      exit(1);
    }
  }

  // Re-connection failure
  void on_failure(const mqtt::token& tok) override {
    std::cerr << "Connection attempt failed" << std::endl;
    if (++nretry_ > N_RETRY_ATTEMPTS)
      exit(1);
    reconnect();
  }

  // (Re)connection success
  // Either this or connected() can be used for callbacks.
  void on_success(const mqtt::token& tok) override {}

  // (Re)connection success
  void connected(const std::string& cause) override {
    sendStatus("ready");
    std::cerr << "MQTT subscribe: '" << TOPIC << "'" << std::endl;
    cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
  }

  // Callback for when the connection is lost.
  // This will initiate the attempt to manually reconnect.
  void connection_lost(const std::string& cause) override {
    std::cerr << "\nConnection lost" << std::endl;
    if (!cause.empty())
      std::cerr << "\tcause: " << cause << std::endl;
    std::cerr << "Reconnecting..." << std::endl;
    nretry_ = 0;
    reconnect();
  }

  // Callback for when a message arrives.
  void message_arrived(mqtt::const_message_ptr msg) override {
    std::cerr << "MQTT " << msg->get_topic() << " >> " << msg->to_string() << std::endl;
    GstStateChangeReturn ret;
    if (msg->to_string() == START_RECORDING_CMD) {
      if (running_) {
        sendStatus("already_started");
        return;
      }
      sendStatus("starting");
      pipeline_ = setupPipeline(pipelineType_, serial_, outdir_);
      ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
      if (ret == GST_STATE_CHANGE_ASYNC) {
        ret = gst_element_get_state(pipeline_, NULL, NULL, GST_CLOCK_TIME_NONE);
      }
      if (ret == GST_STATE_CHANGE_SUCCESS) {
	sendStatus("started");
	running_ = true;
      } else {
	sendStatus("error " + std::to_string(ret));
        std::cerr << "action error, state change result " << ret << std::endl;
      }
    } else if (msg->to_string() == STOP_RECORDING_CMD) {
      if (running_) {
	sendStatus("stopping");
	gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
      }
      sendStatus("stopped");
      running_ = false;
    } else {
      sendStatus("error invalid_command");
      return;
    }

  }

  void delivery_complete(mqtt::delivery_token_ptr token) override {}

public:
  callback(mqtt::async_client& cli, mqtt::connect_options& connOpts, PipelineType ptype, std::string serial, std::string outdir)
        : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription"), pipelineType_(ptype), serial_(serial), outdir_(outdir), running_(false) {
    statusTopic_ = STATUS_TOPIC_BASE + HOSTNAME;
  }
};

PipelineType parsePtype(const std::string& a) {
  if (a == "rgb") {
    return PIPELINE_TYPE_REALSENSE_RGB;
  } else if (a == "mic") {
    return PIPELINE_TYPE_ALSA_HW2;
  } else if (a == "webcam") {
    return PIPELINE_TYPE_AUTO_VIDEO;
  } else {
    return PIPELINE_TYPE_UNKNOWN;
  }
}

void printHelp(char* argv0) {
  std::cerr << "Workshop network streaming daemon" << std::endl 
	  << "Connects to MQTT and listens on " << TOPIC << " for " << START_RECORDING_CMD << " and " << STOP_RECORDING_CMD << "." << std::endl
	  << "Sends status messages to " << STATUS_TOPIC_BASE << "#" << std::endl << std::endl
	  << "Usage: " << argv0 << " -p [rgb/mic/webcam] -o /path/to/output/dir/including/trailing/slash/" << std::endl 
	  << "Optional: -s [realsense serial number]" << std::endl << std::endl;
}

int main (int argc, char *argv[]) {
  char hostname[1024];
  gethostname(hostname, 1024);
  HOSTNAME = std::string(hostname);

  int opt;
  bool flagA = false;
  bool flagB = false;

  // Retrieve the (non-option) argument:
  if ( (argc <= 1) || (argv[argc-1] == NULL) || (argv[argc-1][0] == '-') ) {  // there is NO input...
      printHelp(argv[0]);
      return 1;
  }

  // Shut GetOpt error messages down (return '?'): 
  opterr = 0;

  // Retrieve the options:
  PipelineType ptype = PIPELINE_TYPE_UNKNOWN;
  std::string serial;
  std::string outdir;
  while ( (opt = getopt(argc, argv, "p:s:o:h")) != -1 ) {  // for each option...
    switch (opt) {
      case 'p':
	if (optarg != NULL) {
	  ptype = parsePtype(std::string(optarg)); 
	} else {
          std::cerr << "Warning: no value given for -p argument" << std::endl;
	}
        break;
      case 's':
	serial = optarg;
	break;
      case 'o':
	outdir = optarg;
	break;
      case 'h':
        printHelp(argv[0]);
	return 0;
      case '?':  // unknown option...
      default:
        std::cerr << "Unknown option: '" << char(optopt) << "'!" << std::endl;
	printHelp(argv[0]);
        break;
    }
  }

  if (ptype == PIPELINE_TYPE_UNKNOWN) {
    std::cerr << "Could not resolve pipeline type; exiting" << std::endl;
    return 2;
  }

  gst_init (&argc, &argv);
  mqtt::async_client client(ADDRESS, HOSTNAME);
  mqtt::connect_options connOpts;
  connOpts.set_clean_session(true);    
  callback cb(client, connOpts, ptype, serial, outdir);
  client.set_callback(cb);
  client.connect(connOpts, nullptr, cb);
  std::cin.get();
  std::cerr << "Disconnecting..." << std::flush;
  client.disconnect()->wait();
  std::cerr << "OK" << std::endl;
  return 0;
}
