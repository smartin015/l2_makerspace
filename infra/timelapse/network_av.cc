#include <gst/gst.h>
#include <iostream>
#include <mqtt/async_client.h>

// Much of MQTT work inspired by https://github.com/debihiga/mqtt_paho_cpp_subscriber/blob/master/main.cpp
const std::string ADDRESS("mqtt:1883");
const std::string CLIENT_ID("rs1");
const std::string TOPIC("/av/command");
const int  QOS = 1;
const int  N_RETRY_ATTEMPTS = 5;

class action_listener : public virtual mqtt::iaction_listener
{
	std::string name_;

	void on_failure(const mqtt::token& tok) override {
		std::cout << name_ << " failure";
		if (tok.get_message_id() != 0)
			std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
		std::cout << std::endl;
	}

	void on_success(const mqtt::token& tok) override {
		std::cout << name_ << " success";
		if (tok.get_message_id() != 0)
			std::cout << " for token: [" << tok.get_message_id() << "]" << std::endl;
		auto top = tok.get_topics();
		if (top && !top->empty())
			std::cout << "\ttoken topic: '" << (*top)[0] << "', ..." << std::endl;
		std::cout << std::endl;
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
		std::cout << "Connection attempt failed" << std::endl;
		if (++nretry_ > N_RETRY_ATTEMPTS)
			exit(1);
		reconnect();
	}

	// (Re)connection success
	// Either this or connected() can be used for callbacks.
	void on_success(const mqtt::token& tok) override {}

	// (Re)connection success
	void connected(const std::string& cause) override {
		std::cout << "\nConnection success" << std::endl;
		std::cout << "\nSubscribing to topic '" << TOPIC << "'\n"
			<< "\tfor client " << CLIENT_ID
			<< " using QoS" << QOS << "\n"
			<< "\nPress Q<Enter> to quit\n" << std::endl;

		cli_.subscribe(TOPIC, QOS, nullptr, subListener_);
	}

	// Callback for when the connection is lost.
	// This will initiate the attempt to manually reconnect.
	void connection_lost(const std::string& cause) override {
		std::cout << "\nConnection lost" << std::endl;
		if (!cause.empty())
			std::cout << "\tcause: " << cause << std::endl;

		std::cout << "Reconnecting..." << std::endl;
		nretry_ = 0;
		reconnect();
	}

	// Callback for when a message arrives.
	void message_arrived(mqtt::const_message_ptr msg) override {
		std::cout << "Message arrived" << std::endl;
		std::cout << "\ttopic: '" << msg->get_topic() << "'" << std::endl;
		std::cout << "\tpayload: '" << msg->to_string() << "'\n" << std::endl;

	  GstStateChangeReturn ret;
	  if (msg->to_string() == "start_recording") {
	    ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
	  } else if (msg->to_string() == "stop_recording") {
            ret = gst_element_set_state(pipeline_, GST_STATE_NULL);
	  } else {
	    std::cout << "Invalid command" << std::endl;
	    return;
	  }

	  if (ret == GST_STATE_CHANGE_ASYNC) {
	    ret = gst_element_get_state(pipeline_, NULL, NULL, GST_CLOCK_TIME_NONE);
	  }
	  if (ret == GST_STATE_CHANGE_SUCCESS) {
	    std::cout << "action successful" << std::endl;
	  } else {
	    std::cout << "action error, state change result " << ret << std::endl;
	  }
	}

	void delivery_complete(mqtt::delivery_token_ptr token) override {}

public:
	callback(mqtt::async_client& cli, mqtt::connect_options& connOpts, GstElement* pipeline)
				: nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription"), pipeline_(pipeline) {}
};

GstElement* setupPipeline() {
  return gst_parse_launch(
                    "realsensesrc serial=819112070701 timestamp-mode=clock_all enable-color=true "
                    "! rgbddemux name=demux demux.src_color ! queue ! videoconvert ! omxh264enc ! video/x-h264, stream-format=(string)byte-stream "
                    "! h264parse ! mpegtsmux ! hlssink playlist-root=http://192.168.1.8:8080 location=/tmp/ramdisk/segment_%05d.ts target-duration=5 max-files=5",
      NULL);
}

void runForever(GstElement *pipeline) {
  gst_element_set_state (pipeline, GST_STATE_PLAYING);
  GstBus * bus = gst_element_get_bus (pipeline);

  /*
  GstState st;
  try {
  while (gst_element_get_state(pipeline, &st) ) {
    GstMessage * msg =
        gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
        (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
      g_error ("An error occurred! Re-run with the GST_DEBUG=*:WARN environment "
          "variable set for more details.");
    }
  } 
  }
  catch (e) {
    gst_message_unref (msg);
    gst_object_unref (bus);
    gst_element_set_state (pipeline, GST_STATE_NULL);
    throw(e);
  }*/
}

int main (int argc, char *argv[]) {
  gst_init (&argc, &argv);
  GstElement *pipeline = setupPipeline();
  mqtt::async_client client(ADDRESS, CLIENT_ID);
  mqtt::connect_options connOpts;
  connOpts.set_clean_session(true);    
  callback cb(client, connOpts, pipeline);
  client.set_callback(cb);
  client.connect(connOpts, nullptr, cb);
  //runForever(pipeline);
  while (std::tolower(std::cin.get()) != 'q') {}
  gst_object_unref (pipeline);
  std::cout << "Disconnecting..." << std::flush;
  client.disconnect()->wait();
  std::cout << "OK" << std::endl;
  return 0;
}
