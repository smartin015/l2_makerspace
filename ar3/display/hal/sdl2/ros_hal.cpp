// https://github.com/iamscottmoyers/simple-libwebsockets-example/blob/master/client.c
#include "ros_hal.h"
#include <libwebsockets.h>
#include <vector>
#include <string>
#include "state_generated.h"
#include "flatbuffers/idl.h"
#include "flatbuffers/util.h"
#define EXAMPLE_RX_BUFFER_BYTES 256

flatbuffers::Parser parser;

struct lws *web_socket = NULL;
bool valid_state = false;

static int ws_cb( struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len ) {
  const State *s;
  switch( reason ) {
		case LWS_CALLBACK_CLIENT_ESTABLISHED:
			lws_callback_on_writable( wsi );
			break;

		case LWS_CALLBACK_CLIENT_RECEIVE:
			/* Handle incomming messages here. */
      if (!parser.Parse((char*)in)) {
        printf("Parser error: %s\n", parser.error_.c_str());
      } else {
        valid_state = true;
      }
			break;

		case LWS_CALLBACK_CLIENT_WRITEABLE:
      /* Write messages here */
      break;

		case LWS_CALLBACK_CLOSED:
		case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
			web_socket = NULL;
			break;

		default:
			break;
	}
  return 0;
}

enum protocol_enums
{
	PROTOCOL_EXAMPLE = 0,
	PROTOCOL_COUNT
};

struct lws_protocols protocols[] = {
	{"example-protocol", ws_cb, 0, EXAMPLE_RX_BUFFER_BYTES},
	{ NULL, NULL, 0, 0 } /* terminator */
};

struct lws_context_creation_info info;
struct lws_context *context;
void ros_hal::init() {
	memset( &info, 0, sizeof(info) );
	info.port = CONTEXT_PORT_NO_LISTEN;
	info.protocols = protocols;
	info.gid = -1;
	info.uid = -1;
  context = lws_create_context( &info );
  printf("lws context created\n");
  
  // Load schema into parser
  std::string schemafile;
  bool ok = flatbuffers::LoadFile("lib/proto/state.fbs", false, &schemafile);
  if (!parser.Parse(schemafile.c_str())) {
    printf("WARNING: parser could not parse schema\n");
  }
}

const char* ros_hal::get_status() {
  return "TODO status";
}

const State* ros_hal::get_state() {
  if (!valid_state) {
    return nullptr;
  }
  return GetState(parser.builder_.GetBufferPointer());
}

void ros_hal::spin() {
	if(!web_socket) {
			struct lws_client_connect_info ccinfo = {0};
			ccinfo.context = context;
			ccinfo.address = "localhost";
			ccinfo.port = 8000;
			ccinfo.path = "/";
			ccinfo.host = lws_canonical_hostname( context );
			ccinfo.origin = "origin";
			ccinfo.protocol = protocols[PROTOCOL_EXAMPLE].name;
			web_socket = lws_client_connect_via_info(&ccinfo);
      printf("attempting to connect\n");
	}
  // -1 unofficial hack to not use built-in poll delay introduced in recent libwebsockets version
  // https://github.com/warmcat/libwebsockets/issues/1735
	lws_service( context, -1);
}

