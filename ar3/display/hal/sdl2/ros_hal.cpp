// https://github.com/iamscottmoyers/simple-libwebsockets-example/blob/master/client.c
#include "ros_hal.h"
#include "flatbuffers/idl.h"
#include "flatbuffers/util.h"
#include "state_generated.h"
#include "config_generated.h"
#include <fstream>
#include <libwebsockets.h>
#include <string>
#include <vector>
#define EXAMPLE_RX_BUFFER_BYTES 256

#define CFG_PATH "./data/secret.bin"
std::string cfg_buf;
const Config *cfg;

#define BUFSZ 1024
char out[BUFSZ];

flatbuffers::Parser parser;

struct lws *web_socket = NULL;
bool valid_state = false;

static int ws_cb(struct lws *wsi, enum lws_callback_reasons reason, void *user,
                 void *in, size_t len) {
  const State *s;
  switch (reason) {
  case LWS_CALLBACK_CLIENT_ESTABLISHED:
    printf("WS connection established\n");
    lws_callback_on_writable(wsi);
    break;

  case LWS_CALLBACK_CLIENT_RECEIVE:
    /* Handle incomming messages here. */
    printf("RECV %s\n", (char*)in);
    if (!parser.Parse((char *)in)) {
      printf("Parser error: %s\n", parser.error_.c_str());
    } else {
      valid_state = true;
    }
    break;

  case LWS_CALLBACK_CLIENT_WRITEABLE:
  {
    // Tell WS server we want joint state details
    std::string cmd = "JOINT_STATE";
    uint8_t buf[LWS_SEND_BUFFER_PRE_PADDING + cmd.size() + LWS_SEND_BUFFER_POST_PADDING];
    uint8_t *p = &buf[LWS_SEND_BUFFER_PRE_PADDING];
    size_t n = sprintf((char*)p, cmd.c_str(), rand());
    lws_write(wsi, p, n, LWS_WRITE_TEXT);  
    break;
  }
  case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
    printf("WS error %s\n", in ? (char*)in : "");
  case LWS_CALLBACK_CLOSED:
    printf("WS connection closed\n");
    web_socket = NULL;
    break;

  default:
    break;
  }
  return 0;
}

enum protocol_enums { PROTOCOL_EXAMPLE = 0, PROTOCOL_COUNT };

struct lws_protocols protocols[] = {
    {"example-protocol", ws_cb, 0, EXAMPLE_RX_BUFFER_BYTES},
    {NULL, NULL, 0, 0} /* terminator */
};

struct lws_context_creation_info info;
struct lws_context *context;
void ros_hal::init() {
  std::ifstream cfg_stream;
  cfg_stream.open(CFG_PATH);
  if (cfg_stream.fail()) {
    sprintf(out, "ERR FOPEN");
    return;
  }
  cfg_buf = std::string(std::istreambuf_iterator<char>(cfg_stream),
                        std::istreambuf_iterator<char>());
  cfg = GetConfig(cfg_buf.c_str());
  if (!cfg) {
    sprintf(out, "ERR CFGPARSE"); 
    return;
  }
  if (!cfg->ws_ip()) {
    sprintf(out, "ERR NOWSIP");
    return;
  }
  if (!cfg->ws_port()) {
    sprintf(out, "ERR NOWSPORT");
    return;
  }
  
  memset(&info, 0, sizeof(info));
  info.port = CONTEXT_PORT_NO_LISTEN;
  info.protocols = protocols;
  info.gid = -1;
  info.uid = -1;
  context = lws_create_context(&info);
  sprintf(out, "Config OK, ws con...");

  // Load schema into parser
  std::string schemafile;
  bool ok = flatbuffers::LoadFile("lib/proto/state.fbs", false, &schemafile);
  if (!parser.Parse(schemafile.c_str())) {
    printf("WARNING: parser could not parse schema\n");
  }
}

const char *ros_hal::get_status() { return out; }

const State *ros_hal::get_state() {
  if (!valid_state) {
    return nullptr;
  }
  return GetState(parser.builder_.GetBufferPointer());
}

bool ws_connected = false;
void ros_hal::spin() {
  if (!web_socket) {
    struct lws_client_connect_info ccinfo = {0};
    ccinfo.context = context;
    ccinfo.address = cfg->ws_ip()->c_str();
    ccinfo.port = cfg->ws_port();
    ccinfo.path = "/";
    ccinfo.host = lws_canonical_hostname(context);
    ccinfo.origin = "origin";
    ccinfo.protocol = protocols[PROTOCOL_EXAMPLE].name;
    web_socket = lws_client_connect_via_info(&ccinfo);
    if (ws_connected) {
      ws_connected = false;
      sprintf(out, "WS CONN...");
    }
  } else {
    if (!ws_connected) {
      ws_connected = true;
      sprintf(out, "CONNECTED");
    }
  }
  // -1 unofficial hack to not use built-in poll delay introduced in recent
  // libwebsockets version https://github.com/warmcat/libwebsockets/issues/1735
  lws_service(context, -1);
}
