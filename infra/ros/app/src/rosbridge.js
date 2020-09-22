// Connect and handle rosbridge communications

class ROSBridge {
  constructor() {
    this.ns = "l2";
  }

  setUser(username) {
    this.username = username;
  }

  setHandlers(topicHandlers) {
    this.handlers = topicHandlers;
  }

  setAdvertisedTopics(topics) {
    this.advertised = topics;
  }

  connect(uri) {
    this.socket = new WebSocket("ws://localhost:8001");
    this.socket.onopen = this._onOpen.bind(this);
    this.socket.onmessage = this._onMessage.bind(this);
    this.socket.onerror = this._onError.bind(this);
    this.socket.onclose = this._onClose.bind(this);
  }

  _onOpen() {
    console.log("[open] Connection established");
    this.on_connection_state_change(true);
    this.socket.send("PROJECTS?");
  }

  _onClose(event) {
    this.on_connection_state_change(false);
    if (event.wasClean) {
      console.log(`[close] Connection closed cleanly, code=${event.code} reason=${event.reason}`);
    } else {
      // e.g. server process killed or network down
      // event.code is usually 1006 in this case
      console.log('[close] Connection died');
    }
  }

  _onError(error) {
    console.log(`[error] ${error.message}`);
  }

  _onMessage(event) {
    // console.log(`[message] Data received from server: ${event.data}`);
    try {
      const msg = JSON.parse(event.data);
      console.log(msg);
      if (msg.l2app !== undefined) {
        this.handlers[msg.l2app](msg);
      }
    } catch (e) {
      console.log("Failed to parse message:", e.toString());
      return;
    }
  }

  publish(topic, msg) {
    this.socket.send(JSON.stringify({
      op: "publish",
      topic: `/${this.ns}/${this.username}/${topic}`,
      msg,
    }));
  }
}

const bridge = new ROSBridge();
export default bridge;
