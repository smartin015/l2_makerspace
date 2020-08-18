// Connect and handle rosbridge communications

class ROSBridge {
  constructor() {
    this.ns = "l2";
  }

  setUser(username) {
    this.username = username;
  }

  setHandlers(topicHandlers) {
    this.handlers = {};
    for (let topic of Object.keys(topicHandlers)) {
      if (topic.startsWith("/")) {
        this.handlers[topic] = topicHandlers[topic];
      } else {
        this.handlers[`/${this.ns}/${this.username}/${topic}`] = topicHandlers[topic];
      }
    }
  }

  setAdvertisedTopics(topics) {
    this.advertised = topics;
  }

  connect(uri) {
    this.socket = new WebSocket("ws://penguin.linux.test:3000");
    this.socket.onopen = this._onOpen.bind(this);
    this.socket.onmessage = this._onMessage.bind(this);
    this.socket.onerror = this._onError.bind(this);
    this.socket.onclose = this._onClose.bind(this);
  }

  _onOpen() {
    console.log("[open] Connection established");
    this.on_connection_state_change(true);
    for (let advert of Object.keys(this.advertised)) {
      const topic = `/${this.ns}/${this.username}/${advert}`;
      this.socket.send(JSON.stringify({
        op: 'advertise',
        topic,
        type: this.advertised[advert],
      }));
      console.log(`Advertising ${topic}`);
    }
    for (let topic of Object.keys(this.handlers)) {
      this.socket.send(JSON.stringify({
        op: 'subscribe', 
        topic,
        type: this.handlers[topic][0],
      }));
      console.log(`Subscribed to ${topic}`);
    }
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
      if (msg.op === 'publish' && this.handlers[msg.topic]) {
        this.handlers[msg.topic][1](msg.msg);
      }
    } catch (e) {
      console.log("Failed to parse message:", text);
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
