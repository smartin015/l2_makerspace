import actions from './actions.js'
import bridge from './rosbridge.js'
import {store, Page} from './store.js'
import render from './render.js'

const anchor = window.location.hash.substr(1);
if (anchor !== "") {
  store.page = Object.keys(Page).indexOf(anchor);
  console.log("Setting store to page " + store.page);
}

const constants = {
  USER: 'smartin015',
  URI: "ws://penguin.linux.test:4243",
};
console.log('main entered');
bridge.on_connection_state_change = (conn) => {
  actions.connectionStateChanged(conn);
  render();
};
bridge.setUser(constants.USER);
bridge.setHandlers({
  "active_project_id": ["std_msgs/Int64", (msg) => {
    actions.setActiveProject(msg);
    render();
  }],
  "/l2/debug_json": ["std_msgs/String", (msg) => {
    actions.mergeDebugJSON(msg);
    render();
  }],
});
bridge.setAdvertisedTopics({
  "emergency_stop": "std_msgs/Bool",
});
bridge.connect(constants.URI); 
render();
