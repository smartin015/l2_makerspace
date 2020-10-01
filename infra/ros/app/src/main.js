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
window.onhashchange = () => {
  render();
}
bridge.on_connection_state_change = (conn) => {
  actions.connectionStateChanged(conn);
  render();
};
bridge.setUser(constants.USER);
bridge.setHandlers({
  "active_project_id": (msg) => {
    actions.setActiveProject(msg);
    render();
  },
  "project_list": (msg) => {
    actions.loadProjects(msg.projects);
    render();
  },
  "debug_json": (msg) => {
    actions.mergeDebugJSON(msg);
    render();
  },
});
bridge.connect(constants.URI); 
render();
