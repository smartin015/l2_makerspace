import {store, Page} from './store.js'
import bridge from './rosbridge.js'
const actions = {};

actions.setActiveProject = (msg) => {
  store.active_project = msg.data;
  console.log(msg);
}

actions.connectionStateChanged = (connected) => {
  store.connected = connected;
}

actions.mergeDebugJSON = (msg) => {
  const j = JSON.parse(msg.data);
  store.debug_json = {...store.debug_json, ...j};
}

actions.setPage = (page) => {
  store.page = page;
}

actions.loadProjects = (projects) => {
  for (const p of projects) {
    console.log(p.id);
    store.projects[p.id] = p;
  }
}

actions.projectClicked = (evt) => {
  console.log(evt);
}

actions.controlClicked = (evt) => {
  const id = evt.target.closest("button").id;
  if (id === "estop") {
    console.warn("Sending emergency stop command")
    bridge.publish('emergency_stop', {data: true})
  } else if (id === "debug_json") {
    window.location.hash = Page.DEBUG_JSON;
  } else if (id === "settings") {
    window.location.hash = Page.SETTINGS;
  } else if (id === "project_list") {
    window.location.hash = Page.PROJECT_LIST;
  }
};


export default actions;
