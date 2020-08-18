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

actions.controlClicked = (evt) => {
  const id = evt.target.closest("button").id;
  if (id === "estop") {
    console.warn("Sending emergency stop command")
    bridge.publish('emergency_stop', {data: true})
    return;
  } else if (id === "debug_json") {
    store.page = Page.DEBUG_JSON;
    window.location.hash = "DEBUG_JSON";
    return;
  } else if (id === "settings") {
    store.page = Page.SETTINGS;
    window.location.hash = "SETTINGS";
    return;
  } else if (id === "project_list") {
    store.page = Page.PROJECT_LIST;
    window.location.hash = "PROJECT_LIST";
    return;
  }

  if (!store.active_project) {
    return;
  }
  console.log("CLICKED", id);
  const ids = Object.keys(store.projects).sort();
  const i = ids.indexOf(store.active_project.toString());
  let next = id;
  switch (id) {
    case "next":
      next = ids[(i+1) % ids.length];
      break;
    case "prev":
      next = ids[(i+ids.length-1) % ids.length];
      break;
    default:
      console.error("Unknown control id", id);
      return;
  }
  console.log('next ID is', next, "ids", ids, "i", i);
  bridge.publish('set_active_project', {data: next});
};


export default actions;
