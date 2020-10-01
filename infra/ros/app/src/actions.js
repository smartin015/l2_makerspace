import {store, Page} from './store.js'
import bridge from './rosbridge.js'
const actions = {};

actions.setActiveProject = (id) => {
  store.active_project = id;
  console.log(`Active project now: ${store.active_project}`);
  actions.pubActiveProject(id);
}

actions.pubActiveProject = (id) => {
  bridge.publish(JSON.stringify({"l2app": "active_project", "id": id}));
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

actions.emergencyStop = () => {
  console.warn("Sending emergency stop command")
  bridge.publish('STOP')
}

actions.nav = (page) => {
  window.location.hash = page;
}

export default actions;
