import store from './store.js'
import bridge from './rosbridge.js'
const actions = {};

actions.setActiveProject = (msg) => {
  store.active_project = msg.data;
  console.log(msg);
}

actions.controlClicked = (evt) => {
  if (evt.target.id === "estop") {
    console.warn("Sending emergency stop command")
    bridge.publish('emergency_stop', {data: true})
    return;
  }

  if (!store.active_project) {
    return;
  }
  const id = evt.target.id;
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
