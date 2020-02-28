import actions from './actions.js'
import bridge from './rosbridge.js'
import store from './store.js'
import render from './render.js'

const constants = {
  USER: 'smartin015',
  URI: "ws://penguin.linux.test:4243",
};
console.log('main entered');
bridge.setUser(constants.USER);
bridge.setHandlers({
  "active_project_id": (msg) => {
    actions.setActiveProject(msg);
    render();
  },
});
bridge.setAdvertisedTopics({
  "emergency_stop": "std_msgs/Bool",
});
bridge.connect(constants.URI); 
render();
