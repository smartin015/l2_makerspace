export const Page = {
  PROJECT_LIST: "#PROJECT_LIST",
  DEBUG_JSON: "#DEBUG_JSON",
  SETTINGS: "#SETTINGS",
};

export const store = {
  connected: false,
  projects: {
    /*
    "8675309":
    {name: 'proj', id: 8675309, owner: 'testuser', items:[
      {content: 'item1'},
      {content: 'item2'},
      {content: 'item3'},
    ]},
    "12345":
    {name: 'anotherproj', id: 12345, owner: 'smartin015', items:[
      {content: 'theonlyitem'},
    ]},
    */
  },
  active_project: null,
  debug_json: {},
};

// For debugging; assigned as object reference
// so you can interrogate window.store in the console
window.store = store;

export default store;
