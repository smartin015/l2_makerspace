export const Page = {
  PROJECT_LIST: 0,
  PROJECT_DETAILS: 1,
  DEBUG_JSON: 2,
  SETTINGS: 3,
};

export const store = {
  connected: false,
  projects: {
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
  },
  active_project: null,
  page: Page.PROJECT_LIST,
  debug_json: {},
};

export default store;
