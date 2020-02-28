

const store = {
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
};

export default store;
