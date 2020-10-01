import {store, Page} from './store.js'
import actions from './actions.js'

function renderControls(page) {
  return `
    <button id="${Page.PROJECT_LIST}" ${(page === Page.PROJECT_LIST) ? 'disabled' : ''}><img class="svg" src="img/view_list-white-18dp.svg"/></button>
    <button id="estop"><img class="svg" src="img/report-white-18dp.svg"/></button>
    <button id="${Page.DEBUG_JSON}" ${(page === Page.DEBUG_JSON) ? 'disabled' : ''}><img class="svg" src="img/notes-white-18dp.svg"/></button>
    <button id="${Page.SETTINGS}" ${(page === Page.SETTINGS) ? 'disabled' : ''}><img class="svg" src="img/settings-white-18dp.svg"/></button>
  `;
}

function renderProjects(projects, active_project = null) {
  projects = Object.values(projects);
  return projects.map((p) => renderProjectThumbnail(
    p.id, 
    p.name, 
    p.owner.name, 
    p.items,
    p.id == active_project
  )).join("\n");
}

function renderProjectThumbnail(id, name, owner, items, active = false) {
  return `<a class="project_thumb ${active ? "active" : ""}" 
           href="#/project/${id}">
      <div class="projectname">${name}</div>
      <div class="projectowner">${owner}</div>
      <div class="itemcount">${items.length} items</div>
    </a>`;
}

function renderProject(name, owner, items, active=false) {
  return [name, `<div>
      <button id="set_active" ${active ? "disabled" : ""}>Set active project</button>
      <div class="projectowner">Owner: ${owner}</div>
      <ul>
        ${items.map((item) => `<li>${item.content}</li>`).join("\n")}
      </ul>
    </div>`];
}

function renderNoProjectSet() {
  return `<div>No project set</div>`;
}

function renderDebugJSON(dbj) {
  if (Object.keys(dbj).length === 0) {
    return `<em>Waiting for debug JSON...</em>`;
  }
  return `
    ${Object.entries(dbj).map((ent) => `<div><strong>${ent[0]}:</strong> ${JSON.stringify(ent[1])}</div>`).join("\n")}
  `;
}

const status = document.querySelector(".status");
const controls = document.querySelector(".controls");
const content = document.querySelector(".project > .content");
const header = document.querySelector(".project > .header");
function render() {
  status.innerHTML = (store.connected) ? "Connected" : "Not connected";
  controls.innerHTML = renderControls(window.location.hash);
  for (let c of controls.children) {
    c.addEventListener("click", (evt) => {
			const id = evt.target.closest("button").id;
			if (id === "estop") {
        actions.emergencyStop()
			} else {
        actions.nav(id); 
      }
      render();
    });
  }
  
  switch (window.location.hash) {
    case '':
    case Page.PROJECT_LIST:
      header.innerHTML = "Projects";
      content.innerHTML = renderProjects(store.projects, store.active_project);
      return;
    case Page.DEBUG_JSON:
      header.innerHTML = "/l2/debug_json";
      content.innerHTML = renderDebugJSON(store.debug_json);
      return;
    case Page.SETTINGS:
      header.innerHTML = "Settings";
      content.innerHTML = "TODO settings";
      return;
  }

  const proj_re = window.location.hash.match(/#\/project\/(\d+)/);
  console.log(proj_re);
  if (proj_re) {
    let p = store.projects[proj_re[1]];
    if (!p) {
      header.innerHTML = renderNoProjectSet();
    } else { 
      const rp = renderProject(p.name, p.owner.name, p.items, p.id === store.active_project);
      header.innerHTML = rp[0];
      content.innerHTML = rp[1];
      content.querySelector("#set_active").addEventListener("click", (evt) => {
        actions.setActiveProject(p.id);
        render();
      });
    }
    return;
  }
 
  console.error("Unknown page " + store.page);
}

export default render;
