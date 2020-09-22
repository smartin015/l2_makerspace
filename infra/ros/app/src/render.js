import {store, Page} from './store.js'
import actions from './actions.js'

function renderControls(onClick, page) {
  const result = document.createElement("div");
  result.innerHTML = `
    <button id="project_list" ${(page === Page.PROJECT_LIST) ? 'disabled' : ''}><img class="svg" src="img/view_list-white-18dp.svg"/></button>
    <button id="estop"><img class="svg" src="img/report-white-18dp.svg"/></button>
    <button id="debug_json" ${(page === Page.DEBUG_JSON) ? 'disabled' : ''}><img class="svg" src="img/notes-white-18dp.svg"/></button>
    <button id="settings" ${(page === Page.SETTINGS) ? 'disabled' : ''}><img class="svg" src="img/settings-white-18dp.svg"/></button>
  `;
  for (let c of result.children) {
    c.addEventListener("click", onClick);
  }
  return result;
}

function renderProjects(projects) {
  projects = Object.values(projects);
  return `<div>
    ${projects.map((item) => renderProjectThumbnail(
      item.id, item.name, item.owner.name, item.items)).join("\n")}
  </div>`;
}

function renderProjectThumbnail(id, name, owner, items) {
  return `<a class="project_thumb" href="#/project/${id}">
      <div class="projectname">${name}</div>
      <div class="projectowner">${owner}</div>
      <div class="itemcount">${items.length} items</div>
    </a>`;
}

function renderProject(name, owner, items) {
  return [name, `<div>
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
  controls.innerHTML = "";
  status.innerHTML = (store.connected) ? "Connected" : "Not connected";
  controls.appendChild(renderControls((evt) => {
    actions.controlClicked(evt);
    render();
  }, window.location.hash));
  
  switch (window.location.hash) {
    case Page.PROJECT_LIST:
      header.innerHTML = "Projects";
      content.innerHTML = renderProjects(store.projects);
      for (let c of content.children) {
        c.addEventListener("click", (evt) => {
          actions.projectClicked(evt);
          render();
        });
      }
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
      const rp = renderProject(p.name, p.owner.name, p.items);
      header.innerHTML = rp[0];
      content.innerHTML = rp[1];
    }
    return;
  }
 
  console.error("Unknown page " + store.page);
}

export default render;
