import {store, Page} from './store.js'
import actions from './actions.js'

function renderControls(onClick, page) {
  const result = document.createElement("div");
  result.innerHTML = `
    <button id="prev"><img class="svg" src="img/keyboard_arrow_left-white-18dp.svg"/></button>
    <button id="project_list" ${(page === Page.PROJECT_LIST) ? 'disabled' : ''}><img class="svg" src="img/view_list-white-18dp.svg"/></button>
    <button id="estop"><img class="svg" src="img/report-white-18dp.svg"/></button>
    <button id="debug_json" ${(page === Page.DEBUG_JSON) ? 'disabled' : ''}><img class="svg" src="img/notes-white-18dp.svg"/></button>
    <button id="settings" ${(page === Page.SETTINGS) ? 'disabled' : ''}><img class="svg" src="img/settings-white-18dp.svg"/></button>
    <button id="next"><img class="svg" src="img/keyboard_arrow_right-white-18dp.svg"/></button>
  `;
  for (let c of result.children) {
    c.addEventListener("click", onClick);
  }
  return result;
}

function renderProject(name, owner, items) {
  return `<div>
      <div class="projectname">${name}</div>
      <div class="projectowner">${owner}</div>
      <ul>
        ${items.map((item) => `<li>${item.content}</li>`).join("\n")}
      </ul>
    </div>`;
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
  }, store.page));
  
  switch (store.page) {
    case Page.PROJECT_LIST:
      header.innerHTML = "Projects";
      content.innerHTML = "todo project list";
      break;
    case Page.PROJECT_DETAILS:
      let p = store.projects[store.active_project];
      if (!p) {
        header.innerHTML = renderNoProjectSet();
      } else {
        header.innerHTML = renderProject(p.name, p.owner, p.items);
      }
      content.innerHTML = "todo";
      break;
    case Page.DEBUG_JSON:
      header.innerHTML = "/l2/debug_json";
      content.innerHTML = renderDebugJSON(store.debug_json);
      break;
    case Page.SETTINGS:
      header.innerHTML = "Settings";
      content.innerHTML = "TODO settings";
      break;
    default:
      console.error("Unknown page " + store.page);
  }
}

export default render;
