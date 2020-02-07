import store from './store.js'
import actions from './actions.js'

function renderControls(onClick) {
  const result = document.createElement("div")
  result.innerHTML = `
    <button id="prev">&lt;</button>
    <button id="next">&gt;</button>
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

const controls = document.querySelector(".controls");
const project = document.querySelector(".project");
function render() {
  let p = store.projects[store.active_project];
  if (!p) {
    project.innerHTML = renderNoProjectSet();
  } else {
    project.innerHTML = renderProject(p.name, p.owner, p.items);
  }
  controls.innerHTML = "";
  controls.appendChild(renderControls(actions.controlClicked));
}

export default render;
