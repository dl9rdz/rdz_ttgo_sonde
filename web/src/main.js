import './style.css';
import { getJson, sessionUser } from './api.js';
import { applyTheme } from './theme.js';
import { mountHome } from './panels/home.js';
import { mountDecoder } from './panels/decoder.js';
import { mountQrg } from './panels/qrg.js';
import { mountWifi } from './panels/wifi.js';
import { mountControl } from './panels/control.js';

applyTheme();

let saveAction = null;

const ctx = {
  meta: null,
  onMetaPaint: null,
  onMeta(meta) {
    ctx.meta = meta || ctx.meta;
    if (typeof ctx.onMetaPaint === 'function') ctx.onMetaPaint();
    /* Refresh auth bar version text if visible. */
    if (!saveAction) paintAuthBar();
  },
  /** @param {null|{disabled?:boolean,status?:string,statusClass?:string,onSave:Function}} opts */
  setSaveBar(opts) {
    if (!opts) {
      saveAction = null;
      paintAuthBar();
      return;
    }
    saveAction = opts.onSave || null;
    paintSaveBar(opts);
  },
};

const panels = {
  home: document.getElementById('panel-home'),
  decoder: document.getElementById('panel-decoder'),
  map: document.getElementById('panel-map'),
  qrg: document.getElementById('panel-qrg'),
  wifi: document.getElementById('panel-wifi'),
  config: document.getElementById('panel-config'),
  control: document.getElementById('panel-control'),
};

const mounts = {
  home: (el) => mountHome(el, ctx),
  decoder: mountDecoder,
  qrg: (el) => mountQrg(el, ctx),
  wifi: (el) => mountWifi(el, ctx),
  control: (el) => mountControl(el, ctx),
};

let unmount = null;
const configFrame = panels.config.querySelector('iframe');
const mapFrame = panels.map.querySelector('iframe');

function paintAuthBar() {
  const left = document.getElementById('bar-left');
  const right = document.getElementById('bar-right');
  const user = sessionUser();
  left.innerHTML = user
    ? `<span class="bar-user">${user}</span> <a href="/logout">Logout</a>`
    : `<a class="button primary bar-save" href="login.html">Login</a>`;
  const ver = (ctx.meta && (ctx.meta.versionId || ctx.meta.fullId)) || '';
  right.textContent = ver ? `rdzTTGOsonde ${ver}` : 'rdzTTGOsonde';
}

function paintSaveBar({ disabled = true, status = '', statusClass = 'muted' } = {}) {
  const left = document.getElementById('bar-left');
  const right = document.getElementById('bar-right');
  left.innerHTML = `<button type="button" class="primary bar-save" id="bar-save"${disabled ? ' disabled' : ''}>Save changes</button>`;
  right.innerHTML = `<span class="status-msg ${statusClass}">${status || ''}</span>`;
  const btn = document.getElementById('bar-save');
  btn.onclick = () => {
    if (typeof saveAction === 'function') saveAction();
  };
}

function setNav(name) {
  document.querySelectorAll('.nav-item[data-panel]').forEach((el) => {
    el.classList.toggle('active', el.dataset.panel === name);
  });
}

function show(name) {
  if (!panels[name]) name = 'home';
  if (unmount) {
    unmount();
    unmount = null;
  }
  saveAction = null;

  Object.entries(panels).forEach(([key, el]) => {
    const on = key === name;
    el.hidden = !on;
    el.classList.toggle('active', on);
  });

  if (name === 'config') {
    if (!configFrame.getAttribute('src')) configFrame.setAttribute('src', 'config.html');
  } else {
    configFrame.setAttribute('src', '');
  }

  if (name === 'map') {
    if (!mapFrame.getAttribute('src')) mapFrame.setAttribute('src', 'livemap.html');
  }

  if (mounts[name]) {
    unmount = mounts[name](panels[name], ctx) || null;
  }

  /* Form panels that own the save bar already called setSaveBar; others get auth. */
  if (!saveAction) paintAuthBar();

  const main = document.querySelector('.main');
  const fixed = name === 'map' || name === 'config';
  main.classList.toggle('main-fixed', fixed);

  setNav(name);
  document.getElementById('nav').classList.remove('open');
}

function routeFromHash() {
  const name = (location.hash || '#home').replace(/^#/, '') || 'home';
  show(name);
}

document.getElementById('nav-toggle').onclick = () => {
  document.getElementById('nav').classList.toggle('open');
};

document.querySelectorAll('.nav-item[data-panel]').forEach((btn) => {
  btn.addEventListener('click', () => {
    location.hash = btn.dataset.panel;
  });
});

window.addEventListener('hashchange', routeFromHash);

paintAuthBar();
getJson('/meta.json')
  .then((data) => ctx.onMeta(data))
  .catch(() =>
    getJson('/status.json')
      .then((data) => {
        if (data.meta) ctx.onMeta(data.meta);
      })
      .catch(() => {}),
  )
  .finally(routeFromHash);
