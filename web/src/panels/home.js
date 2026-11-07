import { getJson } from '../api.js';
import { applyTheme } from '../theme.js';

export function mountHome(root, ctx) {
  const theme = localStorage.getItem('rdz-theme') || 'auto';

  root.innerHTML = `
    <div class="card">
      <div class="home-hero">
        <div class="home-hero-main">
          <h1 id="home-title">rdzTTGOSonde</h1>
          <p>Copyright © 2019–2026 by Hansi Reiser, DL9RDZ and contributors</p>
          <p class="muted">Free software under the
            <a href="https://www.gnu.org/licenses/gpl-2.0.txt" target="_blank" rel="noopener">GNU GPL v2</a>.
            See <a href="https://github.com/dl9rdz/rdz_ttgo_sonde/" target="_blank" rel="noopener">GitHub</a>.</p>
          <p><a href="/upd.html">Check for update</a></p>
        </div>
        <div class="home-theme">
          <label for="theme">Theme</label>
          <select id="theme">
            <option value="auto">System</option>
            <option value="light">Light</option>
            <option value="dark">Dark</option>
          </select>
        </div>
      </div>
    </div>
    <div class="card">
      <div class="card-title status-heading">
        Status
        <button type="button" id="home-refresh" class="linkish">[refresh]</button>
      </div>
      <div class="table-wrap">
        <table class="status-table" id="status-table">
          <thead>
            <tr><th>Component</th><th>Status</th></tr>
          </thead>
          <tbody id="status-body">
            <tr><td colspan="2" class="muted">Loading…</td></tr>
          </tbody>
        </table>
      </div>
    </div>
    <div class="card">
      <div class="card-title">External sites</div>
      <div class="link-buttons">
        <a class="button" target="_blank" rel="noopener" href="https://www.wettersonde.net">wettersonde.net</a>
        <a class="button" target="_blank" rel="noopener" href="https://www.radiosondy.info">radiosondy.info</a>
        <a class="button" target="_blank" rel="noopener" href="https://www.sondehub.org">sondehub.org</a>
      </div>
    </div>
  `;

  const title = root.querySelector('#home-title');
  const body = root.querySelector('#status-body');
  const sel = root.querySelector('#theme');
  sel.value = theme;
  sel.onchange = () => {
    localStorage.setItem('rdz-theme', sel.value);
    applyTheme(sel.value);
  };

  function paintMeta(meta) {
    if (!meta) return;
    title.textContent = `${meta.versionName || 'rdzTTGOSonde'} ${meta.versionId || ''}`.trim();
  }

  paintMeta(ctx.meta);

  let abort = null;
  let timer = null;
  let stopped = false;

  async function refresh() {
    if (abort) abort.abort();
    abort = new AbortController();
    try {
      const data = await getJson('/status.json', { signal: abort.signal });
      if (data.meta) {
        ctx.onMeta(data.meta);
        paintMeta(data.meta);
      }
      body.innerHTML = '';
      let rows = 0;
      for (const [key, val] of Object.entries(data)) {
        if (key === 'meta') continue;
        const tr = document.createElement('tr');
        tr.innerHTML =
          `<td class="status-name">${escapeHtml(key)}</td>` +
          `<td class="status-html">${val}</td>`;
        body.appendChild(tr);
        rows++;
      }
      if (!rows) {
        body.innerHTML = '<tr><td colspan="2" class="muted">No status</td></tr>';
      }
    } catch (e) {
      if (e.name === 'AbortError' || e.message === 'auth') return;
      body.innerHTML = `<tr><td colspan="2" class="err">${escapeHtml(e.message)}</td></tr>`;
    }
  }

  function tick() {
    if (stopped) return;
    refresh().finally(() => {
      if (!stopped) timer = setTimeout(tick, 5000);
    });
  }

  root.querySelector('#home-refresh').onclick = () => refresh();
  tick();

  return () => {
    stopped = true;
    if (timer) clearTimeout(timer);
    if (abort) abort.abort();
  };
}

function escapeHtml(s) {
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}
