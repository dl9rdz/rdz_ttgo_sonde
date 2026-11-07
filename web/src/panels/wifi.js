import { getJson, putJson, toast } from '../api.js';

export function mountWifi(root, ctx) {
  root.innerHTML = `
    <div class="card">
      <div class="card-title">Wi-Fi networks</div>
      <p class="muted">AP config is used in access point mode. Station mode tries all other SSID slots.</p>
      <div class="table-wrap" id="wifi-list"><p class="muted">Loading…</p></div>
    </div>
  `;

  const list = root.querySelector('#wifi-list');
  let slots = [];
  let dirty = false;
  let statusText = '';
  let statusClass = 'muted';

  function syncBar() {
    ctx.setSaveBar({
      disabled: !dirty,
      status: statusText,
      statusClass,
      onSave: doSave,
    });
  }

  function setDirty(v) {
    dirty = v;
    syncBar();
  }

  function render() {
    let html =
      '<table class="form"><thead><tr><th>Slot</th><th>SSID</th><th>Password</th></tr></thead><tbody>';
    slots.forEach((s, i) => {
      const title = i === 0 ? 'AP' : String(i);
      html += `<tr>
        <td class="ch">${title}</td>
        <td><input data-i="${i}" data-f="ssid" type="text" value="${escapeAttr(s.ssid || '')}" autocomplete="off"></td>
        <td><input data-i="${i}" data-f="password" type="text" value="${escapeAttr(s.password || '')}" autocomplete="off"></td>
      </tr>`;
    });
    html += '</tbody></table>';
    list.innerHTML = html;
    list.querySelectorAll('input').forEach((el) => {
      el.addEventListener('input', () => {
        const i = Number(el.dataset.i);
        slots[i][el.dataset.f] = el.value;
        setDirty(true);
      });
    });
    setDirty(false);
  }

  async function load() {
    statusText = '';
    statusClass = 'muted';
    syncBar();
    try {
      const data = await getJson('/wifi.json');
      slots = (data.slots || []).map((s, i) => ({
        role: s.role || (i === 0 ? 'ap' : 'sta'),
        ssid: s.ssid || '',
        password: s.password || '',
      }));
      render();
    } catch (e) {
      if (e.message === 'auth') return;
      list.innerHTML = `<p class="err">${escapeHtml(e.message)}</p>`;
    }
  }

  async function doSave() {
    dirty = false;
    statusText = 'Saving…';
    statusClass = 'muted';
    syncBar();
    try {
      await putJson('/wifi.json', { slots });
      statusText = 'Saved';
      statusClass = 'ok';
      toast('Wi-Fi saved');
      syncBar();
      await load();
    } catch (e) {
      if (e.message === 'auth') return;
      dirty = true;
      statusText = e.message;
      statusClass = 'err';
      syncBar();
    }
  }

  syncBar();
  load();
  return () => ctx.setSaveBar(null);
}

function escapeAttr(s) {
  return String(s).replace(/&/g, '&amp;').replace(/"/g, '&quot;').replace(/</g, '&lt;');
}

function escapeHtml(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}
