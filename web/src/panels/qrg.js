import { getJson, postForm, toast } from '../api.js';

const STYPES = [
  ['4', 'RS41'],
  ['R', 'RS92'],
  ['D', 'DFM'],
  ['M', 'M10/M20'],
  ['3', 'MP3H'],
];

export function mountQrg(root, ctx) {
  root.innerHTML = `
    <div class="card">
      <div class="card-title">QRG channels</div>
      <p class="muted">Active channels are scanned / decoded. Save writes qrg.txt on the device.</p>
      <div class="table-wrap" id="qrg-table"><p class="muted">Loading…</p></div>
    </div>
  `;

  const tableHost = root.querySelector('#qrg-table');
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

  function typeOptions(selected) {
    let sel = selected === '2' ? 'M' : selected;
    return STYPES.map(
      ([v, label]) =>
        `<option value="${v}"${v === sel ? ' selected' : ''}>${label}</option>`,
    ).join('');
  }

  function render(channels) {
    let html =
      '<table class="form"><thead><tr><th>Ch</th><th>Active</th><th>Frequency</th><th>Decoder</th><th>Launchsite</th></tr></thead><tbody>';
    channels.forEach((ch, i) => {
      const n = i + 1;
      const type = ch.typeCode || '4';
      html += `<tr>
        <td class="ch">${n}</td>
        <td class="act"><input name="A${n}" type="checkbox" ${ch.active ? 'checked' : ''}></td>
        <td><input name="F${n}" type="text" value="${escapeAttr(String(ch.freq))}"></td>
        <td><select name="T${n}">${typeOptions(type)}</select></td>
        <td><input name="S${n}" type="text" value="${escapeAttr(ch.launchsite || '')}"></td>
      </tr>`;
    });
    html += '</tbody></table>';
    tableHost.innerHTML = html;
    tableHost.querySelectorAll('input, select').forEach((el) => {
      el.addEventListener('input', () => setDirty(true));
      el.addEventListener('change', () => setDirty(true));
    });
    setDirty(false);
  }

  async function load() {
    statusText = '';
    statusClass = 'muted';
    syncBar();
    try {
      const data = await getJson('/qrg.json');
      render(data.channels || []);
    } catch (e) {
      if (e.message === 'auth') return;
      tableHost.innerHTML = `<p class="err">${escapeHtml(e.message)}</p>`;
    }
  }

  async function doSave() {
    const params = {};
    tableHost.querySelectorAll('input, select').forEach((el) => {
      if (!el.name) return;
      if (el.type === 'checkbox') {
        if (el.checked) params[el.name] = 'on';
      } else {
        params[el.name] = el.value;
      }
    });
    dirty = false;
    statusText = 'Saving…';
    statusClass = 'muted';
    syncBar();
    try {
      await postForm('/qrg.html', params);
      statusText = 'Saved';
      statusClass = 'ok';
      toast('QRG saved');
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
