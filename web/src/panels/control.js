import { postJson, toast } from '../api.js';

/** @type {Record<string, {main: string, sub?: string}>} */
const LABELS = {
  rx: { main: 'Short', sub: 'Receiver / next frequency' },
  scan: { main: 'Double', sub: 'Scanner' },
  spec: { main: 'Medium', sub: 'Spectrum' },
  wifi: { main: 'Long', sub: 'Wi-Fi screen' },
  rx2: { main: 'Short', sub: 'Next screen' },
  scan2: { main: 'Double' },
  spec2: { main: 'Medium' },
  wifi2: { main: 'Long' },
  rinex: { main: 'Update RS92 RINEX eph' },
  format: { main: 'Format SD card' },
  reboot: { main: 'Reboot' },
};

const GROUPS = [
  { title: 'Button 1', ids: ['rx', 'scan', 'spec', 'wifi'] },
  { title: 'Button 2', ids: ['rx2', 'scan2', 'spec2', 'wifi2'] },
  { title: 'Maintenance', ids: ['rinex', 'format', 'reboot'] },
];

export function mountControl(root, ctx) {
  const actions = new Set((ctx.meta && ctx.meta.actions) || Object.keys(LABELS));

  root.innerHTML = `
    <div class="card">
      <div class="card-title">Remote control</div>
      <p class="muted">Same actions as the physical buttons on the device.</p>
      <div id="ctl-groups"></div>
      <p id="ctl-status" class="muted"></p>
    </div>
  `;

  const host = root.querySelector('#ctl-groups');
  const status = root.querySelector('#ctl-status');

  GROUPS.forEach((g) => {
    const ids = g.ids.filter((id) => actions.has(id));
    if (!ids.length) return;
    const wrap = document.createElement('div');
    wrap.className = 'ctl-group';
    const h = document.createElement('h2');
    h.textContent = g.title;
    wrap.appendChild(h);
    ids.forEach((id) => {
      const label = LABELS[id] || { main: id };
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'ctl-btn';
      if (id === 'reboot' || id === 'format') btn.classList.add('danger');
      const main = document.createElement('span');
      main.className = 'ctl-main';
      main.textContent = label.main;
      btn.appendChild(main);
      if (label.sub) {
        const sub = document.createElement('span');
        sub.className = 'ctl-sub';
        sub.textContent = label.sub;
        btn.appendChild(sub);
      }
      btn.onclick = () => run(id, btn);
      wrap.appendChild(btn);
    });
    host.appendChild(wrap);
  });

  let stopped = false;
  let pollTimer = null;

  /* After Reboot: poll /update.json (no auth, tiny) until a new boot answers.
   * The firmware reports phase "reboot" until it restarts; a fresh boot reports "idle". */
  function waitForReboot() {
    const start = Date.now();
    status.textContent = 'Rebooting…';
    status.className = 'muted';
    async function poll() {
      if (stopped) return;
      let phase = null;
      const ac = new AbortController();
      const t = setTimeout(() => ac.abort(), 2000);
      try {
        const r = await fetch('/update.json', { credentials: 'same-origin', cache: 'no-store', signal: ac.signal });
        phase = (await r.json()).phase || null;
      } catch (e) {
        /* device down or still booting */
      } finally {
        clearTimeout(t);
      }
      if (stopped) return;
      if (phase && phase !== 'reboot') {
        status.textContent = 'Back online';
        status.className = 'ok';
        pollTimer = setTimeout(() => { if (!stopped) location.href = '/'; }, 1000);
        return;
      }
      if (Date.now() - start > 90000) {
        status.textContent = 'Device did not come back. Check power and Wi-Fi.';
        status.className = 'err';
        return;
      }
      pollTimer = setTimeout(poll, 1000);
    }
    pollTimer = setTimeout(poll, 1500);
  }

  async function run(cmd, btn) {
    if (cmd === 'reboot' && !confirm('Reboot the device now?')) return;
    if (cmd === 'format' && !confirm('Format the SD card? This erases all SD data.')) return;
    btn.disabled = true;
    status.textContent = 'Sending…';
    status.className = 'muted';
    try {
      await postJson('/control', { cmd });
      if (cmd === 'reboot') {
        toast('Reboot sent');
        waitForReboot();
        return;
      }
      status.textContent = 'OK';
      status.className = 'ok';
      toast('Command sent');
    } catch (e) {
      if (e.message === 'auth') return;
      /* Network error on reboot (e.g. older firmware restarting before it answers): wait anyway. */
      if (cmd === 'reboot' && e instanceof TypeError) {
        waitForReboot();
        return;
      }
      status.textContent = e.message;
      status.className = 'err';
      btn.disabled = false;
    } finally {
      if (cmd !== 'reboot') btn.disabled = false;
    }
  }

  return () => {
    stopped = true;
    clearTimeout(pollTimer);
  };
}
