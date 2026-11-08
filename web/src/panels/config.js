import { getJson, postForm } from '../api.js';

function escapeHtml(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
}

const CFGS = [
  ['', 'General configuration', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/General-configuration'],
  ['wifi', 'Wifi mode (0=off, 1=client, 2=AP, 3=client or AP, 4=client-noscan)'],
  ['mdnsname', 'Network mDNS name'],
  ['ephftp', 'FTP server for ephemeris data (RS92 decoder)'],
  ['debug', 'Debug level (0=err/1=warn/2=info/3=all;+10=color)'],
  ['maxsonde', 'Maximum number of QRG entries (must be ≤ 50)'],
  ['rxlat', 'Receiver fixed latitude'],
  ['rxlon', 'Receiver fixed longitude'],
  ['rxalt', 'Receiver fixed altitude'],
  ['b2mute', 'Button 2/medium press mutes LED/Buzzer (minutes)'],
  ['', 'OLED/TFT display configuration', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Display-configuration'],
  ['screenfile', 'Screen config (0=automatic; 1-5=predefined; other=custom)'],
  ['display', 'Display screens (scan, default, ...)'],
  ['dispsaver', 'Display saver (0=never/1=always/2=ifnorx [+10*n: after n sec.])'],
  ['dispcontrast', 'OLED contrast (-1=use default; 0..255=set contrast)'],
  ['norx_timeout', 'No-RX-timeout in seconds (-1=disabled)'],
  ['tft_orient', 'TFT orientation (0/1/2/3), OLED flip: 3'],
  ['', 'Spectrum display configuration', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Spectrum-configuration'],
  ['spectrum', 'Show spectrum on start (-1=no, 0=forever, >0=time [sec])'],
  ['startfreq', 'Start frequency (MHz, default 400)'],
  ['channelbw', 'Bandwidth (kHz)'],
  ['marker', 'Spectrum MHz marker'],
  ['noisefloor', 'Spectrum noisefloor'],
  ['', 'Receiver configuration', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Receiver-configuration'],
  ['freqofs', 'RX frequency offset (Hz)'],
  ['lnaboost', 'LNA boost (0=off, 1=on)'],
  ['rs41.agcbw', 'RS41 AGC bandwidth'],
  ['rs41.rxbw', 'RS41 RX bandwidth'],
  ['rs92.rxbw', 'RS92 RX (and AGC) bandwidth'],
  ['rs92.alt2d', 'RS92 2D fix default altitude'],
  ['dfm.agcbw', 'DFM AGC bandwidth'],
  ['dfm.rxbw', 'DFM RX bandwidth'],
  ['m10m20.agcbw', 'M10/M20 AGC bandwidth'],
  ['m10m20.rxbw', 'M10/M20 RX bandwidth'],
  ['mp3h.agcbw', 'MP3H AGC bandwidth'],
  ['mp3h.rxbw', 'MP3H RX bandwidth'],
  ['', 'KISS TNC/AXUDP/AXTCP data feed configuration', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Data-feed-configuration'],
  ['call', 'Call'],
  ['passcode', 'Passcode'],
  ['kisstnc.active', 'KISS TNC (port 14590) (needs reboot)'],
  ['axudp.active', 'AXUDP active'],
  ['axudp.host', 'AXUDP host[:port]'],
  ['axudp.ratelimit', 'Rate limit'],
  ['tcp.active', 'APRS TCP active'],
  ['tcp.timeout', 'APRS TCP timeout [s] (0=off, 25=on)'],
  ['tcp.host', 'APRS TCP host[:port] (default port 14580)'],
  ['tcp.host2', 'APRS TCP host2[:port]'],
  ['tcp.highrate', 'Rate limit'],
  ['tcp.objcall', 'APRS object call'],
  ['tcp.beaconsym', 'APRS tracker symbol'],
  ['tcp.chase', 'APRS location reporting (0=off, 1=fixed, 2=chase/GPS, 3=auto)'],
  ['tcp.comment', 'APRS location comment'],
  ['', 'MQTT data feed configuration', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/MQTT-configuration'],
  ['mqtt.active', 'MQTT message selection bitfield: 1=Sondes, 2=Uptime, 4=PMU, 8=GPS, 16=Scanner/Spectrum Peak, 128=Debug. 0 to disable MQTT (needs reboot)'],
  ['mqtt.id', 'MQTT client ID'],
  ['mqtt.host', 'MQTT server hostname'],
  ['mqtt.port', 'MQTT port'],
  ['mqtt.username', 'MQTT username'],
  ['mqtt.password', 'MQTT password'],
  ['mqtt.prefix', 'MQTT prefix'],
  ['mqtt.report_interval', 'MQTT reporting interval (ms)'],
  ['', 'Chasemapper settings', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Chasemapper-configuration'],
  ['cm.active', 'Chasemapper active (0=disabled, 1=active)'],
  ['cm.host', 'Chasemapper UDP host'],
  ['cm.port', 'Chasemapper UDP port'],
  ['', 'Sondeseeker settings', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Sondeseeker-configuration'],
  ['ss.active', 'Sondeseeker active (0=disabled, 1=active)'],
  ['ss.host', 'Sondeseeker UDP host'],
  ['ss.port', 'Sondeseeker UDP port'],
  ['', 'SondeHub settings', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/SondeHub-settings'],
  ['sondehub.active', 'SondeHub reporting (0=disabled, 1=active)'],
  ['sondehub.chase', 'SondeHub location reporting (0=off, 1=fixed, 2=chase/GPS, 3=auto)'],
  ['sondehub.host', 'SondeHub host (DO NOT CHANGE)'],
  ['sondehub.callsign', 'Callsign'],
  ['sondehub.antenna', 'Antenna (optional, visible on SondeHub tracker)'],
  ['sondehub.email', 'SondeHub email (optional, only used to contact in case of upload errors)'],
  ['', 'SondeHub frequency import', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/SondeHub-import'],
  ['sondehub.fiactive', 'SondeHub frequency import active (0=disabled, 1=active)'],
  ['sondehub.fiinterval', 'Import frequency (minutes, ≥ 5)'],
  ['sondehub.fimaxdist', 'Import maximum distance (km, ≤ 700)'],
  ['sondehub.fimaxage', 'Import maximum age (hours, ≤ 48)'],
  ['', 'SD card logger configuration', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/SDcard-configuration'],
  ['sd.cs', 'SD card CS'],
  ['sd.miso', 'SD card MISO/DI'],
  ['sd.mosi', 'SD card MOSI/DO'],
  ['sd.clk', 'SD card CLK'],
  ['sd.sync', 'SD card sync interval [s]'],
  ['sd.name', 'SD card naming (0=flat, 1=YYMM folders)'],
  ['sd.speed', 'SD card SPI speed (Hz, 0=default, e.g. 4000000, 8000000, 16000000)'],
  ['', 'Hardware configuration (requires reboot)', 'https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Hardware-configuration'],
  ['disptype', 'Display type (0=OLED/SSD1306, 1=ILI9225, 2=OLED/SH1106, 3=ILI9341, 4=ILI9342, 5=ST7789, 6=ST7796)'],
  ['oled_sda', 'OLED SDA/TFT SDA'],
  ['oled_scl', 'OLED SCL/TFT CLK'],
  ['oled_rst', 'OLED RST/TFT RST (needs reboot)'],
  ['tft_rs', 'TFT RS'],
  ['tft_cs', 'TFT CS'],
  ['tft_spifreq', 'TFT SPI speed'],
  ['button_pin', 'Button input port'],
  ['button2_pin', 'Button 2 input port'],
  ['button2_axp', 'Use AXP192 PWR as Button 2'],
  ['touch_thresh', 'Touch button threshold (0 for calib mode)'],
  ['power_pout', 'Power control port'],
  ['led_pout', 'LED output port'],
  ['gps_rxd', 'GPS RXD pin (-1 to disable)'],
  ['gps_txd', 'GPS TXD pin (optional, only for GPS reset)'],
  ['batt_adc', 'Battery measurement pin'],
  ['sx1278_ss', 'SX1278 SS'],
  ['sx1278_miso', 'SX1278 MISO'],
  ['sx1278_mosi', 'SX1278 MOSI'],
  ['sx1278_sck', 'SX1278 SCK'],
];

const TOCHECK = [
  'sd.cs', 'sd.miso', 'sd.mosi', 'sd.clk', 'oled_sda', 'oled_scl', 'oled_rst',
  'tft_rs', 'tft_cs', 'tft_spifreq', 'button_pin', 'button2_pin', 'led_pout',
  'gps_rxd', 'gps_txd', 'batt_adc', 'sx1278_ss', 'sx1278_miso', 'sx1278_mosi', 'sx1278_sck',
];

const ALLOWED_DUPS = [
  ['sd.mosi', 'oled_sda'],
  ['sd.clk', 'oled_scl'],
];

function labelOf(key) {
  const row = CFGS.find((item) => item[0] === key);
  return row ? row[1] : key;
}

function pinValue(root, name) {
  const el = root.querySelector(`input[data-key="${CSS.escape(name)}"]`);
  if (!el) return NaN;
  return parseInt(el.value, 10);
}

function duplicateWarning(root) {
  const seen = new Map();
  const dups = [];
  for (const name of TOCHECK) {
    const value = pinValue(root, name);
    if (!Number.isFinite(value) || value === -1) continue;
    const prev = seen.get(value);
    if (prev) {
      const allowed = ALLOWED_DUPS.some(
        (pair) => (pair[0] === prev && pair[1] === name) || (pair[0] === name && pair[1] === prev),
      );
      if (!allowed) dups.push({ value, a: labelOf(prev), b: labelOf(name) });
    } else {
      seen.set(value, name);
    }
  }
  if (!dups.length) return true;
  const lines = dups.map((d) => `Pin ${d.value} in '${d.a}' and '${d.b}'`).join('\n');
  return confirm(`Duplicated PIN assignments found:\n${lines}\n\nDo you want to save anyway?`);
}

function formParams(root) {
  const params = {};
  root.querySelectorAll('input[data-key]').forEach((el) => {
    if (el.type === 'checkbox') {
      if (el.checked) params[`${el.dataset.key}#`] = 'on';
      return;
    }
    params[el.dataset.key] = el.value;
  });
  return params;
}

function snapshot(root) {
  return JSON.stringify(formParams(root));
}

function buttonValue(raw) {
  const n = parseInt(raw, 10);
  if (!Number.isFinite(n) || n === -1) return { value: raw == null ? '' : String(raw), touch: false };
  return { value: String(n & 127), touch: (n & 128) !== 0 };
}

function screenNote(data) {
  const idx = Number(data.screenIndex);
  const file = Number.isFinite(idx) ? `/screens${idx}.txt` : 'screens file';
  const layouts = Array.isArray(data.layouts) ? data.layouts : [];
  const lines = layouts.map((label, i) => `${i}=${label}`);
  return [`Using ${file}`, ...lines].join('\n');
}

export function mountConfig(root, ctx) {
  let alive = true;
  let baseline = '';

  root.innerHTML = `
    <div class="card">
      <h2>Config</h2>
      <div id="config-body"><p class="muted">Loading…</p></div>
    </div>`;

  const body = root.querySelector('#config-body');

  function mark() {
    const dirty = snapshot(root) !== baseline;
    ctx.setSaveBar({
      disabled: !dirty,
      status: dirty ? 'Unsaved changes' : '',
      statusClass: 'muted',
      onSave: save,
    });
  }

  function render(data) {
    const values = data.values || {};
    const rows = [];
    for (const item of CFGS) {
      const key = item[0];
      if (!key) {
        const wiki = item[2]
          ? ` <a href="${escapeHtml(item[2])}" target="_blank" rel="noopener">[wiki]</a>`
          : '';
        rows.push(`<tr class="cfg-sec"><th colspan="2">${escapeHtml(item[1])}${wiki}</th></tr>`);
        continue;
      }
      const raw = values[key] == null ? '' : String(values[key]);
      if (key === 'button_pin' || key === 'button2_pin') {
        const pin = buttonValue(raw);
        rows.push(`<tr><td class="cfg-name">${escapeHtml(item[1])}</td><td>
          <input class="cfg-pin" type="text" data-key="${escapeHtml(key)}" value="${escapeHtml(pin.value)}">
          <label class="cfg-touch"><input type="checkbox" data-key="${escapeHtml(key)}"${pin.touch ? ' checked' : ''}> Touch</label>
        </td></tr>`);
      } else {
        rows.push(`<tr><td class="cfg-name">${escapeHtml(item[1])}</td><td><input type="text" data-key="${escapeHtml(key)}" value="${escapeHtml(raw)}"></td></tr>`);
      }
      if (key === 'display') {
        rows.push(`<tr><td class="cfg-note" colspan="2">${escapeHtml(screenNote(data))}</td></tr>`);
      }
    }
    body.innerHTML = `<table class="form"><tbody>${rows.join('')}</tbody></table>`;
    baseline = snapshot(root);
    body.querySelectorAll('input').forEach((el) => el.addEventListener('input', mark));
    body.querySelectorAll('input[type="checkbox"]').forEach((el) => el.addEventListener('change', mark));
    mark();
  }

  async function save() {
    if (!duplicateWarning(root)) return;
    ctx.setSaveBar({ disabled: true, status: 'Saving…', statusClass: 'muted', onSave: save });
    try {
      await postForm('/config.html', formParams(root));
      if (!alive) return;
      ctx.setSaveBar({ disabled: true, status: 'Saved', statusClass: 'ok', onSave: save });
      const data = await getJson('/config.json');
      if (alive) render(data);
    } catch (err) {
      if (!alive || err.message === 'auth') return;
      ctx.setSaveBar({
        disabled: false,
        status: err.message === 'HTTP 401' ? 'Login required' : 'Save failed',
        statusClass: 'err',
        onSave: save,
      });
    }
  }

  ctx.setSaveBar({ disabled: true, status: '', onSave: save });

  getJson('/config.json')
    .then((data) => {
      if (alive) render(data);
    })
    .catch((err) => {
      if (!alive) return;
      body.innerHTML = `<p class="err">${escapeHtml(err.message || 'Load failed')}</p>`;
    });

  return () => {
    alive = false;
    ctx.setSaveBar(null);
  };
}
