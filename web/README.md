# Device web UI

Vite source for the on-device LittleFS pages. Production output is copied into `RX_FSK/data/`.

## Develop against a board

```bash
cd web
npm install
VITE_DEVICE=http://192.168.4.1 npm run dev
```

Open the printed local URL. API/JSON and control endpoints are proxied to the device.
Map assets (`livemap.html` / `livemap.js` / `style.css`) and standalone pages
(`login.html` / `upd.html` / `sdfiles.html`) are served from the repo so edits show
without `uploadfs`. Flash the filesystem before testing those same files on-device.

## Build into LittleFS

```bash
cd web
npm run build
```

Writes `index.html`, `app.js`, and `app.css` into `RX_FSK/data/`. Then flash the filesystem (`pio run -t uploadfs`).

`style.css`, `edit.html`, `login.html`, `livemap.*`, `cfg.js`, and `rdz.js` stay as separate LittleFS files for the config iframe and map.
