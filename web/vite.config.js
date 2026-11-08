import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { defineConfig } from 'vite';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

/** Device IP for `npm run dev` proxy. Override: VITE_DEVICE=http://10.0.0.12 npm run dev */
function deviceTarget() {
  let t = process.env.VITE_DEVICE || 'http://192.168.4.1';
  if (!/^https?:\/\//i.test(t)) {
    t = `http://${t}`;
  }
  return t.replace(/\/$/, '');
}

const PROXY_PATHS = [
  '/status.json',
  '/meta.json',
  '/decoder.json',
  '/live.json',
  '/qrg.json',
  '/qrg.html',
  '/wifi.json',
  '/control',
  '/preauth',
  '/config.html',
  '/config.json',
  '/file',
  '/files.json',
  '/sd',
  '/sdrm.html',
  /* GET login/upd/sdfiles + livemap.* / style.css are served locally (see littlefsStaticDev). */
  '/login.html',
  '/logout',
  '/update.html',
  '/edit.html',
  '/cfg.js',
  '/rdz.js',
  '/sha256.min.js',
];

function proxyTable(target) {
  /** @type {Record<string, object>} */
  const table = {};
  for (const path of PROXY_PATHS) {
    table[path] = {
      target,
      changeOrigin: true,
      secure: false,
      configure(proxy) {
        proxy.on('error', (err, req, res) => {
          console.warn(`[proxy] ${req?.url || '?'}: ${err.message}`);
          if (res && typeof res.writeHead === 'function' && !res.headersSent) {
            res.writeHead(502, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ error: 'device unreachable', detail: err.message }));
          }
        });
      },
    };
  }
  return table;
}

/** Production only: ESP32 gets a classic IIFE script. Dev must keep type=module. */
function classicScript() {
  return {
    name: 'classic-script',
    transformIndexHtml: {
      order: 'post',
      handler(html, ctx) {
        if (ctx.server) {
          return html;
        }
        return html
          .replace(/\s+type="module"/g, '')
          .replace(/\s+crossorigin(?:="[^"]*")?/g, '')
          .replace(
            /<script src="(\.\/app(?:\.[^"]+)?\.js)"><\/script>/,
            '<script defer src="$1"></script>',
          );
      },
    },
  };
}

/**
 * Serve LittleFS-backed static pages from the repo in dev (no uploadfs needed).
 * - web/: login, upd, sdfiles (rewrite hashed app*.css → /src/style.css)
 * - RX_FSK/data/: livemap.html / livemap.js / style.css
 * POST /login.html still reaches the device via proxy.
 */
function littlefsStaticDev() {
  const webPages = new Set(['/login.html', '/upd.html', '/sdfiles.html']);
  const dataFiles = new Map([
    ['/livemap.html', 'text/html; charset=utf-8'],
    ['/livemap.js', 'text/javascript; charset=utf-8'],
    ['/style.css', 'text/css; charset=utf-8'],
  ]);
  const assetTypes = {
    '.png': 'image/png',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon',
    '.json': 'application/json',
  };
  const dataRoot = path.resolve(__dirname, '../RX_FSK/data');
  return {
    name: 'littlefs-static-dev',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const urlPath = (req.url || '').split('?')[0];
        if (req.url && /^\/app(?:\.[a-zA-Z0-9_-]+)?\.css(?:\?.*)?$/.test(req.url)) {
          req.url = '/src/style.css';
          next();
          return;
        }
        if (req.method === 'GET' && webPages.has(urlPath)) {
          const file = path.join(__dirname, urlPath.slice(1));
          if (!fs.existsSync(file)) {
            next();
            return;
          }
          let html = fs.readFileSync(file, 'utf8');
          html = html.replace(
            /href="\.\/app(?:\.[a-zA-Z0-9_-]+)?\.css"/g,
            'href="/src/style.css"',
          );
          res.setHeader('Content-Type', 'text/html; charset=utf-8');
          res.end(html);
          return;
        }
        if (req.method === 'GET' && dataFiles.has(urlPath)) {
          const file = path.join(dataRoot, urlPath.slice(1));
          if (!fs.existsSync(file)) {
            next();
            return;
          }
          res.setHeader('Content-Type', dataFiles.get(urlPath));
          res.setHeader('Cache-Control', 'no-store');
          fs.createReadStream(file).pipe(res);
          return;
        }
        if (req.method === 'GET' && urlPath.startsWith('/') && !urlPath.slice(1).includes('/')) {
          const base = urlPath.slice(1);
          const ext = path.extname(base).toLowerCase();
          const ctype = assetTypes[ext];
          const file = path.join(dataRoot, base);
          if (ctype && fs.existsSync(file)) {
            res.setHeader('Content-Type', ctype);
            res.setHeader('Cache-Control', 'no-store');
            fs.createReadStream(file).pipe(res);
            return;
          }
        }
        next();
      });
    },
  };
}

export default defineConfig(({ command }) => {
  const target = deviceTarget();
  if (command === 'serve') {
    console.log(`[vite] proxy → ${target}`);
  }
  return {
    root: '.',
    /* Relative base breaks Vite HMR/client in dev; keep it for LittleFS builds. */
    base: command === 'build' ? './' : '/',
    plugins: [classicScript(), littlefsStaticDev()],
    build: {
      outDir: 'dist',
      emptyOutDir: true,
      cssCodeSplit: false,
      assetsInlineLimit: 100000,
      modulePreload: false,
      target: 'es2018',
      rollupOptions: {
        input: 'index.html',
        output: {
          format: 'iife',
          /* Stable LittleFS names; copy-to-data rewrites HTML to app.<hash>.* for cache busting. */
          entryFileNames: 'app.js',
          chunkFileNames: 'app.js',
          assetFileNames: 'app[extname]',
          inlineDynamicImports: true,
        },
      },
    },
    server: {
      proxy: proxyTable(target),
    },
  };
});
