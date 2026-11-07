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
  '/file',
  '/files.json',
  '/sd',
  '/sdrm.html',
  '/livemap.html',
  '/livemap.js',
  '/style.css',
  /* GET login/upd/sdfiles are served locally; POST /login.html still proxies for auth. */
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
 * Serve local login/upd/sdfiles HTML in dev (so edits show without uploadfs).
 * POST /login.html still reaches the device via proxy.
 * Map hashed app*.css to the live source stylesheet.
 */
function standalonePagesDev() {
  const pages = new Set(['/login.html', '/upd.html', '/sdfiles.html']);
  return {
    name: 'standalone-pages-dev',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const urlPath = (req.url || '').split('?')[0];
        if (req.url && /^\/app(?:\.[a-zA-Z0-9_-]+)?\.css(?:\?.*)?$/.test(req.url)) {
          req.url = '/src/style.css';
          next();
          return;
        }
        if (req.method === 'GET' && pages.has(urlPath)) {
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
    plugins: [classicScript(), standalonePagesDev()],
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
