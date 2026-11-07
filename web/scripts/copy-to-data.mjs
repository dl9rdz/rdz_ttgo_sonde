import crypto from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const webRoot = path.resolve(__dirname, '..');
const dist = path.resolve(webRoot, 'dist');
const data = path.resolve(webRoot, '../RX_FSK/data');

if (!fs.existsSync(dist)) {
  console.error('web/dist missing — run vite build first');
  process.exit(1);
}

/** Remove previous app bundles (stable or hashed). */
for (const name of fs.readdirSync(data)) {
  if (/^app(\.[a-zA-Z0-9_-]+)?\.(js|css)$/.test(name)) {
    fs.unlinkSync(path.join(data, name));
  }
}

function shortHash(filePath) {
  const buf = fs.readFileSync(filePath);
  return crypto.createHash('sha256').update(buf).digest('hex').slice(0, 8);
}

function copyFromDist(name) {
  const src = path.join(dist, name);
  if (!fs.existsSync(src)) {
    console.error('missing', src);
    process.exit(1);
  }
  fs.copyFileSync(src, path.join(data, name));
  console.log('copied', name);
}

copyFromDist('index.html');

const jsSrc = path.join(dist, 'app.js');
const cssSrc = path.join(dist, 'app.css');
if (!fs.existsSync(jsSrc) || !fs.existsSync(cssSrc)) {
  console.error('expected dist/app.js and dist/app.css (stable names)');
  process.exit(1);
}

fs.copyFileSync(jsSrc, path.join(data, 'app.js'));
fs.copyFileSync(cssSrc, path.join(data, 'app.css'));
const jsHash = shortHash(jsSrc);
const cssHash = shortHash(cssSrc);
const jsRef = `app.${jsHash}.js`;
const cssRef = `app.${cssHash}.css`;
console.log('copied app.js / app.css');
console.log('cache-bust refs', jsRef, cssRef);

function rewriteAppRefs(html) {
  return html
    .replace(/(\.?\/)?app(?:\.[a-zA-Z0-9_-]+)?\.js/g, `./${jsRef}`)
    .replace(/(\.?\/)?app(?:\.[a-zA-Z0-9_-]+)?\.css/g, `./${cssRef}`);
}

/** index.html from Vite references ./app.js|css — rewrite to hashed URLs. */
{
  const p = path.join(data, 'index.html');
  const next = rewriteAppRefs(fs.readFileSync(p, 'utf8'));
  fs.writeFileSync(p, next);
  console.log('patched index.html →', jsRef, cssRef);
}

/** Standalone pages live in web/; copy to LittleFS with hashed CSS (and JS if any). */
function copyStandalone(file) {
  const src = path.join(webRoot, file);
  if (!fs.existsSync(src)) {
    console.error('missing', src);
    process.exit(1);
  }
  const html = rewriteAppRefs(fs.readFileSync(src, 'utf8'));
  fs.writeFileSync(path.join(data, file), html);
  console.log('copied', file, '→', cssRef);
}

copyStandalone('upd.html');
copyStandalone('login.html');
copyStandalone('sdfiles.html');

console.log('RX_FSK/data updated');
