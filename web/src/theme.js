export function applyTheme(v) {
  const mode = v || localStorage.getItem('rdz-theme') || 'auto';
  if (mode === 'auto') document.documentElement.removeAttribute('data-theme');
  else document.documentElement.setAttribute('data-theme', mode);
}
