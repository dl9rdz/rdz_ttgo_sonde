function livemapIsEmbedded() {
  try { return window.parent !== window; } catch (e) { return true; }
}

function livemapMarkEmbedded() {
  if (!livemapIsEmbedded()) return false;
  document.documentElement.classList.add('embedded');
  if (document.body) document.body.classList.add('embedded');
  return true;
}

function livemapApplyTheme(mode) {
  var root = document.documentElement;
  if (!mode || mode === 'auto') root.removeAttribute('data-theme');
  else root.setAttribute('data-theme', mode);
}

function livemapSyncTheme() {
  try {
    if (livemapIsEmbedded()) {
      var parentRoot = window.parent.document.documentElement;
      var theme = parentRoot.getAttribute('data-theme');
      if (theme) {
        livemapApplyTheme(theme);
        return;
      }
      try {
        var stored = window.parent.localStorage.getItem('rdz-theme');
        if (stored) { livemapApplyTheme(stored); return; }
      } catch (e2) {}
    } else {
      try {
        var local = localStorage.getItem('rdz-theme');
        if (local) { livemapApplyTheme(local); return; }
      } catch (e3) {}
    }
  } catch (e) {}
  livemapApplyTheme('auto');
}

function livemapCloseInParent() {
  try { window.parent.location.hash = 'home'; } catch (e) {}
}

function livemapCdnFallback() {
  livemapMarkEmbedded();
  livemapSyncTheme();
  var html = '<br /><br />In order to use this functionality, there must be an internet connection.<br /><br/><a href="livemap.html">retry</a>';
  if (livemapIsEmbedded()) {
    html += '<br /><br/><a href="#home" id="livemap-close">close map</a>';
  }
  var mapEl = document.getElementById('map');
  if (!mapEl) return;
  mapEl.innerHTML = html;
  var close = document.getElementById('livemap-close');
  if (close) {
    close.addEventListener('click', function (ev) {
      ev.preventDefault();
      livemapCloseInParent();
    });
  }
}

try {
  var check = $(document);
} catch (e) {
  document.addEventListener('DOMContentLoaded', livemapCdnFallback);
}

$(document).ready(function(){

  livemapMarkEmbedded();
  livemapSyncTheme();
  setInterval(livemapSyncTheme, 2000);

  var map = L.map('map', { attributionControl: false, zoomControl: false });
  map.on('mousedown touchstart', function () { follow = false; });
  map.on('click', function() { clearTrackInspect(); });

  L.control.scale({ position: 'bottomleft', metric: true, imperial: false }).addTo(map);
  L.control.attribution({ prefix: false }).addTo(map);

  if (mapcenter) map.setView(mapcenter, 5);
  else map.setView([51.163361, 10.447683], 5); // Mitte DE

  var basemapChoices = [];
  var currentBasemapId = null;
  var currentBasemapLayer = null;

  function builtinBasemapsConfig() {
    return {
      default: 'osm',
      basemaps: [
        {
          id: 'osm',
          label: 'OSM',
          url: 'https://{s}.tile.openstreetmap.de/{z}/{x}/{y}.png',
          attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
          maxZoom: 19
        },
        {
          id: 'opentopo',
          label: 'OpenTopo',
          url: 'https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',
          attribution: 'Map data: &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, <a href="http://viewfinderpanoramas.org">SRTM</a><br />Map style: &copy; <a href="https://opentopomap.org">OpenTopoMap</a> (<a href="https://creativecommons.org/licenses/by-sa/3.0/">CC-BY-SA</a>)',
          maxZoom: 17
        },
        {
          id: 'cyclosm',
          label: 'CyclOSM',
          url: 'https://{s}.tile-cyclosm.openstreetmap.fr/cyclosm/{z}/{x}/{y}.png',
          attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, style &copy; <a href="https://www.cyclosm.org">CyclOSM</a> hosted by <a href="https://openstreetmap.fr/">OpenStreetMap France</a>',
          maxZoom: 20
        },
        {
          id: 'esri',
          label: 'Esri',
          url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
          attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye,<br />Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community',
          maxZoom: 21
        },
        {
          id: 'esridark',
          label: 'Esri Dark Gray',
          url: 'https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Dark_Gray_Base/MapServer/tile/{z}/{y}/{x}',
          attribution: 'Tiles &copy; Esri &mdash; Esri, HERE, Garmin, &copy; OpenStreetMap contributors, and the GIS user community',
          maxZoom: 16
        },
        {
          id: 'osmdark',
          label: 'CARTO Gray',
          url: 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png',
          attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors &copy; <a href="https://carto.com/attributions">CARTO</a>',
          maxZoom: 19
        }
      ]
    };
  }

  function parseBasemapsConfig(data) {
    if (!data || !Array.isArray(data.basemaps) || data.basemaps.length === 0) return null;
    for (var i = 0; i < data.basemaps.length; i++) {
      var e = data.basemaps[i];
      if (!e || !e.id || !e.url) return null;
    }
    return data;
  }

  function resolveTileUrl(entry) {
    var url = entry.url || '';
    var key = entry.apikey != null ? String(entry.apikey) : '';
    if (key) url = url.split('{APIKEY}').join(key);
    return url;
  }

  function entryNeedsApiKey(entry) {
    return resolveTileUrl(entry).indexOf('{APIKEY}') >= 0;
  }

  function makeTileLayer(entry) {
    var opts = { attribution: entry.attribution || '' };
    if (entry.maxZoom != null) opts.maxZoom = entry.maxZoom;
    return L.tileLayer(resolveTileUrl(entry), opts);
  }

  function findBasemapChoice(id) {
    for (var i = 0; i < basemapChoices.length; i++) {
      if (basemapChoices[i].id === id) return basemapChoices[i];
    }
    return null;
  }

  function syncBasemapRadios() {
    var inputs = basemapMenu.querySelectorAll('input[name="basemap"]');
    for (var i = 0; i < inputs.length; i++) {
      inputs[i].checked = (inputs[i].value === currentBasemapId);
    }
  }

  function rebuildBasemapMenu() {
    var html = '<div class="basemap-popover-title">Basemap</div>';
    basemapChoices.forEach(function(b) {
      var checked = b.id === currentBasemapId ? ' checked' : '';
      var needs = entryNeedsApiKey(b.entry) ? ' <span class="basemap-key-hint">key</span>' : '';
      html += '<label class="basemap-option">' +
        '<input type="radio" name="basemap" value="' + b.id + '"' + checked + '/> ' +
        (b.label || b.id) + needs + '</label>';
    });
    basemapMenu.innerHTML = html;
  }

  setBasemap = function(id) {
    var next = findBasemapChoice(id);
    if (!next) return false;
    if (entryNeedsApiKey(next.entry)) {
      alert('This basemap needs an API key. Copy basemaps.json to usermaps.json, set "apikey", and edit via /edit.html?file=usermaps.json');
      syncBasemapRadios();
      return false;
    }
    if (!next.layer) next.layer = makeTileLayer(next.entry);
    if (currentBasemapLayer && map.hasLayer(currentBasemapLayer)) {
      map.removeLayer(currentBasemapLayer);
    }
    next.layer.addTo(map);
    currentBasemapLayer = next.layer;
    currentBasemapId = id;
    syncBasemapRadios();
    return true;
  };

  function pickDefaultBasemapId(cfg) {
    var preferDark = document.documentElement.getAttribute('data-theme') === 'dark' ||
      (!document.documentElement.getAttribute('data-theme') &&
        window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches);
    if (cfg.default && findBasemapChoice(cfg.default)) return cfg.default;
    if (preferDark && findBasemapChoice('osmdark')) return 'osmdark';
    if (findBasemapChoice('osm')) return 'osm';
    return basemapChoices[0].id;
  }

  function applyBasemapConfig(cfg) {
    basemapChoices = cfg.basemaps.map(function(e) {
      return { id: e.id, label: e.label || e.id, entry: e, layer: null };
    });
    var startId = pickDefaultBasemapId(cfg);
    if (!setBasemap(startId)) {
      for (var i = 0; i < basemapChoices.length; i++) {
        if (!entryNeedsApiKey(basemapChoices[i].entry) && setBasemap(basemapChoices[i].id)) break;
      }
    }
    rebuildBasemapMenu();
  }

  function fetchBasemapsConfig(done) {
    function tryUrl(url, next) {
      $.ajax({
        url: url,
        dataType: 'json',
        timeout: 4000,
        success: function(data) {
          var cfg = parseBasemapsConfig(data);
          if (cfg) done(cfg);
          else next();
        },
        error: function() { next(); }
      });
    }
    tryUrl('usermaps.json', function() {
      tryUrl('basemaps.json', function() {
        done(builtinBasemapsConfig());
      });
    });
  }

  var reddot = '<span class="ldot rbg"></span>';
  var yellowdot = '<span class="ldot ybg"></span>';
  var greendot = '<span class="ldot gbg"></span>';
  var bluedot = '<span class="ldot bbg"></span>';
  var lastframe = 0;
  var HIST_LINE_STYLE = { color: '#c45c26', weight: 3, opacity: 0.85 };
  var LOW_VS_CLEAR_MS = 5 * 60 * 1000;
  var predictInflight = false;
  var predictHold = false;
  var lowVsSince = null;
  var predictorIntervalSec = 30;
  var predictor = false;
  var lastRxEntries = [];
  var lastRxShown = {}; /* ser -> true/false, default true */
  var lastRxMarkers = {};
  var lastRxTracks = {};
  /* SD / file-list overlays: key -> { path, label, pts, data, layer, marker, shown } */
  var sdOverlays = {};

  $('#map .leaflet-control-container').append(L.DomUtil.create('div', 'leaflet-top leaflet-center leaflet-header'));
  var header = '';
  header += '<div id="sonde_main"><span class="map-title">LiveMap</span>';
  header += '<div class="sonde-idline"><span id="sonde_id"></span> · <span id="sonde_freq"></span> MHz · <span id="sonde_type"></span></div></div>';
  header += '<div id="sonde_detail"><span id="sonde_alt"></span>m · <span id="sonde_climb"></span>m/s · <span id="sonde_speed"></span>km/h · <span id="sonde_dir"></span>°';
  header += '<div class="sonde-meta"><span id="sonde_time"></span> · −<span id="sonde_rssi"></span>dBm</div></div>';
  header += '<div id="sonde_status"><span id="sonde_statbar"></span></div>';
  header += '<div id="settings"><b>Prediction settings</b>';
  header += '<label for="burst">Burst at</label><input type="text" id="burst" maxlength="5" value="..." /> m';
  header += '<label for="overwrite_descend">Descending</label><input type="text" id="overwrite_descend" maxlength="2" value="..." /> m/s';
  header += '<label for="overwrite_descend_till">Use descending until</label><input type="text" id="overwrite_descend_till" maxlength="5" value="..." /> m';
  header += '<small>Below that, observed fall speed is converted to sea-level.</small>';
  header += '<div id="submit"><input type="button" value="save" onclick="settings_save();"/> <input type="button" value="reset" onclick="settings_reset();"/></div>';
  header += '</div>';
  $('.leaflet-header').append(header);

  $('#map .leaflet-control-container').append(L.DomUtil.create('div', 'leaflet-bottom leaflet-center leaflet-footer'));
  $('.leaflet-footer').append('<div id="gps_main"><span class="gps-label">Dir</span> <span class="gps_dir">...</span>° · <span class="gps-label">Dist</span> <span class="gps_dist">...</span>m</div>');

  function statusDotClass(res) {
    if (res === 0) return 'gbg';
    if (res === 2) return 'ybg';
    if (res === 4) return 'bbg';
    return 'rbg'; /* 1 timeout, 3 unset, other */
  }

  function statusDotHtml(res, hasId) {
    var cls = 'ldot ' + statusDotClass(res);
    /* Hollow only for a received frame without confirmed id (e.g. DFM).
       No-RX / error dots stay filled so the scanner strip does not flicker. */
    if (!hasId && (res === 0 || res === 4)) cls += ' hollow';
    return '<span class="' + cls + '"></span>';
  }

  function formatSondeTime(unixSec) {
    if (unixSec == null || unixSec === '' || isNaN(Number(unixSec))) return '';
    var n = Number(unixSec);
    if (!(n > 0)) return '';
    return new Date(n * 1000).toISOString().slice(0, 19) + 'Z';
  }

  function formatLastRxListTime(unixSec) {
    var full = formatSondeTime(unixSec);
    if (!full) return '';
    /* List line: yyyy-mm-dd hh:mm (UTC, no seconds) */
    return full.slice(0, 16).replace('T', ' ');
  }

  var trackInspectMarker = null;

  function clearTrackInspect() {
    if (trackInspectMarker) {
      map.removeLayer(trackInspectMarker);
      trackInspectMarker = null;
    }
  }

  function ptsToLatLngs(pts) {
    if (!pts) return [];
    return pts.map(function(p) {
      if (Array.isArray(p)) return [p[0], p[1]];
      return [p.lat, p.lon];
    });
  }

  function nearestTrackPt(pts, latlng) {
    if (!pts || !pts.length || !latlng) return null;
    var best = null;
    var bestD = Infinity;
    for (var i = 0; i < pts.length; i++) {
      var p = pts[i];
      var lat = Array.isArray(p) ? p[0] : p.lat;
      var lon = Array.isArray(p) ? p[1] : p.lon;
      var d = map.distance(latlng, [lat, lon]);
      if (d < bestD) {
        bestD = d;
        best = p;
      }
    }
    return best;
  }

  function showTrackInspect(pt, label) {
    if (!pt) return;
    var lat = Array.isArray(pt) ? pt[0] : pt.lat;
    var lon = Array.isArray(pt) ? pt[1] : pt.lon;
    var alt = Array.isArray(pt) ? pt[2] : pt.alt;
    var t = Array.isArray(pt) ? pt[3] : pt.t;
    clearTrackInspect();
    var html = '';
    if (label) html += '<b>' + label + '</b><br/>';
    html += lat.toFixed(5) + ', ' + lon.toFixed(5);
    if (alt != null && !isNaN(alt)) html += '<br/>' + Math.round(alt) + 'm';
    var ts = formatSondeTime(t);
    if (ts) html += '<br/>' + ts;
    trackInspectMarker = L.circleMarker([lat, lon], {
      radius: 8,
      color: '#1a5fb4',
      fillColor: '#62a0ea',
      fillOpacity: 0.95,
      weight: 2
    }).addTo(map).bindPopup(html, { closeOnClick: true, autoPan: true }).openPopup();
  }

  function bindTrackInspect(layer, pts, label) {
    if (!layer || !pts || pts.length < 1) return;
    layer._rdzPts = pts;
    layer._rdzLabel = label || '';
    layer.off('click');
    layer.on('click', function(ev) {
      L.DomEvent.stopPropagation(ev);
      if (ev.originalEvent) L.DomEvent.stop(ev.originalEvent);
      var pt = nearestTrackPt(pts, ev.latlng);
      showTrackInspect(pt, label);
    });
  }

  function pushStatDot(html) {
    var box = document.getElementById('sonde_statbar');
    if (!box) return;
    var wrap = document.createElement('span');
    wrap.innerHTML = html;
    var node = wrap.firstChild;
    if (!node) return;
    box.insertBefore(node, box.firstChild);
    while (box.childNodes.length > 10) box.removeChild(box.lastChild);
  }

  function setStatusButton(html) {
    $('#status').html(html);
  }

  headtxt = function(data, res) {
    var hasId = !!(data && data.id);
    var dot = statusDotHtml(typeof res === 'number' ? res : 1, hasId);
    pushStatDot(dot);
    setStatusButton(dot);

    if (data.res === 0) {
      if (data.id) {
        lastframe = data.vframe;
        $('#sonde_id').html(data.id);
      } else if (data.launchsite) {
        $('#sonde_id').html(String(data.launchsite).trim());
      }
      if (data.alt != null) $('#sonde_alt').html(data.alt);
      if (data.climb != null) $('#sonde_climb').html(data.climb);
      if (data.speed != null) $('#sonde_speed').html(mr(data.speed * 3.6 * 10) / 10);
      if (data.dir != null) $('#sonde_dir').html(data.dir);
      if (data.time != null) $('#sonde_time').html(formatSondeTime(data.time));
      if (data.rssi != null) $('#sonde_rssi').html(data.rssi / 2);
      $('#sonde_detail').show();
    } else if (!data.id && data.launchsite) {
      $('#sonde_id').html(String(data.launchsite).trim());
    }
    if (data.freq != null) $('#sonde_freq').html(data.freq);
    if (data.type != null) $('#sonde_type').html(data.type);
  };

  // Control strip: zoom + GPS left; basemap/status/follow/settings right (fixed size — no expanding layers control)
  L.control.zoom({ position: 'topleft' }).addTo(map);
  map.addControl(new L.Control.Button([{
    position: 'topleft',
    title: 'Phone GPS',
    html: '<img class="map-btn-icon" src="gps.svg" width="16" height="16" alt="GPS"/>',
    href: 'javascript:request_phone_gps();'
  }]));
  map.addControl(new L.Control.Button([{
    position: 'topleft',
    id: 'lastrx-btn',
    title: 'Last RX',
    text: '◷',
    className: 'lastrx-toggle',
    href: '#'
  }]));
  map.addControl(new L.Control.Button([{
    position: 'topright',
    id: 'basemap-btn',
    title: 'Basemap',
    html: '<img class="map-btn-layers" src="layers.png" srcset="layers.png 1x, layers-2x.png 2x" width="26" height="26" alt="Basemap"/>',
    className: 'basemap-layers-toggle',
    href: '#'
  }]));
  map.addControl(new L.Control.Button([{
    position: 'topright',
    id: 'status',
    title: 'Refresh',
    text: '',
    href: 'javascript:get_data();'
  }]));
  map.addControl(new L.Control.Button([
    {
      position: 'topright',
      title: 'Follow sonde',
      html: '<img class="map-btn-icon" src="ballon.png" width="16" height="16" alt="Sonde"/>',
      href: "javascript:show(marker[last_id],'marker');"
    },
    {
      title: 'Fit track',
      text: '〰',
      href: 'javascript:show_line();'
    },
    {
      title: 'Burst',
      html: '<img class="map-btn-icon map-btn-icon-sm" src="pop-marker.png" width="14" height="14" alt="Burst"/>',
      href: "javascript:show(marker_burst[last_id],'burst');"
    },
    {
      title: 'Landing',
      html: '<img class="map-btn-icon map-btn-icon-sm" src="landing.png" width="14" height="14" alt="Landing"/>',
      href: "javascript:show(marker_landing[last_id],'landing');"
    }
  ]));
  map.addControl(new L.Control.Button([{
    position: 'topright',
    title: 'Prediction settings',
    text: '⚙',
    href: 'javascript:show_settings();'
  }]));

  // Basemap popover: fixed-size button + panel beside it (opens toward map, not expanding the strip)
  var basemapMenu = L.DomUtil.create('div', 'basemap-popover', map.getContainer());
  basemapMenu.setAttribute('role', 'menu');
  basemapMenu.setAttribute('aria-hidden', 'true');
  basemapMenu.innerHTML = '<div class="basemap-popover-title">Basemap</div>';
  L.DomEvent.disableClickPropagation(basemapMenu);
  L.DomEvent.disableScrollPropagation(basemapMenu);

  positionBasemapMenu = function() {
    var btn = document.getElementById('basemap-btn');
    if (!btn || !basemapMenu) return;
    var mapEl = map.getContainer();
    var br = btn.getBoundingClientRect();
    var mr = mapEl.getBoundingClientRect();
    basemapMenu.style.top = Math.max(4, br.top - mr.top) + 'px';
    basemapMenu.style.right = Math.max(4, mr.right - br.left + 6) + 'px';
    basemapMenu.style.left = 'auto';
  };

  closeBasemapMenu = function() {
    basemapMenu.classList.remove('open');
    basemapMenu.setAttribute('aria-hidden', 'true');
    var btn = document.getElementById('basemap-btn');
    if (btn) btn.classList.remove('active');
  };

  openBasemapMenu = function() {
    positionBasemapMenu();
    basemapMenu.classList.add('open');
    basemapMenu.setAttribute('aria-hidden', 'false');
    var btn = document.getElementById('basemap-btn');
    if (btn) btn.classList.add('active');
  };

  toggleBasemapMenu = function() {
    if (basemapMenu.classList.contains('open')) closeBasemapMenu();
    else openBasemapMenu();
  };

  (function wireBasemapUi() {
    var btn = document.getElementById('basemap-btn');
    if (!btn) return;
    var hoverTimer = null;
    var canHover = window.matchMedia && window.matchMedia('(hover: hover) and (pointer: fine)').matches;

    L.DomEvent.on(btn, 'click', function(ev) {
      L.DomEvent.preventDefault(ev);
      L.DomEvent.stopPropagation(ev);
      toggleBasemapMenu();
    });

    if (canHover) {
      L.DomEvent.on(btn, 'mouseenter', function() {
        clearTimeout(hoverTimer);
        openBasemapMenu();
      });
      L.DomEvent.on(btn, 'mouseleave', function() {
        hoverTimer = setTimeout(closeBasemapMenu, 180);
      });
      L.DomEvent.on(basemapMenu, 'mouseenter', function() {
        clearTimeout(hoverTimer);
      });
      L.DomEvent.on(basemapMenu, 'mouseleave', function() {
        hoverTimer = setTimeout(closeBasemapMenu, 180);
      });
    }

    basemapMenu.addEventListener('change', function(ev) {
      var t = ev.target;
      if (t && t.name === 'basemap') {
        setBasemap(t.value);
        closeBasemapMenu();
      }
    });

    map.on('click', closeBasemapMenu);
    map.on('resize moveend', function() {
      if (basemapMenu.classList.contains('open')) positionBasemapMenu();
    });
    document.addEventListener('keydown', function(ev) {
      if (ev.key === 'Escape') closeBasemapMenu();
    });
  })();

  fetchBasemapsConfig(applyBasemapConfig);

  show = function(e, p) {
    if (p == 'landing') { get_predict(last_data, true); }
    if (e) {
      map.closePopup();
      map.setView(map._layers[e._leaflet_id].getLatLng());
      map._layers[e._leaflet_id].openPopup();
      follow = p;
    }
  };

  getTwoBounds = function(a, b) {
    var sW = new L.LatLng(
      (a._southWest.lat > b._southWest.lat) ? b._southWest.lat : a._southWest.lat,
      (a._southWest.lng > b._southWest.lng) ? b._southWest.lng : a._southWest.lng
    );
    var nE = new L.LatLng(
      (a._northEast.lat < b._northEast.lat) ? b._northEast.lat : a._northEast.lat,
      (a._northEast.lng < b._northEast.lng) ? b._northEast.lng : a._northEast.lng
    );
    return new L.LatLngBounds(sW, nE);
  };

  show_line = function() {
    $('.i_position, .i_landing').remove();
    map.closePopup();
    if (line[last_id]._latlngs.length != 0 && line_predict[last_id]._latlngs.length != 0) {
      map.fitBounds(getTwoBounds(line[last_id].getBounds(), line_predict[last_id].getBounds()));
    } else if (line[last_id]._latlngs.length != 0) {
      map.fitBounds(line[last_id].getBounds());
    } else if (line_predict[last_id]._latlngs.length != 0) {
      map.fitBounds(line_predict[last_id].getBounds());
    }
  };

  last_data = false;
  last_id = false;
  follow = 'marker';

  icon_balloon = L.icon({
    iconUrl: 'ballon.png',
    iconSize: [28, 28],
    iconAnchor: [14, 28],
    popupAnchor: [0, -28],
    tooltipAnchor: [10, -18],
    className: 'map-marker-balloon'
  });
  icon_chute = L.icon({
    iconUrl: 'chute.svg',
    iconSize: [28, 28],
    iconAnchor: [14, 28],
    popupAnchor: [0, -28],
    tooltipAnchor: [10, -18],
    className: 'map-marker-chute'
  });
  icon_landing = L.icon({
    iconUrl: 'landing.png',
    iconSize: [22, 22],
    iconAnchor: [11, 11],
    popupAnchor: [0, 0],
    className: 'map-marker-landing'
  });
  icon_burst = L.icon({
    iconUrl: 'pop-marker.png',
    iconSize: [16, 16],
    iconAnchor: [8, 8],
    popupAnchor: [0, 0],
    className: 'map-marker-burst'
  });
  icon_gps = L.icon({
    iconUrl: 'gps.svg',
    iconSize: [18, 18],
    iconAnchor: [9, 9],
    popupAnchor: [0, -9],
    className: 'leaflet-gps-icon map-marker-gps'
  });

  marker_landing = [];
  dots_predict = [];
  line_predict = [];
  marker_burst = [];
  marker = [];
  dots = [];
  line = [];

  sondeIconFor = function(data) {
    return (data && data.climb < 0) ? icon_chute : icon_balloon;
  };

  tiphtml = function(data) {
    var label = data.id || data.ser || '';
    var alt = (typeof data.alt === 'number') ? data.alt.toFixed(0) : data.alt;
    var vs = (typeof data.climb === 'number') ? data.climb.toFixed(1) : data.climb;
    var hs = mr((data.speed || 0) * 3.6 * 10) / 10;
    var freqHtml = '';
    if (data.freq != null && data.freq !== '' && !isNaN(Number(data.freq))) {
      freqHtml = '<span class="tooltip-freq">' + Number(data.freq).toFixed(3) + ' MHz</span>';
    }
    return '<div class="tooltip-container">' +
      '<div class="tooltip-idrow"><span class="tooltip-id">' + label + '</span>' + freqHtml + '</div>' +
      '<div class="text-speed">' + alt + 'm ' + vs + 'm/s ' + hs + 'km/h</div></div>';
  };

  function freqMarkerKey(freq) {
    if (freq == null || freq === '' || isNaN(freq)) return null;
    return 'f:' + Number(freq).toFixed(3);
  }

  function liveMarkerKey(data) {
    if (data && data.id) return data.id;
    return freqMarkerKey(data && data.freq);
  }

  function isTempFreqKey(key) {
    return !!(key && String(key).indexOf('f:') === 0);
  }

  function liveSerOf(data) {
    if (!data) return '';
    if (data.ser) return String(data.ser);
    if (data.id && data.id.charAt(0) === 'D' && data.id.length > 1) return data.id.slice(1);
    return data.id ? String(data.id) : '';
  }

  function liveOwnsSer(ser) {
    if (!ser) return false;
    var keys = Object.keys(marker);
    for (var i = 0; i < keys.length; i++) {
      var m = marker[keys[i]];
      if (!m) continue;
      var mser = m._rdzSer || keys[i];
      if (mser === ser) return true;
      if (keys[i] === ser) return true;
      if (keys[i] === 'D' + ser) return true;
      if (mser === 'D' + ser) return true;
      if (('D' + mser) === ser) return true;
    }
    return false;
  }

  function migrateFreqMarker(freqKey, realId, data) {
    if (!freqKey || !realId || freqKey === realId) return;
    if (!marker[freqKey]) return;
    if (marker[realId] && marker[realId] !== marker[freqKey]) {
      map.removeLayer(marker[freqKey]);
      delete marker[freqKey];
    } else {
      marker[realId] = marker[freqKey];
      delete marker[freqKey];
    }
    if (dots[freqKey]) {
      if (!dots[realId]) dots[realId] = [];
      dots[realId] = dots[freqKey].concat(dots[realId] || []);
      delete dots[freqKey];
    }
    if (line[freqKey]) {
      if (line[realId] && line[realId] !== line[freqKey]) {
        map.removeLayer(line[freqKey]);
      } else {
        line[realId] = line[freqKey];
      }
      delete line[freqKey];
      if (line[realId] && dots[realId]) line[realId].setLatLngs(dots[realId]);
    }
    if (marker[realId]) {
      marker[realId]._rdzSer = liveSerOf(data);
      marker[realId]._rdzKey = realId;
      marker[realId].setPopupContent(poptxt('position', data));
      marker[realId].setTooltipContent(tiphtml(data));
    }
  }

  function ensureLiveLightTrack(data) {
    var ser = liveSerOf(data);
    if (!ser) return;
    if (lastRxShown[ser] === false) return;
    if (!lastRxTracks[ser]) loadLightTrack(ser);
  }

  function liveKeyForSer(ser) {
    if (!ser) return null;
    var keys = Object.keys(marker);
    for (var i = 0; i < keys.length; i++) {
      var m = marker[keys[i]];
      if (!m) continue;
      var mser = m._rdzSer || keys[i];
      if (mser === ser || keys[i] === ser || keys[i] === 'D' + ser ||
          mser === 'D' + ser || ('D' + mser) === ser) {
        return m._rdzKey || keys[i];
      }
    }
    return null;
  }

  /* Prefer last lt_* sample; else last_rx.csv position. */
  function bridgeStartForSer(ser) {
    if (!ser) return null;
    var track = lastRxTracks[ser];
    if (track && track._rdzPts && track._rdzPts.length) {
      var p = track._rdzPts[track._rdzPts.length - 1];
      return { lat: p.lat, lon: p.lon, source: 'lt' };
    }
    for (var i = 0; i < lastRxEntries.length; i++) {
      if (lastRxEntries[i].ser === ser) {
        return {
          lat: lastRxEntries[i].lat,
          lon: lastRxEntries[i].lon,
          source: 'last_rx'
        };
      }
    }
    return null;
  }

  function ensureLiveTrailBridge(key, ser, liveLoc) {
    if (!key || !ser || isTempFreqKey(key) || !dots[key] || !liveLoc) return;
    var start = bridgeStartForSer(ser);
    var prev = dots[key]._rdzBridgeSource;
    if (!start) {
      if (!prev) dots[key]._rdzBridgeSource = 'none';
      return;
    }
    /* Already bridged from lt — keep it. Allow upgrade from last_rx/none → lt. */
    if (prev === 'lt') return;
    if (prev === 'last_rx' && start.source === 'last_rx') return;

    if (prev === 'last_rx' && start.source === 'lt' && dots[key].length > 0) {
      dots[key].shift(); /* drop previous last_rx bridge point */
    }

    var anchor = (dots[key].length > 0) ? dots[key][0] : liveLoc;
    try {
      if (map.distance([start.lat, start.lon], anchor) < 5) {
        dots[key]._rdzBridgeSource = start.source;
        if (line[key]) line[key].setLatLngs(dots[key]);
        return;
      }
    } catch (e) {}

    dots[key].unshift([start.lat, start.lon]);
    dots[key]._rdzBridgeSource = start.source;
    if (line[key]) line[key].setLatLngs(dots[key]);
  }

  function updateLowVsState(data) {
    if (!data || data.res !== 0) return;
    var vs = data.climb;
    var moving = (typeof vs === 'number' && isFinite(vs) && Math.abs(vs) >= 1);
    if (moving) {
      lowVsSince = null;
      if (predictHold) predictHold = false;
    } else {
      if (lowVsSince == null) lowVsSince = Date.now();
      if (!predictHold && last_id && !isTempFreqKey(last_id) &&
          (Date.now() - lowVsSince) >= LOW_VS_CLEAR_MS) {
        clearPredictForId(last_id);
      }
    }
  }

  function canAutoPredict(data) {
    if (!data || !data.id || isTempFreqKey(data.id)) return false;
    var vs = data.climb;
    return (typeof vs === 'number' && isFinite(vs) && Math.abs(vs) >= 1);
  }

  function nextPredictIntervalSec(data) {
    if (!data) return 30;
    if (data.climb > 0) return 5 * 60;
    if (data.climb < 0 && data.alt > 5000) return 2 * 60;
    return 30;
  }

  function armPredictTimer(sec) {
    if (sec == null || sec < 0) sec = 30;
    predictorIntervalSec = sec;
    clearTimeout(predictor);
    predictor = setTimeout(predictTick, sec * 1000);
  }

  function predictTick() {
    predictor = false;
    var data = last_data;
    if (!data) {
      armPredictTimer(30);
      return;
    }
    if (canAutoPredict(data)) {
      get_predict(data, false);
      armPredictTimer(nextPredictIntervalSec(data));
    } else {
      armPredictTimer(30);
    }
  }

  draw = function(data) {
    var res = (typeof data.res === 'number') ? data.res : 1;
    data.res = res;

    if (res === 0) {
      var key = liveMarkerKey(data);
      if (data.id) {
        var fkey = freqMarkerKey(data.freq);
        if (fkey) migrateFreqMarker(fkey, data.id, data);
        last_id = data.id;
      } else if (key) {
        last_id = key;
      }

      if (key && data.lat && data.lon && data.alt) {
        var location = [data.lat, data.lon, data.alt];
        var ic = sondeIconFor(data);
        var mapData = {
          id: data.id || key,
          ser: liveSerOf(data),
          lat: data.lat,
          lon: data.lon,
          alt: data.alt,
          climb: data.climb,
          speed: data.speed,
          type: data.type,
          time: data.time,
          freq: data.freq
        };
        if (!marker[key]) {
          map.setView(location, 14);
          marker[key] = L.marker(location, { icon: ic }).addTo(map)
            .bindPopup(poptxt('position', mapData), { closeOnClick: false, autoPan: false })
            .bindTooltip(tiphtml(mapData), {
              permanent: true,
              direction: 'right',
              className: 'sondeTooltip',
              offset: [10, -16],
              opacity: 0.75,
              interactive: false
            })
            .openPopup();
          marker[key]._rdzSer = liveSerOf(data);
          marker[key]._rdzKey = key;
        } else {
          marker[key].setIcon(ic);
          marker[key]._rdzSer = liveSerOf(data);
          marker[key]._rdzKey = key;
          marker[key].slideTo(location, {
            duration: 500,
            keepAtCenter: (follow == 'marker')
          })
            .setPopupContent(poptxt('position', mapData))
            .setTooltipContent(tiphtml(mapData));
        }
        if (!dots[key]) dots[key] = [];
        dots[key].push(location);
        if (!line[key]) {
          line[key] = L.polyline(dots[key]).addTo(map);
        } else {
          line[key].setLatLngs(dots[key]);
        }
        ensureLiveTrailBridge(key, liveSerOf(data), location);
        ensureLiveLightTrack(data);
        syncLastRxMarkers();
      }

      if (data.id) storage_write(data);
      last_data = data;
      updateLowVsState(data);
      if (!predictor) armPredictTimer(0);
    } else {
      if (data.id) last_id = data.id;
      if (last_data) {
        /* Keep last good fix for follow/prediction; refresh type/freq/launchsite. */
        last_data.freq = data.freq;
        last_data.type = data.type;
        last_data.launchsite = data.launchsite;
        last_data.res = res;
        last_data.rssi = data.rssi;
      }
    }

    headtxt(data, res);
  };

  marker_gps = false;
  circ_gps = false;
  phone_gps_active = false;
  phone_gps_win = null;

  request_phone_gps = function() {
    var url = (typeof phoneGpsUrl === 'string') ? phoneGpsUrl : '';
    if (!url) {
      alert('Missing phoneGpsUrl in livemap.html.');
      return;
    }
    if (phone_gps_win && !phone_gps_win.closed) {
      phone_gps_win.focus();
      return;
    }
    phone_gps_win = window.open(url, 'rdz-phone-gps', 'width=380,height=320');
  };

  window.addEventListener('message', function(ev) {
    var d = ev.data;
    if (!d || d.type !== 'rdz-phone-gps') { return; }
    if (phone_gps_win && ev.source !== phone_gps_win) { return; }
    if (typeof d.lat !== 'number' || typeof d.lon !== 'number') { return; }
    phone_gps_active = true;
    gps(d);
  });

  gps = function(e) {
    gps_location = [e.lat, e.lon];
    gps_accuracy = e.hdop * 2;

    if (last_data && last_data.lat != '0.000000') {
      if ($('.leaflet-footer').css('display') == 'none') { $('.leaflet-footer').show(); }

      var distance = Math.round(map.distance(gps_location, [last_data.lat, last_data.lon]));
      distance = (distance > 1000) ? (distance / 1000) + 'k' : distance;
      $('.leaflet-footer .gps_dist').html(distance);
      $('.leaflet-footer .gps_dir').html(bearing(gps_location, [last_data.lat, last_data.lon]));
    }

    if (!marker_gps) {
      map.addControl(new L.Control.Button([{
        position: 'topleft',
        title: 'Follow GPS',
        text: '◎',
        href: "javascript:show(marker_gps,'gps');"
      }]));

      marker_gps = L.marker(gps_location, { icon: icon_gps }).addTo(map)
        .bindPopup(poptxt('gps', e), { closeOnClick: false, autoPan: false });
      circ_gps = L.circle(gps_location, gps_accuracy).addTo(map);
    } else {
      marker_gps.slideTo(gps_location, {
        duration: 500,
        keepAtCenter: (follow == 'gps') ? true : false
      })
        .setPopupContent(poptxt('gps', e));
      circ_gps.slideTo(gps_location, { duration: 500 });
      circ_gps.setRadius(gps_accuracy);
    }
  };

  get_data = function() {
    $.ajax({
      url: 'live.json',
      success: (function(data) {
        if (typeof data != 'object') { data = $.parseJSON(data); }
        if (data.sonde) {
          draw(data.sonde);
        } else {
          setStatusButton(yellowdot);
        }
        if (data.gps && !phone_gps_active) {
          gps(data.gps);
        }
      }),
      error: function() {
        setStatusButton(reddot);
      },
      timeout: 1000
    });
  };

  storage = (typeof (Storage) !== 'undefined') ? true : false;

  settings_std = {
    burst: 32500,
    overwrite_descend: 6,
    overwrite_descend_till: 12000
  };

  settings_read = function() {
    if (storage) {
      if (sessionStorage.settings) {
        return JSON.parse(sessionStorage.settings);
      } else {
        settings_write(settings_std);
        return settings_std;
      }
    } else {
      return settings_std;
    }
    return false;
  };

  settings_write = function(data) {
    if (storage) {
      sessionStorage.settings = JSON.stringify(data);
      settings = data;
    }
  };

  settings = settings_read();

  settings_save = function() {
    settings.burst = parseInt($('#settings #burst').val());
    settings.overwrite_descend = parseInt($('#settings #overwrite_descend').val());
    settings.overwrite_descend_till = parseInt($('#settings #overwrite_descend_till').val());
    if (Number.isInteger(settings.burst) && Number.isInteger(settings.overwrite_descend) && Number.isInteger(settings.overwrite_descend_till)) {
      settings_write(settings);
      $('#settings').slideUp();
      get_predict(last_data, true);
    } else {
      alert('Error: only numeric values allowed!');
    }
  };

  settings_reset = function() {
    if (confirm('Reset to default?')) {
      settings_write(settings_std);
      show_settings();
    }
  };

  show_settings = function() {
    $('#settings #burst').val(settings.burst);
    $('#settings #overwrite_descend').val(settings.overwrite_descend);
    $('#settings #overwrite_descend_till').val(settings.overwrite_descend_till);
    $('#settings').slideToggle();
  };

  // Same NASA density Tawhiri uses. descent_rate is sea-level: v(h) = 1.1045 * v_sea / sqrt(rho).
  air_density = function(alt) {
    var temp, pressure;
    if (alt > 25000) {
      temp = -131.21 + 0.00299 * alt;
      pressure = 2.488 * Math.pow((temp + 273.1) / 216.6, -11.388);
    } else if (alt > 11000) {
      temp = -56.46;
      pressure = 22.65 * Math.exp(1.73 - 0.000157 * alt);
    } else {
      temp = 15.04 - 0.00649 * alt;
      pressure = 101.29 * Math.pow((temp + 273.1) / 288.08, 5.256);
    }
    return pressure / (0.2869 * (temp + 273.1));
  };

  sea_level_descent = function(rate, alt) {
    var v = Math.abs(rate);
    if (!(v > 0)) { return settings.overwrite_descend; }
    return v * Math.sqrt(air_density(alt)) / 1.1045;
  };

  predCacheSave = function(id, prediction, data) {
    if (!id || !storage || isTempFreqKey(id)) return;
    try {
      localStorage.setItem('rdz.pred.' + id, JSON.stringify({
        ts: Date.now(),
        prediction: prediction,
        data: { id: data.id, lat: data.lat, lon: data.lon, alt: data.alt, climb: data.climb, time: data.time }
      }));
    } catch (e) {}
  };

  predCacheLoad = function(id) {
    if (!id || !storage || isTempFreqKey(id)) return null;
    try {
      var raw = localStorage.getItem('rdz.pred.' + id);
      if (!raw) return null;
      var o = JSON.parse(raw);
      if (!o || !o.prediction || !o.ts) return null;
      if (Date.now() - o.ts > 72 * 3600 * 1000) return null;
      return o;
    } catch (e) { return null; }
  };

  get_predict = function(data, force) {
    if (!data) return;
    if (!data.id || isTempFreqKey(data.id)) return;
    if (!data.lat || !data.lon || data.alt == null) return;
    if (!force && !canAutoPredict(data)) return;
    if (predictInflight) return;

    if (force) predictHold = true;

    var ascent = (data.climb > 0) ? data.climb : 15;
    var descent, burst;
    if (data.climb > 0) {
      descent = settings.overwrite_descend;
      burst = (data.alt > settings.burst) ? data.alt + 100 : settings.burst;
    } else {
      burst = parseInt(data.alt, 10) + 7;
      descent = (data.alt > settings.overwrite_descend_till)
        ? settings.overwrite_descend
        : sea_level_descent(data.climb, data.alt);
    }

    var m = new Date((data.time || 0) * 1000);
    var datetime = m.toISOString().slice(0, 19) + 'Z';
    var url = 'https://api.v2.sondehub.org/tawhiri';
    url += '?launch_latitude=' + data.lat + '&launch_longitude=' + tawhiri_lon(data.lon);
    url += '&launch_altitude=' + data.alt + '&launch_datetime=' + datetime;
    url += '&ascent_rate=' + ascent + '&burst_altitude=' + burst + '&descent_rate=' + descent;

    predictInflight = true;
    $.getJSON(url, function(prediction) {
      predictInflight = false;
      predCacheSave(data.id, prediction, data);
      draw_predict(prediction, data, false);
    }).fail(function() {
      predictInflight = false;
      var cached = predCacheLoad(data.id);
      if (cached) draw_predict(cached.prediction, data, true);
    });
  };

  draw_predict = function(prediction, data, fromCache) {
    if (!data || !data.id || isTempFreqKey(data.id)) return;
    var ascending = prediction.prediction[0].trajectory;
    var highest = ascending[ascending.length - 1];
    var highest_location = [highest.latitude, sanitize_lon(highest.longitude)];

    var descending = prediction.prediction[1].trajectory;
    var landing = descending[descending.length - 1];
    var landing_location = [landing.latitude, sanitize_lon(landing.longitude)];

    if (!marker_landing[data.id]) {
      marker_landing[data.id] = L.marker(landing_location, { icon: icon_landing }).addTo(map)
        .bindPopup(poptxt('landing', landing), { closeOnClick: false, autoPan: false });
    } else {
      marker_landing[data.id].slideTo(landing_location, {
        duration: 500,
        keepAtCenter: (follow == 'landing')
      })
        .setPopupContent(poptxt('landing', landing));
    }

    dots_predict[data.id] = [];

    if (data.climb > 0) {
      ascending.forEach(function(p) { dots_predict[data.id].push([p.latitude, sanitize_lon(p.longitude)]); });

      if (!marker_burst[data.id]) {
        marker_burst[data.id] = L.marker(highest_location, { icon: icon_burst }).addTo(map)
          .bindPopup(poptxt('burst', highest), { closeOnClick: false, autoPan: false });
      } else {
        marker_burst[data.id].slideTo(highest_location, {
          duration: 500,
          keepAtCenter: (follow == 'burst')
        }).setPopupContent(poptxt('burst', highest));
      }
    }

    descending.forEach(function(p) { dots_predict[data.id].push([p.latitude, sanitize_lon(p.longitude)]); });

    var predColor = fromCache ? '#f0a020' : 'yellow';
    if (!line_predict[data.id]) {
      line_predict[data.id] = L.polyline(dots_predict[data.id], { color: predColor, dashArray: fromCache ? '6 6' : null }).addTo(map);
    } else {
      line_predict[data.id].setStyle({ color: predColor, dashArray: fromCache ? '6 6' : null });
      line_predict[data.id].setLatLngs(dots_predict[data.id]);
    }
    /* Timer is armed from the live poll / predictTick, not from here. */
  };

  sanitize_lon = function(lon) {
    if (lon > 180) { return lon - 360; }
    return lon;
  };
  tawhiri_lon = function(lon) {
    if (lon < 0) { return lon + 360; }
    return lon;
  };

  poptxt = function(t, i) {
    var lat_input = (i.id) ? i.lat : i.latitude;
    var lon_input = sanitize_lon((i.id) ? i.lon : i.longitude);

    var lat = Math.round(lat_input * 1000000) / 1000000;
    var lon = Math.round(lon_input * 1000000) / 1000000;

    var add =
      '<br /><b>Position:</b> ' + lat + ',  ' + lon + '<br />' +
      '<b>Open:</b> <a href="https://www.google.de/maps/?q=' + lat + ', ' + lon + '" target="_blank">GMaps</a> | <a href="https://www.openstreetmap.org/?mlat=' + lat + '&mlon=' + lon + '&zoom=15" target="_blank">OSM</a> | <a href="geo:' + lat + ',' + lon + '">GeoApp</a>';

    if (t == 'position') { return '<div class="i_position"><b>' + i.id + '</b>' + add + '</div>'; }
    if (t == 'burst') { return '<div class="i_burst"><b>Predicted Burst:</b><br />' + fd(i.datetime) + ' at ' + mr(i.altitude) + 'm' + add + '</div>'; }
    if (t == 'highest') { return '<div class="i_burst"><b>Burst:</b> ' + mr(i.altitude) + 'm' + add + '</div>'; }
    if (t == 'landing') { return '<div class="i_landing"><b>Predicted Landing:</b><br />' + fd(i.datetime) + ' at ' + mr(i.altitude) + 'm' + add + '</div>'; }
    if (t == 'gps') { return '<div class="i_gps">Position: ' + (i.lat) + ',' + (i.lon) + '<br />Altitude: ' + i.alt + 'm<br />Speed: ' + mr(i.speed * 3.6 * 10) / 10 + 'km/h ' + i.dir + '°<br />Sat: ' + i.sat + ' Hdop:' + (i.hdop / 10) + '</div>'; }
  };

  fd = function(date) {
    var d = new Date(Date.parse(date));
    return az(d.getUTCHours()) + ':' + az(d.getUTCMinutes()) + ' UTC';
  };
  az = function(n) { return (n < 10) ? '0' + n : n; };
  mr = function(n) { return Math.round(n); };

  storage = (typeof (Storage) !== 'undefined') ? true : false;
  storage_write = function(data) {
    if (storage) {
      if (sessionStorage.sonde) {
        storage_data = JSON.parse(sessionStorage.sonde);
      } else {
        storage_data = [];
      }
      if (JSON.stringify(data) != JSON.stringify(storage_data[storage_data.length - 1])) {
        storage_data.push(data);
        sessionStorage.sonde = JSON.stringify(storage_data);
      }
    }
  };

  storage_read = function() {
    if (storage) {
      if (sessionStorage.sonde) {
        storage_data = JSON.parse(sessionStorage.sonde);
        return storage_data;
      }
    }
    return false;
  };

  storage_remove = function() {
    sessionStorage.removeItem('sonde');
  };

  session_storage = storage_read();
  if (session_storage) {
    session_storage.forEach(function(d) {
      session_storage_last = d;
    });
    if (session_storage_last) {
      session_storage_last.res = 0;
      draw(session_storage_last);
    }
  }

  setInterval(get_data, 1000);

  /* ---- last_rx.csv history pins + light tracks + SD overlays ---- */
  var lastRxMenu = L.DomUtil.create('div', 'basemap-popover lastrx-popover', map.getContainer());
  lastRxMenu.setAttribute('role', 'menu');
  lastRxMenu.setAttribute('aria-hidden', 'true');
  lastRxMenu.innerHTML = '<div class="basemap-popover-title">Last RX</div><div id="last-rx-list"></div>';
  L.DomEvent.disableClickPropagation(lastRxMenu);
  L.DomEvent.disableScrollPropagation(lastRxMenu);

  function positionLastRxMenu() {
    var btn = document.getElementById('lastrx-btn');
    if (!btn || !lastRxMenu) return;
    var mapEl = map.getContainer();
    var br = btn.getBoundingClientRect();
    var mr = mapEl.getBoundingClientRect();
    lastRxMenu.style.top = Math.max(4, br.top - mr.top) + 'px';
    lastRxMenu.style.left = Math.max(4, br.right - mr.left + 6) + 'px';
    lastRxMenu.style.right = 'auto';
  }

  function closeLastRxMenu() {
    lastRxMenu.classList.remove('open');
    lastRxMenu.setAttribute('aria-hidden', 'true');
    var btn = document.getElementById('lastrx-btn');
    if (btn) btn.classList.remove('active');
  }

  function openLastRxMenu() {
    positionLastRxMenu();
    lastRxMenu.classList.add('open');
    lastRxMenu.setAttribute('aria-hidden', 'false');
    var btn = document.getElementById('lastrx-btn');
    if (btn) btn.classList.add('active');
  }

  function toggleLastRxMenu() {
    if (lastRxMenu.classList.contains('open')) closeLastRxMenu();
    else openLastRxMenu();
  }

  (function wireLastRxUi() {
    var btn = document.getElementById('lastrx-btn');
    if (!btn) return;
    var hoverTimer = null;
    var canHover = window.matchMedia && window.matchMedia('(hover: hover) and (pointer: fine)').matches;

    L.DomEvent.on(btn, 'click', function(ev) {
      L.DomEvent.preventDefault(ev);
      L.DomEvent.stopPropagation(ev);
      toggleLastRxMenu();
    });

    if (canHover) {
      L.DomEvent.on(btn, 'mouseenter', function() {
        clearTimeout(hoverTimer);
        openLastRxMenu();
      });
      L.DomEvent.on(btn, 'mouseleave', function() {
        hoverTimer = setTimeout(closeLastRxMenu, 180);
      });
      L.DomEvent.on(lastRxMenu, 'mouseenter', function() {
        clearTimeout(hoverTimer);
      });
      L.DomEvent.on(lastRxMenu, 'mouseleave', function() {
        hoverTimer = setTimeout(closeLastRxMenu, 180);
      });
    }

    map.on('click', closeLastRxMenu);
    map.on('resize moveend', function() {
      if (lastRxMenu.classList.contains('open')) positionLastRxMenu();
    });
    document.addEventListener('keydown', function(ev) {
      if (ev.key === 'Escape') closeLastRxMenu();
    });
  })();

  function parseCsvLines(txt) {
    var rows = [];
    String(txt || '').split(/\r?\n/).forEach(function(line) {
      line = line.trim();
      if (!line || line.charAt(0) === '#' || line.indexOf('ser,') === 0 || line.indexOf('lat,') === 0) return;
      var p = line.split(',');
      if (p.length < 4) return;
      rows.push(p);
    });
    return rows;
  }

  function downsamplePts(pts, maxPts) {
    if (!pts || pts.length <= maxPts) return pts;
    var out = [];
    var step = (pts.length - 1) / (maxPts - 1);
    for (var i = 0; i < maxPts; i++) {
      out.push(pts[Math.round(i * step)]);
    }
    return out;
  }

  function parseTrackCsv(txt) {
    var pts = [];
    parseCsvLines(txt).forEach(function(p) {
      var lat = parseFloat(p[0]), lon = parseFloat(p[1]);
      var alt = parseFloat(p[2]);
      var t = parseInt(p[3], 10);
      if (isNaN(lat) || isNaN(lon)) return;
      pts.push({
        lat: lat,
        lon: lon,
        alt: isNaN(alt) ? null : alt,
        t: isNaN(t) ? 0 : t
      });
    });
    return pts;
  }

  function parseSdTrackCsv(txt) {
    var pts = [];
    var last = null;
    String(txt || '').split(/\r?\n/).forEach(function(line, idx) {
      line = line.trim();
      if (!line) return;
      if (idx === 0 && /lat/i.test(line) && /lon/i.test(line)) return;
      var p = line.split(',');
      /* SD log: validID,ser,typestr,subtype,lat,lon,alt,vs,hs,dir,sats,validPos,time,... */
      var lat, lon, alt, vs, hs, ser, type, t;
      if (p.length >= 6 && (p[0] === '0' || p[0] === '1')) {
        ser = p[1] || '';
        type = p[2] || '';
        lat = parseFloat(p[4]); lon = parseFloat(p[5]);
        alt = parseFloat(p[6]);
        vs = parseFloat(p[7]);
        hs = parseFloat(p[8]);
        t = parseInt(p[12], 10);
      } else if (p.length >= 5 && isNaN(parseFloat(p[0])) && !isNaN(parseFloat(p[2]))) {
        /* hist header rows: ser,type,lat,lon,alt,freq,time,... */
        ser = p[0] || '';
        type = p[1] || '';
        lat = parseFloat(p[2]); lon = parseFloat(p[3]);
        alt = parseFloat(p[4]);
        t = parseInt(p[6], 10);
        vs = parseFloat(p[7]);
        hs = parseFloat(p[8]);
      } else {
        /* light-track style: lat,lon,alt,t */
        lat = parseFloat(p[0]); lon = parseFloat(p[1]);
        alt = parseFloat(p[2]);
        t = parseInt(p[3], 10);
        vs = NaN; hs = NaN; ser = ''; type = '';
      }
      if (isNaN(lat) || isNaN(lon) || (lat === 0 && lon === 0)) return;
      var pt = {
        lat: lat,
        lon: lon,
        alt: isNaN(alt) ? null : alt,
        t: isNaN(t) ? 0 : t
      };
      pts.push(pt);
      last = {
        id: ser || '',
        ser: ser || '',
        type: type || '',
        lat: lat,
        lon: lon,
        alt: isNaN(alt) ? 0 : alt,
        climb: isNaN(vs) ? 0 : vs,
        speed: isNaN(hs) ? 0 : hs,
        time: isNaN(t) ? 0 : t
      };
    });
    return { pts: downsamplePts(pts, 600), last: last };
  }

  function sdOverlayKey(path) {
    return 'sd:' + path;
  }

  function sdOverlayBasename(path) {
    var parts = String(path || '').split('/');
    return parts[parts.length - 1] || path;
  }

  function removeSdOverlayLayers(key) {
    var o = sdOverlays[key];
    if (!o) return;
    if (o.layer) { map.removeLayer(o.layer); o.layer = null; clearTrackInspect(); }
    if (o.marker) { map.removeLayer(o.marker); o.marker = null; }
  }

  function showSdOverlay(key) {
    var o = sdOverlays[key];
    if (!o || !o.pts || o.pts.length < 2 || o.shown === false) return;
    if (!o.layer) {
      o.layer = L.polyline(ptsToLatLngs(o.pts), HIST_LINE_STYLE).addTo(map);
      bindTrackInspect(o.layer, o.pts, o.label || sdOverlayBasename(o.path));
    }
    if (!o.marker && o.data && o.data.lat && o.data.lon) {
      var ic = sondeIconFor(o.data);
      var loc = [o.data.lat, o.data.lon];
      var ts = formatSondeTime(o.data.time);
      var popup =
        '<b>' + (o.data.id || o.label || '') + '</b> (file)<br/>' +
        (o.data.type || '') +
        '<br/>' + o.data.lat.toFixed(5) + ', ' + o.data.lon.toFixed(5) +
        '<br/>' + Math.round(o.data.alt) + 'm' +
        (ts ? '<br/>' + ts : '');
      o.marker = L.marker(loc, { icon: ic }).addTo(map)
        .bindPopup(popup, { closeOnClick: false, autoPan: false })
        .bindTooltip(tiphtml(o.data), {
          permanent: true,
          direction: 'right',
          className: 'sondeTooltip',
          offset: [10, -16],
          opacity: 0.75,
          interactive: false
        });
    }
  }

  function clearPredictForId(id) {
    if (!id) return;
    if (marker_landing[id]) { map.removeLayer(marker_landing[id]); delete marker_landing[id]; }
    if (marker_burst[id]) { map.removeLayer(marker_burst[id]); delete marker_burst[id]; }
    if (line_predict[id]) { map.removeLayer(line_predict[id]); delete line_predict[id]; }
    delete dots_predict[id];
  }

  function removeLastRxCircle(ser) {
    if (lastRxMarkers[ser]) {
      map.removeLayer(lastRxMarkers[ser]);
      delete lastRxMarkers[ser];
    }
  }

  function removeLastRxTrack(ser) {
    if (lastRxTracks[ser]) {
      map.removeLayer(lastRxTracks[ser]);
      delete lastRxTracks[ser];
      clearTrackInspect();
    }
  }

  function hideLastRxSer(ser) {
    removeLastRxCircle(ser);
    removeLastRxTrack(ser);
    /* Only clear prediction if this id is not the live sonde marker */
    if (!liveOwnsSer(ser)) clearPredictForId(ser);
  }

  function showLastRxPredict(e) {
    if (liveOwnsSer(e.ser)) return;
    var cached = predCacheLoad(e.ser);
    if (cached) {
      draw_predict(cached.prediction, {
        id: e.ser, lat: e.lat, lon: e.lon, alt: e.alt, climb: e.vs, time: e.time
      }, true);
    }
  }

  function lastRxTipData(e) {
    return {
      id: e.ser,
      ser: e.ser,
      alt: e.alt,
      climb: e.vs,
      speed: e.hs,
      freq: e.freq
    };
  }

  function renderLastRxList() {
    var box = document.getElementById('last-rx-list');
    if (!box) return;
    var sdKeys = Object.keys(sdOverlays);
    if (!lastRxEntries.length && !sdKeys.length) {
      box.innerHTML = '<div class="last-rx-empty">No last_rx.csv</div>';
      return;
    }
    var html = '';
    lastRxEntries.forEach(function(e) {
      var on = lastRxShown[e.ser] !== false;
      var when = formatLastRxListTime(e.time);
      html += '<label class="basemap-option last-rx-item">' +
        '<input type="checkbox" data-ser="' + e.ser + '"' + (on ? ' checked' : '') + '/>' +
        '<span class="last-rx-body">' +
          '<span class="last-rx-main"><span class="last-rx-id">' + e.ser + '</span> ' +
          '<span class="basemap-key-hint">' + (e.type || '') + ' ' + Math.round(e.alt) + 'm</span></span>' +
          (when ? '<span class="last-rx-time">' + when + '</span>' : '') +
        '</span></label>';
    });
    sdKeys.forEach(function(key) {
      var o = sdOverlays[key];
      var on = o.shown !== false;
      var type = (o.data && o.data.type) ? o.data.type : '';
      var hint = type;
      if (o.data && o.data.alt) {
        hint += (hint ? ' ' : '') + Math.round(o.data.alt) + 'm';
      } else if (!hint && o.path) {
        hint = sdOverlayBasename(o.path);
      }
      var when = formatLastRxListTime(o.data && o.data.time);
      html += '<label class="basemap-option last-rx-item">' +
        '<input type="checkbox" data-sd-key="' + key.replace(/"/g, '') + '"' + (on ? ' checked' : '') + '/>' +
        '<span class="last-rx-body">' +
          '<span class="last-rx-main"><span class="last-rx-id">' + (o.label || sdOverlayBasename(o.path)) + '</span> ' +
          '<span class="basemap-key-hint">' + hint +
          ' <span class="last-rx-file-icon" title="from file">📄</span></span></span>' +
          (when ? '<span class="last-rx-time">' + when + '</span>' : '') +
        '</span></label>';
    });
    box.innerHTML = html;
    box.querySelectorAll('input[data-ser]').forEach(function(inp) {
      inp.addEventListener('change', function() {
        var ser = inp.getAttribute('data-ser');
        lastRxShown[ser] = inp.checked;
        if (!inp.checked) hideLastRxSer(ser);
        else syncLastRxMarkers();
      });
    });
    box.querySelectorAll('input[data-sd-key]').forEach(function(inp) {
      inp.addEventListener('change', function() {
        var key = inp.getAttribute('data-sd-key');
        var o = sdOverlays[key];
        if (!o) return;
        o.shown = inp.checked;
        if (!inp.checked) removeSdOverlayLayers(key);
        else showSdOverlay(key);
      });
    });
  }

  function syncLastRxMarkers() {
    var keepCircle = {};
    var keepTrack = {};
    lastRxEntries.forEach(function(e) {
      if (lastRxShown[e.ser] === false) return;
      keepTrack[e.ser] = true;
      if (!lastRxTracks[e.ser]) loadLightTrack(e.ser);

      var owned = liveOwnsSer(e.ser);
      if (owned) {
        removeLastRxCircle(e.ser);
        return;
      }

      keepCircle[e.ser] = true;
      var loc = [e.lat, e.lon];
      var tip = lastRxTipData(e);
      var ts = formatSondeTime(e.time);
      var popupHtml =
        '<b>' + e.ser + '</b> (last RX)<br/>' + (e.type || '') +
        '<br/>' + e.lat.toFixed(5) + ', ' + e.lon.toFixed(5) +
        '<br/>' + Math.round(e.alt) + 'm' +
        (ts ? '<br/>' + ts : '');
      if (!lastRxMarkers[e.ser]) {
        lastRxMarkers[e.ser] = L.circleMarker(loc, {
          radius: 7, color: '#888', fillColor: '#bbb', fillOpacity: 0.85, weight: 2
        }).addTo(map)
          .bindPopup(popupHtml)
          .bindTooltip(tiphtml(tip), {
            permanent: true,
            direction: 'right',
            className: 'sondeTooltip',
            offset: [10, -16],
            opacity: 0.75,
            interactive: false
          });
        showLastRxPredict(e);
      } else {
        lastRxMarkers[e.ser].setLatLng(loc);
        lastRxMarkers[e.ser].setPopupContent(popupHtml);
        lastRxMarkers[e.ser].setTooltipContent(tiphtml(tip));
      }
    });
    Object.keys(lastRxMarkers).forEach(function(ser) {
      if (!keepCircle[ser]) removeLastRxCircle(ser);
    });
    /* Keep tracks for checked last_rx rows and for the live sonde (even before last_rx lists it). */
    Object.keys(lastRxTracks).forEach(function(ser) {
      if (lastRxShown[ser] === false) {
        removeLastRxTrack(ser);
        return;
      }
      if (keepTrack[ser] || liveOwnsSer(ser)) return;
      removeLastRxTrack(ser);
    });
  }

  function loadLightTrack(ser) {
    if (lastRxShown[ser] === false) return;
    $.ajax({
      url: '/file/lt_' + encodeURIComponent(ser) + '.csv',
      dataType: 'text',
      success: function(txt) {
        if (lastRxShown[ser] === false) return;
        var pts = parseTrackCsv(txt);
        if (pts.length < 2) return;
        if (lastRxTracks[ser]) map.removeLayer(lastRxTracks[ser]);
        lastRxTracks[ser] = L.polyline(ptsToLatLngs(pts), HIST_LINE_STYLE).addTo(map);
        bindTrackInspect(lastRxTracks[ser], pts, ser);
        /* Late lt_* load: upgrade live trail bridge from last_rx end to hist end. */
        if (liveOwnsSer(ser)) {
          var liveKey = liveKeyForSer(ser);
          if (liveKey && dots[liveKey] && dots[liveKey].length) {
            var liveLoc = dots[liveKey][dots[liveKey].length - 1];
            ensureLiveTrailBridge(liveKey, ser, liveLoc);
          }
        }
      }
    });
  }

  function loadLastRx() {
    $.ajax({
      url: '/file/last_rx.csv',
      dataType: 'text',
      success: function(txt) {
        lastRxEntries = [];
        parseCsvLines(txt).forEach(function(p) {
          if (p.length < 7) return;
          var e = {
            ser: p[0], type: p[1],
            lat: parseFloat(p[2]), lon: parseFloat(p[3]),
            alt: parseFloat(p[4]), freq: parseFloat(p[5]),
            time: parseInt(p[6], 10) || 0,
            vs: parseFloat(p[7]) || 0, hs: parseFloat(p[8]) || 0,
            dir: parseFloat(p[9]) || 0
          };
          if (!e.ser || isNaN(e.lat) || isNaN(e.lon)) return;
          if (lastRxShown[e.ser] === undefined) lastRxShown[e.ser] = true;
          lastRxEntries.push(e);
        });
        renderLastRxList();
        syncLastRxMarkers();
      },
      error: function() {
        lastRxEntries = [];
        renderLastRxList();
        syncLastRxMarkers();
      }
    });
  }

  function loadSdOverlays() {
    var list = [];
    try {
      list = JSON.parse(sessionStorage.getItem('rdzMapOverlay') || '[]');
      if (!Array.isArray(list)) list = [];
    } catch (e) { list = []; }
    if (!list.length) return;
    sessionStorage.removeItem('rdzMapOverlay');
    list.forEach(function(path) {
      $.ajax({
        url: path,
        dataType: 'text',
        success: function(txt) {
          var parsed = parseSdTrackCsv(txt);
          if (!parsed.pts || parsed.pts.length < 2) return;
          var key = sdOverlayKey(path);
          var base = sdOverlayBasename(path);
          var data = parsed.last || {};
          if (!data.id) data.id = data.ser || base.replace(/\.csv$/i, '');
          if (typeof data.climb !== 'number') data.climb = 0;
          if (typeof data.speed !== 'number') data.speed = 0;
          if (typeof data.alt !== 'number') data.alt = 0;
          /* End of track may differ from downsampled polyline end — use last raw point */
          if ((!data.lat || !data.lon) && parsed.pts.length) {
            var end = parsed.pts[parsed.pts.length - 1];
            data.lat = end.lat != null ? end.lat : end[0];
            data.lon = end.lon != null ? end.lon : end[1];
            if (!data.time && end.t) data.time = end.t;
          }
          removeSdOverlayLayers(key);
          sdOverlays[key] = {
            path: path,
            label: data.id || base,
            pts: parsed.pts,
            data: data,
            shown: true,
            layer: null,
            marker: null
          };
          showSdOverlay(key);
          renderLastRxList();
          try {
            if (sdOverlays[key].layer) {
              map.fitBounds(sdOverlays[key].layer.getBounds(), { padding: [20, 20] });
            }
          } catch (e2) {}
        }
      });
    });
  }

  loadLastRx();
  setInterval(loadLastRx, 60000);
  loadSdOverlays();

});

L.Control.Button = L.Control.extend({
  onAdd: function() {
    var container = L.DomUtil.create('div', 'leaflet-bar leaflet-control leaflet-control-btnstrip');
    var options = this.options;
    Object.keys(options).forEach(function(key) {
      if (key === 'position') return;
      var opt = options[key];
      if (!opt || typeof opt !== 'object') return;
      var link = L.DomUtil.create('a', opt.className || '', container);
      if (opt.html != null && opt.html !== '') link.innerHTML = opt.html;
      else if (opt.text) link.textContent = opt.text;
      link.href = opt.href || '#';
      if (opt.id) link.id = opt.id;
      if (opt.title) link.title = opt.title;
      link.setAttribute('role', 'button');
    });
    if (this.options[0] && this.options[0].position) {
      this.options.position = this.options[0].position;
    }
    return container;
  }
});

// https://github.com/makinacorpus/Leaflet.GeometryUtil/blob/master/src/leaflet.geometryutil.js#L682
// modified to fit
function bearing(latlng1, latlng2) {
  var rad = Math.PI / 180,
    lat1 = latlng1[0] * rad,
    lat2 = latlng2[0] * rad,
    lon1 = latlng1[1] * rad,
    lon2 = latlng2[1] * rad,
    y = Math.sin(lon2 - lon1) * Math.cos(lat2),
    x = Math.cos(lat1) * Math.sin(lat2) -
      Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
  var brng = ((Math.atan2(y, x) * 180 / Math.PI) + 360) % 360;
  brng = brng < 0 ? brng - 360 : brng;
  return Math.round(brng);
}
