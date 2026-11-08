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
  var lastframe = 0;

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

  var statbar = '';
  headtxt = function(data, stat) {
    var staticon = (stat == '1') ? greendot : yellowdot;
    statbar = staticon + statbar;
    if ((statbar.length) > 10 * greendot.length) { statbar = statbar.substring(0, 10 * greendot.length); }
    if (data.id && data.vframe != lastframe) {
      lastframe = data.vframe;
      $('#sonde_id').html(data.id);
      $('#sonde_alt').html(data.alt);
      $('#sonde_climb').html(data.climb);
      $('#sonde_speed').html(mr(data.speed * 3.6 * 10) / 10);
      $('#sonde_dir').html(data.dir);
      $('#sonde_time').html(new Date(data.time * 1000).toISOString());
      $('#sonde_rssi').html(data.rssi / 2);
      $('#sonde_detail').show();
    } else {
      if (!data.id) {
        $('#sonde_id').html(data.launchsite.trim());
      }
    }
    $('#sonde_freq').html(data.freq);
    $('#sonde_type').html(data.type);
    $('#sonde_statbar').html('&nbsp;' + statbar);
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
    if (p == 'landing') { get_predict(last_data); }
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
    var alt = (typeof data.alt === 'number') ? data.alt.toFixed(0) : data.alt;
    var vs = (typeof data.climb === 'number') ? data.climb.toFixed(1) : data.climb;
    var hs = mr(data.speed * 3.6 * 10) / 10;
    return '<div class="tooltip-container">' + data.id +
      '<div class="text-speed">' + alt + 'm ' + vs + 'm/s ' + hs + 'km/h</div></div>';
  };

  draw = function(data) {
    var stat;
    if (data.id) {
      last_id = data.id;
      // data.res: 0: ok  1: no rx (timeout), 2: crc err, >2 some other error
      if ((data.lat && data.lon && data.alt) && (lastframe != 0)) {
        var location = [data.lat, data.lon, data.alt];
        var ic = sondeIconFor(data);
        if (!marker[data.id]) {
          map.setView(location, 14);
          marker[data.id] = L.marker(location, { icon: ic }).addTo(map)
            .bindPopup(poptxt('position', data), { closeOnClick: false, autoPan: false })
            .bindTooltip(tiphtml(data), {
              permanent: true,
              direction: 'right',
              className: 'sondeTooltip',
              offset: [10, -16],
              opacity: 0.75,
              interactive: false
            })
            .openPopup();
          get_predict(data);
        } else {
          marker[data.id].setIcon(ic);
          marker[data.id].slideTo(location, {
            duration: 500,
            keepAtCenter: (follow == 'marker') ? true : false
          })
            .setPopupContent(poptxt('position', data))
            .setTooltipContent(tiphtml(data));
        }
        if (!dots[data.id]) { dots[data.id] = []; }
        dots[data.id].push(location);
        if (!line[data.id]) {
          line[data.id] = L.polyline(dots[data.id]).addTo(map);
        } else {
          line[data.id].setLatLngs(dots[data.id]);
        }
      }
      if (data.res == 0) {
        storage_write(data);
        $('#status').html(greendot);
        stat = 1;
      } else {
        $('#status').html(yellowdot);
        stat = 0;
      }
      headtxt(data, stat);
      last_data = data;
    } else {
      $('#status').html(yellowdot);
      headtxt(data, 0);
    }
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
    $('#status').html(reddot);
    $.ajax({
      url: 'live.json',
      success: (function(data) {
        if (typeof data != 'object') { data = $.parseJSON(data); }
        if (data.sonde) {
          draw(data.sonde);
        } else {
          setTimeout(function() { $('#status').html(yellowdot); }, 100);
        }
        if (data.gps && !phone_gps_active) {
          gps(data.gps);
        }
      }),
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
      get_predict(last_data);
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

  predictor = false;

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

  get_predict = function(data) {
    if (!data) { return; }
    var ascent = (data.climb > 0) ? data.climb : 15;
    var descent, burst;
    if (data.climb > 0) {
      descent = settings.overwrite_descend;
      burst = (data.alt > settings.burst) ? data.alt + 100 : settings.burst;
    } else {
      burst = parseInt(data.alt) + 7;
      descent = (data.alt > settings.overwrite_descend_till)
        ? settings.overwrite_descend
        : sea_level_descent(data.climb, data.alt);
    }

    var m = new Date(data.time * 1000);
    var datetime = m.toISOString().slice(0, 19) + 'Z';
    var url = 'https://api.v2.sondehub.org/tawhiri';
    url += '?launch_latitude=' + data.lat + '&launch_longitude=' + tawhiri_lon(data.lon);
    url += '&launch_altitude=' + data.alt + '&launch_datetime=' + datetime;
    url += '&ascent_rate=' + ascent + '&burst_altitude=' + burst + '&descent_rate=' + descent;

    $.getJSON(url, function(prediction) {
      draw_predict(prediction, data);
    });
  };

  draw_predict = function(prediction, data) {
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
        keepAtCenter: (follow == 'landing') ? true : false
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
          keepAtCenter: (follow == 'burst') ? true : false
        }).setPopupContent(poptxt('burst', highest));
      }
    }

    descending.forEach(function(p) { dots_predict[data.id].push([p.latitude, sanitize_lon(p.longitude)]); });

    if (!line_predict[data.id]) {
      line_predict[data.id] = L.polyline(dots_predict[data.id], { color: 'yellow' }).addTo(map);
    } else {
      line_predict[data.id].setLatLngs(dots_predict[data.id]);
    }

    if (data.climb > 0) {
      predictor_time = 5 * 60; // ascending, every 5 min
    } else if (data.climb < 0 && data.alt > 5000) {
      predictor_time = 2 * 60; // descending, above 5km, every 2 min
    } else {
      predictor_time = 30; // descending, below 5km, every 30 sec
    }
    clearTimeout(predictor);
    predictor = setTimeout(function() { get_predict(last_data); }, predictor_time * 1000);
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
      dots.push([d.lat, d.lon, d.alt]);
      session_storage_last = d;
    });
    draw(session_storage_last);
  }

  setInterval(get_data, 1000);

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
