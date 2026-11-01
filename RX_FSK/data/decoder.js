(function () {
  var POLL_MS = 1000;
  var STALE_SEC = 5; // after this, last OK/error is shown as stale
  var lastNow = null;
  var cards = {}; // ch -> { el, data }
  var placeholderCh = null;
  var currentCh = null;

  var statusEl = document.getElementById("decoder-status");
  var statusText = document.getElementById("decoder-status-text");
  var listEl = document.getElementById("decoder-list");
  var spectrumEl = document.getElementById("spectrum");
  var spectrumCanvas = null;
  var spectrumLabel = null;

  function esc(s) {
    if (s == null) return "";
    return String(s)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  function fmtNum(v, digits) {
    if (v == null || typeof v !== "number" || isNaN(v)) return "—";
    return v.toFixed(digits);
  }

  function stampRxAt(d) {
    if (d.empty) {
      d._rxAt = null;
      d._ageFrozen = null;
      return;
    }
    var age = (typeof d.age === "number") ? d.age : 0;
    d._rxAt = Date.now() - age * 1000;
    d._ageFrozen = (age >= 86400) ? 86400 : null;
  }

  function effectiveAge(d) {
    if (!d || d.empty || d._rxAt == null) return d && d.age != null ? d.age : null;
    if (d._ageFrozen) return d._ageFrozen;
    var sec = Math.max(0, Math.floor((Date.now() - d._rxAt) / 1000));
    if (sec >= 86400) {
      d._ageFrozen = 86400;
      return 86400;
    }
    return sec;
  }

  function formatAge(sec) {
    if (sec == null) return "";
    if (sec >= 86400) return "> 1 day ago";
    if (sec >= 3600) return Math.floor(sec / 3600) + "h ago";
    if (sec >= 60) return Math.floor(sec / 60) + "m ago";
    return sec + "s ago";
  }

  function resLabel(res) {
    switch (res) {
      case 0: return { text: "OK", cls: "res-ok" };
      case 2: return { text: "partial/error", cls: "res-err" };
      case 4: return { text: "no position", cls: "res-nopos" };
      case 1: return { text: "timeout", cls: "res-timeout" };
      default: return { text: "?", cls: "res-unk" };
    }
  }

  function resLabelFor(d) {
    var age = effectiveAge(d);
    if (age != null && age > STALE_SEC) return { text: "stale", cls: "res-stale" };
    return resLabel(d.res);
  }

  function buildLinks(d) {
    var parts = [];
    var hasPos = d.lat != null && d.lon != null && ((d.validPos || 0) & 0x03) === 0x03;
    if (d.id) {
      parts.push('<a target="_blank" href="https://radiosondy.info/sonde_archive.php?sondenumber=' +
        encodeURIComponent(d.id) + '">radiosondy.info</a>');
    }
    if (d.ser) {
      parts.push('<a target="_blank" href="https://tracker.sondehub.org/' +
        encodeURIComponent(d.ser) + '">SondeHub Tracker</a>');
    }
    if (hasPos) {
      parts.push('<a target="_blank" href="https://www.openstreetmap.org/?mlat=' +
        d.lat + '&mlon=' + d.lon + '&zoom=14">OSM</a>');
      parts.push('<a target="_blank" href="https://www.google.com/maps/search/?api=1&query=' +
        d.lat + ',' + d.lon + '">Google</a>');
      parts.push('<a target="_blank" href="geo:' + d.lat + ',' + d.lon + '">Geo-App</a>');
    }
    return parts.length ? parts.join(" · ") : "";
  }

  function cardHtml(d) {
    var empty = !!d.empty;
    var rl = resLabelFor(d);
    var age = effectiveAge(d);
    var hasPos = !empty && d.lat != null && d.lon != null && ((d.validPos || 0) & 0x03) === 0x03;
    var site = d.launchsite ? esc(d.launchsite) : "";

    var idText = "";
    if (empty) {
      idText = "No decode yet";
    } else if (d.id || d.ser) {
      idText = esc(d.id || "");
      if (d.ser && d.ser !== d.id) {
        idText += (idText ? " (ser: " + esc(d.ser) + ")" : esc(d.ser));
      }
      if (!idText) idText = "<?>";
      if (d.type) idText += " · " + esc(d.type);
    } else {
      idText = d.type ? esc(d.type) : "<?>";
    }

    var stats = [];
    if (!empty) {
      if (d.frame != null) stats.push("Frame# " + d.frame);
      if (d.time) {
        var t = new Date(d.time * 1000);
        if (!isNaN(t.getTime())) {
          stats.push(t.toISOString().replace("T", " ").replace(/\.\d+Z$/, "Z"));
        }
      }
      if (d.rssi != null) stats.push("RSSI " + d.rssi);
      if (d.sats != null) stats.push("Sats=" + d.sats);
      if (d.batt != null) stats.push(fmtNum(d.batt, 1) + " V");
    }

    var posParts = [];
    if (hasPos) {
      posParts.push("Pos " + fmtNum(d.lat, 6) + " " + fmtNum(d.lon, 6));
      if (d.alt != null) posParts.push("Alt " + fmtNum(d.alt, 0) + "m");
    }
    if (!empty) {
      if (d.vs != null) posParts.push("Clb " + fmtNum(d.vs, 1) + " m/s");
      if (d.hs != null) posParts.push("Spd " + fmtNum(d.hs, 1) + " m/s");
      if (d.dir != null) posParts.push("Dir " + fmtNum(d.dir, 0) + "°");
    }
    var posLine = posParts.length ? posParts.join(", ") : (empty ? "" : "—");

    var meteo = [];
    if (!empty) {
      if (d.temp != null) meteo.push(fmtNum(d.temp, 1) + " °C");
      if (d.humidity != null) meteo.push(fmtNum(d.humidity, 1) + " %RH");
      if (d.pressure != null) meteo.push(fmtNum(d.pressure, 1) + " hPa");
    }

    var kt = "";
    if (!empty && (d.burstKT != null || d.launchKT != null || d.countKT != null)) {
      kt = "Burst-KT=" + (d.burstKT != null ? d.burstKT : "?") +
        " Launch-KT=" + (d.launchKT != null ? d.launchKT : "?") +
        " Countdown=" + (d.countKT != null ? d.countKT : "?");
    }

    var links = empty ? "" : buildLinks(d);
    var freqHtml = fmtNum(d.freq, 3) + " MHz" + (site ? " (" + site + ")" : "");

    return (
      '<div class="decoder-card-head">' +
        '<span class="decoder-ch">Ch ' + esc(d.ch) + '</span>' +
        '<span class="decoder-freq">' + freqHtml + '</span>' +
        (empty ? "" : '<span class="decoder-res ' + rl.cls + '">' + rl.text + '</span>') +
        '<span class="decoder-age">' + (empty || age == null ? "" : formatAge(age)) + '</span>' +
      '</div>' +
      '<div class="decoder-card-id">' +
        '<span class="decoder-id">' + idText + '</span>' +
        (stats.length ? '<span class="decoder-stats">' + esc(stats.join(", ")) + '</span>' : '') +
      '</div>' +
      (posLine ? '<div class="decoder-card-qth">' + esc(posLine) + '</div>' : '') +
      (meteo.length ? '<div class="decoder-card-meteo">' + esc(meteo.join(" · ")) + '</div>' : '') +
      (kt ? '<div class="decoder-card-kt">' + esc(kt) + '</div>' : '') +
      (links ? '<div class="decoder-card-links">' + links + '</div>' : '')
    );
  }

  function ensureCard(ch) {
    if (cards[ch]) return cards[ch];
    var el = document.createElement("div");
    el.className = "decoder-card";
    el.dataset.ch = String(ch);
    listEl.appendChild(el);
    cards[ch] = { el: el, data: null };
    return cards[ch];
  }

  function removeCard(ch) {
    var c = cards[ch];
    if (!c) return;
    if (c.el.parentNode) c.el.parentNode.removeChild(c.el);
    delete cards[ch];
    if (placeholderCh === ch) placeholderCh = null;
  }

  function sortCards() {
    var keys = Object.keys(cards).map(Number);
    keys.sort(function (a, b) {
      var da = cards[a].data || {};
      var db = cards[b].data || {};
      if (da.empty && !db.empty) return 1;
      if (!da.empty && db.empty) return -1;
      var aa = effectiveAge(da);
      var ab = effectiveAge(db);
      if (aa == null) aa = 1e9;
      if (ab == null) ab = 1e9;
      if (aa !== ab) return aa - ab;
      return a - b;
    });
    keys.forEach(function (ch) {
      listEl.appendChild(cards[ch].el);
    });
  }

  function renderCard(ch) {
    var c = cards[ch];
    if (!c || !c.data) return;
    c.el.className = "decoder-card" +
      (ch === currentCh ? " current" : "") +
      (c.data.empty ? " empty" : "");
    c.el.innerHTML = cardHtml(c.data);
  }

  function refreshAges() {
    Object.keys(cards).forEach(function (k) {
      renderCard(Number(k));
    });
    sortCards();
  }

  function applyStatus(data) {
    statusText.textContent = data.detail || data.state || "";
    statusEl.className = "decoder-status state-" + (data.state || "disabled");
    currentCh = data.ch;
  }

  function renderSpectrum(spec) {
    if (!spectrumEl) return;
    if (!spec || !spec.rssi || !spec.rssi.length) {
      spectrumEl.hidden = true;
      return;
    }
    spectrumEl.hidden = false;
    if (!spectrumCanvas) {
      spectrumCanvas = document.createElement("canvas");
      spectrumCanvas.width = 600;
      spectrumCanvas.height = 240;
      spectrumEl.appendChild(spectrumCanvas);
      spectrumLabel = document.createElement("div");
      spectrumLabel.className = "spectrum-label";
      spectrumEl.appendChild(spectrumLabel);
    }
    // Same fixed Y scale as LCD/TFT: PLOT_MIN = noisefloor*2, PLOT_SCALE = (x-PLOT_MIN)/2
    var rssi = spec.rssi;
    var n = rssi.length;
    var floor = (spec.noisefloor != null ? spec.noisefloor : -125) * 2;
    var plotMax = spec.plotMax > 0 ? spec.plotMax : 128;
    var ctx = spectrumCanvas.getContext("2d");
    var w = spectrumCanvas.width;
    var h = spectrumCanvas.height;
    ctx.fillStyle = "#222";
    ctx.fillRect(0, 0, w, h);
    var barW = w / n;
    ctx.fillStyle = "#4c4";
    for (var j = 0; j < n; j++) {
      var scaled = rssi[j] < floor ? 0 : (rssi[j] - floor) / 2;
      var bh = Math.min(h, Math.round((scaled / plotMax) * h));
      if (bh > 0) {
        ctx.fillRect(Math.floor(j * barW), h - bh, Math.ceil(barW) || 1, bh);
      }
    }
    if (spec.peak != null && spec.start != null && spec.step != null && spec.step > 0) {
      var px = ((spec.peak - spec.start) / spec.step) * barW;
      ctx.strokeStyle = "#f66";
      ctx.beginPath();
      ctx.moveTo(px, 0);
      ctx.lineTo(px, h);
      ctx.stroke();
    }
    var end = (spec.start != null && spec.step != null)
      ? spec.start + spec.step * n : null;
    spectrumLabel.textContent =
      (spec.start != null ? spec.start.toFixed(3) : "?") + " … " +
      (end != null ? end.toFixed(3) : "?") + " MHz" +
      (spec.peak != null ? "  |  peak " + spec.peak.toFixed(3) + " MHz" : "") +
      (spec.peakRssi != null ? " (" + spec.peakRssi + ")" : "");
  }

  function applyFull(data) {
    Object.keys(cards).forEach(function (ch) { removeCard(Number(ch)); });
    placeholderCh = null;
    var entries = data.entries || [];
    entries.forEach(function (e) {
      stampRxAt(e);
      var c = ensureCard(e.ch);
      c.data = e;
      if (e.empty) placeholderCh = e.ch;
      renderCard(e.ch);
    });
    sortCards();
  }

  function applyDelta(data) {
    var entries = data.entries || [];
    entries.forEach(function (e) {
      if (e.empty) return;
      if (placeholderCh != null) removeCard(placeholderCh);
      stampRxAt(e);
      var c = ensureCard(e.ch);
      c.data = e;
      renderCard(e.ch);
    });

    Object.keys(cards).forEach(function (k) {
      renderCard(Number(k));
    });
    sortCards();
  }

  function apply(data) {
    if (!data || typeof data !== "object") return;
    applyStatus(data);
    renderSpectrum(data.spectrum);
    if (data.full || lastNow == null) {
      applyFull(data);
    } else {
      applyDelta(data);
    }
    if (typeof data.now === "number") lastNow = data.now;
  }

  function poll() {
    var url = "/decoder.json";
    if (lastNow != null) url += "?since=" + lastNow;
    fetch(url)
      .then(function (r) {
        if (!r.ok) throw new Error("HTTP " + r.status);
        return r.json();
      })
      .then(apply)
      .catch(function (err) {
        console.error("decoder poll:", err);
        lastNow = null;
        statusText.textContent = "Connection error — retrying…";
        statusEl.className = "decoder-status state-disabled";
      })
      .finally(function () {
        setTimeout(poll, POLL_MS);
      });
  }

  setInterval(refreshAges, 1000);
  poll();
})();
