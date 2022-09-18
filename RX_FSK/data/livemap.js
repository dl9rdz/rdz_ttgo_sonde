try {
  var check = $(document);
} catch (e) {
  document.addEventListener("DOMContentLoaded", function(event) {
    document.getElementById('map').innerHTML = '<br /><br />In order to use this functionality, there must be an internet connection.<br /><br/><a href="livemap.html">retry</a><br /><br/><a href="index.html">go back</a>';
  });
}

$(document).ready(function(){

  var map = L.map('map', { attributionControl: false, zoomControl: false });
  map.on('mousedown touchstart',function () { follow=false; });

  L.control.scale().addTo(map);
  L.control.attribution({prefix:false}).addTo(map);
  
  var osmlight = L.tileLayer('https://{s}.tile.openstreetmap.de/{z}/{x}/{y}.png', {
  	maxZoom: 19,
  	attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
  });

  var osmdark = L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png', {
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors &copy; <a href="https://carto.com/attributions">CARTO</a>',
    maxZoom: 19
  });
  
  var opentopo = L.tileLayer('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png', {
  	maxZoom: 17,
  	attribution: 'Map data: &copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors, <a href="http://viewfinderpanoramas.org">SRTM</a><br />Map style: &copy; <a href="https://opentopomap.org">OpenTopoMap</a> (<a href="https://creativecommons.org/licenses/by-sa/3.0/">CC-BY-SA</a>)'
  });

  var esri = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
  	attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye,<br />Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community',
    maxZoom: 21
  });
  
  var basemap;
  if (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches) {
    map.addLayer(osmdark);
    basemap='osmdark';
  } else {
    map.addLayer(osmlight);
    basemap='osmlight';
  }
  
  basemap_change = function () {
    if (basemap == 'osmlight') {
      map.removeLayer(osmlight);
      map.addLayer(opentopo);
      basemap = 'opentopo';
    } else if (basemap == 'opentopo') {
      map.removeLayer(opentopo);
      map.addLayer(esri);
      basemap = 'esri';
    } else if (basemap == 'esri') {
      map.removeLayer(esri);
      map.addLayer(osmdark);
      basemap = 'osmdark';
    } else {
      map.removeLayer(osmdark);
      map.addLayer(osmlight);
      basemap = 'osmlight';
    }
  };

  if(mapcenter) map.setView(mapcenter, 5); 
  else map.setView([51.163361,10.447683], 5); // Mitte DE

var reddot = '<span class="ldot rbg"></span>';
var yellowdot = '<span class="ldot ybg"></span>';
var greendot = '<span class="ldot gbg"></span>';
var lastframe = 0;

$('#map .leaflet-control-container').append(L.DomUtil.create('div', 'leaflet-top leaflet-center leaflet-header'));
var header = '';
header += '<div id="sonde_main"><b>rdzTTGOSonde LiveMap</b><br />🎈 <b><span id="sonde_id"></span> - <span id="sonde_freq"></span> MHz - <span id="sonde_type"></span></b></div>';
header += '<div id="sonde_detail"><span id="sonde_alt"></span>m | <span id="sonde_climb"></span>m/s | <span id="sonde_speed"></span>km/h | <span id="sonde_dir"></span>°<br /><span id="sonde_time"></span> | -<span id="sonde_rssi"></span>dBm</div>';
header += '<div id="sonde_status"><span id="sonde_statbar"></span></div>';
header += '<div id="settings"><br /><b>Prediction-Settings</b><br />';

header += '<label for="burst">Burst at:</label><input type="text" id="burst" maxlength="5" value="..."  /> m<br />';
header += '<label for="overwrite_descend">Descending:</label><input type="text" id="overwrite_descend" maxlength="2" value="..."  /> m/s<br />';
header += '<label for="overwrite_descend_till">Use this descending until:</label><input type="text" id="overwrite_descend_till" maxlength="5" value="..."  /> m<br />';
header += '<small>after the transmitted descend will be used</small>';
header += '<div id="submit"><input type="button" value="save" onclick="settings_save();"/>&nbsp;&nbsp;&nbsp;<input type="button" id="submit" value="reset" onclick="settings_reset();"/></div>';
header += '</div>';
$('.leaflet-header').append(header);


$('#map .leaflet-control-container').append(L.DomUtil.create('div', 'leaflet-bottom leaflet-center leaflet-footer'));
var footer = '';
footer += '<div id="gps_main"><b>Direction: </b><span class="gps_dir">...</span>°<br /><b>Distance: </b><span class="gps_dist">...</span>m</div>';
$('.leaflet-footer').append(footer);

var statbar = '';
headtxt = function(data,stat) {
  var staticon = (stat == '1')?greendot:yellowdot; 
  statbar = staticon + statbar;
  if ((statbar.length) > 10*greendot.length) { statbar = statbar.substring(0,10*greendot.length); }
  if (data.id && data.vframe != lastframe ) {
    lastframe = data.vframe;
    $('#sonde_id').html(data.id);
    $('#sonde_alt').html(data.alt);
    $('#sonde_climb').html(data.climb);
    $('#sonde_speed').html( mr(data.speed * 3.6 * 10) / 10 );
    $('#sonde_dir').html(data.dir);
    $('#sonde_time').html(new Date(data.time * 1000).toISOString());
    $('#sonde_rssi').html(data.rssi / 2 );
    $('#sonde_detail').show();
  } else {
    if (!data.id) {
    $('#sonde_id').html(data.launchsite.trim());
    // $('#sonde_detail').hide();
    }
  }
  $('#sonde_freq').html(data.freq);
  $('#sonde_type').html(data.type);
  $('#sonde_statbar').html('&nbsp;'+statbar);
};

map.addControl(new L.Control.Button([ { position: 'topleft', text: '🔙', href: 'index.html' } ]));
  
L.control.zoom({ position:'topleft' }).addTo(map);

map.addControl(new L.Control.Button([ { position: 'topleft', text: '🗺️', href: 'javascript:basemap_change();' } ]));

map.addControl(new L.Control.Button([ { position: 'topright', id: "status", text: '', href: 'javascript:get_data();' } ]));

map.addControl(new L.Control.Button([
  { position:'topright', text: '🎈', href: 'javascript:show(marker[last_id],\'marker\');' },
  { text: '〰️', href: 'javascript:show_line();' },
  { text: '💥', href: 'javascript:show(marker_burst[last_id],\'burst\');' },
  { text: '🎯', href: 'javascript:show(marker_landing[last_id],\'landing\');' }
]));

map.addControl(new L.Control.Button([ { position:'topright', text: '⚙️', href: 'javascript:show_settings();' } ]));

  
    
  show = function(e,p) {
    if (p == 'landing') { get_predict(last_data); }
    if (e) {
      map.closePopup();
      map.setView(map._layers[e._leaflet_id].getLatLng());
      map._layers[e._leaflet_id].openPopup();
      follow = p;
    }
  };


  getTwoBounds = function (a,b) {
    var sW = new L.LatLng((a._southWest.lat > b._southWest.lat)?b._southWest.lat:a._southWest.lat, (a._southWest.lng > b._southWest.lng)?b._southWest.lng:a._southWest.lng);
    var nE = new L.LatLng((a._northEast.lat < b._northEast.lat)?b._northEast.lat:a._northEast.lat, (a._northEast.lng < b._northEast.lng)?b._northEast.lng:a._northEast.lng);

    return new L.LatLngBounds(sW, nE);
  };

  show_line = function() {
      $('.i_position, .i_landing').remove();
      map.closePopup();
      if (line[last_id]._latlngs.length != 0 && line_predict[last_id]._latlngs.length != 0) {
        map.fitBounds(getTwoBounds(line[last_id].getBounds(),line_predict[last_id].getBounds()));
      } else if (line[last_id]._latlngs.length != 0) {
        map.fitBounds(line[last_id].getBounds());
      } else if (line_predict[last_id]._latlngs.length != 0) {
        map.fitBounds(line_predict[last_id].getBounds());
      }
  };



  last_data = false;
  last_id = false;
  follow = 'marker';

  marker_landing = [];
  icon_landing = L.divIcon({className: 'leaflet-landing'});
  dots_predict = [];
  line_predict = [];
  marker_burst = []; 
  icon_burst = L.divIcon({className: 'leaflet-burst'});

  marker = [];
  dots = [];
  line = [];

  draw = function(data) {
    var stat;
    if (data.id) {
      last_id = data.id;
      // data.res: 0: ok  1: no rx (timeout), 2: crc err, >2 some other error
      if ((data.lat && data.lon && data.alt) && (lastframe != 0)) {
        var location = [data.lat,data.lon,data.alt];
        if (!marker[data.id]) {
          map.setView(location, 14);
          marker[data.id] = L.marker(location).addTo(map)
          .bindPopup(poptxt('position',data),{closeOnClick:false, autoPan:false}).openPopup();
          get_predict(data);
        } else {
          marker[data.id].slideTo(location, {
              duration: 500,
              keepAtCenter: (follow=='marker')?true:false
          })
          .setPopupContent(poptxt('position',data));
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
      headtxt(data,stat);
      last_data = data;
    } else {
      $('#status').html(yellowdot);
      headtxt(data,0);
    }
  };


  marker_gps = false;
  icon_gps = L.divIcon({className: 'leaflet-gps'});
  circ_gps = false;

  gps = function(e) {
    gps_location = [e.lat,e.lon];
    gps_accuracy = e.hdop*2;

    if (last_data && last_data.lat != '0.000000') {
      if ($('.leaflet-footer').css('display') == 'none') { $('.leaflet-footer').show(); }

      var distance = Math.round(map.distance(gps_location,[last_data.lat, last_data.lon]));
      distance = (distance > 1000)?(distance / 1000) + 'k':distance;
      $('.leaflet-footer .gps_dist').html(distance);

      $('.leaflet-footer .gps_dir').html( bearing(gps_location,[last_data.lat, last_data.lon]) );
    }

    if (!marker_gps) {
      map.addControl(new L.Control.Button([{ position: 'topleft', text: '🛰️', href: 'javascript:show(marker_gps,\'gps\');' }]));

      marker_gps = L.marker(gps_location,{icon:icon_gps}).addTo(map)
      .bindPopup(poptxt('gps',e),{closeOnClick:false, autoPan:false});
      circ_gps = L.circle(gps_location, gps_accuracy).addTo(map);
    } else {
      marker_gps.slideTo(gps_location, {
          duration: 500,
          keepAtCenter: (follow=='gps')?true:false
      })
      .setPopupContent(poptxt('gps',e));
      circ_gps.slideTo(gps_location, { duration: 500 });
      circ_gps.setRadius(gps_accuracy);
    }
  };

  get_data = function() {
      $('#status').html(reddot);
      $.ajax({url: 'live.json', success: (function( data ) {
        if (typeof data != "object") { data = $.parseJSON(data); }
        if (data.sonde) {
          draw(data.sonde);
        } else {
          setTimeout(function() {$('#status').html(yellowdot);},100);
        }
        if (data.gps) {
          gps(data.gps);
        }
        }),
        timeout: 1000}
     );
  };
  
  storage = (typeof(Storage) !== "undefined")?true:false;
  
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
  
  settings_write = function (data) {
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
      $("#settings").slideUp();
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
    $("#settings").slideToggle();
  };

  predictor = false;
  get_predict = function(data) {
    if (!data) { return; }
    var ascent = (data.climb > 0)? data.climb : 15;
    var descent = (data.climb > 0)? settings.overwrite_descend : data.climb * -1;

    var burst;
    if (data.climb > 0) {
      burst = (data.alt > settings.burst )?data.alt + 100 : settings.burst;
    } else {
      burst = parseInt(data.alt) + 7;
      if (data.alt > settings.overwrite_descend_till ) { descent = settings.overwrite_descend; }
    }

    var m = new Date();
    var datetime = m.getUTCFullYear() + "-" + az(m.getUTCMonth()+1) + "-" + az(m.getUTCDate()) + "T" +
      az(m.getUTCHours()) + ":" + az(m.getUTCMinutes()) + ":" + az(m.getUTCSeconds()) + "Z";
    var url = 'https://api.v2.sondehub.org/tawhiri';
    url += '?launch_latitude='+data.lat + '&launch_longitude='+tawhiri_lon(data.lon);
    url += '&launch_altitude='+data.alt + '&launch_datetime='+datetime;
    url += '&ascent_rate='+ascent + '&burst_altitude=' + burst + '&descent_rate='+descent;

    $.getJSON(url, function( prediction ) {
      draw_predict(prediction,data);
    });
  };

  draw_predict = function(prediction,data) {  
    var ascending = prediction.prediction[0].trajectory;
    var highest = ascending[ascending.length-1];
    var highest_location = [highest.latitude,sanitize_lon(highest.longitude)];

    var descending = prediction.prediction[1].trajectory;
    var landing = descending[descending.length-1];
    var landing_location = [landing.latitude,sanitize_lon(landing.longitude)];

    if (!marker_landing[data.id]) {
      marker_landing[data.id] = L.marker(landing_location,{icon: icon_landing}).addTo(map)
      .bindPopup(poptxt('landing',landing),{closeOnClick:false, autoPan:false});
    } else {
      marker_landing[data.id].slideTo(landing_location, {
          duration: 500,
          keepAtCenter: (follow=='landing')?true:false
      })
      .setPopupContent(poptxt('landing',landing));
    }

    dots_predict[data.id]=[];

    if (data.climb > 0) {
      ascending.forEach(p => dots_predict[data.id].push([p.latitude,sanitize_lon(p.longitude)]));

      if (!marker_burst[data.id]) {
        marker_burst[data.id] = L.marker(highest_location,{icon:icon_burst}).addTo(map).bindPopup(poptxt('burst',highest),{closeOnClick:false, autoPan:false});
      } else {
        marker_burst[data.id].slideTo(highest_location, {
          duration: 500,
          keepAtCenter: (follow=='burst')?true:false
        }).setPopupContent(poptxt('burst',highest));
      }
    }

    descending.forEach(p => dots_predict[data.id].push([p.latitude,sanitize_lon(p.longitude)]));
    
    if (!line_predict[data.id]) { 
      line_predict[data.id] = L.polyline(dots_predict[data.id],{color: 'yellow'}).addTo(map);
    } else {
      line_predict[data.id].setLatLngs(dots_predict[data.id]);
    }

    if (data.climb > 0) {
      predictor_time =  5 * 60; // ascending, every 5 min
    } else if (data.climb < 0 && data.alt > 5000) {
      predictor_time =  2 * 60; // descending, above 5km, every 2 min
    } else {
      predictor_time =  30; // descending, below 5km, every 30 sec
    }
    clearTimeout(predictor);
    predictor = setTimeout(function() {get_predict(last_data);}, predictor_time*1000);
  };

  sanitize_lon = function(lon) {
    if (lon > 180) { return lon - 360; }
    return lon;
  }
  tawhiri_lon = function(lon) {
    if (lon < 0) { return lon + 360; }
    return lon;
  }  

  poptxt = function(t,i) {
    var lat_input = (i.id)?i.lat:i.latitude;
    var lon_input = sanitize_lon((i.id)?i.lon:i.longitude);

    var lat = Math.round(lat_input * 1000000) / 1000000;
    var lon = Math.round(lon_input * 1000000) / 1000000;

    var add =
    '<br /><b>Position:</b> '+lat+',  '+lon+'<br />'+
    '<b>Open:</b> <a href="https://www.google.de/maps/?q='+lat+', '+lon+'" target="_blank">GMaps</a> | <a href="https://www.openstreetmap.org/?mlat='+lat+'&mlon='+lon+'&zoom=15" target="_blank">OSM</a> | <a href="geo://'+lat+','+lon+'">GeoApp</a>';

    if (t == 'position') { return '<div class="i_position"><b>🎈 '+i.id+'</b>'+add+'</div>'; }
    if (t == 'burst') { return '<div class="i_burst"><b>💥 Predicted Burst:</b><br />'+fd(i.datetime)+' in '+mr(i.altitude)+'m'+add+'</div>'; }
    if (t == 'highest') { return '<div class="i_burst"><b>💥 Burst:</b> '+mr(i.altitude)+'m'+add+'</div>';}
    if (t == 'landing') { return '<div class="i_landing"><b>🎯 Predicted Landing:</b><br />'+fd(i.datetime)+' at '+mr(i.altitude)+'m'+add+'</div>'; }
    if (t == 'gps') { return '<div class="i_gps">Position: '+(i.lat)+','+(i.lon)+'<br />Altitude: '+i.alt+'m<br />Speed: '+mr(i.speed * 3.6 * 10)/10+'km/h '+i.dir+'°<br />Sat: '+i.sat+' Hdop:'+(i.hdop/10)+'</div>'; }
  };

  fd = function(date) {
    var d = new Date(Date.parse(date));
    return az(d.getUTCHours()) +':'+ az(d.getUTCMinutes())+' UTC';
  };
  az = function(n) { return (n<10)?'0'+n:n; };
  mr = function(n) { return Math.round(n); };

  storage = (typeof(Storage) !== "undefined")?true:false;
  storage_write = function (data) {
    if (storage) {
      if (sessionStorage.sonde) {
        storage_data = JSON.parse(sessionStorage.sonde);
      } else {
        storage_data = [];
      }
      if (JSON.stringify(data) !=  JSON.stringify(storage_data[storage_data.length - 1])) {
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
      dots.push([d.lat,d.lon,d.alt]);
      session_storage_last = d;
    });
    draw(session_storage_last);
  }

  setInterval(get_data,1000);

});

L.Control.Button = L.Control.extend({
  onAdd: function (map) {
    var container = L.DomUtil.create('div', 'leaflet-bar leaflet-control');
    options = this.options;
    Object.keys(options).forEach(function(key) {
      this.link = L.DomUtil.create('a', '', container);
      this.link.text = options[key].text;
      this.link.href = options[key].href;
      this.link.id =  options[key].id;
    });

    this.options.position = this.options[0].position;
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
    var bearing = ((Math.atan2(y, x) * 180 / Math.PI) + 360) % 360;
    bearing = bearing < 0 ? bearing-360 : bearing;
    return Math.round(bearing);
}
