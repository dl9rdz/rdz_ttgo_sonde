var cfgs = [
[ "", "Software configuration" ],
[ "wifi", "Wifi mode (0=off, 1=client, 2=AP, 3=auto client/AP on startup" ],
[ "debug", "Debug mode (0/1)" ],
[ "maxsonde", "max. number of QRG entries (must be <100)" ],
[ "screenfile", "Screen config (0=automatic; 1-5=predefined; other=custom)" ],
[ "display", "Display screens (scan, default, ...)" ],
[ "", "Spectrum display settings" ],
[ "spectrum", "Show spectrum on start (-1=no, 0=forever, >0=time [sec])" ],
[ "startfreq", "Start frequency (MHz, default 400)" ],
[ "channelbw", "Bandwidth (kHz)" ],
[ "marker", "Spectrum MHz marker" ],   // maybe remove, assume always ==1?
[ "noisefloor", "Spectrum noisefloor" ],
[ "", "Receiver configuration" ],
[ "freqofs", "RX frequency offset (Hz)"],
[ "rs41.agcbw", "RS41 AGC bandwidth"],
[ "rs41.rxbw", "RS41 RX bandwidth"],
[ "rs92.rxbw", "RS92 RX (and AGC) bandwidth"],
[ "rs92.alt2d", "RS92 2D fix default altitude"],
[ "dfm.agcbw", "DFM AGC bandwidth"],
[ "dfm.rxbw", "DFM RX bandwidth"],
[ "m10m20.agcbw", "M10/M20 AGC bandwidth"],
[ "m10m20.rxbw", "M10/M20 RX bandwidth"],
[ "mp3h.agcbw", "MP3H AGC bandwidth"],
[ "mp3h.rxbw", "MP3H RX bandwidth"],
[ "ephftp", "FTP for eph (RS92)"],
[ "", "Data feed configuration"],
[ "call", "Call"],
[ "passcode", "Passcode"],
[ "kisstnc.active", "KISS TNC (port 14590) (needs reboot)"],
[ "kisstnc.idformat", "KISS TNC ID Format"],
[ "axudp.active", "AXUDP active"],
[ "axudp.host", "AXUDP Host"],
[ "axudp.port", "AXUDP Port"],
[ "axudp.idformat", "DFM ID Format"],
[ "axudp.highrate", "Rate limit"],
[ "tcp.active", "APRS TCP active"],
[ "tcp.host", "ARPS TCP Host"],
[ "tcp.port", "APRS TCP Port"],
[ "tcp.idformat", "DFM ID Format"],
[ "tcp.highrate", "Rate limit"],
[ "mqtt.active", "MQTT Active (needs reboot)"],
[ "mqtt.id", "MQTT client ID"],
[ "mqtt.host", "MQTT server hostname"],
[ "mqtt.port", "MQTT Port"],
[ "mqtt.username", "MQTT Username"],
[ "mqtt.password", "MQTT Password"],
[ "mqtt.prefix", "MQTT Prefix"],
[ "", "Hardware configuration (requires reboot)"],
[ "disptype", "Display type (0=OLED/SSD1306, 1=ILI9225, 2=OLED/SH1106, 3=ILI9341, 4=ILI9342)"],
[ "norx_timeout", "No-RX-Timeout in seconds (-1=disabled)"],
[ "oled_sda", "OLED SDA/TFT SDA"],
[ "oled_scl", "OLED SCL/TFT CLK"],
[ "oled_rst", "OLED RST/TFT RST (needs reboot)"],
[ "tft_rs", "TFT RS"],
[ "tft_cs", "TFT CS"],
[ "tft_orient", "TFT orientation (0/1/2/3), OLED flip: 3"],
[ "tft_spifreq", "TFT SPI speed"],
[ "button_pin", "Button input port"],
[ "button2_pin", "Button 2 input port"],
[ "button2_axp", "Use AXP192 PWR as Button 2"],
[ "touch_thresh", "Touch button threshold<br>(0 for calib mode)"],
[ "power_pout", "Power control port"],
[ "led_pout", "LED output port"],
[ "gps_rxd", "GPS RXD pin (-1 to disable)"],
[ "gps_txd", "GPS TXD pin (not really needed)"],
[ "batt_adc", "Battery measurement pin"],
[ "sx1278_ss", "SX1278 SS"],
[ "sx1278_miso", "SX1278 MISO"],
[ "sx1278_mosi", "SX1278 MOSI"],
[ "sx1278_sck", "SX1278 SCK"],
[ "mdnsname", "mDNS name"],
[ "", "SondeHub settings"],
[ "sondehub.active", "SondeHub reporting (0=disabled, 1=active)"],
[ "sondehub.chase", "SondeHub location reporting (0=off, 1=fixed, 2=chase/GPS, 3=auto)"],
[ "sondehub.host", "SondeHub host (DO NOT CHANGE)"],
[ "sondehub.callsign", "Callsign"],
[ "sondehub.lat", "Latitude (optional, required to show station on SondeHub Tracker)"],
[ "sondehub.lon", "Longitude (optional, required to show station on SondeHub Tracker)"],
[ "sondehub.alt", "Altitude (optional, visible on SondeHub tracker)"],
[ "sondehub.antenna", "Antenna (optional, visisble on SondeHub tracker)"],
[ "sondehub.email", "SondeHub email (optional, only used to contact in case of upload errors)"],
[ "", "SondeHub frequency import" ],]
[ "sondehub.fiactive", "SondeHub frequency import active (0=disabled, 1=active)" ],
[ "sondehub.fiinterval", "Import frequency (minutes, &geq;5)" ],
[ "sondehub.fimaxdist", "Import maximum distance (km, &leq;500)" ],
[ "sondehub.fimaxage", "Import maximum age (hours, &leq;24" ],
];

function mkcfg(id, key, label, value) {
 var s = "<tr class=\"cfgpanel\"><td>" + label + "</td><td><input name=\"" + key + "\" type=\"text\" value=\"" + value + "\"/></td></tr>\n";
 return s;
}
function mkcfgbtn(id, key, label, value) {
  var touch = "";
  var v = value;
  if(v != -1 && (v&128)) {
    touch = " checked";
    v = v & 127;
  }
  var s = "<tr class=\"cfgpanel\"><td>" + label + "</td><td><input name=\"" + key + "\" type=\"text\" size=\"3\" value=\"" + v + "\"/>";
  s += "<input type=\"checkbox\" name=\"" + key + "#\" "+touch+"> Touch</td></tr>\n";
  return s;
}

function mksep(id,label) {
  return "<tr class=\"cfgheader\"><th align=\"left\" colspan=\"2\">"+label+"</th></tr>\n";
}
function rowdisp(id,disp) {
  var matches = document.querySelectorAll("tr."+id);
  matches.forEach(function(e) { if(disp) e.hidden=true; else e.removeAttribute('hidden');});
  hid=id; nid="N"+id;
  if(!disp) { hid=nid; nid=id; }
  document.querySelector("span."+hid).hidden=true;
  document.querySelector("span."+nid).removeAttribute('hidden');
}
function configTable() {
  // iterate over cfgs
  var tab = "<table width=\"100%\"><tr><th>Option</th><th>Value</th></tr>\n";
  var id=0;
  for(i=0; i<cfgs.length; i++) { 
    var key = cfgs[i][0];
    var lbl = cfgs[i][1];
    if(key) {
        if(key=="button_pin" || key=="button2_pin")
	  tab += mkcfgbtn("s"+id, key, lbl, cf.get(key));
	else
          tab += mkcfg("s"+id, key, lbl, cf.get(key));
    } else {
        id++;
        tab += mksep("s"+id, lbl);
    }
  }
  tab += "</table>";
  var cfgdiv = document.getElementById("cfgtab");
  cfgdiv.innerHTML = tab;
  // enable collapse / expand of items below a header
  var acc = document.getElementsByClassName("cfgheader");
  for(i=0; i<acc.length; i++) {
    acc[i].firstChild.innerHTML = "\u2212 " + acc[i].firstChild.innerHTML;
    acc[i].addEventListener("click", function() {
      achar = "\u2212";
      if(this.classList.toggle("active")) achar = "+";
      this.firstChild.innerHTML = achar + this.firstChild.innerHTML.substring(1);
      var panel = this;
      console.log(panel);
      while( panel = panel.nextElementSibling) {
        console.log(panel);
	if ( panel.className!="cfgpanel") { break; }
        if(panel.style.visibility==="collapse") {
          panel.style.visibility="visible";
        } else {
          console.log("none");
          panel.style.visibility="collapse";
        } 
      }
    });
  }
}
