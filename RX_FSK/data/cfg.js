var cfgs = [
[ "", "General configuration", "https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/General-configuration" ],
[ "wifi", "Wifi mode (0=off, 1=client, 2=AP, 3=client or AP autoselect on startup)" ],
[ "mdnsname", "Network mDNS name"],
[ "ephftp", "FTP server for ephemeris data (RS92 decoder)"],
[ "debug", "Debug mode (0/1)" ],
[ "maxsonde", "Maximum number of QRG entries (must be &leq; 50)" ],
[ "rxlat", "Receiver fixed latitude"],
[ "rxlon", "Receiver fixed longitude"],
[ "rxalt", "Receiver fixed altitude"],
[ "", "OLED/TFT display configuration", "https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Display-configuration" ],
[ "screenfile", "Screen config (0=automatic; 1-5=predefined; other=custom)" ],
[ "display", "Display screens (scan, default, ...)" ],
[ "norx_timeout", "No-RX-timeout in seconds (-1=disabled)"],
[ "tft_orient", "TFT orientation (0/1/2/3), OLED flip: 3"],
[ "", "Spectrum display configuration", "https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Spectrum-configuration" ],
[ "spectrum", "Show spectrum on start (-1=no, 0=forever, >0=time [sec])" ],
[ "startfreq", "Start frequency (MHz, default 400)" ],
[ "channelbw", "Bandwidth (kHz)" ],
[ "marker", "Spectrum MHz marker" ],   // maybe remove, assume always ==1?
[ "noisefloor", "Spectrum noisefloor" ],
[ "", "Receiver configuration", "https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Receiver-configuration" ],
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
[ "", "KISS TNC/AXUDP/AXTCP data feed configuration", "https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Data-feed-configuration"],
[ "call", "Call"],
[ "passcode", "Passcode"],
[ "kisstnc.active", "KISS TNC (port 14590) (needs reboot)"],
[ "axudp.active", "AXUDP active"],
[ "axudp.host", "AXUDP host"],
[ "axudp.port", "AXUDP port"],
[ "axudp.highrate", "Rate limit"],
[ "tcp.active", "APRS TCP active"],
[ "tcp.host", "APRS TCP host"],
[ "tcp.port", "APRS TCP port"],
[ "tcp.highrate", "Rate limit"],
[ "tcp.objcall", "APRS object call"],
[ "tcp.beaconsym", "APRS tracker symbol"],
[ "tcp.chase", "APRS location reporting (0=off, 1=fixed, 2=chase/GPS, 3=auto)"],
[ "tcp.comment", "APRS location comment"],
[ "", "MQTT data feed configuration", "https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/MQTT-configuration"],
[ "mqtt.active", "MQTT active (needs reboot)"],
[ "mqtt.id", "MQTT client ID"],
[ "mqtt.host", "MQTT server hostname"],
[ "mqtt.port", "MQTT port"],
[ "mqtt.username", "MQTT username"],
[ "mqtt.password", "MQTT password"],
[ "mqtt.prefix", "MQTT prefix"],
[ "", "Chasemapper settings", "https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Chasemapper-configuration"],
[ "cm.active", "Chasemapper active (0=disabled, 1=active)"],
[ "cm.host", "Chasemapper UDP host"],
[ "cm.port", "Chasemapper UDP port"],
[ "", "SondeHub settings", "https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/SondeHub-settings"],
[ "sondehub.active", "SondeHub reporting (0=disabled, 1=active)"],
[ "sondehub.chase", "SondeHub location reporting (0=off, 1=fixed, 2=chase/GPS, 3=auto)"],
[ "sondehub.host", "SondeHub host (DO NOT CHANGE)"],
[ "sondehub.callsign", "Callsign"],
[ "sondehub.antenna", "Antenna (optional, visisble on SondeHub tracker)"],
[ "sondehub.email", "SondeHub email (optional, only used to contact in case of upload errors)"],
[ "", "SondeHub frequency import", "https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/SondeHub-import" ],
[ "sondehub.fiactive", "SondeHub frequency import active (0=disabled, 1=active)" ],
[ "sondehub.fiinterval", "Import frequency (minutes, &geq; 5)" ],
[ "sondehub.fimaxdist", "Import maximum distance (km, &leq; 700)" ],
[ "sondehub.fimaxage", "Import maximum age (hours, &leq; 48)" ],
[ "", "Hardware configuration (requires reboot)", "https://github.com/dl9rdz/rdz_ttgo_sonde/wiki/Hardware-configuration"],
[ "disptype", "Display type (0=OLED/SSD1306, 1=ILI9225, 2=OLED/SH1106, 3=ILI9341, 4=ILI9342)"],
[ "oled_sda", "OLED SDA/TFT SDA"],
[ "oled_scl", "OLED SCL/TFT CLK"],
[ "oled_rst", "OLED RST/TFT RST (needs reboot)"],
[ "tft_rs", "TFT RS"],
[ "tft_cs", "TFT CS"],
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
];

function mkcfg(id, key, label, value) {
 var s = "<tr style=\"visibility: collapse;\" class=\"cfgpanel\"><td>" + label + "</td><td><input name=\"" + key + "\" type=\"text\" value=\"" + value + "\"/></td></tr>\n";
 return s;
}
function mkcfgbtn(id, key, label, value) {
  var touch = "";
  var v = value;
  if(v != -1 && (v&128)) {
    touch = " checked";
    v = v & 127;
  }
  var s = "<tr style=\"visibility:collapse\" class=\"cfgpanel\"><td>" + label + "</td><td><input name=\"" + key + "\" type=\"text\" size=\"3\" value=\"" + v + "\"/>";
  s += "<input type=\"checkbox\" name=\"" + key + "#\" "+touch+"> Touch</td></tr>\n";
  return s;
}

function mksep(id,label,url) {
  return "<tr class=\"cfgheader\"><th class=\"cfg\" align=\"left\" colspan=\"2\">"+label+" <a href=\""+url+"\" target=\”_blank\">[wiki]</a></th></tr>\n";
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
        if(key=="button_pin" || key=="button2_pin") {
	  tab += mkcfgbtn("s"+id, key, lbl, cf.get(key));
	} else if (key=="display") {
          tab += mkcfg("s"+id, key, lbl, cf.get(key));
	  tab += "<tr style=\"visibility:collapse\" class=\"cfgpanel\"><td>"+scr+"</td><td></td></tr>"
	} else {
          tab += mkcfg("s"+id, key, lbl, cf.get(key));
	}
    } else {
        id++;
        tab += mksep("s"+id, lbl, cfgs[i][2]);
    }
  }
  tab += "</table>";
  var cfgdiv = document.getElementById("cfgtab");
  cfgdiv.innerHTML = tab;
  // enable collapse / expand of items below a header
  var acc = document.getElementsByClassName("cfgheader");
  for(i=0; i<acc.length; i++) {
    acc[i].firstChild.innerHTML = "[+] " + acc[i].firstChild.innerHTML;
    acc[i].addEventListener("click", function(e) {
      if(e.target.nodeName=="A") return;
      achar = "[+]";
      if(this.classList.toggle("active")) achar = "[\u2212]";
      this.firstChild.innerHTML = achar + this.firstChild.innerHTML.substring(3);
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
  acc[0].click();
}
