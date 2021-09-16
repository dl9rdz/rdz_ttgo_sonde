let stypes=new Map();
stypes.set('4', 'RS41');
stypes.set('R', 'RS92');
stypes.set('D', 'DFM');
stypes.set('M', 'M10');
stypes.set('2', 'M20');
stypes.set('3', 'MP3H');

/* (no longer) Used by qrg.html in RX_FSK.ino */
function prep() {
  var stlist=document.querySelectorAll("input.stype");
  for(txt of stlist){
    var val=txt.getAttribute('value'); var nam=txt.getAttribute('name'); 
    var sel=document.createElement('select');
    sel.setAttribute('name',nam);
    for(stype of stypes) { 
      var opt=document.createElement('option');
      opt.value=stype[0];
      opt.innerHTML=stype[1];
      if(stype[0]==val) { opt.setAttribute('selected','selected'); }
      sel.appendChild(opt);
    }
    txt.replaceWith(sel);
  }
} 

function qrgTable() {
  var tab=document.getElementById("divTable");

  var table = "<table><tr><th>ID</th><th>Active</th><th>Freq</th><th>Launchsite</th><th>Mode</th></tr>";
  for(i=0; i<qrgs.length; i++) {
     var ck = "";
     if(qrgs[i][0]) ck="checked";
     table += "<tr><td>" + (i+1) + "</td><td><input name=\"A" + (i+1) + "\" type=\"checkbox\" " + ck + "/></td>";
     table += "<td><input name=\"F" + (i+1) + "\" type=\"text\" width=12 value=\"" + qrgs[i][1] + "\"></td>";
     table += "<td><input name=\"S" + (i+1) + "\" type=\"text\" value=\"" + qrgs[i][2] +"\"></td>";
     table += "<td><input class=\"stype\" name=\"T" + (i+1) + "\" value=\"" + qrgs[i][3] + "\"></td></tr>";
  }
  table += "</table>";
  tab.innerHTML = table;
  prep();
}
