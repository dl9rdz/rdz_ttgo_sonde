let stypes=new Map();
stypes.set('4', 'RS41');
stypes.set('R', 'RS92');
stypes.set('9', 'DFM9 (old)');
stypes.set('6', 'DFM6 (old)');
stypes.set('D', 'DFM');
stypes.set('M', 'M10');
stypes.set('2', 'M20');
stypes.set('3', 'MP3H');

/* Used by qrg.html in RX_FSK.ino */
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

window.onload = prep;
