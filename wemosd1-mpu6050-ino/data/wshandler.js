var head = 0, tail = 0, ring = new Array();
var myJSON;
function get_appropriate_ws_url(extra_url)
{
	var pcol;
	var u = document.URL;

	/*
	 * We open the websocket encrypted if this page came on an
	 * https:// url itself, otherwise unencrypted
	 */

	if (u.substring(0, 5) === "https") {
		pcol = "wss://";
		u = u.substr(8);
	} else {
		pcol = "ws://";
		if (u.substring(0, 4) === "http")
			u = u.substr(7);
	}

	u = u.split("/");
	var p = u[0].split(":");

	/* + "/xxx" bit is for IE10 workaround */

	return pcol + p[0] + ":8181" + "/" + extra_url;
}

function new_ws(urlpath, protocol)
{
	if (typeof MozWebSocket != "undefined")
		return new MozWebSocket(urlpath, protocol);

	return new WebSocket(urlpath, protocol);
}

function isJSONQuatValid(_str) {
	myJSON = null;
	try {
		myJSON = JSON.parse(_str);
	} catch (e) {
		return false;
	}

	if(myJSON.qw == undefined) return false;
	if(myJSON.qw == null) return false;
	if(myJSON.qx == undefined) return false;
	if(myJSON.qx == null) return false;
	if(myJSON.qy == undefined) return false;
	if(myJSON.qy == null) return false;
	if(myJSON.qz == undefined) return false;
	if(myJSON.qz == null) return false;
	if(myJSON.msg == undefined) return false;
	if(myJSON.msg == null) return false;

	return true;
}
document.addEventListener("DOMContentLoaded", function() {

	var ws = new_ws(get_appropriate_ws_url(""), ['arduino']);
	try {
		ws.onopen = function() {
			document.getElementById("r").disabled = 0;
		};
	
		ws.onmessage =function got_packet(msg) {
			var n, s = "";
			var msgdata = msg.data;

			if(isJSONQuatValid(msgdata)) {
				document.getElementById("qw").value = myJSON.qw; 
				document.getElementById("qx").value = myJSON.qx; 
				document.getElementById("qy").value = myJSON.qy; 
				document.getElementById("qz").value = myJSON.qz; 
				document.getElementById("qno").value = myJSON.msg;
				document.getElementById("inputdisplay").hidden = true;
			}

			ring[head] = msgdata + "\n";
			head = (head + 1) % 50;
			if (tail === head)
				tail = (tail + 1) % 50;
	
			n = tail;
			do {
				s = s + ring[n];
				n = (n + 1) % 50;
			} while (n !== head);
			
			document.getElementById("r").value = s; 
			document.getElementById("r").scrollTop =
			document.getElementById("r").scrollHeight;
		
		};
	
		ws.onclose = function(){
			document.getElementById("r").disabled = 1;
		};
	} catch(exception) {
		alert("<p>Error " + exception);  
	}

}, false);
