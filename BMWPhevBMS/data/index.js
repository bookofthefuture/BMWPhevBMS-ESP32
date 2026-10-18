/** @brief excutes when page finished loading. Creates tables and chart */
var output, websocket,chartSpeed

function toggleHidden(id) {
	const element = document.getElementById(id);
	if (element.classList.contains('hidden')) {
		element.classList.remove('hidden');

	} else {
		element.classList.add('hidden');

	}

	return false
}
function show(id) {
	const element = document.getElementById(id);
	if (element.classList.contains('hidden')) {
		element.classList.remove('hidden');

	}

}

function hide(id) {
	const element = document.getElementById(id);
	if (!element.classList.contains('hidden')) {
		element.classList.add('hidden');

	}

}
function setValue(id, value) {
	const element = document.getElementById(id);
	element.innerHTML = value
}

function updateText(key, data) {
		if (key == 'status') {
			setValue('mode', data.replace('"', ""));
		if (data == 'Charge') {
			show('outlander-charger')
			show('chargeContainer');
		} else {
			hide('outlander-charger')
			hide('chargeContainer');

		}
	} else if (key == 'inverterstatus.val') {
		if (data == 1) {
			setValue('inverterStatus', "Run");
		} else {
			setValue('inverterStatus', "Off");

		}

		
	} else if (key == 'minVolt') {
		setValue('celllow', Math.round(data * 1000));
	} else if (key == 'maxVolt') {
		setValue('cellhigh', Math.round(data * 1000));
	} else if (key == 'cellDelta') {
		setValue('celldelta', Math.round(data * 1000));
	} else if (key == 'chargeEnabled') {
		setValue('acPresent', data);
	}  else if (key == 'requestedchargecurrent.val') {
		setValue('requestedChargeCurrent', data / 10);
	} else if (key == 'chargerstatus.val') {
		setValue('chargerStatus', data)
	} else if (key == 'chargevsetpoint.val') {
		setValue('chargeSetpoint', data)
	} else if (key == 'chargecurrentmax.val') {
		setValue('chargeMaxCurrent', data)
	} else if (key == 'evse_duty.val') {
		setValue('evseDuty', data);
	} else if (key == 'socOverride.val') {
		setValue('socOverride', data);
	} else if (key == 'ahUsed') {
		setValue('ah', data.toFixed(1));
	} else if (key == 'capacity.val') {
		setValue('usableAh', data);
	} else if (key == 'balanceActive') {
		setValue('balanceStatus', data);
	}
}

function updateButton(key, value) {
	if (key == 'ac.val') {
		if (value == '1') {
			hide('acOverride');
			show('acOverrideOff');
		} else {
			show('acOverride')
			hide('acOverrideOff');
		}
	}
		
}

function initHandlers() {
	const acOverride = document.getElementById("acOverride");
	acOverride.onclick = function() {
		acOverride.innerHTML = "Loading"
		var xmlhttp = new XMLHttpRequest();
		xmlhttp.onreadystatechange = function() {
			acOverride.innerHTML = "Override AC Present"
		};
		xmlhttp.open("POST", "cmd", true);
		xmlhttp.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
		xmlhttp.send("cmd=a");
	}

	const shuntReset = document.getElementById("shuntReset");
	shuntReset.onclick = function() {
		var xmlhttp = new XMLHttpRequest();
		xmlhttp.open("POST", "cmd", true);
		xmlhttp.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
		xmlhttp.send("cmd=r");
	}

	const acOverrideOff = document.getElementById("acOverrideOff");
	acOverrideOff.onclick = function() {
		acOverrideOff.innerHTML = "Loading"
		var xmlhttp = new XMLHttpRequest();
		xmlhttp.onreadystatechange = function() {
			acOverrideOff.innerHTML = "Remove AC Present Override"
		};
		xmlhttp.open("POST", "cmd", true);
		xmlhttp.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
		xmlhttp.send("cmd=o");
	}

}

function dashboard() {
	var xhr = new XMLHttpRequest();

	xhr.open('GET', '/dashboard')
	xhr.onload = function() {
		const json = JSON.parse(xhr.responseText);
		onMessage(json)
	};
	xhr.send()
}

function onLoad() {
	output = document.getElementById("output");
	initGauges();
	initHandlers();
	chargerWebSocket("ws://"+ location.host +"/ws");
	setInterval(dashboard, 1000);  
}

function onOpen(evt) {
	console.log("Socket Connected");
}
   
function onMessage(json) {
	Object.keys(json).forEach(function(key) {
		updateGauge(key, json[key]);
		updateText(key, json[key]);
		// updateButton(json.message);
	});


 }

 function onError(evt) {
	console.log("Socket Error")
 }
   
 function doSend(message) {
	websocket.send(message);
 }
   
 function writeToScreen(message) {
	var pre = document.createElement("p"); 
	pre.style.wordWrap = "break-word"; 
	pre.innerHTML = message; output.appendChild(pre);
 }

 function chargerWebSocket(wsUri) {
	websocket = new WebSocket(wsUri);
	   
	websocket.onopen = function(evt) {
	   onOpen(evt)
	};

	websocket.onclose = function(evt) {
		console.log(evt)
	};
   
	websocket.onmessage = function(evt) {
		console.log(evt)
	};
   
	websocket.onerror = function(evt) {
	   onError(evt)
	};
 }

function setUpdateStatus(text) {
	document.getElementById("updatestatus").textContent = text;
}

/** @brief uploads a firmware image to /update; the BMS restarts itself on success */
function uploadFirmware()
{
	var input = document.getElementById('firmwarefile');
	if (!input.files.length) {
		setUpdateStatus("Choose a firmware .bin file first");
		return;
	}
	var file = input.files[0];
	if (!confirm("Update the BMS firmware with " + file.name + "?\nThe BMS will restart.")) {
		return;
	}

	var fd = new FormData();
	fd.append("firmware", file, file.name);
	var xhr = new XMLHttpRequest();

	xhr.upload.onprogress = function(e) {
		if (e.lengthComputable) {
			setUpdateStatus("Uploading " + Math.round(100 * e.loaded / e.total) + "%");
		}
	};
	xhr.onload = function() {
		if (xhr.status >= 200 && xhr.status < 300) {
			setUpdateStatus("Update complete, restarting. Reloading in 20 seconds");
			setTimeout(function() { location.reload(); }, 20000);
		} else {
			setUpdateStatus("Update failed: " + xhr.responseText);
		}
	};
	xhr.onerror = function() {
		setUpdateStatus("Update failed: connection lost");
	};

	setUpdateStatus("Uploading 0%");
	xhr.open("POST", "/update");
	xhr.send(fd);
}

/** @brief uploads a web interface file to the BMS (SPIFFS) */
function uploadFile()
{
	var input = document.getElementById('updatefile');
	if (!input.files.length) {
		setUpdateStatus("Choose a file first");
		return;
	}
	var xmlhttp = new XMLHttpRequest();
	var fd = new FormData(document.getElementById('uploadform'));

	xmlhttp.onload = function()
	{
		if (xmlhttp.status >= 200 && xmlhttp.status < 400) {
			setUpdateStatus("Upload complete");
		} else {
			setUpdateStatus("Upload failed: " + xmlhttp.responseText);
		}
	}

	xmlhttp.open("POST", "/edit");
	xmlhttp.send(fd);
}
