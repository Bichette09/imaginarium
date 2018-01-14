
function savesettings()
{
	var lSettings = {};
	$("#settingscontainer .settingentry").each(function(){
		var elem = $(this);
		var lKeyName = elem.attr('paramname');
		var lKeyPath = lKeyName.split('/');
		var lDst = lSettings;
		while(lKeyPath.length > 1)
		{
			if(lKeyPath[0].length != 0)
			{
				if(lDst[lKeyPath[0]] === undefined)
				{
					lDst[lKeyPath[0]] = {};
				}
				lDst = lDst[lKeyPath[0]];
			}
			lKeyPath.shift();
		}
		var lKeyVal = elem.find('.settingvalue').attr('value');
		lDst[lKeyPath[0]] = lKeyVal;
	});
	var a = document.createElement('a');
	var blob = new Blob([JSON.stringify(lSettings, null, 2)], {'type':'application/json'});
	a.href = window.URL.createObjectURL(blob);
	a.download = 'settings.json';
	a.click();
}

function onfilter()
{
	lText = document.getElementById("filterinput").value.trim();
	lVals = [];
	if(lText.length > 0)
	{
		lVals = lText.split(' ');
		for(var idx in lVals)
		{
			lVals[idx] = lVals[idx].trim().toLowerCase();
		}
	}
	
	$("#settingscontainer .settingentry").each(function () {
		var elem = $(this);
		var lShow = false;
		if(lVals.length == 0)
		{
			lShow = true;
		}
		else
		{
			for(var idx in lVals)
			{
				if(lVals[idx].length == 0)
					continue;
				/*if(lVals[idx][0]=='-')
				{
					var lToTest = lVals[idx].substring(1);
					if(elem.attr('val').toLowerCase().indexOf(lToTest) !== -1)
					{
						lShow = false;
						break;
					}
				}
				else*/ if(elem.attr('paramname').toLowerCase().indexOf(lVals[idx]) !== -1)
				{
					lShow = true;
				}
			}
			
		}
		
		if(lShow)
		{
			elem.show();
		}
		else
		{
			elem.hide();
		}

	});
}

function onsettinginputloosefocus(pEvent)
{
	var lElem = $(pEvent.target);
	sRosCtx.callService('/rosapi/set_param','rosapi/GetParam',{name:lElem.parent().attr('paramname'),value:lElem.val()}, undefined);
}

var entityMap = {
  '&': '&amp;',
  '<': '&lt;',
  '>': '&gt;',
  '"': '&quot;',
  "'": '&#39;',
  '/': '&#x2F;',
  '`': '&#x60;',
  '=': '&#x3D;'
};

function escapeHtml (string) {
  return String(string).replace(/[&<>"'`=\/]/g, function (s) {
    return entityMap[s];
  });
}

function addSetting(pName,pValue)
{
	$( "#settingscontainer" ).append( 
		"<div class=\"settingentry\" id=\"settingentry_" + escapeHtml(pName) + "\" paramname=\"" + escapeHtml(pName) + "\">"+
			"<div class=\"settingname\">" + escapeHtml(pName) + "</div>"+
			"<input type=\"text\" onchange=\"onsettinginputloosefocus(event)\" class=\"settingvalue\" originalvalue=\""+escapeHtml(pValue)+"\" value=\""+escapeHtml(pValue)+"\"></input>"+
		"</div>");
	
}

function createGetParamCallback(pName)
{
	return function(pResult){
		addSetting(pName,pResult.value);
		onParamRetrieved();
	}
}

var sSettingsCtx = {
	waitcptr : 0
};

function onParamRetrieved()
{
	--sSettingsCtx.waitcptr;
	if(sSettingsCtx.waitcptr !== 0)
		return;
	
	// sort setting list
	$("#settingscontainer .settingentry").sort(function (a, b) {
		return $(a).attr('paramname').localeCompare($(b).attr('paramname'));
	}).each(function () {
		var elem = $(this);
		$(elem).appendTo("#settingscontainer");
	});
	
	$('#settingscontainer').show();
	$('#waiterroot').hide();
	onfilter();
}

function onload()
{
	$('#settingscontainer').hide();
	
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	++sSettingsCtx.waitcptr;
	sRosCtx.callService('/rosapi/get_param_names','rosapi/GetParamNames',{}, function(pResult) {
		for(var i = 0 ; i < pResult.names.length ; ++i)
		{
			var lName = pResult.names[i];
			
			++sSettingsCtx.waitcptr;
			sRosCtx.callService('/rosapi/get_param','rosapi/GetParam',{name:lName,default:''}, createGetParamCallback(lName));
			
		}
		onParamRetrieved();
	})
}

