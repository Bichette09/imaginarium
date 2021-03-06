﻿
function savesettings()
{
	var lSettings = {};
	$("#paramscontainer .settingentry, #settingscontainer .settingentry").each(function(){
		var elem = $(this);
		var lRoot = "";
		if(elem.parents("#paramscontainer").length)
		{
			lRoot = "rosparams";
		}
		else
		{
			lRoot = "settings"
		}
		
		var lKeyName = lRoot + '/' + elem.attr('paramname');
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
	var blob = new Blob([JSON.stringify(lSettings, null, 4)], {'type':'application/json'});
	a.href = window.URL.createObjectURL(blob);
	a.download = 'settings.json';
	a.click();
}

function deletesetting(event)
{
	var lName = $(event.target).parents(".settingentry").attr('paramname');
	sRosCtx.callService('/settings_store/delete','settings_store/delete',{name:lName}, function(pResult) {
		if(!pResult.error)
		{
			$(event.target).parents(".settingentry").remove();
		}
	})	
}

function sortsettings()
{
	$("#settingscontainer .settingentry").sort(function (a, b) {
		return $(a).attr('paramname').localeCompare($(b).attr('paramname'));
	}).each(function () {
		var elem = $(this);
		$(elem).appendTo("#settingscontainer");
	});
	
	$('#settingscontainer').show();
	$('#waiterroot').hide();
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
	
	$("#settingscontainer .settingentry, #paramscontainer .settingentry, #statescontainer .settingentry").each(function () {
		var elem = $(this);
		var lShow = false;
		if(lVals.length == 0)
		{
			lShow = true;
		}
		else
		{
			var lHide = false;
			var lGotKeepFilter = false;
			for(var idx in lVals)
			{
				if(lVals[idx].length == 0)
					continue;
				if(lVals[idx][0]=='-')
				{
					if(lVals[idx].length > 1)
					{
						var lToTest = lVals[idx].substring(1);
						if(elem.attr('paramname').toLowerCase().indexOf(lToTest) !== -1)
						{
							lHide = true;
						}
					}
				}
				else 
				{
					lGotKeepFilter = true;
					if(elem.attr('paramname').toLowerCase().indexOf(lVals[idx]) !== -1)
					{
						lShow = true;
					}
				}
			}
			lShow = (!lGotKeepFilter || lShow) && !lHide;
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
	sRosCtx.callService('/settings_store/set','settings_store/set',{name:lElem.parent().attr('paramname'),newvalue:lElem.val()}, undefined);
	//sRosCtx.callService('/rosapi/set_param','rosapi/GetParam',{name:lElem.parent().attr('paramname'),value:lElem.val()}, undefined);
}

function onsettinginput(pEvent)
{
	if (e.keyCode == 13) {
		var tb = document.getElementById("scriptBox");
		onsettinginputloosefocus(pEvent);
		return false;
	}
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

function addOrUpdateSetting(pName,pValue,pDescription,pInUse)
{
	if(pName.length <= 0)
		return;
	var lIsNew = false;
	var lId = btoa( "settingentry_" + pName).replace(/=/g,'_');
	
	var lDiv = $("#"+lId);
	
	var lDesc = '';
	if(pDescription.length > 0)
	{
		lDesc = '(' + pDescription + ')'
	}

	if (lDiv.length == 0){
		$( "#settingscontainer" ).append( 
		"<div class=\"settingentry\" id=\"" + lId + "\" paramname=\"" + escapeHtml(pName) + "\">"+
			"<div class=\"settingtitle\"><span class=\"settingname\">" + escapeHtml(pName) + "</span><span class=\"settingdescription\">" + escapeHtml(lDesc) + "</span><div id=\"deletebutton\" class=\"settingsbutton\" onclick=\"deletesetting(event)\">delete</div></div>"+
			"<input type=\"text\" onkeypress=\"onsettinginput(event)\" onchange=\"onsettinginputloosefocus(event)\" class=\"settingvalue\" originalvalue=\"\" value=\"\"></input>"+
		"</div>");
		lIsNew = true;
	}
	var lInput = $('#'+lId + ' input');
	lInput.attr('originalvalue',pValue);
	lInput.attr('value',pValue);
	var lDeleteButton = $('#'+lId + ' #deletebutton');
	if(pInUse)
		lDeleteButton.hide();
	else
		lDeleteButton.show();
	return lIsNew;
}

function addOrUpdateState(pName,pValue)
{
	if(pName.length <= 0)
		return;
	var lIsNew = false;
	var lId = btoa( "stateentry_" + pName).replace(/=/g,'_');
	
	var lDiv = $("#"+lId);
	
	if (lDiv.length == 0){
		
		
		$( "#statescontainer" ).append( 
		"<div class=\"settingentry\" id=\"" + lId + "\" paramname=\"" + escapeHtml(pName) + "\">"+
		"</div>");
		lIsNew = true;
	}
	
	var lValueSpan = "";
		
	var lValueLowered = pValue.toLowerCase();
	
	var lDisplayIcon = undefined;
	if(lValueLowered == 'true' || lValueLowered == 'on' || lValueLowered == 'ok' || lValueLowered == 'enable' || lValueLowered == 'enabled')
	{
		lDisplayIcon = 'check_circle_outline';
		lDisplayColor = 'green';
	}
	else if(lValueLowered == 'false' || lValueLowered == 'off' || lValueLowered == 'error' || lValueLowered == 'warning')
	{
		lDisplayIcon = 'error';
		lDisplayColor = lValueLowered == 'warning' ? 'orange' : 'red';
	}
	else if(lValueLowered == 'disable' || lValueLowered == 'disabled')
	{
		lDisplayIcon = 'highlight_off';
		lDisplayColor = 'orange';
	}
	else if(lValueLowered == 'play' || lValueLowered == 'run')
	{
		lDisplayIcon = 'play_circle_outline';
		lDisplayColor = 'green';
	}
	else if(lValueLowered == 'pause' || lValueLowered == 'stop')
	{
		lDisplayIcon = 'pause_circle_outline';
		lDisplayColor = 'blue';
	}
	
	if(lDisplayIcon !== undefined)
	{
		lValueSpan = "<span class=\"statevalue settingsbutton\" style=\"color:"+lDisplayColor+";font-size:1.4em;\">" + lDisplayIcon + "</span>";
	}
	else
	{
		lValueSpan = "<span class=\"statevalue\">" + escapeHtml(pValue) + "</span>";
	}
	
	$("#"+lId).html("<div class=\"settingtitle\"><span class=\"settingname\">" + escapeHtml(pName) + "</span>" + lValueSpan + "</div>");
	
	return lIsNew;
}

function addParam(pName,pValue)
{
	$( "#paramscontainer" ).append( 
		"<div class=\"settingentry\" id=\"paramentry_" + escapeHtml(pName) + "\" paramname=\"" + escapeHtml(pName) + "\">"+
			"<div class=\"settingname\">" + escapeHtml(pName) + "</div>"+
			"<div class=\"settingvalue\" value=\""+escapeHtml(pValue)+"\">"+escapeHtml(pValue)+"</div>"+
		"</div>");
	
}

function createGetParamCallback(pName)
{
	return function(pResult){
		addParam(pName,JSON.parse(pResult.value));
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
	
	sortsettings();
	onfilter();
}

function onSettingChanged(pMsg)
{
	if(addOrUpdateSetting(pMsg.name,pMsg.value,pMsg.description,pMsg.inuse))
	{
		sortsettings();
		onfilter();
	}
	
}

function onStateChanged(pMsg)
{
	if(addOrUpdateState(pMsg.name,pMsg.value))
	{
		sortsettings();
		onfilter();
	}
	
}

function onseparatorclicked(pName)
{
	$('#' + pName + 'container').toggle();
	if(pName == "params" && $('#' + pName + 'container').attr('paramretrieved') === undefined)
	{
		$('#' + pName + 'container').attr('paramretrieved',true);
		$('#waiterroot').show();
		// we need to retrieve ros parameters
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
}

function onload()
{
	$('#statescontainer').show();
	$('#settingscontainer').show();
	$('#paramscontainer').hide();
	
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	sRosCtx.startListeningTopic('settings_store/Change','settings_store/Change',onSettingChanged);
	sRosCtx.startListeningTopic('settings_store/StateChange','settings_store/Change',onStateChanged);
	
	++sSettingsCtx.waitcptr;
	sRosCtx.callService('/settings_store/multiget','settingsstore/multiget',{requestednames:[]}, function(pResult) {
		for(var i = 0 ; i < pResult.names.length ; ++i)
		{
			addOrUpdateSetting(pResult.names[i],pResult.values[i],pResult.descriptions[i],pResult.inuse[i])
		}
		onParamRetrieved();
	})
	
	++sSettingsCtx.waitcptr;
	sRosCtx.callService('/settings_store/getstates','settingsstore/getstates',{requestednames:[]}, function(pResult) {
		for(var i = 0 ; i < pResult.names.length ; ++i)
		{
			addOrUpdateState(pResult.names[i],pResult.values[i])
		}
		onParamRetrieved();
	})
	
	
}

