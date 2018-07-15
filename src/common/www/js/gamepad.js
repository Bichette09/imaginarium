var sGamePad = {
	mIsInit : false,
	mGamePadId : null,
	mTimer : null,
	mLastPressedButtons : undefined,
	mLastStickCheckSum : undefined
};

function onProcessGamePad(){
	let lGamePads = navigator.getGamepads();
	let lGamePad = undefined;
	// on chrome getGamepads is an array of null elements
	for(var i = 0 ; i < lGamePads.length ; ++i)
	{
		// look for last valid gamepad with at least one button
		if(lGamePads[i] != undefined && lGamePads[i].buttons.length > 0)
			lGamePad = lGamePads[i];
	}
	
	if(!sGamePad.mIsInit)
	{
		if(lGamePad != undefined)
		{
			onConnect(lGamePad);
		}
		return;
	}
	
	if(lGamePad == undefined || lGamePad.id != sGamePad.mGamePadId)
	{
		onDisconnect();
		return;
	}
	let lPressedButtons = '|';
	for(var i = 0 ; i < lGamePad.buttons.length ; ++i)
	{
		let lDiv = $( "#button_" + i.toString());
		if(lGamePad.buttons[i].pressed)
		{
			lDiv.removeClass("gamepad_buttonnotpressed");
			lDiv.addClass("gamepad_buttonpressed");
			
			lPressedButtons += ("0" + i).slice(-3) + "|";
			lPressedButtons += lDiv.text() + "|";
		}
		else
		{
			lDiv.removeClass("gamepad_buttonpressed");
			lDiv.addClass("gamepad_buttonnotpressed");
		}
	}
	
	if(lPressedButtons != sGamePad.mLastPressedButtons)
	{
		sGamePad.mLastPressedButtons = lPressedButtons;
		sRosCtx.writeMsg('GamePadButtons','std_msgs/String',{data:lPressedButtons});
	}
	
	let lStickValues = [];
	let lStickValuesCss = [];
	let lStickCheckSum = 0;
	let lStickPrecision = 50.;
	for( var i = 0 ; i < lGamePad.axes.length ; ++i)
	{
		let lValFiltered = Math.round(lGamePad.axes[i] * lStickPrecision) / lStickPrecision;
		// filter dead zone near zero
		if( Math.abs(lValFiltered) < 0.15)
			lValFiltered = 0.;
		lStickCheckSum += Math.pow(lStickPrecision,i) * lValFiltered;
		lStickValues.push(lValFiltered);
		let lVal = lValFiltered;
		lVal = (lVal + 1.) * 0.5; // [-1;1] => [0;1]
		lVal = (lVal * 90); // [0;1] => [0;90]
		lStickValuesCss.push(lVal);
	}
	
	if(lStickCheckSum != sGamePad.mLastStickCheckSum)
	{
		sGamePad.mLastStickCheckSum = lStickCheckSum;
		sRosCtx.writeMsg('GamePadSticks','std_msgs/Float32MultiArray',{layout:{dim:[{label:'',size:lStickValues.length,stride:0}]},data:lStickValues});
	}
	
	for(var i = 0 ; i < lStickValuesCss.length ; i+=2)
	{
		$( "#axes_" + i.toString()).css('top',lStickValuesCss[i+1].toString() + '%');
		$( "#axes_" + i.toString()).css('left',lStickValuesCss[i].toString() + '%');
	}
}

function onDisconnect(){
	sGamePad.mIsInit = false;
	sGamePad.mGamePadId = undefined;
	sGamePad.mLastPressedButtons = undefined;
	sGamePad.mLastStickCheckSum = undefined;
	$( "#connectinfo" ).show();
	$( "#gamepadroot" ).hide();
}

var sXBox360Layout = ['A','B','X','Y','LB','RB','LT','RT','back','start','lstick','rstick','up','down','left','right','?']

function onConnect(pGamePad){
	sGamePad.mIsInit = true;
	sGamePad.mGamePadId = pGamePad.id;
	$( "#connectinfo" ).hide();
	$( "#gamepadroot" ).show();
	$( "#gamepadid").text(pGamePad.id);
	$( "#gamepadbuttoncontainer" ).html('');
	
	let lButtonLayout = undefined;
	let lGamePadIdLc = pGamePad.id.toLowerCase();
	let lIsXBox = false;
	lIsXBox = lIsXBox || (lGamePadIdLc.indexOf('xbox') != -1 && lGamePadIdLc.indexOf('360') != -1);
	lIsXBox = lIsXBox || (lGamePadIdLc.indexOf('vendor: 045e') != -1 && lGamePadIdLc.indexOf('product: 028e') != -1);
	if( lIsXBox && pGamePad.buttons.length >= 16)
	{
		lButtonLayout = sXBox360Layout;
	}
	
	for(var i = 0 ; i < pGamePad.buttons.length ; ++i)
	{
		let lButtonName = 'B' + i.toString();
		if(lButtonLayout !== undefined)
		{
			lButtonName = lButtonLayout[i];
		}
		$( "#gamepadbuttoncontainer" ).append('<div id="button_' + i.toString() + '" class="gamepad_button gamepad_buttonnotpressed"><div class="gamepad_buttontext">' + lButtonName + '</div></div>');
	}
	$( "#gamepadaxescontainer" ).html('');
	for(var i = 0 ; i < pGamePad.axes.length ; i+=2)
	{
		$( "#gamepadaxescontainer" ).append('<div class="gamepad_axesroot"><div id="axes_' + i.toString() + '" class="gamepad_axes"></div></div>');
	}
}



function onload() {
	
	if(! ("getGamepads" in navigator))
	{
		$( "#supporterror" ).show();
		return;
	}
	$( "#supporterror" ).hide();
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	})
	
	onDisconnect();
	
	sGamePad.mTimer = window.setInterval(onProcessGamePad,25);
	
	
	$(window).on("gamepadconnected", onProcessGamePad);
	$(window).on("gamepaddisconnected",onProcessGamePad);
	
}