var sRosCtx = {
	mHandle : undefined,
	mConnected : false,
	mTopics : {},
	mServices : {},
	
	initRos : function(pOnRosConnectionStatusCallback){
		$('body').addClass('roserror');
		sRosCtx.mHandle = new ROSLIB.Ros({
			url: 'ws://' + window.location.hostname + ':9090'
		});
		if(pOnRosConnectionStatusCallback !== undefined)
			pOnRosConnectionStatusCallback(false);
		sRosCtx.mHandle.on('connection', function() {
			console.log('Connected to websocket server.');
			sRosCtx.mConnected = true;
			$('body').removeClass('roserror');
			if(pOnRosConnectionStatusCallback !== undefined)
				pOnRosConnectionStatusCallback(true);
		});

		sRosCtx.mHandle.on('error', function(error) {
			console.log('Error connecting to websocket server: ', error);
			sRosCtx.mConnected = false;
			$('body').addClass('roserror');
			if(pOnRosConnectionStatusCallback !== undefined)
				pOnRosConnectionStatusCallback(false);
		});

		sRosCtx.mHandle.on('close', function() {
			console.log('Connection to websocket server closed.');
			sRosCtx.mConnected = false;
			$('body').addClass('roserror');
			if(pOnRosConnectionStatusCallback !== undefined)
				pOnRosConnectionStatusCallback(false);
		});
		
		window.addEventListener('unload', function(event) {
			sRosCtx.finalizeRos();
		});
	},
	
	finalizeRos : function()
	{
		if(sRosCtx.mHandle == undefined)
			return;
		for(var lKey in sRosCtx.mTopics)
		{
			if(sRosCtx.mTopics[lKey].writer)
			{
				console.log('unadvertise ' + lKey);
				sRosCtx.mTopics[lKey].unadvertise();
			}
		}
		sRosCtx.mHandle.close();
		sRosCtx.mHandle = undefined;
	},

	ensureTopicIsCreated : function(pName,pType, pIsWriter)
	{
		if( sRosCtx.mTopics[pName] === undefined)
		{
			sRosCtx.mTopics[pName] = { 
				topic : new ROSLIB.Topic({
					ros : sRosCtx.mHandle,
					name : pName,
					messageType : pType
				}),
				writer : pIsWriter,
				callbacks : []
			};
		}
	},

	startListeningTopic : function(pName, pType, pCallback)
	{
		sRosCtx.ensureTopicIsCreated(pName,pType,false);
		if(sRosCtx.mTopics[pName].callbacks.length == 0)
		{
			sRosCtx.mTopics[pName].topic.subscribe(function(message){
				for(var f in sRosCtx.mTopics[pName].callbacks)
				{
					sRosCtx.mTopics[pName].callbacks[f](message);
				}
			})
		}
		sRosCtx.mTopics[pName].callbacks.push(pCallback);
	},

	stopListeningTopic : function (pName)
	{
		if( sRosCtx.mTopics[pName] === undefined)
			return;
		sRosCtx.mTopics[pName].topic.unsubscribe();
		delete sRosCtx.mTopics[pName];
	},

	writeMsg : function (pName,pType,pMsg)
	{
		if(!sRosCtx.mConnected)
			return;
		sRosCtx.ensureTopicIsCreated(pName,pType,true);
		sRosCtx.mTopics[pName].topic.publish(pMsg);
	},
	
	callService : function(pName, pType, pPayload, pResCallback)
	{
		if(sRosCtx.mServices[pName] === undefined)
		{
			sRosCtx.mServices[pName] = new ROSLIB.Service({
				ros : sRosCtx.mHandle,
				name : pName,
				serviceType : pType
			});
		}
		
		var lRequest = new ROSLIB.ServiceRequest(pPayload);
		sRosCtx.mServices[pName].callService(lRequest,pResCallback);
	}
};

