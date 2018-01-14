var sRosCtx = {
	mHandle : undefined,
	mConnected : false,
	mTopics : {},
	mServices : {},
	
	initRos : function(pOnRosConnectionStatusCallback){
		sRosCtx.mHandle = new ROSLIB.Ros({
			url: 'ws://' + window.location.hostname + ':9090'
		});
		if(pOnRosConnectionStatusCallback !== undefined)
			pOnRosConnectionStatusCallback(false);
		sRosCtx.mHandle.on('connection', function() {
			console.log('Connected to websocket server.');
			sRosCtx.mConnected = true;
			if(pOnRosConnectionStatusCallback !== undefined)
				pOnRosConnectionStatusCallback(true);
		});

		sRosCtx.mHandle.on('error', function(error) {
			console.log('Error connecting to websocket server: ', error);
			sRosCtx.mConnected = false;
			if(pOnRosConnectionStatusCallback !== undefined)
				pOnRosConnectionStatusCallback(false);
		});

		sRosCtx.mHandle.on('close', function() {
			console.log('Connection to websocket server closed.');
			sRosCtx.mConnected = false;
			if(pOnRosConnectionStatusCallback !== undefined)
				pOnRosConnectionStatusCallback(false);
		});
	},

	ensureTopicIsCreated : function(pName,pType)
	{
		if( sRosCtx.mTopics[pName] === undefined)
		{
			sRosCtx.mTopics[pName] = { 
				topic : new ROSLIB.Topic({
					ros : sRosCtx.mHandle,
					name : pName,
					messageType : pType
				}),
				callbacks : []
			};
		}
	},

	startListeningTopic : function(pName, pType, pCallback)
	{
		sRosCtx.ensureTopicIsCreated(pName,pType);
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
		sRosCtx.ensureTopicIsCreated(pName,pType);
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

