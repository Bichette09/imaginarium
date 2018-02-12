var sTopologyData = {}

function removeDuplicate(pIn)
{
	var lRes = [];
	$.each(pIn, function(i, el){
		if($.inArray(el, lRes) === -1) lRes.push(el);
	});
	pIn = lRes;
}

function getListenCallback(pTopic)
{
	return function(msg) {
		sTopologyData.topics[pTopic].stats.push(Date.now());
	};
}

function updateTopicUsage()
{
	for(var lTopic in sTopologyData.topics)
	{
		var lTopicInfo = sTopologyData.topics[lTopic];
		if(lTopicInfo.stats === undefined)
			continue;
		var lTopicNodeId = 'topic_' + lTopic;
		var lTopicNode = sTopologyData.nodesdataset.get(lTopicNodeId);
		
		var lStaticLabel = lTopicNode.label.substring(0,lTopicNode.label.lastIndexOf('\n')+1);
		
		if(lTopicInfo.stats === false)
		{
			lTopicInfo.stats = undefined;
			sTopologyData.nodesdataset.update({id: lTopicNodeId, label: lStaticLabel+' '});
			continue;
		}
		
		while(lTopicInfo.stats.length >= 2)
		{
			var lLastTs = lTopicInfo.stats[lTopicInfo.stats.length-1]
			var lFirstTs = lTopicInfo.stats[0]
			// we use a sliding window (1 seconds if there is more than 15 samples else 5 seconds)
			// if no sample were received since 2 seconds, clear the whole array
			if( (lTopicInfo.stats.length > 15 && (lLastTs - lFirstTs) > 1000) ||
				((lLastTs - lFirstTs) > 5000)
				)
			{
				lTopicInfo.stats.shift();
				continue;
			}
			else if((Date.now() - lLastTs) > 2000)
			{
				lTopicInfo.stats = [];
				break;
			}
			break;
		}
		
		var lMsgPerSec = 0.;
		if(lTopicInfo.stats.length >= 2)
		{
			var lLastTs = lTopicInfo.stats[lTopicInfo.stats.length-1]
			var lFirstTs = lTopicInfo.stats[0]
			var lElapsedMs = lLastTs - lFirstTs;
			if(lElapsedMs > 0)
			{
				lMsgPerSec = Math.round(lTopicInfo.stats.length * 10 / (lElapsedMs/1000.))/10.;
			}
		}
		
		sTopologyData.nodesdataset.update({id: lTopicNodeId, label: lStaticLabel + lMsgPerSec + ' msg/s'});
	}
}

function setTopicToMonitor(pTopicList)
{
	// disconnect from current topics if they are no more in lTopicList
	for(var idx in sTopologyData.monitoredTopics)
	{
		var lTopic = sTopologyData.monitoredTopics[idx];
		if($.inArray(lTopic, pTopicList) !== -1)
			continue;
		sRosCtx.stopListeningTopic(lTopic);
		// set to false to trigger clean of node label
		sTopologyData.topics[lTopic].stats = false;
	}
	// for entries in pTopicList that are not in monitoredTopics
	for(var idx in pTopicList)
	{
		var lTopic = pTopicList[idx];
		if($.inArray(lTopic, sTopologyData.monitoredTopics) !== -1)
			continue;
		sTopologyData.topics[lTopic].stats = [];
		sRosCtx.startListeningTopic(lTopic,sTopologyData.topics[lTopic].type,getListenCallback(lTopic));
	}
	sTopologyData.monitoredTopics = pTopicList;
	updateTopicUsage();
}

function keepNode(pNodeName)
{
	return pNodeName.indexOf('/rosapi') === -1 && pNodeName.indexOf('/rosout') === -1; 
}

function keepTopic(pTopicName)
{
	return pTopicName.indexOf('/rosout') === -1;
}

function keepService(pServiceName)
{
	return pServiceName.indexOf('/get_loggers') === -1 && pServiceName.indexOf('/set_logger_level') === -1;
}

// here I use a function to create the callback so pNode will not be overwriten when called in a loop
function createNodeDetailsCallback(pNode) 
{
	return function(pResult){
		sTopologyData.nodes[pNode] = {
			subscribing : [],
			publishing : [],
			services : []
		};
		for(var idx in pResult.subscribing)
		{
			if(!keepTopic(pResult.subscribing[idx]))
				continue;
			sTopologyData.nodes[pNode].subscribing.push(pResult.subscribing[idx]);
		}
		for(var idx in pResult.publishing)
		{
			if(!keepTopic(pResult.publishing[idx]))
				continue;
			sTopologyData.nodes[pNode].publishing.push(pResult.publishing[idx]);
		}
		for(var idx in pResult.services)
		{
			if(!keepService(pResult.services[idx]))
				continue;
			sTopologyData.nodes[pNode].services.push(pResult.services[idx]);
		}
		onTopologyRetrieved();
	}
}

function createServiceTypeCallback(pService) 
{
	return function(pResult){
		sTopologyData.services[pService]['type'] = pResult.type;
		
		/*++sTopologyData.waitcptr;
		sRosCtx.callService('/rosapi/service_request_details','rosapi/ServiceRequestDetails',{type:pResult.type}, function(pInfo){
			console.log(pInfo);
			onTopologyRetrieved();
		});
		++sTopologyData.waitcptr;
		sRosCtx.callService('/rosapi/service_response_details','rosapi/ServiceResponseDetails',{type:pResult.type}, function(pInfo){
			console.log(pInfo);
			onTopologyRetrieved();
		});
		*/
		onTopologyRetrieved();
	}
}


function retrieveTopology()
{
	$('#waiterroot').show();
	$('#graphcontainer').hide();
	setTopicToMonitor([]);
	sTopologyData = {
		nodes : {},
		topics : {},
		services : {},
		waitcptr : 0,
		callcount : 0,
		network:undefined,
		nodesdataset:undefined,
		edgesdataset:undefined,
		monitoredTopics:[]
	}
	
	// request topology from rosapi
	++sTopologyData.waitcptr;
	sRosCtx.callService('/rosapi/nodes','rosapi/Nodes',{}, function(pResult) {
		for(var i = 0 ; i < pResult.nodes.length ; ++i)
		{
			var lNode = pResult.nodes[i];
			if( !keepNode(lNode) )
				continue;
			sTopologyData.nodes[lNode] = {};
			
			++sTopologyData.waitcptr;
			sRosCtx.callService('/rosapi/node_details','rosapi/NodeDetails',{node:lNode}, createNodeDetailsCallback(lNode));
		}
		onTopologyRetrieved();
	})
	
	++sTopologyData.waitcptr;
	sRosCtx.callService('/rosapi/services','rosapi/Services',{}, function(pResult) {
		for(var i = 0 ; i < pResult.services.length ; ++i)
		{
			var lService = pResult.services[i];
			if( !keepService(lService) )
				continue;
			
			sTopologyData.services[lService] = {'providers':[]};
			
			++sTopologyData.waitcptr;
			sRosCtx.callService('/rosapi/service_type','rosapi/ServiceType',{service:lService}, createServiceTypeCallback(lService));
		}
		onTopologyRetrieved();
	})
	
	++sTopologyData.waitcptr;
	sRosCtx.callService('/rosapi/topics','rosapi/Topics',{}, function(pResult) {
		for(var i = 0 ; i < pResult.topics.length ; ++i)
		{
			var lTopic = pResult.topics[i];
			if( !keepNode(lTopic) )
				continue;
			
			var lType = pResult.types[i];
			sTopologyData.topics[lTopic] = {'type':lType,'subscribers':[],'publishers':[],'stats':undefined};
		}
		onTopologyRetrieved();
	})
}


function onTopologyRetrieved()
{
	--sTopologyData.waitcptr;
	if(sTopologyData.waitcptr != 0)
		return;
	$('#waiterroot').hide();
	$('#graphcontainer').show();
	// console.log(sTopologyData.services);
	var lNodes = [];
	var lEdges = [];
	var lCreatedNodes = {};
	
	for( var lNode in sTopologyData.nodes)
	{
		// create a graph node for the ros node
		lNodes.push({
				id:'node_' + lNode,
				font: { multi: true },
				'label':'<b>' + lNode + '</b>',
				shape: 'circle',
				color: {
					background:'rgb(30, 190, 190)',
					highlight:{
						border:'#000000',
						background:'rgb(30, 190, 190)'
					}
				}
			}
		);
		
		for( var lTopicIdx in sTopologyData.nodes[lNode].publishing)
		{
			var lTopic = sTopologyData.nodes[lNode].publishing[lTopicIdx];
			if($.inArray(sTopologyData.topics[lTopic].publishers, lNode) === -1)
				sTopologyData.topics[lTopic].publishers.push(lNode);
		}
		for( var lTopicIdx in sTopologyData.nodes[lNode].subscribing)
		{
			var lTopic = sTopologyData.nodes[lNode].subscribing[lTopicIdx];
			if($.inArray(sTopologyData.topics[lTopic].subscribers, lNode) === -1)
				sTopologyData.topics[lTopic].subscribers.push(lNode);
		}
		
		var lServiceTxt = '';
		for( var lServiceIdx in sTopologyData.nodes[lNode].services)
		{
			var lService = sTopologyData.nodes[lNode].services[lServiceIdx];
			if($.inArray(sTopologyData.services[lService].providers, lNode) === -1)
				sTopologyData.services[lService].providers.push(lNode);
			if(lServiceTxt.length !== 0)
				lServiceTxt += '\n';
			lServiceTxt += '<b>' + lService + '</b> <i>(' + sTopologyData.services[lService].type + ')</i>';
		}
		
		if( lServiceTxt.length !== 0)
		{
			lNodes.push({
					id:'service_' + lNode,
					font: { multi: true },
					'label':lServiceTxt,
					shape: 'box',
					color: {
						background:'rgb(142, 197, 255)',
						highlight:{
							border:'#000000',
							background:'rgb(142, 197, 255)'
						}
					}
				}
			);
			
			lEdges.push({
				from:'node_' + lNode, 
				to: 'service_' + lNode,
				arrows:'to',
				color:{
					color:'black'
				}
			});
		}
		
	}
	
	for( var lTopic in sTopologyData.topics)
	{
		var lTopicInfo = sTopologyData.topics[lTopic];
		var lColor = 'black';
		if(lTopicInfo.subscribers.length == 0 && lTopicInfo.publishers.length == 0)
		{
			continue;
		}
		else if(lTopicInfo.subscribers.length != 0 && lTopicInfo.publishers.length != 0)
		{
			lColor = 'rgb(10, 200, 90)';
		}
		else if(lTopicInfo.publishers.length == 0)
		{
			lColor = 'rgb(250, 20, 20)';
		}
		else
		{
			lColor = 'rgb(200, 150, 50)';
		}
		
		lNodes.push({
				id:'topic_' + lTopic,
				font: { multi: true },
				'label': '<b>' + lTopic + '</b>\n<i>' + lTopicInfo.type + '</i>\n ',
				shape: 'box',
				color: {
					background:lColor,
					highlight:{
						border:'#000000',
						background:lColor
					}
				}
			}
		);
		
		for(var lIdx in lTopicInfo.subscribers)
		{
			lEdges.push({
				from:'node_' + lTopicInfo.subscribers[lIdx], 
				to: 'topic_' + lTopic,
				arrows:'from',
				color:{
					color:'black'
				}
			});
		}
		for(var lIdx in lTopicInfo.publishers)
		{
			lEdges.push({
				from:'node_' + lTopicInfo.publishers[lIdx], 
				to: 'topic_' + lTopic,
				arrows:'to',
				color:{
					color:'black'
				}
			});
		}
		
	}
	
	// create a network
	var container = document.getElementById('graphcontainer');
	sTopologyData.nodesdataset = new vis.DataSet(lNodes);
	sTopologyData.edgesdataset = new vis.DataSet(lEdges);
	var data = {
		nodes: sTopologyData.nodesdataset,
		edges: sTopologyData.edgesdataset
	};
	var options = {
		layout: {
			randomSeed:2,
			improvedLayout:true//,
			// randomSeed:10,
			/* ,hierarchical:{
				 edgeMinimization: false,
				 nodeSpacing:250
				// sortMethod: "directed"
			 }*/
		},
		physics:{
			barnesHut:{
				avoidOverlap:0.2
			},
			forceAtlas2Based:{
				avoidOverlap:1.8,
				centralGravity:0.1
			},
			solver:'barnesHut',
			stabilization: {
				enabled: true,
				iterations: 500,
				updateInterval: 25
			}
		},
		autoResize: true,
		height: '100%',
		width: '100%',
		interaction:{
			multiselect:true
		}
	};
	sTopologyData.network = new vis.Network(container, data, options);
	sTopologyData.network.fit();
	sTopologyData.network.on("stabilizationIterationsDone", function () {
		sTopologyData.network.setOptions( { physics: false } );
	});
	sTopologyData.network.on("click", function (params) {
		
		var lTopicList = [];
		// iterate over selected nodes to find which topics should be listened
		for(var lNodeIdx in params.nodes)
		{
			var lGraphNodeId = params.nodes[lNodeIdx];
			if( lGraphNodeId.indexOf('topic_') == 0)
			{
				// a topic
				lTopicList.push(lGraphNodeId.substring(6));
			}
			else if( lGraphNodeId.indexOf('node_') == 0)
			{
				// a node
				var lRosNodeId = lGraphNodeId.substring(5);
				for(var idx in sTopologyData.nodes[lRosNodeId].publishing)
				{
					lTopicList.push(sTopologyData.nodes[lRosNodeId].publishing[idx]);
				}
				for(var idx in sTopologyData.nodes[lRosNodeId].subscribing)
				{
					lTopicList.push(sTopologyData.nodes[lRosNodeId].subscribing[idx]);
				}
			}
		}
		removeDuplicate(lTopicList);
		
		setTopicToMonitor(lTopicList);
	});
}


function onload()
{
	// create ROS object
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	retrieveTopology();
	setInterval(updateTopicUsage, 500);
}