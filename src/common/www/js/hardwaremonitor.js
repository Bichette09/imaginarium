
function updateScalarValue(pName, pMessage, pPrecision, pUnit, pStyle)
{
	let lVal = pMessage[pName];
	let lStyle = 'hw_scalarentry'
	for(var key in pStyle)
	{
		var lRange = pStyle[key];
		if( lVal >= lRange[0] && lVal <= lRange[1])
		{
			lStyle = lStyle + ' ' + key;
			break;
		}
	}
	$( "#scalar_" + pName).html( lVal.toFixed(pPrecision) + pUnit);
	$( "#scalar_" + pName).parent().attr('class',lStyle)
}

function onload()
{
	// create ROS object
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	sRosCtx.startListeningTopic('/hardware_monitor','hardware_monitor/msg',function(message) {
		updateScalarValue('cpuload',message,0,"%",{'hw_ok':[0,50],'hw_warning':[50,75],'hw_critical':[75,1000]})
		updateScalarValue('cputemp',message,0,"°C",{'hw_ok':[0,50],'hw_warning':[50,60],'hw_critical':[60,1000]})
		updateScalarValue('cpufreq',message,0,"MHz",{'hw_ok':[0,1000],'hw_warning':[1000,5000],'hw_critical':[0,0]})
		updateScalarValue('memload',message,0,"%",{'hw_ok':[0,50],'hw_warning':[50,75],'hw_critical':[75,1000]})
		updateScalarValue('lo',message,0,"kB/s",{'hw_ok':[0,200],'hw_warning':[200,1000],'hw_critical':[1000,100000]})
		updateScalarValue('wifi',message,0,"kB/s",{'hw_ok':[0,200],'hw_warning':[200,1000],'hw_critical':[1000,100000]})
		updateScalarValue('eth',message,0,"kB/s",{'hw_ok':[0,200],'hw_warning':[200,1000],'hw_critical':[1000,100000]})
		updateScalarValue('wifistrength',message,0,"%",{'hw_ok':[60,100],'hw_warning':[40,60],'hw_critical':[0,40]})
	},'hwmonitor');
	
	// create graphs
	sGraphs.init('graphs');
	sGraphs.addGraph(
		{
			"title":"cpu/mem",
			"y1":{
				"unit":"%",
				"range":[-0.1,100.1],
				"rostraces":[
					{
						"name":"cpu",
						"topicname":"/hardware_monitor",
						"messagetype":"/hardware_monitor/msg",
						"messagefield":"cpuload"
					},{
						"name":"mem",
						"topicname":"/hardware_monitor",
						"messagetype":"/hardware_monitor/msg",
						"messagefield":"memload"
					}
				]
			}
		});
	sGraphs.addGraph(
		{ 
			"title":"network",
			"y1":{
				"unit":"kB/s",
				"range":null,
				"rostraces":[
					{
						"name":"wifi",
						"topicname":"/hardware_monitor",
						"messagetype":"/hardware_monitor/msg",
						"messagefield":"wifi"
					},{
						"name":"eth",
						"topicname":"/hardware_monitor",
						"messagetype":"/hardware_monitor/msg",
						"messagefield":"eth"
					},{
						"name":"lo",
						"topicname":"/hardware_monitor",
						"messagetype":"/hardware_monitor/msg",
						"messagefield":"lo"
					}
				]
			},
			"y2":{
				"unit":"%",
				"range":[0,100],
				"rostraces":[
					{
						"name":"wifi strength",
						"topicname":"/hardware_monitor",
						"messagetype":"/hardware_monitor/msg",
						"messagefield":"wifistrength"
					}
				]
			}
		});
	sGraphs.addGraph(
		{
			"title":"temp/freq",
			"y1":{
				"unit":"MHz",
				"range":[99.9,1300.1],
				"rostraces":[
					{
						"name":"cpufreq",
						"topicname":"/hardware_monitor",
						"messagetype":"/hardware_monitor/msg",
						"messagefield":"cpufreq"
					}
				]
			},
			"y2":{
				"unit":"°C",
				"range":[-0.1,120.1],
				"rostraces":[
					{
						"name":"cputemp",
						"topicname":"/hardware_monitor",
						"messagetype":"/hardware_monitor/msg",
						"messagefield":"cputemp"
					}
				]

			}
		});
	sGraphs.updateSizes();
}