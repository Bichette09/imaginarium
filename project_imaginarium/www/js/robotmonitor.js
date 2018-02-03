
function onload()
{
	// create ROS object
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	// create graphs
	sGraphs.init('graphs');
	sGraphs.addGraph(
		{
			"title":"gyro",
			"y1":{
				"unit":"°/sec",
				"range":[-10,10],
				"rostraces":[
					{
						"name":"angular rot",
						"topicname":"/imaginarium_core/Measures",
						"messagetype":"/imaginarium_core/Measures",
						"messagefield":"angular_speed",
						"fieldindex":2
					}
				]
			},
			"y2":{
				"unit":"m/s-2",
				"range":[-5,5],
				"rostraces":[
					{
						"name":"accel X",
						"topicname":"/imaginarium_core/Measures",
						"messagetype":"/imaginarium_core/Measures",
						"messagefield":"acceleration",
						"fieldindex":1
					},{
						"name":"accel Y",
						"topicname":"/imaginarium_core/Measures",
						"messagetype":"/imaginarium_core/Measures",
						"messagefield":"acceleration",
						"fieldindex":2
					}
				]
			}
		});
	sGraphs.addGraph(
		{
			"title":"mem",
			"y1":{
				"unit":"%",
				"range":[-0.1,100.1],
				"rostraces":[
					{
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
			"title":"temp",
			"y1":{
				"unit":"°C",
				"range":[-0.1,100.1],
				"rostraces":[
					{
						"name":"temp",
						"topicname":"/hardware_monitor",
						"messagetype":"/hardware_monitor/msg",
						"messagefield":"cputemp"
					}
				]
			}
		});
	
	sGraphs.updateSizes();
}