
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
			"title":"cpu",
			"y1":{
				"unit":"%",
				"range":[-0.1,100.1],
				"rostraces":[
					{
						"name":"cpu",
						"topicname":"/hardware_monitor",
						"messagetype":"/hardware_monitor/msg",
						"messagefield":"cpuload"
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