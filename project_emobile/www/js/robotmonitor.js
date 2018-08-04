
function onload()
{
	// create ROS object
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	// create graphs
	sGraphs.init('graphs');
	
	lDistanceGraphConfig = {
			"title":"vu8",
			"y1":{
				"unit":"cm",
				"range":[-0.1,400.1],
				"rostraces":[
					
				]
			}
		}
	for(var i = 0 ; i < 8 ; ++i)
	{
		lDistanceGraphConfig["y1"]["rostraces"].push(
				{
						"name":i.toString(),
						"topicname":"/pointcloud",
						"messagetype":"/emobile/PointCloud",
						"messagefield":"distance",
						"fieldindex":i,
						"scale":100.
					});
	}
	
	sGraphs.addGraph(lDistanceGraphConfig);
	/*
	,
					{
						"name":"goal",
						"topicname":"/imaginarium_core/DiagThrust",
						"messagetype":"/imaginarium_core/DiagThrust",
						"messagefield":"goalSpeed"
					}
					*/
	sGraphs.addGraph(
		{
			"title":"actuator",
			"y1":{
				"unit":"power%",
				"range":[-1.01,1.01],
				"rostraces":[
					{
						"name":"power",
						"topicname":"/emobile/CommandThrottle",
						"messagetype":"/emobile/CommandThrottle",
						"messagefield":"throttle"
					}
				]
			},
			"y2":{
				"unit":"°",
				"range":[-1.01,1.01],
				"rostraces":[
					{
						"name":"wheel angle",
						"topicname":"/emobile/CommandSteering",
						"messagetype":"/emobile/CommandSteering",
						"messagefield":"steering"
					}
				]
			}
		});
		
	
	sGraphs.updateSizes();
}