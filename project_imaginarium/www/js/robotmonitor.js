
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
						"topicname":"/imaginarium_core/Gyro",
						"messagetype":"/imaginarium_core/Gyro",
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
						"topicname":"/imaginarium_core/Gyro",
						"messagetype":"/imaginarium_core/Gyro",
						"messagefield":"acceleration",
						"fieldindex":1
					},{
						"name":"accel Y",
						"topicname":"/imaginarium_core/Gyro",
						"messagetype":"/imaginarium_core/Gyro",
						"messagefield":"acceleration",
						"fieldindex":2
					}
				]
			}
		});
	lDistanceGraphConfig = {
			"title":"ultrasound",
			"y1":{
				"unit":"cm",
				"range":[-0.1,400.1],
				"rostraces":[
					
				]
			}
		}
	for(var i = 0 ; i < 10 ; ++i)
	{
		lDistanceGraphConfig["y1"]["rostraces"].push(
				{
						"name":i.toString(),
						"topicname":"/imaginarium_core/Ultrasound",
						"messagetype":"/imaginarium_core/Ultrasound",
						"messagefield":"distance",
						"fieldindex":i
					});
	}
	
	sGraphs.addGraph(lDistanceGraphConfig);
	
	sGraphs.addGraph(
		{
			"title":"speed/dir",
			"y1":{
				"unit":"m/sec",
				"range":[-4,4],
				"rostraces":[
					{
						"name":"speed",
						"topicname":"/imaginarium_core/Speed",
						"messagetype":"/imaginarium_core/Speed",
						"messagefield":"speed"
					},
					{
						"name":"goal",
						"topicname":"/commandThrust/DiagThrust",
						"messagetype":"/commandThrust/DiagThrust",
						"messagefield":"goalSpeed"
					}
				]
			},
			"y2":{
				"unit":"°",
				"range":[-45,45],
				"rostraces":[
					{
						"name":"wheel angle",
						"topicname":"/commandThrust/DiagThrust",
						"messagetype":"/commandThrust/DiagThrust",
						"messagefield":"wheel angle"
					}
				]
			}
		});
	
	sGraphs.updateSizes();
}