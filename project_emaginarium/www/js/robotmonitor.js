
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
						"topicname":"/emaginarium_common/Gyro",
						"messagetype":"/emaginarium_common/Gyro",
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
						"topicname":"/emaginarium_common/Gyro",
						"messagetype":"/emaginarium_common/Gyro",
						"messagefield":"acceleration",
						"fieldindex":1
					},{
						"name":"accel Y",
						"topicname":"/emaginarium_common/Gyro",
						"messagetype":"/emaginarium_common/Gyro",
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
						"topicname":"/emaginarium/Ultrasound",
						"messagetype":"/emaginarium/Ultrasound",
						"messagefield":"distance",
						"fieldindex":i
					});
	}
	
	sGraphs.addGraph(lDistanceGraphConfig);
	/*
	,
					{
						"name":"goal",
						"topicname":"/emaginarium/DiagThrust",
						"messagetype":"/emaginarium/DiagThrust",
						"messagefield":"goalSpeed"
					}
					*/
	sGraphs.addGraph(
		{
			"title":"speed/dir",
			"y1":{
				"unit":"m/sec",
				"range":[-4,4],
				"rostraces":[
					{
						"name":"speed",
						"topicname":"/emaginarium/Speed",
						"messagetype":"/emaginarium/Speed",
						"messagefield":"speed"
					},{
						"name":"speedtarget",
						"topicname":"/emaginarium/SpeedTarget",
						"messagetype":"/emaginarium/SpeedTarget",
						"messagefield":"speedtarget"
					}
				]
			},
			"y2":{
				"unit":"°",
				"range":[-45,45],
				"rostraces":[
					{
						"name":"wheel angle",
						"topicname":"/emaginarium/CommandNosewheel",
						"messagetype":"/emaginarium/CommandNosewheel",
						"messagefield":"nosewheelAngle"
					}
				]
			}
		});
		
	sGraphs.addGraph(
		{
			"title":"wheel_cmd",
			/*"range":[-4,4],*/
			"y1":{
				"unit":"?",
				"rostraces":[
					{
						"name":"propcommand",
						"topicname":"/emaginarium/CommandNosewheel",
						"messagetype":"/emaginarium/CommandNosewheel",
						"messagefield":"propCommand"
					},{
						"name":"preccommand",
						"topicname":"/emaginarium/CommandNosewheel",
						"messagetype":"/emaginarium/CommandNosewheel",
						"messagefield":"precCommand"
					}
				]
			},
			"y2":{
				"unit":"°",
				"rostraces":[
					{
						"name":"left_angle",
						"topicname":"/emaginarium/CommandNosewheel",
						"messagetype":"/emaginarium/CommandNosewheel",
						"messagefield":"leftAngle"
					},{
						"name":"right_angle",
						"topicname":"/emaginarium/CommandNosewheel",
						"messagetype":"/emaginarium/CommandNosewheel",
						"messagefield":"rightAngle"
					}
				]
			}
		});
	
	sGraphs.updateSizes();
}