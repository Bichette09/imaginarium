
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
			"title":"img processor",
			"y1":{
				"unit":"fps",
				"range":[0,30],
				"rostraces":[
					{
						"name":"fps",
						"topicname":"/grading_machine/ImgProcessorStat",
						"messagetype":"/grading_machine/ImgProcessorStat",
						"messagefield":"fps"
					}
				]
			},
			"y2":{
				"unit":"ms",
				"range":[0.,600.],
				"rostraces":[
					{
						"name":"latency",
						"topicname":"/grading_machine/ImgProcessorStat",
						"messagetype":"/grading_machine/ImgProcessorStat",
						"messagefield":"latency"
					},{
						"name":"filter",
						"topicname":"/grading_machine/ImgProcessorStat",
						"messagetype":"/grading_machine/ImgProcessorStat",
						"messagefield":"filter"
					},{
						"name":"areaextraction",
						"topicname":"/grading_machine/ImgProcessorStat",
						"messagetype":"/grading_machine/ImgProcessorStat",
						"messagefield":"areaextraction"
					}
				]
			}
		});
	
	sGraphs.updateSizes();
}