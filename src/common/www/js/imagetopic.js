var sImageTopicData = {}


function createTopicTypeCallback(pTopic) 
{
	return function(pResult){
		// sImageTopicData.services[pService]['type'] = pResult.type;
		if(pResult.type == 'sensor_msgs/CompressedImage')
		{
			sImageTopicData.topics.push(pTopic.replace('/compressed',''));
		}
		onTopicRetrieved();
	}
}


function retrieveTopics()
{
	$('#waiterroot').show();
	$('#graphcontainer').hide();
	sImageTopicData = {
		waitcptr : 0,
		topics : []
	}
	
	// request topology from rosapi
	++sImageTopicData.waitcptr;
	sRosCtx.callService('/rosapi/topics','rosapi/Topics',{}, function(pResult) {
		for(var i = 0 ; i < pResult.topics.length ; ++i)
		{
			var lTopic = pResult.topics[i];
			if(lTopic.indexOf('/compressed') < 0)
				continue;
			 
			++sImageTopicData.waitcptr;
			sRosCtx.callService('/rosapi/topic_type','rosapi/TopicType',{topic:lTopic}, createTopicTypeCallback(lTopic));
		}
		onTopicRetrieved();
	})
	
}

var entityMap = {
  '&': '&amp;',
  '<': '&lt;',
  '>': '&gt;',
  '"': '&quot;',
  "'": '&#39;',
  '/': '&#x2F;',
  '`': '&#x60;',
  '=': '&#x3D;'
};
function escapeHtml (string) {
  return String(string).replace(/[&<>"'`=\/]/g, function (s) {
    return entityMap[s];
  });
}

function onTopicRetrieved()
{
	--sImageTopicData.waitcptr;
	if(sImageTopicData.waitcptr != 0)
		return;
	$('#waiterroot').hide();
	$('#graphcontainer').show();
	// sImageTopicData.topics.push('test');
	// sImageTopicData.topics.push('vroum courm');
	// sImageTopicData.topics.push('aaa courm');
	// sImageTopicData.topics.push('/aaa courm');
	// create a div for each topic
	for(var i = 0 ; i < sImageTopicData.topics.length ; ++i)
	{
		let lTopic = sImageTopicData.topics[i];
		$('#topiclist').append( 
			"<label class=\"topicentry\" topicid=\"" + escapeHtml(lTopic) + "\">" +
				"<input type=\"checkbox\" onchange=\"oncheckchanged(event)\" >"+
				"<span class=\"topicentryspan\">" + escapeHtml(lTopic) + "</span>"+
			"</label>"
			);
	}
	$("#topiclist .topicentry").sort(function (a, b) {
		return $(a).attr('topicid').localeCompare($(b).attr('topicid'));
	}).each(function () {
		var elem = $(this);
		$(elem).appendTo("#topiclist");
	});
	// console.log(sImageTopicData.services);
	
}


function onload()
{
	// create ROS object
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	retrieveTopics();
	
}

function oncheckchanged(event)
{
	// let lIsChecked = $(event.target).is(":checked");
	let lIsChecked = event.target.checked;
	let lTopic = event.target.parentElement.getAttribute('topicid');
	let lTopicImgId = lTopic.replace(new RegExp('/', 'g'),'_');

	if(!lIsChecked)
	{
		// remove image from 
		$('#img_' + lTopicImgId).remove();
	}
	else
	{
		let lStreamUrl = 'http://' +  window.location.hostname + ':8080/stream?topic=' + lTopic + '&type=ros_compressed';
		// create image
		$('#imagecontainer').append( 
			"<img id=\"img_" + escapeHtml(lTopicImgId) + "\" title=\"" + escapeHtml(lTopic) + "\"" +
				"src=\""+ escapeHtml(lStreamUrl) + "\" "+
				"alt=\""+ escapeHtml(lStreamUrl) + "\" "+
				"></img>"
			);
			
		// sort
		$("#imagecontainer img").sort(function (a, b) {
			return $(a).attr('id').localeCompare($(b).attr('id'));
		}).each(function () {
			var elem = $(this);
			$(elem).appendTo("#imagecontainer");
		});
	}
}