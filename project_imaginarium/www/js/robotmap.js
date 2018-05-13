
function redraw()
{
	let canvas = document.getElementById('mapcanvas');
	let ctx = canvas.getContext('2d');
	ctx.fillStyle="red";
	ctx.clearRect(0, 0, canvas.width, canvas.height);
	// ctx.fillStyle="#eeeeee";
	ctx.fillStyle="red";
	ctx.fillRect(0,0,canvas.width, canvas.height);
	
	
	// draw grid 
	
	// ctx.clearRect(20,20,100,50);
}

function onload()
{
	// create ROS object
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	redraw()
}