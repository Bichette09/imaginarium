
function resizeCanvans()
{
	var canvas = document.getElementById("mapcanvas");
	var parent = document.getElementById("maproot");
	
	// don't ask why ... with some brother setting canvas size to parent size, cause the parent to grow ...
	canvas.width = parent.offsetWidth-10;
	canvas.height = parent.offsetHeight-10;
	
	redraw();
}

function redraw()
{
	let canvas = document.getElementById('mapcanvas');
	let ctx = canvas.getContext('2d');
	
	sCanvasHelper.canvas = canvas;
	sCanvasHelper.ctx = ctx;
	
	// clear canvas 
	sCanvasHelper.clear(6.,4.,0.80);
	sCanvasHelper.drawGrid(0.25,'rgba(0, 0, 0, 0.2)',2);
	sCanvasHelper.drawGrid(1.0,'rgba(0, 0, 0, 0.2)',2);
	
	
	// draw center
	sCanvasHelper.drawRectangle(-0.05,-0.05,0.1,0.1,'black',undefined,undefined);
	// sCanvasHelper.drawRectangle(0,0,100,10,undefined,'black',1);
	
	// ctx.beginPath();
	// ctx.moveTo(10,0);
	// ctx.lineTo(100,0);
	// ctx.lineWidth = 10;
	// ctx.strokeStyle = '#000000ff';
	// ctx.stroke();
	
	//drawCircle(ctx,0.,0.,0.5,'red','yellow',5);
	// ctx.fillStyle="red";
	// ctx.rect(0,0,canvas.width, canvas.height);
	// ctx.fillRect(0,0,canvas.width, canvas.height);
	// ctx.fill();
	// ctx.lineWidth=4;
	// ctx.stroke();
	
	// ctx.rect(20,20,150,100);
	// ctx.strokeStyle="black";
	// ctx.lineWidth="4px";
	// ctx.strokeRect(0,0,canvas.width, canvas.height);
	
	// ctx.lineWidth=4;
	// ctx.strokeRect(20,20,80,100); 
	
	// draw grid 
	
	// ctx.clearRect(20,20,100,50);
}

function onload()
{
	$('#maproot').ready(resizeCanvans());
	
	// create ROS object
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	redraw();
}