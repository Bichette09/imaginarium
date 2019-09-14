
function resizeCanvans()
{
	var canvas = document.getElementById("mapcanvas");
	var parent = document.getElementById("maproot");
	
	// don't ask why ... with some brother setting canvas size to parent size, cause the parent to grow ...
	canvas.width = parent.offsetWidth-10;
	canvas.height = parent.offsetHeight-10;
	
	redraw();
}


var sLastLeddarMeasures = [];
var sWheelAngle = 0.;

var sWheelLocation = [[-110,0.],[+110,0.],[-110,265.],[110,265.]]

var sRequestRedraw = true;

function setMeasures(pX,pY)
{
	sLastLeddarMeasures = []
	for(var i = 0 ; i < pX.length ; ++i)
	{
		sLastLeddarMeasures.push([pX[i],pY[i]]);
	}
	sRequestRedraw = true;
}

function setCommandSteering(pMsg)
{
	lAngle = pMsg.steering*-35
	if(sWheelAngle != pMsg.steering)
	{
		sWheelAngle = lAngle;
		sRequestRedraw = true;
	}
}



function updateDraw()
{
	if(!sRequestRedraw)
		return;
	sRequestRedraw = false;
	redraw();
}

function redraw()
{
	let canvas = document.getElementById('mapcanvas');
	let ctx = canvas.getContext('2d');
	
	sCanvasHelper.canvas = canvas;
	sCanvasHelper.ctx = ctx;
	
	// clear canvas 
	sCanvasHelper.clear(5000.,6000.,0.70);
	sCanvasHelper.drawGrid(250.,'rgba(0, 0, 0, 0.2)',2);
	sCanvasHelper.drawGrid(1000.0,'rgba(0, 0, 0, 0.2)',2);
	
	
	// draw center
	sCanvasHelper.drawRectangle(0,0,20,20,'rgba(0,0,0,0.7)',undefined,undefined);
	
	// draw robot border
	ctx.globalAlpha = 0.5;
	sCanvasHelper.drawRectangle(0,250-134,220,500,undefined,'black',1.);
	ctx.globalAlpha = 1.;
	for(var i = 0 ; i < sWheelLocation.length ; ++i)
	{
		var lAngle = i <= 1 ? 0. : sWheelAngle;
		
		ctx.save();
		let lPosTmp = sCanvasHelper.convertUnitToPx([sWheelLocation[i][0],sWheelLocation[i][1]]);
	
		ctx.translate(lPosTmp[0],lPosTmp[1]);
		ctx.rotate(Math.PI * lAngle / 180.);
		sCanvasHelper.drawRectangle(-10,0,40,150,'rgba(0,0,128,0.5)',undefined,undefined);
		ctx.restore();
	}
	
	for(var i = 0 ; i < sLastLeddarMeasures.length ; ++i) // 
	{
		var lMeasuredPos = sLastLeddarMeasures[i];
		sCanvasHelper.drawCircle(lMeasuredPos[0],lMeasuredPos[1],0.030, i < 8 ? 'rgba(0,128,0,0.7)' : 'rgba(0,0,128,0.7)','black',1,true);
	}
	
	sCanvasHelper.drawDebug2dPrimitives(true);
	
}


function onload()
{
	$('#maproot').ready(resizeCanvans());
	
	// create ROS object
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	sRosCtx.startListeningTopic('pointcloud','emobile/PointCloud',function(data){setMeasures(data.x1,data.y1);});
	sRosCtx.startListeningTopic('/emobile/CommandSteering','emobile/CommandSteering',function(data){setCommandSteering(data);});
	sRosCtx.startListeningTopic('emobile/Debug2dPrimitive','emaginarium_common/Debug2dPrimitive',function(data){ if(sCanvasHelper.addDebug2dPrimitive(data)) sRequestRedraw = true;});
	
	sCanvasHelper.setUnitAndFrameTransform(1000.,true);
	
	redraw();
	setInterval(updateDraw, 30);
}