
function resizeCanvans()
{
	var canvas = document.getElementById("mapcanvas");
	var parent = document.getElementById("maproot");
	
	// don't ask why ... with some brother setting canvas size to parent size, cause the parent to grow ...
	canvas.width = parent.offsetWidth-10;
	canvas.height = parent.offsetHeight-10;
	
	redraw();
}

sUltraSoundPosDir = [
		// 0 0°
		[170.,18.,1.,0.],
		// 1a 20°
		//[165.,82.,0.939693,0.34202],
		// 1b 0°
		[145.,-361.,1.,0.],
		// 2 40°
		[132.,132.,0.766044,0.642788],
		// 3 62°
		[86.,174.,0.469472,0.882948],
		// 4 80°
		[30.,194.,0.173648,0.984808],
		// 5 100°
		[-30.,194.,-0.173648,0.984808],
		// 6 123°
		[-92.,169.,-0.544639,0.838671],
		// 7 145°
		[-137.,140.,-0.819152,0.573576],
		// 8a 160°
		// [162.,87.,-0.939693,0.34202],
		// 8b 180°
		[-148.,-355.,-1.,0.],
		// 9 180°
		[-174.,24.,-1.,0.]
];
sUltraSoundSensorOrder = 
	//[0,1,2,3,4,5,6,7,8,9];
	[1,0,2,3,4,5,6,7,9,8];

sLastUltraSoundMeasure = [[1250.,53.],undefined,undefined,undefined,undefined,undefined,undefined,undefined,undefined,[-650.,53.]];
sWheelAngle = 15.;
	
function redraw()
{
	let canvas = document.getElementById('mapcanvas');
	let ctx = canvas.getContext('2d');
	
	sCanvasHelper.canvas = canvas;
	sCanvasHelper.ctx = ctx;
	
	// clear canvas 
	sCanvasHelper.clear(3000.,4000.,0.80);
	sCanvasHelper.drawGrid(250.,'rgba(0, 0, 0, 0.2)',2);
	sCanvasHelper.drawGrid(1000.0,'rgba(0, 0, 0, 0.2)',2);
	
	
	// draw center
	sCanvasHelper.drawRectangle(0,0,20,20,'rgba(0,0,0,0.7)',undefined,undefined);
	
	// draw robot border
	ctx.save();
	ctx.beginPath();
	let lPosTmp = sCanvasHelper.convertUnitToPx(sUltraSoundPosDir[sUltraSoundSensorOrder[0]].slice(0,2));
	ctx.moveTo(lPosTmp[0],lPosTmp[1]);
	
	for(let i = 1 ; i < sUltraSoundSensorOrder.length ; ++i)
	{
		lPosTmp = sCanvasHelper.convertUnitToPx(sUltraSoundPosDir[sUltraSoundSensorOrder[i]].slice(0,2));
		ctx.lineTo(lPosTmp[0],lPosTmp[1]);
	}
	ctx.closePath();
	ctx.lineWidth = 2;
	ctx.strokeStyle = 'rgba(0,0,0,0.7)';
	ctx.stroke();
	ctx.restore();
	
	// draw wheel
	ctx.save();
	ctx.rotate(Math.PI * sWheelAngle / 180.);
	sCanvasHelper.drawRectangle(0,0,30,200,'rgba(0,0,128,0.7)',undefined,undefined);
	ctx.restore();
	
	// draw position of each sensor
	if(sLastUltraSoundMeasure!== undefined && sLastUltraSoundMeasure.length == sUltraSoundSensorOrder.length)
	{
		for(let i = 0 ; i < sUltraSoundSensorOrder.length ; ++i)
		{
			let lSensorId = sUltraSoundSensorOrder[i];
			let lMeasure = sLastUltraSoundMeasure[lSensorId];
			let lSensorDir = sUltraSoundPosDir[lSensorId].slice(2,4);
			let lSensorPos = sUltraSoundPosDir[lSensorId].slice(0,2);
			if(lMeasure === undefined || (lMeasure[0] < 0.5 && lMeasure[1] < 0.5))
			{
				sCanvasHelper.drawRectangle(lSensorPos[0],lSensorPos[1],30,30,'rgba(255,0,0,0.7)',undefined,undefined);
			}
			else
			{
				sCanvasHelper.drawRectangle(lMeasure[0],lMeasure[1],30,30,'rgba(0,128,0,0.8)',undefined,undefined);
				
				// compuse measure pos in px
				let lPosTmp = sCanvasHelper.convertUnitToPx(lMeasure);
				
				// compute distance to sensor
				let lDelta = [lSensorPos[0]-lMeasure[0],lSensorPos[1]-lMeasure[1]];
				let lSensorDist = Math.sqrt(lDelta[0]*lDelta[0]+lDelta[1]*lDelta[1]);
				//tan(7.5°)
				let lHalfSegmentLen = lSensorDist*0.131652*sCanvasHelper.unitToPxScale;
				// rotate by 90°
				let lSegmentVector = [lSensorDir[1],-lSensorDir[0]]
				ctx.beginPath();
				ctx.moveTo(lPosTmp[0] + lSegmentVector[0]*lHalfSegmentLen,lPosTmp[1] + lSegmentVector[1]*lHalfSegmentLen);
				ctx.lineTo(lPosTmp[0] - lSegmentVector[0]*lHalfSegmentLen,lPosTmp[1] - lSegmentVector[1]*lHalfSegmentLen);
				ctx.lineWidth = 2;
				ctx.strokeStyle = 'rgba(0,128,0,0.7)';
				ctx.stroke();
				ctx.beginPath();
				
				
				lPosTmp[0] += lSensorDir[0]*10;
				lPosTmp[1] += lSensorDir[1]*10;
				ctx.save();
				ctx.translate(lPosTmp[0],lPosTmp[1]);
				ctx.scale(1.,-1.);
				ctx.font = "16px Arial";
				ctx.fillStyle = 'green'
				ctx.fillText(lSensorId.toString(),-4.,4.);
				ctx.restore();
			}
		}
	}
	
}

function onload()
{
	$('#maproot').ready(resizeCanvans());
	
	// create ROS object
	sRosCtx.initRos(function(pOk){
		if(pOk) $( "#roserror" ).hide();
		else $( "#roserror" ).show();
	});
	
	sRosCtx.startListeningTopic('emaginarium/Ultrasound','emaginarium/Ultrasound',function(data){console.log(data);});
	sRosCtx.startListeningTopic('emaginarium/CommandNosewheel','emaginarium/CommandNosewheel',function(data){console.log(data);});
	
	redraw();
}