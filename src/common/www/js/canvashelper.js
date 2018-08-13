var sCanvasHelper = {
	
	ctx : undefined,
	canvas : undefined,
	unitToPxScale : undefined,
	pxToUnitScale : undefined,
	/* [xmin,ymin,xmax,ymax]*/
	boundMeter : [],
	boundPx : [],
	debug2dPrimitives : {},
	
	clear : function(pMinWidthMeter, pMinHeightMeter, pCenterYPercent)
	{
		this.ctx.resetTransform();
		
		this.ctx.fillStyle="#eeeeee";
		this.ctx.fillRect(0,0,this.canvas.width, this.canvas.height);
		
		// compute scale
		this.unitToPxScale = Math.min(this.canvas.width / pMinWidthMeter, this.canvas.height / pMinHeightMeter);
		this.pxToUnitScale = 1./this.unitToPxScale;
		
		// apply a transform to move robot origin
		// console.log('size ' + (this.pxToUnitScale * this.canvas.width).toString() + 'x' + (this.pxToUnitScale * this.canvas.height).toString() + 'm ' + this.canvas.width.toString() + 'x' + this.canvas.height.toString() + 'px');
		
		// we do not apply scale here to avoid blurry rendering due to non integer positions after transform
		this.ctx.transform(1.,0.,0.,-1., Math.round(this.canvas.width * 0.5),Math.round(this.canvas.height * pCenterYPercent));
		
		
		this.boundMeter = [-this.pxToUnitScale * this.canvas.width*0.5,-this.pxToUnitScale * this.canvas.height*(1.-pCenterYPercent),this.pxToUnitScale * this.canvas.width*0.5,this.pxToUnitScale * this.canvas.height*pCenterYPercent];
		this.boundPx = [Math.round(this.unitToPxScale*this.boundMeter[0]),Math.round(this.unitToPxScale*this.boundMeter[1]),Math.round(this.unitToPxScale*this.boundMeter[2]),Math.round(this.unitToPxScale*this.boundMeter[3])];
	},
	
	convertUnitToPx : function(pIn)
	{
		let lRes = [];
		for(let i = 0 ; i < pIn.length ; ++i)
		{
			lRes.push(pIn[i]*this.unitToPxScale);
		}
		return lRes;
	},
	
	drawCircle : function (pX,pY,pRadius, pFillColor, pBorderColor, pBorderWidth)
	{
		this.ctx.beginPath();
		this.ctx.arc(Math.round(pX*this.unitToPxScale), Math.round(pY*this.unitToPxScale), Math.round(pRadius*this.unitToPxScale), 0, 2 * Math.PI, false);
		if( pFillColor !== undefined)
		{
			this.ctx.fillStyle = pFillColor;
			this.ctx.fill();
		}
		
		if(pBorderColor !== undefined)
		{
			this.ctx.lineWidth = pBorderWidth;
			this.ctx.strokeStyle = pBorderColor;
			this.ctx.stroke();
		}
	},

	drawRectangle : function (pCenterX,pCenterY,pWidth,pHeight,pFillColor, pBorderColor, pBorderWidth)
	{
		this.ctx.beginPath();
	
		this.ctx.rect(Math.round((pCenterX - pWidth*0.5)*this.unitToPxScale), Math.round((pCenterY-pHeight*0.5)*this.unitToPxScale),Math.round(pWidth*this.unitToPxScale), Math.round(pHeight*this.unitToPxScale));
		if(pFillColor !== undefined)
		{
			this.ctx.fillStyle = pFillColor;
			this.ctx.fill();
		}
		if(pBorderColor !== undefined)
		{
			this.ctx.lineWidth = pBorderWidth;
			this.ctx.strokeStyle = pBorderColor;
			this.ctx.stroke();
		}
	},
	
	drawGrid : function(pStep, pColor, pWidth)
	{
		// this.ctx.lineWidth = 1.;
		// let lOffset = 0.1;// pWidth % 2 == 0 ? 0 : 0.5;
		this.ctx.beginPath();
			
		for(let x = Math.round(this.boundMeter[0] / pStep) - 1 ; x <= Math.round(this.boundMeter[2] / pStep) + 1 ; ++x)
		{
			let lXPx = Math.round(x*pStep*this.unitToPxScale);
			this.ctx.moveTo(lXPx,this.boundPx[1]);
			this.ctx.lineTo(lXPx,this.boundPx[3]);
		}
		
		for(let y = Math.round(this.boundMeter[1] / pStep) - 1 ; y <= Math.round(this.boundMeter[3] / pStep) + 1 ; ++y)
			// for(let y = 0 ; y <= 12 ; ++y)
		{
			let lYPx = Math.round(y*pStep*this.unitToPxScale);
			this.ctx.moveTo(this.boundPx[0],lYPx);
			this.ctx.lineTo(this.boundPx[2],lYPx);
			
		}
		this.ctx.closePath();
		this.ctx.lineWidth = pWidth;
		this.ctx.strokeStyle = pColor;
		this.ctx.stroke();
		// avoid that next call to stroke redraw our path
		this.ctx.beginPath();
		
	},
	
	addDebug2dPrimitive : function(pData)
	{
		if(this.debug2dPrimitives[pData.name] != undefined && this.debug2dPrimitives[pData.name] == pData)
		{
			return false;
		}
		
		this.debug2dPrimitives[pData.name] = pData;
		return true;
	},
	
	drawDebug2dPrimitives : function()
	{
		for(var k in this.debug2dPrimitives)
		{
			var lData = this.debug2dPrimitives[k]
			if (lData.data.length == 0)
				continue;
			if(lData.type == 'line')
			{
				this.drawLine(lData.data[0],lData.data[1],lData.color,2)
			}
			
		}
	},
	
	/** draw a line of equation y = ax+b
	*/
	drawLine : function(pA, pB, pColor, pWidth)
	{
		let lX0 = this.boundMeter[0];
		let lY0 = pA * lX0 + pB;
		let lX1 = this.boundMeter[2];
		let lY1 = pA * lX1 + pB;
		this.ctx.beginPath();
		this.ctx.moveTo(lX0*this.unitToPxScale,lY0*this.unitToPxScale);
		this.ctx.lineTo(lX1*this.unitToPxScale,lY1*this.unitToPxScale);
		this.ctx.closePath();
		this.ctx.lineWidth = pWidth;
		this.ctx.strokeStyle = pColor;
		this.ctx.stroke();
		// avoid that next call to stroke redraw our path
		this.ctx.beginPath();
	}

	
};

