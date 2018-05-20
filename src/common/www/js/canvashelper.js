var sCanvasHelper = {
	
	ctx : undefined,
	canvas : undefined,
	meterToPxScale : undefined,
	pxToMeterScale : undefined,
	boundMeter : [],
	boundPx : [],
	
	clear : function(pMinWidthMeter, pMinHeightMeter, pCenterYPercent)
	{
		this.ctx.resetTransform();
		
		this.ctx.fillStyle="#eeeeee";
		this.ctx.fillRect(0,0,this.canvas.width, this.canvas.height);
		
		// compute scale
		this.meterToPxScale = Math.min(this.canvas.width / pMinWidthMeter, this.canvas.height / pMinHeightMeter);
		this.pxToMeterScale = 1./this.meterToPxScale;
		
		// apply a transform to move robot origin
		console.log('size ' + (this.pxToMeterScale * this.canvas.width).toString() + 'x' + (this.pxToMeterScale * this.canvas.height).toString() + 'm ' + this.canvas.width.toString() + 'x' + this.canvas.height.toString() + 'px');
		
		// we do not apply scale here to avoid blurry rendering due to non integer positions after transform
		this.ctx.transform(1.,0.,0.,-1., Math.round(this.canvas.width * 0.5),Math.round(this.canvas.height * pCenterYPercent));
		
		
		this.boundMeter = [-this.pxToMeterScale * this.canvas.width*0.5,-this.pxToMeterScale * this.canvas.height*(1.-pCenterYPercent),this.pxToMeterScale * this.canvas.width*0.5,this.pxToMeterScale * this.canvas.height*pCenterYPercent];
		this.boundPx = [Math.round(this.meterToPxScale*this.boundMeter[0]),Math.round(this.meterToPxScale*this.boundMeter[1]),Math.round(this.meterToPxScale*this.boundMeter[2]),Math.round(this.meterToPxScale*this.boundMeter[3])];
	},
	
	drawCircle : function (pX,pY,pRadius, pFillColor, pBorderColor, pBorderWidth)
	{
		this.ctx.beginPath();
		this.ctx.arc(Math.round(pX*this.meterToPxScale), Math.round(pY*this.meterToPxScale), Math.round(pRadius*this.meterToPxScale), 0, 2 * Math.PI, false);
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

	drawRectangle : function (pX,pY,pWidth,pHeight,pFillColor, pBorderColor, pBorderWidth)
	{
		this.ctx.rect(Math.round(pX*this.meterToPxScale), Math.round(pY*this.meterToPxScale),Math.round(pWidth*this.meterToPxScale), Math.round(pHeight*this.meterToPxScale));
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
			let lXPx = Math.round(x*pStep*this.meterToPxScale);
			this.ctx.moveTo(lXPx,this.boundPx[1]);
			this.ctx.lineTo(lXPx,this.boundPx[3]);
		}
		
		for(let y = Math.round(this.boundMeter[1] / pStep) - 1 ; y <= Math.round(this.boundMeter[3] / pStep) + 1 ; ++y)
		{
			let lYPx = Math.round(y*pStep*this.meterToPxScale);
			this.ctx.moveTo(this.boundPx[0],lYPx);
			this.ctx.lineTo(this.boundPx[2],lYPx);
			
		}
		this.ctx.closePath();
		this.ctx.lineWidth = pWidth;
		this.ctx.strokeStyle = pColor;
		this.ctx.stroke();
		// avoid that next call to stroke redraw our path
		this.ctx.beginPath();
		
	}
	

	
};

