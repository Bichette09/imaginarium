// define weatherstation object
var sGraphs = {
	mPaused : false,
	mTimeWinsowSizeSec : 30,
	mBiggestXValue : 0,
	mFixedRange : undefined,
	mTimeReference : null,
	mBlockRelayout : false,
	mGraphs :{},
	updateSizes : function (){
		let lStyle = '';
		// if we are in landscape mode, display two graphs in a row, else only one
		if(Object.keys(this.mGraphs).length > 1)
		{
			let lIsLandscape = ($(window).height() < $(window).width()) && $(window).width() > 512;
			if(lIsLandscape)
			{
				lStyle = 'width:50%;max-height:90vh';
			}
			else
			{
				// vh unit is not working on chrome for android ...
				lStyle = 'width:100%;height:50vh;max-height:40vh';
			}
			
		}
		else
		{
			lStyle = 'width:100%';
		}
		// update style of divs
		for(let id in this.mGraphs)
		{
			$( "#" + id).attr('style',lStyle)
		}
		
		// relayout
		for(let id in this.mGraphs)
		{
			Plotly.relayout(id, {});
		}
	},
	
	
	  
};

// refresh graph size on window resize
var sResizeEndTimer = undefined;
window.onresize = function(event) {
	if(sResizeEndTimer === undefined)
	{
		sResizeEndTimer = setTimeout(
		function(){
			sGraphs.updateSizes();
			clearTimeout(sResizeEndTimer);
			sResizeEndTimer = undefined;
		}
		,250);
	}
};

function Graph( pParams) {
	// retrieve div
	this.mParams = pParams;
	
	
	// create graph
	lGridColor = "rgba(0,0,0,0.2)";
	lShowGrid = true;
	
	// init layout
	let lLayout = {
			"title":this.mParams.title,
			paper_bgcolor: 'rgba(0,0,0,0)',
			plot_bgcolor: 'rgba(0,0,0,0)',
			xaxis:{
				showline:lShowGrid,
				showgrid: lShowGrid,
				zeroline: false,
				gridcolor: lGridColor,
				linecolor: lGridColor,
				type:"time"
			},
			yaxis: {
				title:this.mParams.y1.unit,
				fixedrange:true,
				autorange: this.mParams.y1.range === undefined ? true : false,
				showline: lShowGrid,
				showgrid: lShowGrid,
				zeroline: false,
				gridcolor: lGridColor,
				linecolor: lGridColor,
				hoverformat:".1f",
				range:this.mParams.y1.range
			},
			legend:{
				bgcolor: 'rgba(102, 102, 102, 0.15)',
				bordercolor: 'rgba(0, 0, 0, 0)',
				x: 0.,
				y: 1.,
				xanchor: 'left',
				yanchor: 'top'
			},
			margin : {
				l: 50,
				r: 50,
				b: 50,
				t: 50,
				pad: 4
			},
			showlegend: true,
			autosize: true
		};
	if(this.mParams.y1 !== undefined)
	{
		let lAutoRange1 = this.mParams.y1.range == undefined;
		lLayout.yaxis = {
				title:this.mParams.y1.unit,
				fixedrange: !lAutoRange1,
				autorange: lAutoRange1,
				showline: lShowGrid,
				showgrid: lShowGrid,
				zeroline: false,
				gridcolor: lGridColor,
				linecolor: lGridColor,
				hoverformat:".1f",
				range:this.mParams.y1.range
				
		};
	}

	if(this.mParams.y2 !== undefined)
	{
		let lAutoRange2 = this.mParams.y2.range == undefined;
		lLayout.yaxis2 = {
				title: this.mParams.y2.unit,
				fixedrange: !lAutoRange2,
				autorange: lAutoRange2,
				showline: false,
				showgrid: false,
				zeroline: false,
				range:this.mParams.y2.range,
				hoverformat:".1f",
				
				overlaying: "y",
				side: "right"
		};
	}
	
	
	
	// register callback on plotly_relayout to link all xaxis.range
	$("#" + this.mParams.divid).bind(
			'plotly_relayout',
			function(event,eventdata)
			{
				let lRange = undefined;
				if( 'xaxis.range[0]' in eventdata || 'xaxis.range[1]' in eventdata)
				{
					lRange = [0,0];
					if( 'xaxis.range[0]' in eventdata)
						lRange[0] = eventdata['xaxis.range[0]'];
					if( 'xaxis.range[1]' in eventdata)
						lRange[1] = eventdata['xaxis.range[1]'];	
				}
				else if( 'xaxis.range' in eventdata)
				{
					return;
				}
				else if('xaxis.autorange' in eventdata)
				{
					lRange = undefined;
				}
				if(sGraphs.mBlockRelayout)
					return;
				sGraphs.mBlockRelayout = true;
				sGraphs.mFixedRange = lRange;
				for( key in sGraphs.mGraphs)
				{
					sGraphs.mGraphs[key].updateRange();
				}
				sGraphs.mBlockRelayout = false;
			}
		);
	var lGraph = this;
	lGraph.mTraces = [];
	// method to clear graph content
	this.clearData = function()
	{
		let lIndices = [];
		for(var i = 0 ; i < lGraph.mTraces.length ; ++i)
		{
			lIndices.push(i);
		}
		if(lIndices.length > 0)
		{
			Plotly.deleteTraces(lGraph.mParams.divid,lIndices);
		}
		
		lGraph.mTraces = [];
		var lAddRosTrace = function(pGraph,pTrace, pMainAxis) {
			pTrace.mPendingDataX = [];
			pTrace.mPendingDataY = [];
			pGraph.mTraces.push(pTrace);
			pTrace.mMainAxis = pMainAxis;
			sRosCtx.startListeningTopic(pTrace.topicname,pTrace.messagetype,function(message){
					let lX = 0.;
					if( message.header !== undefined)
					{
						let lXMsec = Math.floor(message.header.stamp.nsecs * 1e-6);
						lX = message.header.stamp.secs + lXMsec * 1e-3;
					}
					else
					{
						lX = new Date().getTime() / 1000.;
					}
					
					if(sGraphs.mTimeReference == undefined)
					{
						sGraphs.mTimeReference = lX;
					}
					lX -= sGraphs.mTimeReference;
					let lY = message[pTrace.messagefield];
					if(pTrace.fieldindex !== undefined )
					{
						lY = lY[pTrace.fieldindex];
					}
					
					if(pTrace.scale !== undefined)
					{
						lY *= pTrace.scale;
					}
					
					if(lX !== undefined && lY !== undefined)
					{
						pTrace.mPendingDataX.push(lX);
						pTrace.mPendingDataY.push(lY);
					}
				});
			
			var lTraceDef = {
				x : [],
				y: [],
				type: 'scattergl',
				opacity:0.8,
				mode: 'lines', 
				line: {
					width: 2
				},
				yaxis: pMainAxis ? 'y' : 'y2'
			};
			for(var k in pTrace)
			{
				if( k !== 'x' &&
					k !== 'y' && 
					k !== 'topicname' &&
					k !== 'messagetype'){
					lTraceDef[k] = pTrace[k];
				}
			}
			return lTraceDef;
		}
		var lTraces = [];
		for( var t in lGraph.mParams.y1.rostraces)
		{
			lTraces.push(lAddRosTrace(lGraph,lGraph.mParams.y1.rostraces[t],true));
		}
		
		if(lGraph.mParams.y2 !== undefined)
		{
			for( var t in lGraph.mParams.y2.rostraces)
			{
				lTraces.push(lAddRosTrace(lGraph,lGraph.mParams.y2.rostraces[t],false));
			}
		}
		// create graph
		Plotly.plot(lGraph.mParams.divid, lTraces, lLayout, {showLink: false,displayModeBar: false,displaylogo: false});
		
		
		
		
	};
	this.clearData();
	
	this.updateRange = function(){
		let lRange = sGraphs.mFixedRange !== undefined ? sGraphs.mFixedRange : [sGraphs.mBiggestXValue - sGraphs.mTimeWinsowSizeSec,sGraphs.mBiggestXValue];
		Plotly.relayout(lGraph.mParams.divid,'xaxis.range',lRange);
	}
	
	setInterval(function(){
		if(sGraphs.mPaused)
			return;
		// iterate over all traces to generate append command
		let lUpdate = {x:[],y:[]};
		let lIndices = [];
		for(var i = 0 ; i < lGraph.mTraces.length ; ++i)
		{
			if(lGraph.mTraces[i].mPendingDataY.length > 0)
			{
				lUpdate.x.push(lGraph.mTraces[i].mPendingDataX);
				for(var j = 0 ; j < lGraph.mTraces[i].mPendingDataX.length ; ++j)
				{
					sGraphs.mBiggestXValue = Math.max(sGraphs.mBiggestXValue,lGraph.mTraces[i].mPendingDataX[j]);
				}
				
				lUpdate.y.push(lGraph.mTraces[i].mPendingDataY);
				lGraph.mTraces[i].mPendingDataX = [];
				lGraph.mTraces[i].mPendingDataY = [];
				lIndices.push(i);
			}
		}
		if(lIndices.length > 0)
		{
			Plotly.extendTraces(lGraph.mParams.divid,lUpdate,lIndices);
			lGraph.updateRange();
		}
	}
	,100);
	
}


function graphbuttonclick(event){
	if(event.target.id == "play")
	{
		
		if(event.target.innerHTML == "pause_circle_outline")
		{
			event.target.innerHTML = "play_circle_outline";
			sGraphs.mPaused = true;
		}
		else
		{
			event.target.innerHTML = "pause_circle_outline";
			sGraphs.mPaused = false;
		}
	}
	else if(event.target.id == "clear")
	{
		sGraphs.clearGraph();
	}
	
}

function graphspinchange(event) {
	sGraphs.mTimeWinsowSizeSec = event.target.value;
	for(let id in sGraphs.mGraphs)
	{
		sGraphs.mGraphs[id].updateRange();
	}
}


sGraphs.init = function(pDivId)
{
	let lDiv = $("#" + pDivId);
	lDiv.append("<div id=\"graphctrl\" class=\"graphctrl\"></div>");
	
	$("#graphctrl").append("<div id=\"play\" class=\"graphbutton\" onclick=\"graphbuttonclick(event)\">pause_circle_outline</div>");
	$("#graphctrl").append("<div id=\"clear\" class=\"graphbutton\" onclick=\"graphbuttonclick(event)\">block</div>");
	$("#graphctrl").append("<input id=\"timewindow\" autocomplete=on type=\"number\" class=\"graphspinbox\" min=\"10\" max=\"300\" value=\"30\" oninput=\"graphspinchange(event)\">s</input>");
	lDiv.append("<div id=\"graphroot\" class=\"graphroot\"></div>");
}

sGraphs.addGraph = function(pParams)
{
	let lGraphDivId = "graph_" + pParams.title.replace(/[\/\\]/,'_');
	$("#graphroot").append("<div id=\"" + lGraphDivId + "\" class=\"graph\"></div>\n");
	pParams.divid = lGraphDivId;
	sGraphs.mGraphs[lGraphDivId] = new Graph(pParams);
}


sGraphs.clearGraph = function()
{
	
	if( this.mCurrentRequest != null)
	{
		this.mCurrentRequest.abort();
		this.mCurrentRequest = null;
	}
	this.mCurrentData = null;
	for( key in sGraphs.mGraphs)
	{
		sGraphs.mGraphs[key].clearData();
	}
	sGraphs.mTimeReference = undefined;
	sGraphs.mBiggestXValue = 0;
	sGraphs.mFixedRange = undefined;
}

