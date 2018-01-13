#include "mainwindow.h"


#include <QWheelEvent>
#include <QScrollBar>
#include <QDebug>
#include <QOpenGLWidget>
#include <QGraphicsLineItem>
#include <QGraphicsEllipseItem>
#include <QTextDocument>
#include <QTime>
#include <MyGraphView.h>
#include <ZmqSubscriber.h>

#include <SensorLocItem.h>

#include "ui_mainwindow.h"

using namespace QtCharts;

static const QColor sSensorColors[10] = { QColor(0xbb,0x00,0xff), QColor(0xff,0x00,0x62), QColor(0xbf,0xff,0x00), QColor(0xff,0x09,0x00), QColor(0xff,0x00,0xae), QColor(0x00,0xc8,0xff), QColor(0x00,0x59,0xff), QColor(0xff,0x7b,0x00), QColor(0x0d,0xff,0x00), QColor(0x00,0x95,0xff)};

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
  , mMapScene(NULL)
  , mMapScale(1.)
  , mAddSensorIds(true)
  , mTopChart(NULL)
  , mBottomChart(NULL)
  , mGyroChart(NULL)
  , mLastTimeStamp(0.)
  , mStartTimeStamp(0.)
  , mSub(NULL)
{
    ui->setupUi(this);

	ui->splitter->setSizes(QList<int>()<<width()/2<<width()/2);

	ui->mMapGraphicsView->installEventFilter(this);
	ui->mMapGraphicsView->viewport()->installEventFilter(this);
	ui->mMapGraphicsView->setRenderHints( QPainter::Antialiasing | QPainter::TextAntialiasing | QPainter::SmoothPixmapTransform | QPainter::HighQualityAntialiasing);
	ui->mMapGraphicsView->setTransformationAnchor(QGraphicsView::AnchorViewCenter);




	{
		mTopChart = new MyGraphView(this);
		ui->mTopChartContainer->layout()->addWidget(mTopChart);

		MyGraphView::AxisParameters lLeft;
		lLeft.mLabel = "m/s";
		lLeft.mRangeMinValue = -1;
		lLeft.mRangeMaxValue = 1.;
		lLeft.mAutoRange = true;
		lLeft.mCenterZero = true;
		mTopChart->setAxisParameter(lLeft,true);

		MyGraphView::AxisParameters lRight;
		lRight.mLabel = "%";
		lRight.mRangeMinValue = -1.02;
		lRight.mRangeMaxValue = 1.02;
		lLeft.mAutoRange = false;
		lLeft.mCenterZero = true;
		mTopChart->setAxisParameter(lRight,false);

		mTopChart->declareSerie("speed",true, Qt::darkGreen);
		mTopChart->declareSerie("goal speed",true,Qt::blue);

		mTopChart->declareSerie("thrust",false,Qt::red);
	}

	{
		mBottomChart = new MyGraphView(this);
		ui->mBottomChartContainer->layout()->addWidget(mBottomChart);

		MyGraphView::AxisParameters lLeft;
		lLeft.mLabel = "m";
		lLeft.mRangeMinValue = 0.;
		lLeft.mRangeMaxValue = 1.;
		lLeft.mAutoRange = true;
		lLeft.mCenterZero = false;
		mBottomChart->setAxisParameter(lLeft,true);

		MyGraphView::AxisParameters lRight;
		lRight.mLabel = "°";
		lRight.mRangeMinValue = -30.02;
		lRight.mRangeMaxValue = 30.02;
		lLeft.mAutoRange = false;
		lLeft.mCenterZero = true;
		mBottomChart->setAxisParameter(lRight,false);

		mBottomChart->declareSerie("left",true,Qt::red);
		mBottomChart->declareSerie("right",true,Qt::green);
		mBottomChart->declareSerie("front",true,Qt::blue);

		mBottomChart->declareSerie("wheel angle",false,Qt::darkGreen);
	}

    {
        mGyroChart = new MyGraphView(this);
        ui->mGyroChartContainer->layout()->addWidget(mGyroChart);

        MyGraphView::AxisParameters lLeft;
        lLeft.mLabel = "°/s";
        lLeft.mRangeMinValue = -132.;
        lLeft.mRangeMaxValue = 132.;
        lLeft.mAutoRange = true;
        lLeft.mCenterZero = true;
        mGyroChart->setAxisParameter(lLeft,true);

        MyGraphView::AxisParameters lRight;
		lRight.mLabel = "g";
		lRight.mRangeMinValue = -2.02;
		lRight.mRangeMaxValue = 2.02;
        lLeft.mAutoRange = true;
        lLeft.mCenterZero = true;
        mGyroChart->setAxisParameter(lRight,false);

        mGyroChart->declareSerie("ωZ",true,Qt::red);
        mGyroChart->declareSerie("aX",false,Qt::darkGreen);
        mGyroChart->declareSerie("aY",false,Qt::blue);

    }


	initMapScene();


	mNoDataTimer.setSingleShot(true);
	mNoDataTimer.setInterval(2000);
	mNoDataTimer.stop();
	QObject::connect(&mNoDataTimer,SIGNAL(timeout()),this,SLOT(onNoDataTimeOut()));

	clearAll();
	on_mTimePerdiodSpin_valueChanged(0);
}

MainWindow::~MainWindow()
{
    delete mSub;mSub = NULL;
    delete ui;
	delete mMapScene;
	mMapScene = NULL;
}

void MainWindow::initMapScene()
{
	mMapScene = new QGraphicsScene(this);
	ui->mMapGraphicsView->setScene(mMapScene);

	mMapScene->setBackgroundBrush(QBrush(QColor(238,238,238),Qt::SolidPattern));

	// create center
	QPen lCentralPen(Qt::black);
	lCentralPen.setCosmetic(true);
	mMapScene->addEllipse(-5,-5,10,10,lCentralPen);

	// create grid
	QPen lMainLinePen(QColor(190,190,190));
	lMainLinePen.setWidthF(2.);
	lMainLinePen.setStyle(Qt::DashLine);
	lMainLinePen.setCosmetic(true);
	QPen lSecLinePen(QColor(210,210,210),Qt::DashLine);
	lSecLinePen.setWidthF(2.);
	lSecLinePen.setStyle(Qt::DashDotDotLine);
	lSecLinePen.setCosmetic(true);

	const int lStepSize = 25;
	const int lStepCount = 50;
	for( int lX = -lStepCount ; lX <= lStepCount ; ++lX)
	{
		QPen & lPen = (lX % 4) == 0 ? lMainLinePen : lSecLinePen;

		mMapScene->addLine(lX * lStepSize, lStepCount * lStepSize, lX * lStepSize,-lStepCount * lStepSize,lPen );
		mMapScene->addLine(lStepCount * lStepSize,lX * lStepSize,-lStepCount * lStepSize, lX * lStepSize,lPen );
	}

	recenterMapView();
}

void MainWindow::recenterMapView()
{
	QTransform lXForm;
	lXForm.scale(mMapScale,mMapScale);
	ui->mMapGraphicsView->setTransform(lXForm);

	double lHeight = ui->mMapGraphicsView->mapToScene(ui->mMapGraphicsView->rect()).boundingRect().height();
	ui->mMapGraphicsView->centerOn(QPointF(0., -lHeight / 6.));
}

bool MainWindow::eventFilter(QObject * pWatched, QEvent * pEvent)
{
	if(pWatched == ui->mMapGraphicsView || pWatched == ui->mMapGraphicsView->viewport()	)
	{
		switch(pEvent->type())
		{
		case QEvent::Resize:
			recenterMapView();
			break;
		case QEvent::Wheel:
			{
				QWheelEvent * lEvent = static_cast<QWheelEvent*>(pEvent);
				if(lEvent->delta() > 0)
				{
					mMapScale *= 1.1f;
				}
				else
				{
					mMapScale /= 1.1f;
				}
				mMapScale = std::max(0.5f,std::min(mMapScale,3.f));
				recenterMapView();
				lEvent->accept();
				return true;
			}
			break;
		default:
			break;
		}
	}
	return QMainWindow::eventFilter(pWatched,pEvent);
}

void MainWindow::onNoDataTimeOut()
{
	ui->statusBar->showMessage("offline");
}

void MainWindow::on_mTimePerdiodSpin_valueChanged(int)
{
	double lTMin = mLastTimeStamp - mStartTimeStamp - ui->mTimePerdiodSpin->value();
	double lTMax = mLastTimeStamp - mStartTimeStamp;
	mTopChart->setTimeWindow(lTMin,lTMax);
	mBottomChart->setTimeWindow(lTMin,lTMax);
	mGyroChart->setTimeWindow(lTMin,lTMax);
}

void MainWindow::on_mAddressLineEdit_returnPressed()
{
    delete mSub;mSub = NULL;
    mSub = new ZmqSubscriber(ui->mAddressLineEdit->text().toUtf8().constData(),"mapviewer");
    QObject::connect(mSub,SIGNAL(newMessage(QVariantMap)),this,SLOT(onNewData(QVariantMap)),Qt::QueuedConnection);
    mLastTimeStamp = 0;
    mStartTimeStamp = 0;
}

 void MainWindow::on_mPauseButton_pressed()
 {
    if(ui->mPauseButton->isChecked())
    {
        mLastTimeStamp = 0;
        mStartTimeStamp = 0;
        ui->statusBar->showMessage("pause");
    }
    else
    {
        ui->statusBar->showMessage("offline");
    }
 }

QPointF extractPoint(const QVariant & pVal)
{
	QVariantList lList = pVal.toList();
	if(lList.size() != 2)
		return QPointF(0.,0.);
	return QPointF(lList[0].toDouble(),-lList[1].toDouble());
}

QPointF extractPoint(const QVariantMap & pMap, const char * pKey)
{
	return extractPoint(pMap.value(QString::fromUtf8(pKey)));
}

void MainWindow::clearAll()
{
	mLastTimeStamp = 0.;
	mStartTimeStamp = 0.;
	mNoDataTimer.stop();
	onNoDataTimeOut();
	mTopChart->clearValues();
	mBottomChart->clearValues();

}


void MainWindow::onNewData(const QVariantMap & pMap)
{
	mNoDataTimer.stop();
	mNoDataTimer.start();

    if(ui->mPauseButton->isChecked())
        return;

	double lTs = 0.;
	double lDelta = 0.;
	{
		double lLastTs = pMap.value("ts").toDouble();
        if(mStartTimeStamp == 0 || mStartTimeStamp > lLastTs || mLastTimeStamp > lLastTs)
		{
			clearAll();
		}
		if(mStartTimeStamp == 0.)
			mStartTimeStamp = lLastTs;
		lDelta = mLastTimeStamp > 0. ? (lLastTs - mLastTimeStamp) : 0.;
		mLastTimeStamp = lLastTs;
		lTs = lLastTs - mStartTimeStamp;
	}

	// update time window
	on_mTimePerdiodSpin_valueChanged(0);


	// update status labels
	QString lStatusMessage;
	lStatusMessage += (pMap.value("islock",false).toBool() ? "disabled" : "ready");
	lStatusMessage += " / ";
	lStatusMessage += (pMap.value("isautomatic",false).toBool() ? "automatic" : "manual");
	lStatusMessage += " / ";
	lStatusMessage += "door cptr " + QString::number(pMap.value("doorcptr",0).toInt());
	lStatusMessage += " / ";
	lStatusMessage += "refresh @" + QString::number( lDelta > 0. ? 1./lDelta : 0.,'f',1) + "Hz";
	ui->statusBar->showMessage(lStatusMessage);

	// add sensor loc
	if(mAddSensorIds)
	{
		QVariantList lSensorLocs = pMap.value("sensorloc").toList();
		for(int i = 0 ; i < 10 && i < lSensorLocs.size() ; ++i)
		{
			mAddSensorIds = false;
			QPointF lSensorLoc = extractPoint(lSensorLocs[i]);

			SensorLocItem * lRectItem = new SensorLocItem(QString::number(i),QRectF(lSensorLoc.x()-5,lSensorLoc.y()-5,10.,10.),sSensorColors[i]);
			mMapScene->addItem(lRectItem);

		}
	}

	// update map
	{
		QVariantList lHitPoints = pMap.value("hitpoints").toList();

		// ensure we have enought items
		while(mSensorLines.size() < lHitPoints.size())
		{
			mSensorLines.push_back(mMapScene->addLine(0.,0.,0.,0.));
			mSensorPoints.push_back(mMapScene->addEllipse(0.,0.,0.,0.));
		}

		// hide previous items
		foreach(QGraphicsLineItem * lItem, mSensorLines)
		{
			lItem->setVisible(false);
		}
		foreach(QGraphicsEllipseItem * lItem, mSensorPoints)
		{
			lItem->setVisible(false);
		}

		// iterate over hit points
		QVariantList::const_iterator lHitPointIt = lHitPoints.begin();
		const QVariantList::const_iterator lHitPointItEnd = lHitPoints.end();
		tLineItems::iterator lLineIt = mSensorLines.begin();
		tEllipseItems::iterator lCenterIt = mSensorPoints.begin();
		for( ; lHitPointIt != lHitPointItEnd ; ++lHitPointIt , ++lLineIt , ++lCenterIt)
		{
			QVariantMap lHitPoint = lHitPointIt->toMap();
			QPointF lPt = extractPoint(lHitPoint,"pt");
			QPointF lDir = extractPoint(lHitPoint,"dir");
			int lSensorId = lHitPoint.value("sensorid",0).toInt();
			double lInfluencedist = lHitPoint.value("influencedist",1.).toDouble();
			//double lTs = lHitPoint.value("ts",0.).toDouble();
			//double lDistToSensor = lHitPoint.value("dist2sensor",0.).toDouble();

			QGraphicsEllipseItem * lCenter = *lCenterIt;
			lCenter->setRect(lPt.x()-2.,lPt.y()-2.,4.,4.);
			lCenter->setPen(QPen(sSensorColors[lSensorId]));
			lCenter->setBrush(QBrush(sSensorColors[lSensorId],Qt::SolidPattern));
			lCenter->setVisible(true);

			QPointF lDirCross(lDir.y(),-lDir.x());
			QPointF lA = lPt + lDirCross * lInfluencedist * 0.5;
			QPointF lB = lPt - lDirCross * lInfluencedist * 0.5;
			QGraphicsLineItem * lLine = *lLineIt;
			lLine->setLine(lA.x(),lA.y(),lB.x(),lB.y());
			QPen lLinePen(sSensorColors[lSensorId]);
			lLinePen.setCosmetic(true);
			lLine->setPen(lLinePen);
			lLine->setVisible(true);
		}


	}

	// update series
	{
		double lTs = mLastTimeStamp - mStartTimeStamp;

		mTopChart->addValue("speed",lTs,pMap.value("speedmeasured").toDouble());
		mTopChart->addValue("goal speed",lTs,pMap.value("speedgoal").toDouble());
		mTopChart->addValue("thrust",lTs,pMap.value("thrust").toDouble());

		//mTopChart->removeOldest(lTs - 120);
		mTopChart->updateAxisRanges();

		mBottomChart->addValue("right",lTs,pMap.value("rightdist").toDouble());
		mBottomChart->addValue("left",lTs,pMap.value("leftdist").toDouble());
		mBottomChart->addValue("front",lTs,pMap.value("frontdist").toDouble());

		mBottomChart->addValue("wheel angle",lTs,pMap.value("wheelangle").toDouble());

		mBottomChart->updateAxisRanges();


		mGyroChart->addValue("ωZ",lTs,pMap.value("wz").toDouble());
		mGyroChart->addValue("aX",lTs,pMap.value("ax").toDouble());
		mGyroChart->addValue("aY",lTs,pMap.value("ay").toDouble());

        mGyroChart->updateAxisRanges();
	}
}
