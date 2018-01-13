#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QVariantMap>
#include <QTimer>
#include <QList>

class QGraphicsLineItem;
class QGraphicsEllipseItem;

class MyGraphView;

namespace Ui {
class MainWindow;
}

class ZmqSubscriber;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

	virtual bool eventFilter(QObject * pWatched, QEvent * pEvent);
public slots:
	void onNewData(const QVariantMap & pMap);
	void onNoDataTimeOut();
	void on_mTimePerdiodSpin_valueChanged(int);
    void on_mAddressLineEdit_returnPressed();
    void on_mPauseButton_pressed();
private:
	void initMapScene();
	void recenterMapView();
	void clearAll();

	QTimer				mNoDataTimer;

	typedef QList<QGraphicsLineItem*> tLineItems;
	typedef QList<QGraphicsEllipseItem*> tEllipseItems;
	typedef QList<QGraphicsTextItem*> tTextItems;

	MyGraphView *				mTopChart;
	MyGraphView *				mBottomChart;
    MyGraphView *               mGyroChart;
	double						mLastTimeStamp;
	double						mStartTimeStamp;

    ZmqSubscriber *     mSub;

	QGraphicsScene *	mMapScene;
	float				mMapScale;
	tLineItems			mSensorLines;
	tEllipseItems		mSensorPoints;
	bool				mAddSensorIds;

	Ui::MainWindow *	ui;
};

#endif // MAINWINDOW_H
