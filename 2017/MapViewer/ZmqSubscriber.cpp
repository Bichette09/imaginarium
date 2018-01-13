#include "ZmqSubscriber.h"

#include <QDebug>
#include <QJsonDocument>


#include <zmq.h>


ZmqSubscriber::ZmqSubscriber(const std::string & pListenTo, const std::string & pMsgType)
	: QThread(NULL)
	, mMessageType(pMsgType)
	, mListenTo(pListenTo)
	, mQuit(false)
{

	start();
}

ZmqSubscriber::~ZmqSubscriber()
{
	mQuit = true;
	wait();
}

void ZmqSubscriber::run()
{
	void * lZmqCtx = zmq_ctx_new ();

	//  Create a subscriber
	void * lZmqSocket = zmq_socket (lZmqCtx, ZMQ_SUB);
	{
		if( zmq_connect (lZmqSocket, mListenTo.c_str()) != 0)
		{
			qCritical()<<"Fail to open socket "<<errno;
			lZmqSocket = NULL;
			goto EndRun;
		}

		//  Subscribe for all messages.
		if( zmq_setsockopt (lZmqSocket, ZMQ_SUBSCRIBE, mMessageType.c_str(), 0) != 0)
		{
			qCritical()<<"Fail to subscribe to "<<QString::fromUtf8(mMessageType.c_str());
			goto EndRun;
		}

		std::vector<char> lBuffer;
		lBuffer.resize(1024*1024);

		while(!mQuit)
		{
			int lMsgSize = zmq_recv (lZmqSocket,lBuffer.data(),lBuffer.size(), ZMQ_DONTWAIT);
			if( lMsgSize > (int)mMessageType.size() )
			{
				// got a message
				QByteArray lJsonBuffer = QByteArray(lBuffer.data() + mMessageType.size(),lMsgSize - (int)mMessageType.size());


				QVariantMap lJsonObject = QJsonDocument::fromJson(lJsonBuffer).toVariant().toMap();
//				static bool sOnce = true;
//				if(sOnce)
//				{
//					qInfo()<<QString::fromUtf8(lJsonBuffer);
//					qInfo()<<lJsonObject;
//				}
//				sOnce = false;
				emit newMessage(lJsonObject);
			}
			else
			{
				msleep(10);
			}
		}
	}
	EndRun:

	if(lZmqSocket)
	{
		zmq_close (lZmqSocket);
		lZmqSocket = NULL;
	}

	zmq_ctx_term (lZmqCtx);
}
