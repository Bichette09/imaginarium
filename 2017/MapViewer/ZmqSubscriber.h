#pragma once

#include <string>

#include <QThread>

class ZmqSubscriber : public QThread
{
	Q_OBJECT
public:
	explicit ZmqSubscriber(const std::string & pListenTo, const std::string & pMessageType);
	virtual ~ZmqSubscriber();

signals:
	void newMessage(const QVariantMap & pMsg);

protected:
	virtual void run();

	const std::string	mListenTo;
	const std::string	mMessageType;
	bool				mQuit;
};
