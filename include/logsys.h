#ifndef LOGSYS_H
#define LOGSYS_H

#include <QStringListModel>

namespace lidar_log {

class LogSys
{
public:
    enum LogLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    LogSys(){}
    virtual ~LogSys() = 0;

    /*
    *** Usage: you need to declare a Q_SIGNAL in the derived class,
    *** and emit your signal with Q_EMIT after call log().
    */
    virtual void log_pipe(const LogLevel&, const QString&) = 0;

    void log(const LogLevel&, const QString&);
    QStringListModel* loggingModel()
        { return &logging_model; }


protected:
    QStringListModel logging_model;

};

} // namespace lidar_log

#endif // LOGSYS_H
