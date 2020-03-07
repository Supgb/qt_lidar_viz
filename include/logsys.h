#ifndef LOGSYS_H
#define LOGSYS_H

class QStringListModel;
class QString;

namespace lidar_base {

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

    LogSys(QStringListModel* lm): logging_model(lm) {}
    ~LogSys() = default;

    /*
    *** Usage: you need to declare a Q_SIGNAL in the derived class,
    *** and emit your signal with Q_EMIT after call log().
    *** A simple implementation of log_pipe looks like:
    *** """
    *** void log_pipe(const LogLevel& level, const QString& msg)
    *** { log(level, msg); Q_EMIT UPDATE_LOG(); }
    *** """
    */
    virtual void log_pipe(const LogLevel&, const QString&) = 0;

    void log(const LogLevel&, const QString&);
    QStringListModel* loggingModel()
        { return logging_model; }


protected:
    QStringListModel *logging_model;

};

} // namespace lidar_base

#endif // LOGSYS_H
