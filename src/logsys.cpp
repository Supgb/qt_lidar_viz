#include "../include/logsys.h"

#include <QDateTime>
#include <QTextStream>

using namespace lidar_log;

LogSys::~LogSys()
{}

void LogSys::log(const LogLevel &level, const QString &msg)
{
    logging_model.insertRows(logging_model.rowCount(), 1);
    QString logging_msg;
    QTextStream logging_model_msg(&logging_msg);
    switch (level) {
        case (Debug) :
            logging_model_msg << "[DEBUG] [" << QDateTime::currentDateTime().toString("yyyy-MM-dd")
                              << "]: " << msg;
        break;

        case (Info) :
            logging_model_msg << "[INFO] [" << QDateTime::currentDateTime().toString("mm::ss::zz")
                              << "]: " << msg;
        break;

        case (Warn) :
            logging_model_msg << "[WARN] [" << QDateTime::currentDateTime().toString("mm::ss::zz")
                              << "]: " << msg;
        break;

        case (Error) :
            logging_model_msg << "[ERROR] [" << QDateTime::currentDateTime().toString("mm::ss::zz")
                              << "]: " << msg;
        break;

        case (Fatal) :
            logging_model_msg << "[FATAL] [" << QDateTime::currentDateTime().toString("mm::ss::zz")
                              << "]: " << msg;
        break;
    }

    logging_model.setData(logging_model.index(logging_model.rowCount()-1),
                          logging_msg);
}
