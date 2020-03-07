#include "../include/logsys.h"

#include <QDateTime>
#include <QTextStream>
#include <QMutex>
#include <QStringListModel>

namespace lidar_base
{

QMutex loggingModel_mutex;

void LogSys::log(const LogLevel &level, const QString &msg)
{
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
    QMutexLocker(&lidar_base::loggingModel_mutex);   // qualifier required.
    logging_model->insertRows(logging_model->rowCount(), 1);
    logging_model->setData(logging_model->index(logging_model->rowCount()-1),
                          logging_msg);
}

}   // namespace lidar_base
