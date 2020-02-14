#include "lidarinfo.h"
#include <QAbstractItemView>

using namespace lidar_ctrl;

LidarInfo::LidarInfo(_ld_assembler *assem):
    LidarCtrlPanel(assem)
{
    ui_constructor();
}

void LidarInfo::ui_constructor()
{
    grid_layout = new QGridLayout();
    log_view = new QListView();
    log_view->setEditTriggers(QAbstractItemView::NoEditTriggers);
    log_view->setModel(_assembler->loggingModel());
    grid_layout->addWidget(log_view, 0, 0);

    setLayout(grid_layout);

    QObject::connect(_assembler, SIGNAL(UPDATE_LOG()), this,
                     SLOT(reloadLogInfo()),
                     Qt::QueuedConnection);

}

void LidarInfo::reloadLogInfo()
{
    log_view->scrollToBottom();
}
