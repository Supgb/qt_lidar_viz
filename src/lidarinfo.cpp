#include "lidarinfo.h"
#include <QAbstractItemView>
#include <QGridLayout>
#include <QListView>
#include <QScrollBar>

namespace lidar_base
{

LidarInfo::LidarInfo(_ld_assembler *assem, _ld_driver_t* driver,
                     _ld_decoder_t* decoder, _ld_model_t* model):
    LidarCtrlPanel(assem, driver, decoder, model)
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

}   // namespace lidar_base
