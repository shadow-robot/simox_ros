#include "sr_grasp_mesh_planner/coin_viewer.hpp"

//-------------------------------------------------------------------------------

CoinViewer::CoinViewer(QWidget *parent,
                       const char *name,
                       SbBool embed,
                       SoQtFullViewer::BuildFlag flag,
                       SoQtViewer::Type type)
: SoQtExaminerViewer(parent, name, embed, flag, type)
{
}

//-------------------------------------------------------------------------------

CoinViewer::~CoinViewer()
{
}

//-------------------------------------------------------------------------------

void CoinViewer::lock()
{
  sbmutex_.lock();
}

//-------------------------------------------------------------------------------

void CoinViewer::unlock()
{
  sbmutex_.unlock();
}

//-------------------------------------------------------------------------------

void CoinViewer::actualRedraw()
{
  sbmutex_.lock();

  SoQtExaminerViewer::actualRedraw();

  sbmutex_.unlock();
}

//-------------------------------------------------------------------------------
