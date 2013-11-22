#include "sr_grasp_mesh_planner/coin_viewer.hpp"

CoinViewer::CoinViewer(QWidget *parent,
                       const char *name,
                       SbBool embed,
                       SoQtFullViewer::BuildFlag flag,
                       SoQtViewer::Type type)
: /*_lock(mutex, boost::defer_lock),*/ SoQtExaminerViewer(parent, name, embed, flag, type)
{
}

CoinViewer::~CoinViewer()
{
}

void CoinViewer::lock()
{
  //_lock.lock();
  mutex.lock();
}

void CoinViewer::unlock()
{
  //_lock.unlock();
  mutex.unlock();
}

void CoinViewer::actualRedraw()
{
  //_lock.lock();
  mutex.lock();
  SoQtExaminerViewer::actualRedraw();
  //_lock.unlock();
  mutex.unlock();
}
