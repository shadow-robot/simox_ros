#pragma once

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/threads/SbMutex.h>

//-------------------------------------------------------------------------------

namespace sr_grasp_mesh_planner
{

class CoinViewer : public SoQtExaminerViewer
{
public:
  CoinViewer(QWidget *parent=NULL,
             const char *name=NULL,
             SbBool embed=TRUE,
             SoQtFullViewer::BuildFlag flag=BUILD_ALL,
             SoQtViewer::Type type=BROWSER);
  virtual ~CoinViewer();

  void lock();
  void unlock();

protected:
  virtual void actualRedraw(void);

  SbThreadMutex sbmutex_;
};

} // end of namespace sr_grasp_mesh_planner

//-------------------------------------------------------------------------------
