/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   coin_viewer.hpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Use mutex to protect SoQtExaminerViewer::actualRedraw.
 **/

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
