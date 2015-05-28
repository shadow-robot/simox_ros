/*
 * Copyright (c) 2013 Shadow Robot Company Ltd.
 *  All rights reserved.
 *
 * This code is proprietary and may not be used, copied, distributed without
 *  prior authorisation and agreement from Shadow Robot Company Ltd.
 */

/**
 * @file   coin_viewer.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Use mutex to protect SoQtExaminerViewer::actualRedraw.
 **/

#include "sr_grasp_mesh_planner/coin_viewer.hpp"

//-------------------------------------------------------------------------------

using namespace sr_grasp_mesh_planner;

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
