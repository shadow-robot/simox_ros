 #pragma once

 #include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
 #include <Inventor/threads/SbMutex.h>
 #include <boost/thread.hpp>

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

     SbThreadMutex mutex;
     //mutable boost::recursive_mutex mutex;
     //boost::unique_lock<boost::recursive_mutex> _lock;
 };
