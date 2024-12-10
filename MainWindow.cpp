#include "MainWindow.h"
bool Running = true;
MainWindow::MainWindow(QWidget* parent):QMainWindow(parent)
{
    lab_image = new QLabel(this);
    central_w = new QWidget(this);
    this->setCentralWidget(central_w);
    QVBoxLayout* layout = new QVBoxLayout();
    central_w->setLayout(layout);
    layout->addWidget(lab_image);
    QMenuBar* bar = this->menuBar();
    QAction* act = bar->addAction("open");

    
    yolov5s = new YOLOV5s(this);
    connect(act,&QAction::triggered,this,[=](){
        QString source = QFileDialog::getOpenFileName(this,"open file","..","*");
        if(!source.isNull())
        {
            yolov5s->mpp_dec->source_file = source;
            connect(yolov5s,&YOLOV5s::UpdateImage_signal,this,&MainWindow::UpdateImage);
            
            yolov5s->start();
        }
    });
    this->resize(660,660);
}
MainWindow::~MainWindow()
{   
    Running = false;
    yolov5s->quit();
    yolov5s->wait();
}

void MainWindow::UpdateImage(QImage image)
{
    lab_image->setPixmap(QPixmap::fromImage(image));    
}