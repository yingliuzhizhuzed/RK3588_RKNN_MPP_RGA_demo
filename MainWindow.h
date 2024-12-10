#ifndef __MAINWINDOW_H
#define __MAINWINDOW_H
#include <QMainWindow>
#include <QLabel>
#include <QVBoxLayout>
#include <QImage>
#include <QPixmap>
#include <QAction>
#include <QMenuBar>
#include <QFileDialog>
#include "YOLOV5s.h"
class MainWindow:public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();
public slots:
    void UpdateImage(QImage image);
private:
    QLabel* lab_image;
    QWidget* central_w;

    YOLOV5s* yolov5s;
};

#endif