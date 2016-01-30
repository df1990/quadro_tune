#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtCore/QtGlobal>

#include <QMainWindow>

#include <QtSerialPort/QSerialPort>
#include <QDialog>
#include <QByteArray>
#include <QPlainTextEdit>

#include "reg_map.h"

namespace Ui {
class MainWindow;
}

class Console;

class MainWindow : public QMainWindow
{
    Q_OBJECT

private slots:
    void showPortInfo(int idx);
    void readSerialData();

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    struct Settings {
        QString name;
        qint32 baudRate;
        QString stringBaudRate;
        QSerialPort::DataBits dataBits;
        QString stringDataBits;
        QSerialPort::Parity parity;
        QString stringParity;
        QSerialPort::StopBits stopBits;
        QString stringStopBits;
        QSerialPort::FlowControl flowControl;
        QString stringFlowControl;
        bool localEchoEnabled;
    };

    struct frame{
        int length;
        int command;
        int reg_addr;
        int reg_count;
        int reg_values[32];
    };


    Settings settings() const;

private slots:
    void on_connectButton_clicked();

    void on_pushButton_clicked();

    void updatePIDData();

    void on_pushButton_PIDTest_clicked();

private:
    Ui::MainWindow *ui;

    QTimer *mDataTimer;
    QSerialPort *mSerial;
    Settings mCurrentSettings;
    QByteArray mSerialRxData;
    frame mRxFrame;
    int mRegArray[REG_COUNT];
    int mDataIndex;


    void regArrayInit();
    int getFrame();
    void sendFrame(frame f);
    void printFrame(frame f, bool send);
    void fillPortsParameters();
    void fillPortsInfo();
    void updateSettings();
    void updateRegMap(frame f);
    void initPlot(void);
};

#endif // MAINWINDOW_H
