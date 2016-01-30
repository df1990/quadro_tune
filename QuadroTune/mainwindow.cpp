#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtSerialPort/QSerialPortInfo>
#include <QIntValidator>
#include <QLineEdit>
#include <QMessageBox>
#include <QScrollBar>

static const char blankString[] = QT_TRANSLATE_NOOP("SettingsDialog", "N/A");


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    mSerial = new QSerialPort(this);
    connect(ui->serialPortInfoListBox, SIGNAL(currentIndexChanged(int)),
            this, SLOT(showPortInfo(int)));
    connect(mSerial, SIGNAL(readyRead()), this, SLOT(readSerialData()));

    mDataTimer = new QTimer(this);

    int mDataIndex = 0;

    connect(mDataTimer, SIGNAL(timeout()), this, SLOT(updatePIDData()));

    fillPortsParameters();
    fillPortsInfo();
    updateSettings();
    regArrayInit();
    initPlot();
}

MainWindow::~MainWindow()
{
    if (mSerial->isOpen())
    {
        mSerial->close();
    }
    delete mSerial;
    //delete dataTimer;

    delete ui;
}

void MainWindow::initPlot(void)
{
    ui->PIDPlot->addGraph();// blue line
    ui->PIDPlot->graph(0)->setPen(QPen(Qt::blue));
    //ui->PIDPlot->graph(0)->setBrush(QBrush(QColor(240, 255, 200)));
    ui->PIDPlot->graph(0)->setAntialiasedFill(false);
    ui->PIDPlot->graph(0)->setVisible(true);

    ui->PIDPlot->addGraph(); // blue dot
    ui->PIDPlot->graph(1)->setPen(QPen(Qt::blue));
    ui->PIDPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->PIDPlot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->PIDPlot->graph(1)->setVisible(true);

    /*ui->PIDPlot->addGraph();// red line
    ui->PIDPlot->graph(2)->setPen(QPen(Qt::red));
    //ui->PIDPlot->graph(2)->setBrush(QBrush(QColor(240, 255, 200)));
    //ui->PIDPlot->graph(2)->setAntialiasedFill(false);
    ui->PIDPlot->graph(2)->setVisible(false);

    ui->PIDPlot->addGraph(); // red dot
    ui->PIDPlot->graph(3)->setPen(QPen(Qt::red));
    ui->PIDPlot->graph(3)->setLineStyle(QCPGraph::lsNone);
    ui->PIDPlot->graph(3)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->PIDPlot->graph(3)->setVisible(false);*/

    //ui->PIDPlot->xAxis->setTickLabelType(QCPAxis::ltNumber);
    ui->PIDPlot->xAxis->setTickLabelType(QCPAxis::ltDateTime);
    ui->PIDPlot->xAxis->setDateTimeFormat("hh:mm:ss");
    ui->PIDPlot->xAxis->setAutoTickStep(false);
    ui->PIDPlot->xAxis->setTickStep(1);
    ui->PIDPlot->axisRect()->setupFullAxesBox();
    ui->PIDPlot->yAxis->setRange(-34000,34000);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->PIDPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->PIDPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->PIDPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->PIDPlot->yAxis2, SLOT(setRange(QCPRange)));
    //ui->PIDPlot->repaint();
}

void MainWindow::regArrayInit()
{
    memset(mRegArray,0,__REG_END__ * sizeof(int));
}

void MainWindow::showPortInfo(int idx)
{
    if (idx == -1)
        return;

    QStringList list = ui->serialPortInfoListBox->itemData(idx).toStringList();
    ui->descriptionLabel->setText(tr("Description: %1").arg(list.count() > 1 ? list.at(1) : tr(blankString)));
    ui->manufacturerLabel->setText(tr("Manufacturer: %1").arg(list.count() > 2 ? list.at(2) : tr(blankString)));
    ui->serialNumberLabel->setText(tr("Serial number: %1").arg(list.count() > 3 ? list.at(3) : tr(blankString)));
    ui->locationLabel->setText(tr("Location: %1").arg(list.count() > 4 ? list.at(4) : tr(blankString)));
    ui->vidLabel->setText(tr("Vendor Identifier: %1").arg(list.count() > 5 ? list.at(5) : tr(blankString)));
    ui->pidLabel->setText(tr("Product Identifier: %1").arg(list.count() > 6 ? list.at(6) : tr(blankString)));
}

void MainWindow::fillPortsParameters()
{
    ui->baudRateBox->addItem(QStringLiteral("9600"), QSerialPort::Baud9600);
    ui->baudRateBox->addItem(QStringLiteral("19200"), QSerialPort::Baud19200);
    ui->baudRateBox->addItem(QStringLiteral("38400"), QSerialPort::Baud38400);
    ui->baudRateBox->addItem(QStringLiteral("57600"), QSerialPort::Baud57600);
    ui->baudRateBox->addItem(QStringLiteral("115200"), QSerialPort::Baud115200);
    ui->baudRateBox->addItem(tr("Custom"));

    ui->dataBitsBox->addItem(QStringLiteral("5"), QSerialPort::Data5);
    ui->dataBitsBox->addItem(QStringLiteral("6"), QSerialPort::Data6);
    ui->dataBitsBox->addItem(QStringLiteral("7"), QSerialPort::Data7);
    ui->dataBitsBox->addItem(QStringLiteral("8"), QSerialPort::Data8);
    ui->dataBitsBox->setCurrentIndex(3);

    ui->parityBox->addItem(tr("None"), QSerialPort::NoParity);
    ui->parityBox->addItem(tr("Even"), QSerialPort::EvenParity);
    ui->parityBox->addItem(tr("Odd"), QSerialPort::OddParity);
    ui->parityBox->addItem(tr("Mark"), QSerialPort::MarkParity);
    ui->parityBox->addItem(tr("Space"), QSerialPort::SpaceParity);

    ui->stopBitsBox->addItem(QStringLiteral("1"), QSerialPort::OneStop);
#ifdef Q_OS_WIN
    ui->stopBitsBox->addItem(tr("1.5"), QSerialPort::OneAndHalfStop);
#endif
    ui->stopBitsBox->addItem(QStringLiteral("2"), QSerialPort::TwoStop);

    ui->flowControlBox->addItem(tr("None"), QSerialPort::NoFlowControl);
    ui->flowControlBox->addItem(tr("RTS/CTS"), QSerialPort::HardwareControl);
    ui->flowControlBox->addItem(tr("XON/XOFF"), QSerialPort::SoftwareControl);
}

void MainWindow::fillPortsInfo()
{
    ui->serialPortInfoListBox->clear();
    QString description;
    QString manufacturer;
    QString serialNumber;
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        QStringList list;
        description = info.description();
        manufacturer = info.manufacturer();
        serialNumber = info.serialNumber();
        list << info.portName()
             << (!description.isEmpty() ? description : blankString)
             << (!manufacturer.isEmpty() ? manufacturer : blankString)
             << (!serialNumber.isEmpty() ? serialNumber : blankString)
             << info.systemLocation()
             << (info.vendorIdentifier() ? QString::number(info.vendorIdentifier(), 16) : blankString)
             << (info.productIdentifier() ? QString::number(info.productIdentifier(), 16) : blankString);

        ui->serialPortInfoListBox->addItem(list.first(), list);
    }

    ui->serialPortInfoListBox->addItem(tr("Custom"));
}

void MainWindow::updateSettings()
{
    mCurrentSettings.name = ui->serialPortInfoListBox->currentText();

    if (ui->baudRateBox->currentIndex() == 4) {
        mCurrentSettings.baudRate = ui->baudRateBox->currentText().toInt();
    } else {
        mCurrentSettings.baudRate = static_cast<QSerialPort::BaudRate>(
                    ui->baudRateBox->itemData(ui->baudRateBox->currentIndex()).toInt());
    }
    mCurrentSettings.stringBaudRate = QString::number(mCurrentSettings.baudRate);

    mCurrentSettings.dataBits = static_cast<QSerialPort::DataBits>(
                ui->dataBitsBox->itemData(ui->dataBitsBox->currentIndex()).toInt());
    mCurrentSettings.stringDataBits = ui->dataBitsBox->currentText();

    mCurrentSettings.parity = static_cast<QSerialPort::Parity>(
                ui->parityBox->itemData(ui->parityBox->currentIndex()).toInt());
    mCurrentSettings.stringParity = ui->parityBox->currentText();

    mCurrentSettings.stopBits = static_cast<QSerialPort::StopBits>(
                ui->stopBitsBox->itemData(ui->stopBitsBox->currentIndex()).toInt());
    mCurrentSettings.stringStopBits = ui->stopBitsBox->currentText();

    mCurrentSettings.flowControl = static_cast<QSerialPort::FlowControl>(
                ui->flowControlBox->itemData(ui->flowControlBox->currentIndex()).toInt());
    mCurrentSettings.stringFlowControl = ui->flowControlBox->currentText();
}

void MainWindow::on_connectButton_clicked()
{
    if(ui->connectButton->text() == "Connect")
    {
        updateSettings();
        mSerial->setPortName(mCurrentSettings.name);
        mSerial->setBaudRate(mCurrentSettings.baudRate);
        mSerial->setDataBits(mCurrentSettings.dataBits);
        mSerial->setParity(mCurrentSettings.parity);
        mSerial->setStopBits(mCurrentSettings.stopBits);
        mSerial->setFlowControl(mCurrentSettings.flowControl);
        if (mSerial->open(QIODevice::ReadWrite))
        {
                //ui->actionConnect->setEnabled(false);
                //ui->actionDisconnect->setEnabled(true);
                //ui->actionConfigure->setEnabled(false);

                //ui->pushButton_CMD1->setEnabled(true);
                //ui->pushButton_CMD2->setEnabled(true);
                ui->connectButton->setText("Disconnect");
                ui->statusBar->showMessage(tr("Connected to %1 : %2, %3, %4, %5, %6")
                                           .arg(mCurrentSettings.name).arg(mCurrentSettings.stringBaudRate).arg(mCurrentSettings.stringDataBits)
                                           .arg(mCurrentSettings.stringParity).arg(mCurrentSettings.stringStopBits).arg(mCurrentSettings.stringFlowControl));
        } else
        {
            QMessageBox::critical(this, tr("Error"), mSerial->errorString());

            ui->statusBar->showMessage(tr("Open error"));
        }

    }
    else
    {
        if (mSerial->isOpen())
        {
            mSerial->close();
            ui->statusBar->showMessage(tr("Disconnected"));
            ui->connectButton->setText("Connect");
            //ui->pushButton_CMD1->setEnabled(false);
            //ui->pushButton_CMD1->setEnabled(false);
        }
    }
}

#define FRAME_NO_PAYLOAD_LENGTH 7
#define FRAME_HEADER_LENGTH 6
#define HEADER_START_CODE '$'
#define HEADER_END_CODE '#'
#define FRAME_END_CODE '@'

#define FRAME_START         0
#define FRAME_LENGTH        1
#define FRAME_CMD           2
#define FRAME_REG_ADDR      3
#define FRAME_REG_COUNT     4
#define FRAME_HEADER_END    5
#define FRAME_REG_VALUES    6

int MainWindow::getFrame()
{
    static bool header_found = false;
    int ret_val = 0;

    while(!header_found && (mSerialRxData.size() >= FRAME_HEADER_LENGTH))
    {

        if((mSerialRxData.at(FRAME_START) == HEADER_START_CODE) &&
           (mSerialRxData.at(FRAME_HEADER_END) == HEADER_END_CODE))
           {
                header_found = 1;
                break;
           }
           else
           {
                mSerialRxData.remove(0,1);
           }
    }

    if(header_found)
    {
        int tmp_length;
        tmp_length = mSerialRxData.at(FRAME_LENGTH);

        if(mSerialRxData.size() >= tmp_length)
        {
            if(mSerialRxData.at(tmp_length-1) == FRAME_END_CODE)
            {
                mRxFrame.length = mSerialRxData.at(FRAME_LENGTH);
                mRxFrame.command = mSerialRxData.at(FRAME_CMD);
                mRxFrame.reg_addr = mSerialRxData.at(FRAME_REG_ADDR);
                mRxFrame.reg_count = mSerialRxData.at(FRAME_REG_COUNT);

                for(int index = 0; index < (tmp_length - FRAME_NO_PAYLOAD_LENGTH); index++)
                {
                    mRxFrame.reg_values[index] = mSerialRxData.at(FRAME_REG_VALUES+index);
                }

                mSerialRxData.remove(0,tmp_length);
                ret_val = 1;
            }
            else
            {
                mSerialRxData.remove(0,FRAME_HEADER_LENGTH);
            }
            header_found = 0;
        }
    }
    return ret_val;
}

void MainWindow::sendFrame(frame f)
{
    static int txCount = 0;
    if((NULL != mSerial) && (mSerial->isOpen()))
    {

        txCount++;
        QByteArray txFrame;
        //Setup Header
        txFrame.append(HEADER_START_CODE);
        txFrame.append(f.length);//length placeholder
        txFrame.append(f.command);

        txFrame.append(f.reg_addr);
        txFrame.append(f.reg_count);
        txFrame.append(HEADER_END_CODE);
        if(f.command == 'w')
        {
            for(int index = 0; index < f.reg_count; index++)
            {
                txFrame.append(f.reg_values[index]);
            }
        }

        txFrame.append(FRAME_END_CODE);
        mSerial->write(txFrame);
        //ui->statusBar->showMessage(tr("TX count=%1").arg(QString::number(txCount)));
    }
}

void MainWindow::updateRegMap(frame f)
{
    if((f.reg_addr < REG_COUNT) &&
       (f.reg_addr + f.reg_count - 1 < REG_COUNT) &&
       (f.reg_count > 0 ))
    {
        for(int index = 0; index < f.reg_count; index++)
        {
            mRegArray[f.reg_addr + index] = f.reg_values[index];
        }
    }
}

void MainWindow::printFrame(frame f, bool send)
{

    QString frame_text;
    if(send)
    {
        frame_text += "<-- ";
    }
    else
    {
        frame_text += "--> ";
    }
    frame_text += "CMD= ";
    frame_text += (char)(f.command);
    frame_text += "\t";
    frame_text += "L=";
    frame_text += QString::number(f.length);
    frame_text += "\t";
    frame_text += "A=";
    frame_text += QString::number(f.reg_addr);
    frame_text += "\t";
    frame_text += "C=";
    frame_text += QString::number(f.reg_count);
    frame_text += "\t";
    //frame_text += " ";
    if(send)
    {
        if(f.command == 'w')
        {
            for(int index = 0; index < f.reg_count; index++)
            {

                frame_text += QString("[%1:%2]").arg(QString::number(f.reg_addr+index)).arg(QString::number(f.reg_values[index]));
            }
        }

    }
    else
    {
        if(f.command == 'r')
        {
            for(int index = 0; index < f.reg_count; index++)
            {

                frame_text += QString("[%1:%2]").arg(QString::number(f.reg_addr+index)).arg(QString::number(f.reg_values[index]));
            }
        }
    }

    frame_text += "\n";
    if(send)
    {
        ui->plainTextEdit_log->insertPlainText("-----------------------------------------------------------------------------\n");
    }

    ui->plainTextEdit_log->insertPlainText(frame_text);
    QScrollBar *bar = ui->plainTextEdit_log->verticalScrollBar();
    bar->setValue(bar->maximum());
}

void MainWindow::readSerialData()
{
    static int rxCount = 0;
    rxCount++;
    mSerialRxData += mSerial->readAll();

    while(getFrame())
    {
        printFrame(mRxFrame, false);
        updateRegMap(mRxFrame);
        rxCount++;


    }
}

void MainWindow::on_pushButton_clicked()
{
    frame tx_frame;
    tx_frame.command = ui->plainTextEdit_CMD->toPlainText().at(0).toLatin1();
    tx_frame.reg_addr = ui->plainTextEdit_ADDR->toPlainText().toInt();
    tx_frame.reg_count = ui->plainTextEdit_CNT->toPlainText().toInt();
    if(tx_frame.command == 'w')
    {


        QStringList argumentsList = ui->plainTextEdit_VALUES->toPlainText().split(",",QString::SkipEmptyParts);
        if(argumentsList.size() < tx_frame.reg_count)
        {
            tx_frame.reg_count = argumentsList.size();
        }
        tx_frame.length = FRAME_NO_PAYLOAD_LENGTH + tx_frame.reg_count;
        for(int index = 0; index < tx_frame.reg_count; index++)
        {
            tx_frame.reg_values[index] = argumentsList.at(index).toInt();
        }
    }
    else
    {
        tx_frame.length = FRAME_NO_PAYLOAD_LENGTH;
    }
    printFrame(tx_frame,true);
    sendFrame(tx_frame);
}

void MainWindow::on_pushButton_PIDTest_clicked()
{
    if(ui->radioButton_X_Axis->isChecked())
    {
        mDataIndex = REG_GYRO_XH;

    }
    else if(ui->radioButton_Y_Axis->isChecked())
    {
        mDataIndex = REG_GYRO_YH;

    }
    else if(ui->radioButton_Z_Axis->isChecked())
    {
        mDataIndex = REG_GYRO_ZH;

    }
    else if(ui->radioButton_X_PID->isChecked())
    {
        mDataIndex = REG_PID_XH;

    }
    else if(ui->radioButton_Y_PID->isChecked())
    {
        mDataIndex = REG_PID_YH;

    }
    else if(ui->radioButton_Z_PID->isChecked())
    {
        mDataIndex = REG_PID_ZH;

    }
    else
    {
        mDataIndex = 0;
    }

    if(ui->pushButton_PIDTest->text() == "Start")
    {
           ui->pushButton_PIDTest->setText("Stop");
           if(mDataTimer != NULL)
           {
               mDataTimer->start(50);
           }
    }
    else
    {
        ui->pushButton_PIDTest->setText("Start");
        if(mDataTimer != NULL)
        {
            mDataTimer->stop();
        }
    }
}

void MainWindow::updatePIDData()
{
    static int timer_count = 0;


    int16_t rawValue = 0;
    int16_t temp = static_cast<int16_t>(mRegArray[mDataIndex]);
    temp = (temp & 0x00FF);
    rawValue |= temp;

    rawValue = (rawValue << 8);
    temp = static_cast<int16_t>(mRegArray[mDataIndex + 1]);
    temp = (temp & 0x00FF);
    rawValue |= temp;

    double value0 = static_cast<double>(rawValue);
    double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

    ui->PIDPlot->graph(0)->addData(key,rawValue);

    ui->PIDPlot->graph(1)->clearData();
    ui->PIDPlot->graph(1)->addData(key,rawValue);

    ui->PIDPlot->graph(0)->removeDataBefore(key-8);
    ui->PIDPlot->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
    ui->PIDPlot->replot();

    timer_count++;

    ui->statusBar->showMessage(tr("Timeout count=%1, gyro=%2").arg(QString::number(timer_count)).arg(value0));

    frame f;
    f.command = 'r';

    f.reg_addr = mDataIndex;
    f.reg_count = 2;
    f.length = FRAME_NO_PAYLOAD_LENGTH;
    printFrame(f,true);
    sendFrame(f);
}
