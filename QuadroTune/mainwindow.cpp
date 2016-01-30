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

    mDataIndex = 0;
    mPIDIndex = 0;
    mRxCount = 0;
    mTxCount = 0;
    mSetPoint = 0;
    mInit = 0;
    mDeinit = 0;

    mControlIndex = 0;

    /*
    mControlData[0] = 0;
    mControlData[1] = 0;
    mControlData[2] = 0;
    mControlData[3] = 0;
    */

    connect(mDataTimer, SIGNAL(timeout()), this, SLOT(updatePIDPlot()));

    ui->comboBox_PIDSelect->addItem(QStringLiteral("PID_X"));
    ui->comboBox_PIDSelect->addItem(QStringLiteral("PID_Y"));
    ui->comboBox_PIDSelect->addItem(QStringLiteral("PID_Z"));

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
    ui->PIDPlot->graph(0)->setAntialiasedFill(false);
    ui->PIDPlot->graph(0)->setVisible(true);

    ui->PIDPlot->addGraph(); // blue dot
    ui->PIDPlot->graph(1)->setPen(QPen(Qt::blue));
    ui->PIDPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->PIDPlot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->PIDPlot->graph(1)->setVisible(true);

    ui->PIDPlot->addGraph();// setValue
    ui->PIDPlot->graph(2)->setPen(QPen(Qt::red));
    ui->PIDPlot->graph(2)->setAntialiasedFill(false);
    ui->PIDPlot->graph(2)->setVisible(true);


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
    if((NULL != mSerial) && (mSerial->isOpen()))
    {

        mTxCount++;
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
        ui->statusBar->showMessage(tr("Rx=%1, Tx=%2").arg(mRxCount).arg(mTxCount));
    }
}

void MainWindow::updateRegMap(frame f)
{
    if((f.reg_addr < REG_COUNT) &&
       (f.reg_addr + f.reg_count - 1 < REG_COUNT) &&
       (f.reg_count > 0 ) &&
        (f.command == 'r'))
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
    mSerialRxData += mSerial->readAll();

    while(getFrame())
    {
        printFrame(mRxFrame, false);
        updateRegMap(mRxFrame);
        mRxCount++;
        ui->statusBar->showMessage(tr("Rx=%1, Tx=%2").arg(mRxCount).arg(mTxCount));

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
        //mPIDIndex = REG_PID_X_PH;
    }
    else if(ui->radioButton_Y_Axis->isChecked())
    {
        mDataIndex = REG_GYRO_YH;
        //mPIDIndex = REG_PID_Y_PH;
    }
    else if(ui->radioButton_Z_Axis->isChecked())
    {
        mDataIndex = REG_GYRO_ZH;
        //mPIDIndex = REG_PID_Z_PH;
    }
    else if(ui->radioButton_X_PID->isChecked())
    {
        mDataIndex = REG_PID_XH;
        //mPIDIndex = REG_PID_X_PH;
    }
    else if(ui->radioButton_Y_PID->isChecked())
    {
        mDataIndex = REG_PID_YH;
        //mPIDIndex = REG_PID_Y_PH;
    }
    else if(ui->radioButton_Z_PID->isChecked())
    {
        mDataIndex = REG_PID_ZH;
        //mPIDIndex = REG_PID_Z_PH;
    }
    else
    {
        mDataIndex = 0;
        mPIDIndex = 0;
    }


    if(ui->pushButton_PIDTest->text() == "Start")
    {
        mInit = 0;
        QTimer::singleShot(100, this, SLOT(initTimer()));
    }
    else
    {
        mDeinit = 0;
        QTimer::singleShot(100, this, SLOT(deinitTimer()));
        if(mDataTimer != NULL)
        {
            mDataTimer->stop();
        }
    }
}

void MainWindow::initTimer()
{
    frame f;
    if(mInit == 0)
    {
        //Enable log
        f.command = 'w';
        f.reg_addr = REG_LOG_ENABLE;
        f.reg_count = 1;
        f.reg_values[0] = 1;
        f.length = FRAME_NO_PAYLOAD_LENGTH + f.reg_count;
        printFrame(f,true);
        sendFrame(f);
        mInit = 1;
        QTimer::singleShot(100, this, SLOT(initTimer()));
    }
    else if(mInit == 1)
    {
        //Chagne State to flying mode
        f.command = 'w';
        f.reg_addr = REG_STATE;
        f.reg_count = 1;
        f.reg_values[0] = 1;//flying mode
        f.length = FRAME_NO_PAYLOAD_LENGTH + f.reg_count;
        printFrame(f,true);
        sendFrame(f);
        mInit = 2;
        QTimer::singleShot(100, this, SLOT(initTimer()));
    }
    else if(mInit == 2)
    {
        //Turn motor ON
        f.command = 'w';
        f.reg_addr = REG_MOTOR_ENABLE;
        f.reg_count = 1;
        f.reg_values[0] = 0x0F;
        f.length = FRAME_NO_PAYLOAD_LENGTH + f.reg_count;
        printFrame(f,true);
        sendFrame(f);
        mInit = 3;

        ui->pushButton_PIDTest->setText("Stop");
        if(mDataTimer != NULL)
        {
               mDataTimer->start(25);
        }
    }

}

void MainWindow::deinitTimer()
{
    frame f;
    if(mDeinit == 0)
    {
        //Disable log
        f.command = 'w';
        f.reg_addr = REG_LOG_ENABLE;
        f.reg_count = 1;
        f.reg_values[0] = 0;
        f.length = FRAME_NO_PAYLOAD_LENGTH + f.reg_count;
        printFrame(f,true);
        sendFrame(f);
        mDeinit = 1;
        QTimer::singleShot(100, this, SLOT(deinitTimer()));
    }
    else if(mDeinit == 1)
    {
        //Chagne State to idle mode
        f.command = 'w';
        f.reg_addr = REG_STATE;
        f.reg_count = 1;
        f.reg_values[0] = 0;//flying mode
        f.length = FRAME_NO_PAYLOAD_LENGTH + f.reg_count;
        printFrame(f,true);
        sendFrame(f);
        mDeinit = 2;
        QTimer::singleShot(100, this, SLOT(deinitTimer()));
    }
    else
    {
        //Turn motor FF
        f.command = 'w';
        f.reg_addr = REG_MOTOR_ENABLE;
        f.reg_count = 1;
        f.reg_values[0] = 0x00;
        f.length = FRAME_NO_PAYLOAD_LENGTH + f.reg_count;
        printFrame(f,true);
        sendFrame(f);
        mDeinit = 3;
        ui->pushButton_PIDTest->setText("Start");
    }
}


void MainWindow::updatePIDPlot()
{
    static int toggle = 0;
    static int timer_count = 0;
    frame f;

    if(toggle)
    {
        toggle = 0;

        int16_t rawValue = 0;
        int16_t temp = static_cast<int16_t>(mRegArray[mDataIndex]);
        temp = (temp & 0x00FF);
        rawValue |= temp;

        rawValue = (rawValue << 8);
        temp = static_cast<int16_t>(mRegArray[mDataIndex + 1]);
        temp = (temp & 0x00FF);
        rawValue |= temp;

        //double value0 = static_cast<double>(rawValue);
        double key = QDateTime::currentDateTime().toMSecsSinceEpoch()/1000.0;

        ui->PIDPlot->graph(0)->addData(key,rawValue);
        ui->PIDPlot->graph(2)->addData(key,mSetPoint);


        ui->PIDPlot->graph(1)->clearData();
        ui->PIDPlot->graph(1)->addData(key,rawValue);

        ui->PIDPlot->graph(0)->removeDataBefore(key-8);
        ui->PIDPlot->graph(2)->removeDataBefore(key-8);
        ui->PIDPlot->xAxis->setRange(key+0.25, 8, Qt::AlignRight);
        ui->PIDPlot->replot();

        timer_count++;

        //ui->statusBar->showMessage(tr("Timeout count=%1, gyro=%2").arg(QString::number(timer_count)).arg(value0));


        f.command = 'r';
        f.reg_addr = mDataIndex;
        f.reg_count = 2;
        f.length = FRAME_NO_PAYLOAD_LENGTH;
        printFrame(f,true);
        sendFrame(f);
    }
    else
    {
        toggle = 1;

        f.command = 'w';
        f.reg_addr = REG_THRUST;
        f.reg_count = 4;
        f.length = FRAME_NO_PAYLOAD_LENGTH + f.reg_count;

        f.reg_values[0] = mRegArray[REG_THRUST];
        f.reg_values[1] = mRegArray[REG_PITCH];
        f.reg_values[2] = mRegArray[REG_ROLL];
        f.reg_values[3] = mRegArray[REG_YAW];


        printFrame(f,true);
        sendFrame(f);
    }




}




void MainWindow::on_pushButton_PIDTest_2_clicked()
{
    frame f;
    f.command = 'w';
    f.reg_addr = mPIDIndex;
    f.reg_count = 7;
    f.length = f.reg_count + FRAME_NO_PAYLOAD_LENGTH;

    int value,valueH,valueL;

    value = ui->textEdit_PID_PValue->toPlainText().toInt();
    valueH = (value & 0x0000FF00);
    valueL = (value & 0x000000FF);
    valueH = (valueH >> 8);

    f.reg_values[0] = valueH;
    f.reg_values[1] = valueL;

    value = ui->textEdit_PID_IValue->toPlainText().toInt();
    valueH = (value & 0x0000FF00);
    valueL = (value & 0x000000FF);
    valueH = (valueH >> 8);

    f.reg_values[2] = valueH;
    f.reg_values[3] = valueL;

    value = ui->textEdit_PID_DValue->toPlainText().toInt();
    valueH = (value & 0x0000FF00);
    valueL = (value & 0x000000FF);
    valueH = (valueH >> 8);

    f.reg_values[4] = valueH;
    f.reg_values[5] = valueL;

    f.reg_values[6] = 1;//reinit PID

    if((NULL != mSerial) && mSerial->isOpen())
    {
        printFrame(f,true);
        sendFrame(f);
    }

}

void MainWindow::on_comboBox_PIDSelect_currentIndexChanged(const QString &arg1)
{

    mRegArray[REG_PITCH] = 0;
    mRegArray[REG_ROLL] = 0;
    mRegArray[REG_YAW] = 0;

    if(arg1 == "PID_X")
    {
        mPIDIndex = REG_PID_X_PH;
        mControlIndex = REG_PITCH;
    }
    else if(arg1 == "PID_Y")
    {
        mPIDIndex = REG_PID_Y_PH;
        mControlIndex = REG_ROLL;
    }
    else if(arg1 == "PID_Z")
    {
        mPIDIndex = REG_PID_Z_PH;
        mControlIndex = REG_YAW;
    }

    mRegArray[REG_THRUST] = ui->verticalSlider_Thrust->value();
    mRegArray[mControlIndex] = ui->verticalSlider_Setpoint->value();


    frame f;
    f.command = 'r';
    f.reg_addr = mPIDIndex;
    f.reg_count = 6;
    f.length = FRAME_NO_PAYLOAD_LENGTH;
    if((NULL != mSerial) && mSerial->isOpen())
    {
        printFrame(f,true);
        sendFrame(f);
    }
    QTimer::singleShot(500, this, SLOT(updatePIDValues()));
}

void MainWindow::updatePIDValues()
{
    u_int16_t rawValue = 0;
    u_int16_t temp = static_cast<u_int16_t>(mRegArray[mPIDIndex]);
    temp = (temp & 0x00FF);
    rawValue |= temp;


    rawValue = (rawValue << 8);
    temp = static_cast<u_int16_t>(mRegArray[mPIDIndex + 1]);
    temp = (temp & 0x00FF);
    rawValue |= temp;
    ui->textEdit_PID_PValue->setText(QString::number(static_cast<int>(rawValue)));

    rawValue = 0;
    temp = static_cast<u_int16_t>(mRegArray[mPIDIndex + 2]);
    temp = (temp & 0x00FF);
    rawValue |= temp;


    rawValue = (rawValue << 8);
    temp = static_cast<u_int16_t>(mRegArray[mPIDIndex + 3]);
    temp = (temp & 0x00FF);
    rawValue |= temp;
    ui->textEdit_PID_IValue->setText(QString::number(static_cast<int>(rawValue)));

    rawValue = 0;
    temp = static_cast<u_int16_t>(mRegArray[mPIDIndex + 4]);
    temp = (temp & 0x00FF);
    rawValue |= temp;


    rawValue = (rawValue << 8);
    temp = static_cast<u_int16_t>(mRegArray[mPIDIndex + 5]);
    temp = (temp & 0x00FF);
    rawValue |= temp;
    ui->textEdit_PID_DValue->setText(QString::number(static_cast<int>(rawValue)));
}

#define GYRO_MAX_RANGE 250L
#define MAX_INT16 32767L
#define MIN_INT16 -32768L

#define PITCH_MAX_RANGE 30L //max pitch rotation speed in deg/sec
#define ROLL_MAX_RANGE 30L //max roll rotation speed in deg/sec
#define YAW_MAX_RANGE 30L //max yaw rotation speed in deg/sec

#define THRUST_MAX_VALUE MAX_INT16
#define THRUST_MIN_VALUE 0

#define PITCH_MAX_VALUE ((PITCH_MAX_RANGE * MAX_INT16)/GYRO_MAX_RANGE)
#define PITCH_MIN_VALUE (-PITCH_MAX_VALUE)

#define ROLL_MAX_VALUE ((ROLL_MAX_RANGE * MAX_INT16)/GYRO_MAX_RANGE)
#define ROLL_MIN_VALUE (-ROLL_MAX_VALUE)

#define YAW_MAX_VALUE ((YAW_MAX_RANGE * MAX_INT16)/GYRO_MAX_RANGE)
#define YAW_MIN_VALUE (-YAW_MAX_VALUE)

int16_t map(int16_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    return (int16_t)(((int32_t)x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void MainWindow::on_verticalSlider_Setpoint_sliderMoved(int position)
{
    ui->label_SetpointValue->setText(QString::number(position));
}

void MainWindow::on_verticalSlider_Thrust_sliderMoved(int position)
{
    ui->label_ThrustValue->setText(QString::number(position));
}

void MainWindow::on_pushButton_setControl_clicked()
{
    mRegArray[REG_THRUST] = ui->verticalSlider_Thrust->value();
    mRegArray[mControlIndex] = ui->verticalSlider_Setpoint->value();
    mSetPoint =  map(ui->verticalSlider_Setpoint->value(),-127,127,PITCH_MIN_VALUE,PITCH_MAX_VALUE);
}
