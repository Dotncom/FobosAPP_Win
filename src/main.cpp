#include "main.h"

fobos_dev_t* device = nullptr;
float* iqData = nullptr; 
float* dataq = nullptr;
//size_t dataqSize = DEFAULT_BUF_LEN/8;
double globalFrequency = 100000000; 
double actualFrequency = 100000000; 
double listeningFrequency = 100000000; 
double globalSampleRate = 80000000;
double globalBandwidth = 200000;
double minFrequency = 60000000;
double maxFrequency = 140000000;
int globalModulationType = 0;
int globalMode = 0;
std::vector<float> fftMagnitudes;
std::vector<float> fftFrequencies;
int fftLength = 16384;
int DEFAULT_BUF_LEN = 65536;
int currentScale = 100;
bool secondGraph = false;
bool syncWariable = false;
float sensitivity = 10;
float contrast = 10;
bool colorf = false;

YourClassName::YourClassName(QWidget *parent) 
    : QMainWindow(parent), deviceOpened(false)
    {

    setFixedSize(1920, 1000);

    QStringList devices = getFobosDevices();

    QWidget *centralWidget = new QWidget(this);
    QScrollArea *scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    scrollArea->setWidget(centralWidget);
    setCentralWidget(scrollArea);
    QHBoxLayout *scaleLayout = new QHBoxLayout();
    QVBoxLayout *contrastLayout = new QVBoxLayout();
    QVBoxLayout *sensLayout = new QVBoxLayout();
     QHBoxLayout *hboxLayout = new QHBoxLayout();
    QVBoxLayout *layout = new QVBoxLayout();
    QGridLayout *checkboxLayout = new QGridLayout();
     QVBoxLayout *graphLayout = new QVBoxLayout();
    
    for (int i = 0; i < 8; ++i) {
        checkBoxes[i] = new QCheckBox(QString("GPIO %1").arg(i + 1), this);
        checkboxLayout->addWidget(checkBoxes[i], i / 4, i % 4);
        connect(checkBoxes[i], &QCheckBox::stateChanged, this, &YourClassName::onCheckboxStateChanged);
    }
    fftComboBox = new QComboBox(this);
    fftComboBox->addItem("2048");
    fftComboBox->addItem("4096");
    fftComboBox->addItem("8192");
    fftComboBox->addItem("16384");
    fftComboBox->addItem("32768");
    fftComboBox->addItem("65536");
    fftComboBox->addItem("131072");
    fftComboBox->addItem("262144");
    fftComboBox->addItem("524288");
    fftComboBox->setCurrentIndex(3);
    
    lnaGainSlider = new QSlider(Qt::Horizontal, this);
    lnaGainSlider->setRange(1, 3);
    lnaGainSlider->setValue(1);
    
    vgaGainSlider = new QSlider(Qt::Horizontal, this);
    vgaGainSlider->setRange(0, 15);
    vgaGainSlider->setValue(3);
    
    lnaGainLabel = new QLabel("LNA Gain: 1", this);
    vgaGainLabel = new QLabel("VGA Gain: 3", this);
    
    scaleSlider = new QSlider(Qt::Horizontal, this); 
    scaleSlider->setRange(1, 100);
    scaleSlider->setValue(currentScale);    
    scaleLabel = new QLabel(QString("Scale: %1").arg(currentScale), this);
    
    audioDeviceComboBox = new QComboBox(this);
    comboBox = new QComboBox(this);
    modeBox = new QComboBox(this);
    sampleBox = new QComboBox(this);
    clkBox = new QComboBox(this);
    
    audioCheckbox = new QCheckBox("Audio", this);
    syncCheckbox = new QCheckBox("Sync", this);
    graphCheckbox = new QCheckBox("Spectr 2", this);
    colorCheckbox = new QCheckBox("Colorful", this);

    QHBoxLayout* chckbox = new QHBoxLayout();
    chckbox->addWidget(audioCheckbox);
    chckbox->addWidget(syncCheckbox);
    chckbox->addWidget(graphCheckbox);
    chckbox->addWidget(colorCheckbox);

    comboBox->addItems(getFobosDevices());
    modeBox->addItem("RF", 0);
    modeBox->addItem("HF1 + HF2", 1);
    modeBox->addItem("HF1", 2);
    modeBox->addItem("HF2", 3);
    
    clkBox->addItem("Internal", 0);
    clkBox->addItem("External", 1);

    processor = new DataProcessor(device, this);
    audioProcessor = new AudioProcessor(this);
    
    graphWidget = new MyGraphWidget(this);
    graphWidget->setFixedSize(1536, 256);
    
    scaleWidget = new ScaleWidget(this);
    scaleWidget->setFixedSize(1536, 50);
    
    waterfallWidget = new MyWaterfallWidget(this);
    waterfallWidget->setFixedSize(1536, 608);
    
    refreshButton = new QPushButton("Refresh USB Devices", this);
    fobosButton = new QPushButton("Show Fobos Details", this);
    startButton = new QPushButton("Start", this);
    stopButton = new QPushButton("Stop", this);
    
    contrastSlider = new QSlider(Qt::Horizontal, this);
    contrastSlider->setRange(1, 15);
    contrastSlider->setValue(10);
    contrastLabel = new QLabel(QString("Contrast: %1").arg(contrast), this);
    
    sensitivitySlider = new QSlider(Qt::Horizontal, this);
    sensitivitySlider->setRange(1, 30);
    sensitivitySlider->setValue(10);
    sensitivityLabel = new QLabel(QString("Sensitivity: %1").arg(sensitivity), this);
    
    sensLayout->addWidget(sensitivityLabel);
    sensLayout->addWidget(sensitivitySlider);
    contrastLayout->addWidget(contrastLabel);
    contrastLayout->addWidget(contrastSlider);
    
    QLabel *centralFrequencyLabel = new QLabel("Central Frequency (Hz):", this);
    frequencyLineEdit = new QLineEdit(this);
    frequencyLineEdit->setPlaceholderText("Enter frequency (Hz)");
    frequencyLineEdit->setText("100000000");
    
    QLabel *listeningFrequencyLabel = new QLabel("Listening Frequency (Hz):", this);
    listeningFrequencyLineEdit = new QLineEdit(this);
    listeningFrequencyLineEdit->setPlaceholderText("Enter listening frequency (Hz)");
    listeningFrequencyLineEdit->setText("100000000");
    
    QLabel *fftLabel = new QLabel("FFT Length", this);
    
    bandwidthLineEdit = new QLineEdit(this);
    bandwidthLineEdit->setText("200000");
   
    QButtonGroup* buttonGroup = new QButtonGroup(this);
    QStringList modulationNames = {"AM", "FM", "SSB", "USB", "LSB", "DSB", "CW", "FT8", "RTTY", "FSK", "PSK", "QAM"};
    
    QHBoxLayout* row1 = new QHBoxLayout();
    QHBoxLayout* row2 = new QHBoxLayout();
    QHBoxLayout* row3 = new QHBoxLayout();
    
    scaleLayout->addLayout(contrastLayout);
    scaleLayout->addLayout(sensLayout);
    
    graphLayout->addWidget(graphWidget);
    graphLayout->addWidget(scaleWidget); 
    graphLayout->addWidget(waterfallWidget);
    graphLayout->addLayout(scaleLayout);
    
    for (int i = 0; i < modulationNames.size(); ++i) {
        QRadioButton* radioButton = new QRadioButton(modulationNames[i]);
        buttonGroup->addButton(radioButton, i);
        
        if (i < 4) {
            row1->addWidget(radioButton);
        } else if (i < 8) {
        row2->addWidget(radioButton);
        } else {
            row3->addWidget(radioButton);
        }
        
        if (i == 0) {
            radioButton->setChecked(true); 
        }
    }
    
    layout->addWidget(refreshButton);
    layout->addWidget(comboBox);
    layout->addWidget(clkBox);
    layout->addWidget(modeBox);
    layout->addWidget(sampleBox);
    layout->addLayout(checkboxLayout);
    layout->addWidget(centralFrequencyLabel);
    layout->addWidget(frequencyLineEdit);
    layout->addWidget(listeningFrequencyLabel);
    layout->addWidget(listeningFrequencyLineEdit);
    layout->addWidget(fftLabel);
    layout->addWidget(fftComboBox);
    layout->addWidget(scaleLabel);
    layout->addWidget(scaleSlider);
    layout->addWidget(fobosButton);
    layout->addWidget(startButton);
    layout->addWidget(stopButton);
    layout->addWidget(lnaGainLabel);
    layout->addWidget(lnaGainSlider);
    layout->addWidget(vgaGainLabel);
    layout->addWidget(vgaGainSlider);
    layout->addLayout(chckbox);
    layout->addWidget(audioDeviceComboBox);
    layout->addWidget(bandwidthLineEdit);
    layout->addLayout(row1);
    layout->addLayout(row2);
    layout->addLayout(row3);
    
    hboxLayout->addLayout(layout);
    hboxLayout->addLayout(graphLayout); 
    centralWidget->setLayout(hboxLayout);
    
    scaleWidget->setMarkerPosition(0.5);
    scaleWidget->setRange(minFrequency, maxFrequency);
    
    updateTimer = new QTimer(this);
    updateTimer->setInterval(10);
    
    connect(updateTimer, &QTimer::timeout, this, &YourClassName::updateSpectrum);
    connect(graphCheckbox, &QCheckBox::toggled, this, &YourClassName::doubleGraphEnable);
    connect(colorCheckbox, &QCheckBox::toggled, this, &YourClassName::colorGraphEnable);
    connect(syncCheckbox, &QCheckBox::toggled, this, &YourClassName::syncEnable);
    connect(audioDeviceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onAudioDeviceChanged(int)));
    connect(buttonGroup, QOverload<int>::of(&QButtonGroup::idClicked), this, &YourClassName::onModulationChanged);
    connect(scaleSlider, &QSlider::valueChanged, this, &YourClassName::onScaleChanged);
    connect(frequencyLineEdit, &QLineEdit::returnPressed, this, &YourClassName::onFrequencyEntered);
    connect(fftComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onfftLengthEntered()));
    connect(listeningFrequencyLineEdit, &QLineEdit::returnPressed, this, &YourClassName::onListeningFrequencyEntered);
    connect(lnaGainSlider, &QSlider::valueChanged, this, &YourClassName::onLnaGainChanged);
    connect(vgaGainSlider, &QSlider::valueChanged, this, &YourClassName::onVgaGainChanged);
    connect(contrastSlider, &QSlider::valueChanged, this, &YourClassName::onContrastChanged);
    connect(sensitivitySlider, &QSlider::valueChanged, this, &YourClassName::onSensitivityChanged);
    
    connect(startButton, &QPushButton::clicked, this, &YourClassName::startFobosProcessing);
    connect(stopButton, &QPushButton::clicked, this, &YourClassName::stopFobosProcessing);
    connect(modeBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onDirectSamplingChanged);
    connect(clkBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onClkChanged);
    connect(refreshButton, &QPushButton::clicked, [this]() {
        comboBox->clear();
        comboBox->addItems(getFobosDevices());
    });
    connect(fobosButton, &QPushButton::clicked, this, &YourClassName::listFobosDevices);
    connect(sampleBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onSampleRateChanged);
    connect(bandwidthLineEdit, &QLineEdit::textChanged, this, &YourClassName::onBandwidthChanged);
    connect(scaleWidget, SIGNAL(frequencyChanged()), this, SLOT(updateFrequency()));
    connect(scaleWidget, SIGNAL(centralFrequencyChanged()), this, SLOT(updateCentralFrequency()));
    //connect(scaleWidget, &ScaleWidget::frequencyChanged, this, &YourClassName::updateFrequency);
    //connect(scaleWidget, &ScaleWidget::centralFrequencyChanged, this, &YourClassName::updateCentralFrequency);
    //connect(waterfallWidget, SIGNAL(scaleChanged(int delta)), this, SLOT(onWaterfallScaleChanged(int delta)));
    //connect(graphWidget, SIGNAL(scaleChanged(int delta)), this, SLOT(onWaterfallScaleChanged(int delta)));
    connect(waterfallWidget, &MyWaterfallWidget::scaleChanged, this, &YourClassName::onWaterfallScaleChanged);
    connect(graphWidget, &MyGraphWidget::scaleChanged, this, &YourClassName::onWaterfallScaleChanged);
    onFrequencyEntered();
    onVgaGainChanged(3);
    onLnaGainChanged(1);
    populateSampleRates();
    populateAudioDevices();
}

YourClassName::~YourClassName() {
    if (audioProcessor) { 
    delete audioProcessor;
    audioProcessor = nullptr;
    }
    if (processor) {
        delete processor;
        processor = nullptr;
    }

    if (device) {
        fobos_rx_close(device);
        device = nullptr;
    }
    if (iqData) {
        iqData = nullptr;
    }
}

void YourClassName::updateFrequency() {
   listeningFrequencyLineEdit->setText(QString::number(listeningFrequency, 'f', 0));
   onListeningFrequencyEntered();
}

void YourClassName::updateCentralFrequency() {
   frequencyLineEdit->setText(QString::number(globalFrequency, 'f', 0));
   onFrequencyEntered();
}

void YourClassName::onModulationChanged(int id) {
    globalModulationType = id;
    qDebug() << "Modulation type changed to:" << id;
}

void YourClassName::onScaleChanged(int value) {
    currentScale = value;
    fftResult = std::make_unique<FFTResult>();

    scaleLabel->setText(QString("Scale: %1").arg(value));
    settingRange();
}

void YourClassName::onSensitivityChanged(int value) {
    sensitivity = value;
    sensitivityLabel->setText(QString("Sensitivity: %1").arg(value));
    settingRange();
}

void YourClassName::onContrastChanged(int value) {
    contrast = value;
    contrastLabel->setText(QString("Contrast: %1").arg(value));
    settingRange();
}

void YourClassName::onWaterfallScaleChanged(int delta) {
    currentScale += delta;
    currentScale = qBound(minScale, currentScale, maxScale);  // ???????????? ???????? ???????? ????????, ???? ?????

    scaleSlider->setValue(currentScale);
    scaleLabel->setText(QString("Scale: %1").arg(currentScale));
    settingRange();
}

void YourClassName::doubleGraphEnable(bool checked) {
    if (checked){
        secondGraph = true;
        qDebug()<<"secondgraph enabled";
    } else {
        secondGraph = false;
        qDebug()<<"secondgraph disabled";
    }
}

void YourClassName::colorGraphEnable(bool checked) {
    if (checked){
        colorf = true;
        qDebug()<<"color graph enabled";
    } else {
        colorf = false;
        qDebug()<<"color graph disabled";
    }
}

void YourClassName::syncEnable(bool checked) {
    if (checked){
        syncWariable = true;
        qDebug()<<"syncWariable enabled";
    } else {
        syncWariable = false;
        qDebug()<<"syncWariable disabled";
    }
}

void YourClassName::wheelEvent(QWheelEvent *event) {
    if (event->angleDelta().y() != 0) {
        QLineEdit *focusedLineEdit = qobject_cast<QLineEdit*>(focusWidget());
        if (focusedLineEdit) {
            bool ok;
            double currentValue = focusedLineEdit->text().toDouble(&ok);
            if (ok) {
                double delta = event->angleDelta().y() > 0 ? 1.0 : -1.0; 
                currentValue += delta;
                focusedLineEdit->setText(QString::number(currentValue));fftResult = std::make_unique<FFTResult>();

                focusedLineEdit->emit textEdited(focusedLineEdit->text()); 
            }
        }
    }
    QMainWindow::wheelEvent(event); 
}

void YourClassName::populateAudioDevices() {
    UINT numDevices = waveOutGetNumDevs();
    qDebug() << "Number of waveOut devices found:" << numDevices;

    for (UINT i = 0; i < numDevices; i++) {
        WAVEOUTCAPS caps;
        if (waveOutGetDevCaps(i, &caps, sizeof(WAVEOUTCAPS)) == MMSYSERR_NOERROR) {
            QString deviceName = QString::fromLocal8Bit(caps.szPname);  // Исправлено
            qDebug() << "Device" << i << ":" << deviceName;
            audioDeviceComboBox->addItem(deviceName, QVariant(i));
        }
    }
}

void YourClassName::onAudioDeviceChanged(int index) {
    if (index < 0) return;

    QVariant data = audioDeviceComboBox->currentData();
    if (!data.isValid()) {
        qDebug() << "Error: Invalid audio device selected!";
        return;
    }

    // Получаем строку с именем устройства
    QString deviceName = data.toString();
    qDebug() << "Selected audio device:" << deviceName;

    int deviceID = data.toInt();  // Передаём ID вместо имени
    qDebug() << "Selected audio device ID:" << deviceID;

    if (audioProcessor) {
        audioProcessor->setAudioDevice(deviceID);
    } else {
        qDebug() << "Error: audioProcessor is null!";
    }
}


void YourClassName::onBandwidthChanged() {
    globalBandwidth = bandwidthLineEdit->text().toDouble();
}

void YourClassName::onfftLengthEntered() {
    fftLength = fftComboBox->currentText().toInt();
}

QString formatSampleRate(double sampleRate) {
    QString formattedRate;
    if (sampleRate >= 1e9) {
        formattedRate = QString::number(sampleRate / 1e9, 'f', 2) + " GHz";
    } else if (sampleRate >= 1e6) {
        formattedRate = QString::number(sampleRate / 1e6, 'f', 2) + " MHz";
    } else if (sampleRate >= 1e3) {
        formattedRate = QString::number(sampleRate / 1e3, 'f', 2) + " kHz";
    } else {
        formattedRate = QString::number(sampleRate, 'f', 2) + " Hz";
    }

    return formattedRate;
}

void YourClassName::populateSampleRates() {
    int ret = fobos_rx_open(&device, 0);
    if (!device) {
        qDebug() << "Device is not initialized.";
        return;
    }
    double sampleRates[100];
    unsigned int count = 100;
    int result = fobos_rx_get_samplerates(device, sampleRates, &count);
    if (result != FOBOS_ERR_OK) {
        qDebug() << "Failed to get sample rates, error code:" << result;
        return;
    }
    sampleBox->clear();
    for (unsigned int i = 0; i < count; ++i) {
        QString formattedRate = formatSampleRate(sampleRates[i]);
        sampleBox->addItem(formattedRate);
    }
}

void YourClassName::updateSpectrum() {
    //QElapsedTimer timer;
    //timer.start();
    //dataq = new float[dataSize];
        //for (int i = 0; i < 8; ++i){
        //int setrf = fobos_rx_set_frequency(device, globalFrequency + globalSampleRate * i, &actualFrequency);
        //memcpy(iqData + i * DEFAULT_BUF_LEN/8, dataq, DEFAULT_BUF_LEN/8 * sizeof(float));
        //}
    fftResult->storeFFTResults();
    graphWidget->setData(fftFrequencies, fftMagnitudes, minFrequency, maxFrequency, fftLength, colorf);
    waterfallWidget->setData(fftFrequencies, fftMagnitudes, minFrequency, maxFrequency, fftLength, secondGraph, contrast, sensitivity);
    //waterfallWidget->setData(fftFrequencies, fftMagnitudes, minFrequency, maxFrequency, fftLength, secondGraph, contrast, sensitivity);
    //qDebug() << "all took" << timer.elapsed() << "milliseconds";
}
 
void YourClassName::onSampleRateChanged(int index) {
    if (!device) {
        qDebug() << "Device is not initialized.";
        return;
    }
    QString selectedText = sampleBox->currentText();
    bool ok;
    double selectedSampleRate = selectedText.remove(QRegExp("[^\\d.]")).toDouble(&ok);
    if (!ok) {
        qDebug() << "Invalid sample rate selected.";fftResult = std::make_unique<FFTResult>();

        return;
    }
    selectedSampleRate *= 1e6;
    qDebug() << "Attempting to set sample rate to:" << selectedSampleRate;
    int result = fobos_rx_set_samplerate(device, selectedSampleRate, &globalSampleRate);
    if (result != FOBOS_ERR_OK) {
        qDebug() << "Failed to set sample rate, error code:" << result;
    } else {
        QString formattedRate = formatSampleRate(globalSampleRate);
        qDebug() << "Sample rate set to" << formattedRate;
    }
    settingRange();
}
void YourClassName::onListeningFrequencyEntered() {
    if (listeningFrequencyLineEdit) {
        bool ok;
        double frequency = listeningFrequencyLineEdit->text().toDouble(&ok);
        if (!ok) {
        qDebug() << "Invalid frequency entered.";
        return;
    }
        if (ok) {
         listeningFrequency = frequency; 
         qDebug() << "Frequency set to" << listeningFrequency << "Hz";
     }
 settingRange();
}
}    
    
void YourClassName::onFrequencyEntered() {
    if (globalMode == 0) {
    if (frequencyLineEdit) {
        bool ok;
        double frequency = frequencyLineEdit->text().toDouble(&ok);
        if (!ok) {
        qDebug() << "Invalid frequency entered.";
        return;
    }
    if (frequency < 50000000){
        frequency = 50000000;
        qDebug() << "Frequency to set" << frequency << "Hz";
        }
        if (ok) {
            int result = fobos_rx_set_frequency(device, frequency, &actualFrequency);
            globalFrequency = frequency;
            if (result != FOBOS_ERR_OK) {
                qDebug() << "Failed to set frequency, error code:" << result;
            } else {
                QString frequencyStr = QString::number(globalFrequency, 'f', 0);
                QString formattedFreq = formatSampleRate(globalFrequency);
                 frequencyLineEdit->setText(frequencyStr);
                qDebug() << "Frequency set to" << formattedFreq;
            }
        }
        }
    }
    else { frequencyLineEdit->setText("000000000"); 
        listeningFrequencyLineEdit->setText("1250000");
        listeningFrequency = 1250000;
        globalFrequency = 0;
        }
settingRange();
}

QStringList YourClassName::getFobosDevices() {
    QStringList deviceList;
        if (device) {
        fobos_rx_close(device);
        device = nullptr;
    }
    int deviceCount = fobos_rx_get_device_count();
    if (deviceCount == 0) {
        deviceList << "Failed to initialize libfobos";
        return deviceList;
    }
    QString deviceInfo = "Number of Fobos devices connected: " + QString::number(deviceCount) + "\n";
    for (int i = 0; i < deviceCount; ++i) {
        char hvrev[256], fvver[256], manuf[256], prod[256], serialNumber[256];
        fobos_dev_t* tempDevice = nullptr;
        int result = fobos_rx_open(&tempDevice, i);
        if (result == FOBOS_ERR_OK) {
            if (fobos_rx_get_board_info(tempDevice, hvrev, fvver, manuf, prod, serialNumber) == FOBOS_ERR_OK) {
                deviceList << QString("Device %1: Serial Number: %2").arg(i).arg(serialNumber);
            } else {
                deviceList << QString("Failed to get serial number for device %1").arg(i);
            }
            fobos_rx_close(tempDevice);
        } else {
            deviceInfo += "Failed to open device " + QString::number(i) + "\n";
        }
    }
    return deviceList;
}

void YourClassName::listFobosDevices() {
    int deviceCount = fobos_rx_get_device_count();
    if (deviceCount == 0) {
        QMessageBox::information(this, "Devices", "No Fobos devices connected.");
    } else {
        QString deviceInfo = "Number of Fobos devices connected: " + QString::number(deviceCount) + "\n";
        for (int i = 0; i < deviceCount; ++i) {
            char hvrev[256], fvver[256], manuf[256], prod[256], serialNumber[256], libv[256], apiv[256];
                if (fobos_rx_get_board_info(device, hvrev, fvver, manuf, prod, serialNumber) == FOBOS_ERR_OK &&
                    fobos_rx_get_api_info(libv, apiv) == FOBOS_ERR_OK) {
                    deviceInfo += QString("Device %1: hardware revision %2, firmware version %3, manufacturer %4, Product %5, Serial Number: %6, library version %7, API version %8\n")
                        .arg(i).arg(hvrev).arg(fvver).arg(manuf).arg(prod).arg(serialNumber).arg(libv).arg(apiv);
            } else {
                deviceInfo += "Failed to open device " + QString::number(i) + "\n";
            }
        }
        QMessageBox::information(this, "Fobos Devices", deviceInfo);
    }
}

void YourClassName::onDirectSamplingChanged(int index) {
    if (device) {
        int value = modeBox->currentData().toInt();
        int libfobosValue = (value == 0) ? 0 : 1;
        int result = fobos_rx_set_direct_sampling(device, static_cast<unsigned int>(libfobosValue));
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set direct sampling mode, error code:" << result;
        } else {
            qDebug() << "Mode set to" << libfobosValue;
        }
        globalMode = value; 
        if (value != 0){
        frequencyLineEdit->setText("000000000"); 
        listeningFrequencyLineEdit->setText("1250000");
        globalFrequency = 0;
        listeningFrequency = 1250000;
        }
          else  if (value == 0){
        frequencyLineEdit->setText("100000000"); 
        listeningFrequencyLineEdit->setText("100000000");
        listeningFrequency = 100000000;
        globalFrequency = 100000000;
        }   
        qDebug() << "Current mode set to" << globalMode;
    } else {
        qDebug() << "Device is not initialized.";
    }
    settingRange();
}

void YourClassName::settingRange() {
    double newRange = globalSampleRate * (currentScale / 100.0);
	double newMin = listeningFrequency - newRange / 2.0;
    double newMax = listeningFrequency + newRange / 2.0;
    if (globalMode == 0 || globalMode == 1) {		
    double overallMin = globalFrequency - globalSampleRate/ 2.0;
    double overallMax = globalFrequency + globalSampleRate / 2.0;
    if (newMin < overallMin) {
        newMin = overallMin;
        newMax = newMin + newRange;
    }
    if (newMax > overallMax) {
        newMax = overallMax;
        newMin = newMax - newRange;
    }
    } else if (globalMode == 2 || globalMode == 3) {
    double overallMind = 1;
    double overallMaxd = globalSampleRate / 2.0 - 1;
    if (newMin < overallMind) {
        newMin = overallMind;
        newMax = newMin + newRange/2.0;
    }

    if (newMax > overallMaxd) {
        newMax = overallMaxd;
        newMin = newMax - newRange/2.0;
    }
    } 
    minFrequency = newMin;
    maxFrequency = newMax;
    scaleWidget->setRange(minFrequency, maxFrequency);
}

void YourClassName::onCheckboxStateChanged(int state) {
    QCheckBox *senderCheckbox = qobject_cast<QCheckBox*>(sender());
    if (senderCheckbox) {
        uint8_t value = 0;
        for (int i = 0; i < 8; ++i) {
            if (checkBoxes[i]->isChecked()) {
                value |= (1 << i); 
            }
        }
        qDebug() << "Checkbox state changed. New GPO value:" << value;
        int ret = fobos_rx_set_user_gpo(device, value);
        if (ret != FOBOS_ERR_OK) {
            qDebug() << "Failed to set user GPO, error code:" << ret;
        }
    }
}

void YourClassName::onLnaGainChanged(int value) {
   fobos_rx_open(&device, 0);
    if (!device) {
        qDebug() << "Device is not initialized.";
        return;
    }
 
    int result = fobos_rx_set_lna_gain(device, static_cast<unsigned int>(value));
    if (result != FOBOS_ERR_OK) {
        qDebug() << "Failed to set LNA gain, error code:" << result;
    } else {
        lnaGainLabel->setText(QString("LNA Gain: %1").arg(value));
    }
}

void YourClassName::onVgaGainChanged(int value) {
      fobos_rx_open(&device, 0);
    if (!device) {

        qDebug() << "Device is not initialized.";
        return;
    }

    int result = fobos_rx_set_vga_gain(device, static_cast<unsigned int>(value));
    if (result != FOBOS_ERR_OK) {
        qDebug() << "Failed to set VGA gain, error code:" << result;
    } else {
        vgaGainLabel->setText(QString("VGA Gain: %1").arg(value));
    }
}

void YourClassName::onClkChanged(int index) {
    if (device) {
        int value = clkBox->currentData().toInt();
        int result = fobos_rx_set_clk_source(device, static_cast<unsigned int>(value));
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set clock source, error code:" << result;
             } else {
                qDebug() << "Clock source set to" << value;
        }
    } else {
        qDebug() << "Device is not initialized.";
    }
}

void YourClassName::startFobosProcessing() {
    fftResult = std::make_unique<FFTResult>();
    fftComboBox->setEnabled(false);
    if (!deviceOpened) {
        deviceOpened = true;
        processor->start();
        if (audioCheckbox->isChecked()) {
        audioProcessor->startDemodulation();
    }
    updateTimer->start();
        qDebug() << "Fobos Started";
    }
}
 
void YourClassName::stopFobosProcessing() {
    fftComboBox->setEnabled(true);
    if (deviceOpened) {
        updateTimer->stop();
        if (audioCheckbox->isChecked()) {
            audioProcessor->stopDemodulation();
        }
        processor->stop();
        processor->wait();
        fftResult.reset();
        deviceOpened = false;
    }
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    YourClassName window;
    window.show(); 
    return app.exec();
    qDebug() << "App started";
}
