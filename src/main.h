#ifndef MAIN_H
#define MAIN_H

#include <Windows.h>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QTimer>
#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>
#include <QSlider>
#include <QLabel>
#include <memory>
#include <QScrollArea>
#include <QCheckBox>
#include <QMainWindow>
//#include <QAudioOutput>
//#include <QAudioDeviceInfo>
//#include <QAudio>
#include <QWheelEvent>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QCloseEvent>
#include <QMessageBox>
#include <QVector>
#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <cmath>
#include <algorithm>
#include <QRadioButton>
#include <QButtonGroup>
#include <QElapsedTimer>
#include <windows.h>
#include <mmsystem.h>
#include "fft.h"
#include "dataprocessor.h"
#include "audioprocessor.h"
#include "scalewidget.h"
#include "MyGraphWidget.h"
#include "MyWaterfallWidget.h"
#include <fobos.h>
#include <fftw3.h>
//#include <libusb.h>

extern fobos_dev_t *device;
extern float* dataq;
extern float* iqData;
extern double globalFrequency; 
extern double actualFrequency; 
extern double listeningFrequency;
extern double globalSampleRate;
extern double globalBandwidth;
extern int globalModulationType;
extern int globalMode;
extern int fftLength;
extern int currentScale;
extern double minFrequency;
extern double maxFrequency;
extern float sensitivity;
extern float contrast;
class FFTResult;

class YourClassName : public QMainWindow {
    Q_OBJECT

public:
   explicit YourClassName(QWidget *parent = nullptr);
    ~YourClassName();
     std::unique_ptr<FFTResult> fftResult;
    void onFrequencyEntered();
    void onListeningFrequencyEntered();
    void onScaleChanged(int value);
    void updateSpectrum();
    void onLnaGainChanged(int value);
    void onVgaGainChanged(int value);
    void onSensitivityChanged(int value);
    void onContrastChanged(int value);
    void populateSampleRates();
    void populateAudioDevices();
private slots:
	void settingRange();
    void onDirectSamplingChanged(int index);
    void listFobosDevices();
    void startFobosProcessing();
    void stopFobosProcessing();
    void onSampleRateChanged(int index);
    void onfftLengthEntered();
    void onClkChanged(int index);
    void onCheckboxStateChanged(int state);
    void onAudioDeviceChanged(int index);
    void syncEnable(bool checked);
	void onModulationChanged(int id);
    void onBandwidthChanged();
    void updateFrequency();
    void updateCentralFrequency();
    void doubleGraphEnable(bool checked);
    void colorGraphEnable(bool checked);
protected:
	void onWaterfallScaleChanged(int delta);
    void wheelEvent(QWheelEvent *event) override;
        //void closeEvent(QCloseEvent *event) override {
        //int reply = QMessageBox::question(this, "Acception", 
        //                                  "Close this program?",
        //                                  QMessageBox::Yes | QMessageBox::No);
        //if (reply == QMessageBox::Yes) { 
        //    event->accept();
        //} else {
        //    event->ignore();
        //}
		//};

private:
    QWidget *centralWidget;
    
    QStringList getFobosDevices();
    
    QComboBox *clkBox;
    QComboBox *comboBox;
    QComboBox *modeBox;
    QComboBox *sampleBox;
    QComboBox *fftComboBox;
    QComboBox *audioDeviceComboBox;
    
    QPushButton *refreshButton;
    QPushButton *fobosButton;
    QPushButton *startButton;
    QPushButton *stopButton;
    
    QCheckBox *spectrumCheckbox;
    QCheckBox *audioCheckbox;
    QCheckBox *syncCheckbox;
    QCheckBox *graphCheckbox;
    QCheckBox *colorCheckbox;
    QCheckBox *checkBoxes[8]; 
    
    QSlider *scaleSlider;
    QSlider *lnaGainSlider;
    QSlider *vgaGainSlider;
    QSlider *contrastSlider;
    QSlider *sensitivitySlider;
    
    QLabel *lnaGainLabel;
    QLabel *centralFrequencyLabel;
    QLabel *listeningFrequencyLabel;
    QLabel *fftLabel;
    QLabel *contrastLabel;
    QLabel *sensitivityLabel;
    QLabel *vgaGainLabel;
    QLabel *scaleLabel;
       
    QLineEdit *frequencyLineEdit;
    QLineEdit *listeningFrequencyLineEdit;
    QLineEdit *bandwidthLineEdit;
    
    QTimer *updateTimer;

    DataProcessor *processor;
    AudioProcessor *audioProcessor;
    MyGraphWidget *graphWidget;
    MyWaterfallWidget *waterfallWidget;
    ScaleWidget *scaleWidget;
    
    bool deviceOpened;
    int minScale = 1;  
    int maxScale = 100;
};

#endif // MAIN_H
