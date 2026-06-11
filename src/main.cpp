#include "main.h"
#include "iqbuffer.h"
#include "diagnosticlogging.h"
#include "finetunewidget.h"

#include <fobos_sdr.h>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QList>
#include <QMenu>
#include <QMessageLogContext>
#include <QMutexLocker>
#include <QHostAddress>
#include <QIcon>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTabWidget>
#include <QTableWidget>
#include <QHeaderView>
#include <QTextStream>
#include <QTextCursor>
#include <QPushButton>
#include <QToolButton>
#include <QKeyEvent>
#include <QAbstractButton>
#include <QAbstractItemView>
#include <QColor>
#include <QCoreApplication>
#include <QRegularExpression>
#include <QScopeGuard>
#include <QSettings>
#if !defined(_WIN32) && defined(FOBOSAPP_HAS_QT_AUDIO)
#include <QAudioDeviceInfo>
#endif
#include <cmath>
#include <cstring>
#include <limits>
#include <array>
#ifdef _WIN32
#include <psapi.h>
#endif
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <new>
#include <utility>

#ifdef _MSC_VER
#pragma comment(lib, "psapi.lib")
#endif

fobos_dev_t* device = nullptr;
fobos_sdr_dev_t* agileDevice = nullptr;
FobosApiKind activeFobosApiKind = FobosApiKind::Standard;
float* iqData = nullptr; 
float* dataq = nullptr;
//size_t dataqSize = DEFAULT_BUF_LEN/8;
double globalFrequency = 100000000; 
double actualFrequency = 100000000; 
double listeningFrequency = 100000000; 
double globalSampleRate = 80000000;
double globalBandwidth = 10000;
double minFrequency = 60000000;
double maxFrequency = 140000000;
int globalModulationType = 0;
int globalMode = 0;
std::vector<float> fftMagnitudes;
std::vector<float> fftFrequencies;
int fftLength = 32768;
int DEFAULT_BUF_LEN = 32768;
double currentScale = 100.0;
bool secondGraph = false;
bool syncWariable = false;
float sensitivity = 10;
float contrast = 10;
bool colorf = false;
int deviceID = 0;

namespace {

constexpr double RF_MIN_CENTER_FREQUENCY = 50000000.0;
constexpr double RF_MIN_LISTENING_FREQUENCY = 25000000.0;
constexpr double RF_EXPERIMENTAL_MAX_FREQUENCY = 7750000000.0;
constexpr double DIRECT_MIN_FREQUENCY = 1.0;
constexpr int SCALE_SLIDER_FACTOR = 10;
constexpr double MIN_SCALE_PERCENT = 0.1;
constexpr double MAX_SCALE_PERCENT = 100.0;
constexpr int LEVEL_SLIDER_FACTOR = 10;
constexpr int MIN_LEVEL_SLIDER_VALUE = -1600;
constexpr int MAX_LEVEL_SLIDER_VALUE = 200;
constexpr float MIN_LEVEL_GAP = 0.1f;
constexpr int NETWORK_SPECTRUM_MAX_BINS = 2048;
constexpr int NETWORK_CHANNEL_SPECTRUM_MAX_BINS = 768;
constexpr int NETWORK_SPECTRUM_INTERVAL_MS = 100;
constexpr int NETWORK_CHANNEL_SPECTRUM_INTERVAL_MS = 160;
constexpr int NETWORK_FULL_RESOLUTION_SPECTRUM_INTERVAL_MS = 50;
constexpr qint64 NETWORK_IQ_MAX_PENDING_BYTES = 8 * 1024 * 1024;
constexpr qint64 NETWORK_CHANNEL_IQ_LOW_LATENCY_PENDING_BYTES = 2 * 1024 * 1024;
constexpr uint64_t NETWORK_IQ_DROP_LOG_INTERVAL = 200;
constexpr qint64 NETWORK_CLIENT_SETTINGS_GUARD_MS = 1500;
constexpr double NETWORK_AUDIO_PREBUFFER_SECONDS = 0.55;
constexpr qint64 NETWORK_SPECTRUM_MAX_PENDING_BYTES = 4 * 1024 * 1024;
constexpr int AUDIO_RELAY_HEADER_BYTES = 12;
constexpr qint64 AUDIO_HTTP_MAX_PENDING_BYTES = 512 * 1024;
constexpr int AUDIO_LOW_PASS_SLIDER_STEP_HZ = 100;
constexpr int AUDIO_LOW_PASS_SLIDER_MAX = 200;
constexpr int AUDIO_HIGH_PASS_SLIDER_STEP_HZ = 25;
constexpr int AUDIO_HIGH_PASS_SLIDER_MAX = 40;
constexpr int FINE_TUNE_DIAL_MIN = -100;
constexpr int FINE_TUNE_DIAL_MAX = 100;
constexpr double FINE_TUNE_VISIBLE_RANGE_DIVISOR = 20.0;
constexpr double FINE_TUNE_MIN_RANGE_HZ = 500.0;
constexpr double FINE_TUNE_MAX_RANGE_HZ = 500000.0;
constexpr int FINE_TUNE_MODE_SCALE = 0;
constexpr int FINE_TUNE_MODE_DIAL = 1;
constexpr int HF_NOISE_CANCEL_DEPTH_MIN = 0;
constexpr int HF_NOISE_CANCEL_DEPTH_MAX = 200;
constexpr int HF_NOISE_CANCEL_REF_GAIN_MIN = -400;
constexpr int HF_NOISE_CANCEL_REF_GAIN_MAX = 400;
constexpr int HF_NOISE_CANCEL_REF_DELAY_MIN_NS = -2000;
constexpr int HF_NOISE_CANCEL_REF_DELAY_MAX_NS = 2000;
constexpr int HF_NOISE_CANCEL_REF_TILT_MIN = -300;
constexpr int HF_NOISE_CANCEL_REF_TILT_MAX = 300;
constexpr int VIDEO_SNAPSHOT_INTERVAL_MS = 90;
constexpr std::size_t VIDEO_SNAPSHOT_MAX_FLOATS = 262144 * 2;
constexpr qint64 FPV_HUNTER_TRACK_HOLD_MS = 120000;
constexpr int FPV_HUNTER_TRACK_HOLD_FRAMES = 1200;
constexpr int FPV_HUNTER_TRACK_STABLE_MISS_FRAMES = 3;
constexpr int FPV_HUNTER_MAX_EVENTS = 32;
constexpr qint64 LIVE_RETUNE_SETTLE_MS = 80;
constexpr qint64 AGILE_RF_LOW_RATE_RETUNE_SETTLE_MS = 550;
constexpr qint64 AGILE_RF_LOW_RATE_SAMPLE_SETTLE_MS = 900;
constexpr qint64 AGILE_RF_MID_RATE_RETUNE_SETTLE_MS = 320;
constexpr qint64 AGILE_RF_MID_RATE_SAMPLE_SETTLE_MS = 600;
constexpr qint64 AGILE_RF_HIGH_RATE_RETUNE_SETTLE_MS = 160;
constexpr qint64 AGILE_RF_HIGH_RATE_SAMPLE_SETTLE_MS = 280;
constexpr qint64 STANDARD_SCAN_SETTLE_MS = 60;
constexpr int STANDARD_SCAN_MIN_SETTLE_MS = 0;
constexpr int STANDARD_SCAN_MAX_SETTLE_MS = 1000;
constexpr int STANDARD_SCAN_MIN_DWELL_MS = 20;
constexpr int STANDARD_SCAN_MAX_DWELL_MS = 5000;
constexpr int SPECTRUM_UPDATE_AUTO_MS = 0;
constexpr int SPECTRUM_UPDATE_MIN_MS = 10;
constexpr int SPECTRUM_UPDATE_MAX_MS = 250;
constexpr double AGILE_RF_LOW_RATE_AUTO_BANDWIDTH_RATIO = 1.00;
constexpr double AGILE_RF_MID_RATE_AUTO_BANDWIDTH_RATIO = 1.00;
constexpr double AGILE_RF_HIGH_RATE_AUTO_BANDWIDTH_RATIO = 1.00;
constexpr int AGILE_SCAN_MIN_POINTS = 2;
constexpr int AGILE_SCAN_MAX_POINTS = 256;
constexpr double AGILE_SCAN_MIN_STEP_MHZ = 0.001;
constexpr double AGILE_SCAN_MAX_STEP_MHZ = 1000.0;
constexpr double SCAN_MEASUREMENT_MIN_BIN_MHZ = 0.001;
constexpr double SCAN_MEASUREMENT_MAX_BIN_MHZ = 10.0;
constexpr float SCAN_MEASUREMENT_COVERAGE_DELTA_DB = 6.0f;
constexpr int SPUR_CALIBRATION_TARGET_FRAMES = 32;
constexpr float SPUR_CALIBRATION_MIN_PROMINENCE_DB = 8.0f;
constexpr float SPUR_CALIBRATION_MIN_NARROW_DB = 2.0f;
constexpr int SPUR_CALIBRATION_INNER_BINS = 4;
constexpr int SPUR_CALIBRATION_OUTER_BINS = 36;
constexpr int SPUR_MAX_MASK_ENTRIES = 16;
constexpr double SPUR_MIN_MASK_WIDTH_HZ = 50.0;
constexpr double SPUR_MAX_MASK_WIDTH_HZ = 20000.0;
constexpr qint64 DIAGNOSTIC_LOG_MAX_BYTES = 8 * 1024 * 1024;

QMutex gLogMutex;
QFile gLogFile;
qint64 gLogBytesWritten = 0;
int gLogLinesUntilFlush = 0;

qint64 agileRfLiveSettleMs(double sampleRate, bool sampleRateChange) {
    if (!std::isfinite(sampleRate) || sampleRate <= 0.0) {
        return sampleRateChange ? AGILE_RF_MID_RATE_SAMPLE_SETTLE_MS
                                : AGILE_RF_MID_RATE_RETUNE_SETTLE_MS;
    }
    if (sampleRate <= 12500000.5) {
        return sampleRateChange ? AGILE_RF_LOW_RATE_SAMPLE_SETTLE_MS
                                : AGILE_RF_LOW_RATE_RETUNE_SETTLE_MS;
    }
    if (sampleRate <= 20000000.5) {
        return sampleRateChange ? AGILE_RF_MID_RATE_SAMPLE_SETTLE_MS
                                : AGILE_RF_MID_RATE_RETUNE_SETTLE_MS;
    }
    return sampleRateChange ? AGILE_RF_HIGH_RATE_SAMPLE_SETTLE_MS
                            : AGILE_RF_HIGH_RATE_RETUNE_SETTLE_MS;
}

double agileRfAutoBandwidthRatio(double sampleRate) {
    if (!std::isfinite(sampleRate) || sampleRate <= 0.0) {
        return AGILE_RF_MID_RATE_AUTO_BANDWIDTH_RATIO;
    }
    if (sampleRate <= 12500000.5) {
        return AGILE_RF_LOW_RATE_AUTO_BANDWIDTH_RATIO;
    }
    if (sampleRate <= 20000000.5) {
        return AGILE_RF_MID_RATE_AUTO_BANDWIDTH_RATIO;
    }
    return AGILE_RF_HIGH_RATE_AUTO_BANDWIDTH_RATIO;
}

QString persistentSettingsFilePath() {
    return QCoreApplication::applicationDirPath() + QStringLiteral("/FobosAPP.ini");
}

void appendLe32(QByteArray &buffer, quint32 value) {
    buffer.append(static_cast<char>(value & 0xff));
    buffer.append(static_cast<char>((value >> 8) & 0xff));
    buffer.append(static_cast<char>((value >> 16) & 0xff));
    buffer.append(static_cast<char>((value >> 24) & 0xff));
}

void appendLe16(QByteArray &buffer, quint16 value) {
    buffer.append(static_cast<char>(value & 0xff));
    buffer.append(static_cast<char>((value >> 8) & 0xff));
}

quint32 readLe32(const char *data) {
    const auto *bytes = reinterpret_cast<const uchar *>(data);
    return static_cast<quint32>(bytes[0]) |
           (static_cast<quint32>(bytes[1]) << 8) |
           (static_cast<quint32>(bytes[2]) << 16) |
           (static_cast<quint32>(bytes[3]) << 24);
}

QByteArray streamingWavHeader() {
    constexpr quint32 streamDataBytes = 0x7fffffffU;
    constexpr quint16 channels = 1;
    constexpr quint32 sampleRate = 48000;
    constexpr quint16 bitsPerSample = 16;
    constexpr quint16 blockAlign = channels * bitsPerSample / 8;
    constexpr quint32 byteRate = sampleRate * blockAlign;

    QByteArray header;
    header.reserve(44);
    header.append("RIFF", 4);
    appendLe32(header, streamDataBytes + 36U);
    header.append("WAVE", 4);
    header.append("fmt ", 4);
    appendLe32(header, 16);
    appendLe16(header, 1);
    appendLe16(header, channels);
    appendLe32(header, sampleRate);
    appendLe32(header, byteRate);
    appendLe16(header, blockAlign);
    appendLe16(header, bitsPerSample);
    header.append("data", 4);
    appendLe32(header, streamDataBytes);
    return header;
}

const char *messageTypeName(QtMsgType type) {
    switch (type) {
    case QtDebugMsg:
        return "DEBUG";
    case QtInfoMsg:
        return "INFO";
    case QtWarningMsg:
        return "WARN";
    case QtCriticalMsg:
        return "CRITICAL";
    case QtFatalMsg:
        return "FATAL";
    }
    return "LOG";
}

const char *runStateName(RadioRunState state) {
    switch (state) {
    case RadioRunState::Idle:
        return "Idle";
    case RadioRunState::Starting:
        return "Starting";
    case RadioRunState::Running:
        return "Running";
    case RadioRunState::Stopping:
        return "Stopping";
    }
    return "Unknown";
}

const char *fobosApiKindName(FobosApiKind kind) {
    switch (kind) {
    case FobosApiKind::Standard:
        return "standard";
    case FobosApiKind::Agile:
        return "agile";
    }
    return "unknown";
}

QString fobosApiDisplayName(FobosApiKind kind) {
    return kind == FobosApiKind::Agile ? QStringLiteral("Agile") : QStringLiteral("Standard");
}

bool firmwareLooksAgile(const QString &firmwareVersion) {
    bool ok = false;
    const int major = firmwareVersion.trimmed().section('.', 0, 0).toInt(&ok);
    return ok && major >= 3;
}

void *activeFobosDevice() {
    return activeFobosApiKind == FobosApiKind::Agile
               ? static_cast<void*>(agileDevice)
               : static_cast<void*>(device);
}

bool hasActiveFobosDevice() {
    return activeFobosDevice() != nullptr;
}

double directMaxFrequency(double sampleRate) {
    return (std::max)(DIRECT_MIN_FREQUENCY, sampleRate / 2.0 - 1.0);
}

double directMinFrequencyForMode(int inputMode, double sampleRate) {
    return inputMode == INPUT_HF_COMBINED ? -directMaxFrequency(sampleRate) : DIRECT_MIN_FREQUENCY;
}

double autoTuneRoundingStepHz(double frequencyHz, double visibleSpanHz) {
    if (!std::isfinite(frequencyHz)) {
        return 1000.0;
    }

    const double absFrequency = std::abs(frequencyHz);
    double minimumStep = 100.0;
    double maximumStep = 5000.0;

    if (absFrequency >= 3000000000.0) {
        minimumStep = 100000.0;
        maximumStep = 5000000.0;
    } else if (absFrequency >= 1000000000.0) {
        minimumStep = 10000.0;
        maximumStep = 1000000.0;
    } else if (absFrequency >= 300000000.0) {
        minimumStep = 5000.0;
        maximumStep = 12500.0;
    } else if (absFrequency >= 30000000.0) {
        minimumStep = 1000.0;
        maximumStep = 12500.0;
    } else if (absFrequency >= 3000000.0) {
        minimumStep = 100.0;
        maximumStep = 5000.0;
    }

    const double spanStep = std::isfinite(visibleSpanHz) && visibleSpanHz > 0.0
                                ? visibleSpanHz / 2000.0
                                : minimumStep;
    const double targetStep = (std::clamp)(spanStep, minimumStep, maximumStep);
    constexpr std::array<double, 19> niceSteps = {
        10.0,
        50.0,
        100.0,
        500.0,
        1000.0,
        2500.0,
        5000.0,
        6250.0,
        10000.0,
        12500.0,
        25000.0,
        50000.0,
        100000.0,
        250000.0,
        500000.0,
        1000000.0,
        2500000.0,
        5000000.0,
        10000000.0,
    };

    for (const double step : niceSteps) {
        if (step >= targetStep && step >= minimumStep && step <= maximumStep) {
            return step;
        }
    }
    return maximumStep;
}

double roundAutoTuneFrequencyHz(double frequencyHz, double visibleSpanHz) {
    if (!std::isfinite(frequencyHz)) {
        return frequencyHz;
    }
    const double stepHz = autoTuneRoundingStepHz(frequencyHz, visibleSpanHz);
    if (!std::isfinite(stepHz) || stepHz <= 0.0) {
        return frequencyHz;
    }
    return std::round(frequencyHz / stepHz) * stepHz;
}

void normalizeTuning(RadioSettings &settings, bool preserveCenter = false) {
    if (settings.sampleRate <= 0.0) {
        return;
    }

    if (settings.inputMode == INPUT_RF) {
        const double halfRate = settings.sampleRate / 2.0;
        settings.centerFrequency = (std::clamp)(settings.centerFrequency,
                                                RF_MIN_CENTER_FREQUENCY,
                                                RF_EXPERIMENTAL_MAX_FREQUENCY);
        settings.listeningFrequency = (std::clamp)(settings.listeningFrequency,
                                                   RF_MIN_LISTENING_FREQUENCY,
                                                   RF_EXPERIMENTAL_MAX_FREQUENCY);

        if (!preserveCenter && settings.listeningFrequency < settings.centerFrequency - halfRate) {
            settings.centerFrequency = (std::clamp)(settings.listeningFrequency + halfRate,
                                                    RF_MIN_CENTER_FREQUENCY,
                                                    RF_EXPERIMENTAL_MAX_FREQUENCY);
        } else if (!preserveCenter && settings.listeningFrequency > settings.centerFrequency + halfRate) {
            settings.centerFrequency = (std::clamp)(settings.listeningFrequency - halfRate,
                                                    RF_MIN_CENTER_FREQUENCY,
                                                    RF_EXPERIMENTAL_MAX_FREQUENCY);
        }

        const double low = (std::max)(RF_MIN_LISTENING_FREQUENCY,
                                      settings.centerFrequency - halfRate);
        const double high = (std::clamp)(settings.centerFrequency + halfRate,
                                         low,
                                         RF_EXPERIMENTAL_MAX_FREQUENCY);
        settings.listeningFrequency = (std::clamp)(settings.listeningFrequency, low, high);
    } else {
        settings.centerFrequency = 0.0;
        settings.actualFrequency = 0.0;
        const double directMin = directMinFrequencyForMode(settings.inputMode, settings.sampleRate);
        const double directMax = directMaxFrequency(settings.sampleRate);
        settings.listeningFrequency = (std::clamp)(settings.listeningFrequency,
                                                   directMin,
                                                   directMax);
    }
}

constexpr double DMR_CENTER_REALIGN_THRESHOLD_HZ = 100000.0;
constexpr double AGILE_PRE_STREAM_FREQUENCY_NUDGE_HZ = 1000000.0;

bool shouldRealignDmrCenterToListening(const RadioSettings &settings) {
    return settings.inputMode == INPUT_RF &&
           settings.modulationType == MOD_DMR &&
           std::isfinite(settings.centerFrequency) &&
           std::isfinite(settings.listeningFrequency) &&
           std::abs(settings.centerFrequency - settings.listeningFrequency) >
               DMR_CENTER_REALIGN_THRESHOLD_HZ;
}

bool realignDmrCenterToListening(RadioSettings &settings) {
    if (!shouldRealignDmrCenterToListening(settings)) {
        return false;
    }
    settings.centerFrequency = settings.listeningFrequency;
    settings.actualFrequency = settings.listeningFrequency;
    normalizeTuning(settings, true);
    return true;
}

QString agileScanPresetSpec(const QString &rangesMhz, double stepMhz) {
    return QStringLiteral("%1\t%2").arg(rangesMhz.trimmed(),
                                      QString::number(stepMhz, 'f', 6));
}

QString standardScanPresetSpec(const QString &centersMhz, int dwellMs, int settleMs) {
    return QStringLiteral("%1\t%2\t%3").arg(centersMhz.trimmed(),
                                            QString::number(dwellMs),
                                            QString::number(settleMs));
}

QString agileScanPresetRanges(const QString &spec, const QString &fallback = QString()) {
    const QStringList parts = spec.split(QChar('\t'));
    return parts.isEmpty() ? fallback : parts.first().trimmed();
}

QString standardScanPresetCenters(const QString &spec, const QString &fallback = QString()) {
    const QStringList parts = spec.split(QChar('\t'));
    return parts.isEmpty() ? fallback : parts.first().trimmed();
}

double agileScanPresetStepMhz(const QString &spec, double fallback) {
    const QStringList parts = spec.split(QChar('\t'));
    if (parts.size() < 2) {
        return fallback;
    }
    bool ok = false;
    const double value = parts.at(1).toDouble(&ok);
    return ok ? (std::clamp)(value, AGILE_SCAN_MIN_STEP_MHZ, AGILE_SCAN_MAX_STEP_MHZ) : fallback;
}

int standardScanPresetDwellMs(const QString &spec, int fallback) {
    const QStringList parts = spec.split(QChar('\t'));
    if (parts.size() < 2) {
        return fallback;
    }
    bool ok = false;
    const int value = parts.at(1).toInt(&ok);
    return ok ? (std::clamp)(value, STANDARD_SCAN_MIN_DWELL_MS, STANDARD_SCAN_MAX_DWELL_MS) : fallback;
}

int standardScanPresetSettleMs(const QString &spec, int fallback) {
    const QStringList parts = spec.split(QChar('\t'));
    if (parts.size() < 3) {
        return fallback;
    }
    bool ok = false;
    const int value = parts.at(2).toInt(&ok);
    return ok ? (std::clamp)(value, STANDARD_SCAN_MIN_SETTLE_MS, STANDARD_SCAN_MAX_SETTLE_MS) : fallback;
}

QVector<ScanVisualSegment> scanSegmentsFromFrame(const QJsonObject &frame) {
    QVector<ScanVisualSegment> segments;
    const QJsonArray segmentArray = frame.value("scanSegments").toArray();
    segments.reserve(segmentArray.size());
    for (const QJsonValue &value : segmentArray) {
        const QJsonObject item = value.toObject();
        const double startHz = item.value("startHz").toDouble(std::numeric_limits<double>::quiet_NaN());
        const double endHz = item.value("endHz").toDouble(std::numeric_limits<double>::quiet_NaN());
        const double centerHz = item.value("centerHz").toDouble((startHz + endHz) * 0.5);
        const double actualStartHz = item.value("actualStartHz").toDouble(startHz);
        const double actualEndHz = item.value("actualEndHz").toDouble(endHz);
        const double actualCenterHz = item.value("actualCenterHz").toDouble((actualStartHz + actualEndHz) * 0.5);
        if (!std::isfinite(startHz) ||
            !std::isfinite(endHz) ||
            endHz <= startHz) {
            continue;
        }
        QString label = item.value("label").toString();
        if (label.isEmpty()) {
            label = QStringLiteral("%1 MHz").arg(actualCenterHz / 1000000.0, 0, 'f', 3);
        }
        segments.push_back({startHz,
                            endHz,
                            centerHz,
                            actualStartHz,
                            actualEndHz,
                            actualCenterHz,
                            label});
    }
    return segments;
}

std::vector<float> actualFrequenciesFromScanSegments(const std::vector<float> &displayFrequencies,
                                                     const QVector<ScanVisualSegment> &segments) {
    if (displayFrequencies.empty() || segments.isEmpty()) {
        return {};
    }

    std::vector<float> actualFrequencies;
    actualFrequencies.reserve(displayFrequencies.size());
    for (const float displayFrequency : displayFrequencies) {
        double actualFrequency = displayFrequency;
        for (const ScanVisualSegment &segment : segments) {
            if (!std::isfinite(segment.startHz) ||
                !std::isfinite(segment.endHz) ||
                !std::isfinite(segment.actualStartHz) ||
                !std::isfinite(segment.actualEndHz) ||
                segment.endHz <= segment.startHz ||
                displayFrequency < segment.startHz ||
                displayFrequency > segment.endHz) {
                continue;
            }
            const double ratio =
                (static_cast<double>(displayFrequency) - segment.startHz) /
                (segment.endHz - segment.startHz);
            actualFrequency = segment.actualStartHz +
                              (std::clamp)(ratio, 0.0, 1.0) *
                                  (segment.actualEndHz - segment.actualStartHz);
            break;
        }
        actualFrequencies.push_back(static_cast<float>(actualFrequency));
    }
    return actualFrequencies;
}

bool actualFrequencyInsideScanSegments(double frequencyHz, const QVector<ScanVisualSegment> &segments) {
    if (!std::isfinite(frequencyHz) || segments.isEmpty()) {
        return false;
    }
    for (const ScanVisualSegment &segment : segments) {
        if (std::isfinite(segment.actualStartHz) &&
            std::isfinite(segment.actualEndHz) &&
            segment.actualEndHz > segment.actualStartHz &&
            frequencyHz >= segment.actualStartHz &&
            frequencyHz <= segment.actualEndHz) {
            return true;
        }
    }
    return false;
}

double fallbackActualFrequencyForScanSegments(const QVector<ScanVisualSegment> &segments,
                                              double fallbackHz) {
    if (segments.isEmpty()) {
        return fallbackHz;
    }
    const ScanVisualSegment &segment = segments.at(segments.size() / 2);
    if (std::isfinite(segment.actualCenterHz)) {
        return segment.actualCenterHz;
    }
    if (std::isfinite(segment.actualStartHz) &&
        std::isfinite(segment.actualEndHz) &&
        segment.actualEndHz > segment.actualStartHz) {
        return (segment.actualStartHz + segment.actualEndHz) * 0.5;
    }
    return fallbackHz;
}

QVector<double> parseAgileScanFrequenciesMhz(const QString &rangesMhz,
                                             double stepMhz,
                                             QString *error) {
    QVector<double> frequencies;
    QString text = rangesMhz.trimmed();
    text.replace(QChar(0x2013), QLatin1Char('-'));
    text.replace(QChar(0x2014), QLatin1Char('-'));
    text.replace(QLatin1Char('\\'), QLatin1Char(','));
    text.replace(QLatin1Char('/'), QLatin1Char(','));
    text.replace(QLatin1Char(';'), QLatin1Char(','));
    text.replace(QRegularExpression(QStringLiteral("\\s+")), QString());
    text.replace(QRegularExpression(QStringLiteral("mhz|мгц"), QRegularExpression::CaseInsensitiveOption),
                 QString());

    if (text.isEmpty()) {
        if (error) {
            *error = QStringLiteral("Scan ranges are empty");
        }
        return frequencies;
    }

    stepMhz = (std::clamp)(stepMhz, AGILE_SCAN_MIN_STEP_MHZ, AGILE_SCAN_MAX_STEP_MHZ);
    const QStringList tokens = text.split(QLatin1Char(','), Qt::SkipEmptyParts);
    for (const QString &token : tokens) {
        const int dashIndex = token.indexOf(QLatin1Char('-'));
        bool startOk = false;
        bool endOk = false;
        double startMhz = 0.0;
        double endMhz = 0.0;

        if (dashIndex > 0) {
            startMhz = token.left(dashIndex).toDouble(&startOk);
            endMhz = token.mid(dashIndex + 1).toDouble(&endOk);
        } else {
            startMhz = token.toDouble(&startOk);
            endMhz = startMhz;
            endOk = startOk;
        }

        if (!startOk || !endOk || startMhz <= 0.0 || endMhz <= 0.0) {
            if (error) {
                *error = QStringLiteral("Bad scan range: %1").arg(token);
            }
            frequencies.clear();
            return frequencies;
        }
        if (endMhz < startMhz) {
            std::swap(startMhz, endMhz);
        }
        const double startHz = startMhz * 1000000.0;
        const double endHz = endMhz * 1000000.0;
        if (startHz < RF_MIN_CENTER_FREQUENCY ||
            endHz > RF_EXPERIMENTAL_MAX_FREQUENCY) {
            if (error) {
                *error = QStringLiteral("Scan uses RF input only: %1 MHz to %2 MHz")
                             .arg(RF_MIN_CENTER_FREQUENCY / 1000000.0, 0, 'f', 0)
                             .arg(RF_EXPERIMENTAL_MAX_FREQUENCY / 1000000.0, 0, 'f', 0);
            }
            frequencies.clear();
            return frequencies;
        }

        const int countBefore = frequencies.size();
        for (double mhz = startMhz; mhz <= endMhz + stepMhz * 0.25; mhz += stepMhz) {
            frequencies.push_back(mhz * 1000000.0);
            if (frequencies.size() > AGILE_SCAN_MAX_POINTS) {
                if (error) {
                    *error = QStringLiteral("Too many scan points (%1 max). Increase step or split presets.")
                                 .arg(AGILE_SCAN_MAX_POINTS);
                }
                frequencies.clear();
                return frequencies;
            }
        }
        if (frequencies.size() == countBefore) {
            frequencies.push_back(startMhz * 1000000.0);
        }
    }

    std::sort(frequencies.begin(), frequencies.end());
    auto last = std::unique(frequencies.begin(), frequencies.end(), [](double a, double b) {
        return std::abs(a - b) < 0.5;
    });
    frequencies.erase(last, frequencies.end());

    if (frequencies.size() < AGILE_SCAN_MIN_POINTS) {
        if (error) {
            *error = QStringLiteral("Agile scan needs at least two frequencies");
        }
        frequencies.clear();
    }
    return frequencies;
}

QString formatMhzList(const QVector<double> &frequenciesHz) {
    QStringList parts;
    parts.reserve(frequenciesHz.size());
    for (const double frequencyHz : frequenciesHz) {
        QString text = QString::number(frequencyHz / 1000000.0, 'f', 6);
        while (text.contains(QLatin1Char('.')) && text.endsWith(QLatin1Char('0'))) {
            text.chop(1);
        }
        if (text.endsWith(QLatin1Char('.'))) {
            text.chop(1);
        }
        parts << text;
    }
    return parts.join(QStringLiteral(", "));
}

QVector<double> parseStandardScanCentersMhz(const QString &centersMhz,
                                            double sampleRateHz,
                                            int minimumPoints,
                                            QString *error,
                                            bool *adjusted) {
    if (adjusted) {
        *adjusted = false;
    }
    QVector<double> frequencies;
    QString text = centersMhz.trimmed();
    text.replace(QChar(0x2013), QLatin1Char('-'));
    text.replace(QChar(0x2014), QLatin1Char('-'));
    text.replace(QLatin1Char('\\'), QLatin1Char(','));
    text.replace(QLatin1Char('/'), QLatin1Char(','));
    text.replace(QLatin1Char(';'), QLatin1Char(','));
    text.replace(QRegularExpression(QStringLiteral("mhz|мгц|РјРіС†"),
                                    QRegularExpression::CaseInsensitiveOption),
                 QString());
    text.replace(QRegularExpression(QStringLiteral("\\s+")), QStringLiteral(","));

    if (text.isEmpty()) {
        if (minimumPoints > 0 && error) {
            *error = QStringLiteral("Standard scan centers are empty");
        }
        return frequencies;
    }

    const QStringList tokens = text.split(QLatin1Char(','), Qt::SkipEmptyParts);
    for (const QString &token : tokens) {
        bool ok = false;
        const double mhz = token.toDouble(&ok);
        const double frequencyHz = mhz * 1000000.0;
        if (!ok ||
            !std::isfinite(frequencyHz) ||
            frequencyHz < RF_MIN_CENTER_FREQUENCY ||
            frequencyHz > RF_EXPERIMENTAL_MAX_FREQUENCY) {
            if (error) {
                *error = QStringLiteral("Standard scan uses RF centers only: %1 MHz to %2 MHz")
                             .arg(RF_MIN_CENTER_FREQUENCY / 1000000.0, 0, 'f', 0)
                             .arg(RF_EXPERIMENTAL_MAX_FREQUENCY / 1000000.0, 0, 'f', 0);
            }
            frequencies.clear();
            return frequencies;
        }
        frequencies.push_back(frequencyHz);
    }

    std::sort(frequencies.begin(), frequencies.end());
    if (std::isfinite(sampleRateHz) && sampleRateHz > 0.0 && frequencies.size() > 1) {
        const double minimumStepHz = (std::max)(1.0, sampleRateHz);
        for (int i = 1; i < frequencies.size(); ++i) {
            const double minimumFrequency = frequencies.at(i - 1) + minimumStepHz;
            if (frequencies.at(i) < minimumFrequency - 0.5) {
                frequencies[i] = minimumFrequency;
                if (adjusted) {
                    *adjusted = true;
                }
            }
            if (frequencies.at(i) > RF_EXPERIMENTAL_MAX_FREQUENCY) {
                if (error) {
                    *error = QStringLiteral("Standard scan list exceeds receiver range after sample-rate spacing");
                }
                frequencies.clear();
                return frequencies;
            }
        }
    }

    if (frequencies.size() < minimumPoints) {
        if (error) {
            *error = QStringLiteral("Standard scan needs at least two centers");
        }
        frequencies.clear();
    }
    return frequencies;
}

int scalePercentToSliderValue(double scalePercent) {
    const double clamped = (std::clamp)(scalePercent, MIN_SCALE_PERCENT, MAX_SCALE_PERCENT);
    return static_cast<int>(std::lround(clamped * SCALE_SLIDER_FACTOR));
}

double sliderValueToScalePercent(int sliderValue) {
    const int clamped = (std::clamp)(sliderValue,
                                     scalePercentToSliderValue(MIN_SCALE_PERCENT),
                                     scalePercentToSliderValue(MAX_SCALE_PERCENT));
    return clamped / static_cast<double>(SCALE_SLIDER_FACTOR);
}

QString formatScalePercent(double scalePercent) {
    if (std::abs(scalePercent - std::round(scalePercent)) < 0.05) {
        return QString::number(scalePercent, 'f', 0);
    }
    return QString::number(scalePercent, 'f', 1);
}

QString scaleLabelText(double scalePercent) {
    return QString("Scale: %1").arg(formatScalePercent(scalePercent));
}

float sliderValueToLevel(int sliderValue) {
    const int clamped = (std::clamp)(sliderValue, MIN_LEVEL_SLIDER_VALUE, MAX_LEVEL_SLIDER_VALUE);
    return clamped / static_cast<float>(LEVEL_SLIDER_FACTOR);
}

int levelToSliderValue(float level) {
    const int value = static_cast<int>(std::lround(level * LEVEL_SLIDER_FACTOR));
    return (std::clamp)(value, MIN_LEVEL_SLIDER_VALUE, MAX_LEVEL_SLIDER_VALUE);
}

QString levelLabelText(const QString &name, float level) {
    return QString("%1: %2").arg(name, QString::number(level, 'f', 1));
}

double clampAudioLowPassHz(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0) {
        return 0.0;
    }
    return (std::clamp)(hz, static_cast<double>(AUDIO_LOW_PASS_SLIDER_STEP_HZ),
                        static_cast<double>(AUDIO_LOW_PASS_SLIDER_STEP_HZ * AUDIO_LOW_PASS_SLIDER_MAX));
}

double clampAudioHighPassHz(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0) {
        return 0.0;
    }
    return (std::clamp)(hz, static_cast<double>(AUDIO_HIGH_PASS_SLIDER_STEP_HZ),
                        static_cast<double>(AUDIO_HIGH_PASS_SLIDER_STEP_HZ * AUDIO_HIGH_PASS_SLIDER_MAX));
}

double audioLowPassSliderValueToHz(int value) {
    return value <= 0 ? 0.0 : clampAudioLowPassHz(value * AUDIO_LOW_PASS_SLIDER_STEP_HZ);
}

int audioLowPassHzToSliderValue(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0) {
        return 0;
    }
    return (std::clamp)(static_cast<int>(std::lround(hz / AUDIO_LOW_PASS_SLIDER_STEP_HZ)),
                        1,
                        AUDIO_LOW_PASS_SLIDER_MAX);
}

double audioHighPassSliderValueToHz(int value) {
    return value <= 0 ? 0.0 : clampAudioHighPassHz(value * AUDIO_HIGH_PASS_SLIDER_STEP_HZ);
}

int audioHighPassHzToSliderValue(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0) {
        return 0;
    }
    return (std::clamp)(static_cast<int>(std::lround(hz / AUDIO_HIGH_PASS_SLIDER_STEP_HZ)),
                        1,
                        AUDIO_HIGH_PASS_SLIDER_MAX);
}

QString audioFilterFrequencyText(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0) {
        return QString();
    }
    if (hz >= 1000.0) {
        const int decimals = hz >= 10000.0 ? 0 : 1;
        return QString("%1 kHz").arg(hz / 1000.0, 0, 'f', decimals);
    }
    return QString("%1 Hz").arg(static_cast<int>(std::lround(hz)));
}

double clampHfNoiseCancelDepth(double depth) {
    if (!std::isfinite(depth)) {
        return 1.0;
    }
    return (std::clamp)(depth,
                        HF_NOISE_CANCEL_DEPTH_MIN / 100.0,
                        HF_NOISE_CANCEL_DEPTH_MAX / 100.0);
}

int hfNoiseCancelDepthToSliderValue(double depth) {
    return (std::clamp)(static_cast<int>(std::lround(clampHfNoiseCancelDepth(depth) * 100.0)),
                        HF_NOISE_CANCEL_DEPTH_MIN,
                        HF_NOISE_CANCEL_DEPTH_MAX);
}

double hfNoiseCancelSliderValueToDepth(int value) {
    return (std::clamp)(value, HF_NOISE_CANCEL_DEPTH_MIN, HF_NOISE_CANCEL_DEPTH_MAX) / 100.0;
}

QString hfNoiseCancelDepthLabelText(double depth) {
    return QString("HF cancel: %1%").arg(hfNoiseCancelDepthToSliderValue(depth));
}

double clampHfNoiseCancelRefGainDb(double gainDb) {
    if (!std::isfinite(gainDb)) {
        return 0.0;
    }
    return (std::clamp)(gainDb,
                        HF_NOISE_CANCEL_REF_GAIN_MIN / 10.0,
                        HF_NOISE_CANCEL_REF_GAIN_MAX / 10.0);
}

int hfNoiseCancelRefGainToSliderValue(double gainDb) {
    return (std::clamp)(static_cast<int>(std::lround(clampHfNoiseCancelRefGainDb(gainDb) * 10.0)),
                        HF_NOISE_CANCEL_REF_GAIN_MIN,
                        HF_NOISE_CANCEL_REF_GAIN_MAX);
}

double hfNoiseCancelSliderValueToRefGainDb(int value) {
    return (std::clamp)(value, HF_NOISE_CANCEL_REF_GAIN_MIN, HF_NOISE_CANCEL_REF_GAIN_MAX) / 10.0;
}

QString hfNoiseCancelRefGainLabelText(double gainDb) {
    return QString("Ref gain: %1 dB").arg(clampHfNoiseCancelRefGainDb(gainDb), 0, 'f', 1);
}

double clampHfNoiseCancelRefDelayNs(double delayNs) {
    if (!std::isfinite(delayNs)) {
        return 0.0;
    }
    return (std::clamp)(delayNs,
                        static_cast<double>(HF_NOISE_CANCEL_REF_DELAY_MIN_NS),
                        static_cast<double>(HF_NOISE_CANCEL_REF_DELAY_MAX_NS));
}

int hfNoiseCancelRefDelayToSliderValue(double delayNs) {
    return (std::clamp)(static_cast<int>(std::lround(clampHfNoiseCancelRefDelayNs(delayNs))),
                        HF_NOISE_CANCEL_REF_DELAY_MIN_NS,
                        HF_NOISE_CANCEL_REF_DELAY_MAX_NS);
}

double hfNoiseCancelSliderValueToRefDelayNs(int value) {
    return (std::clamp)(value,
                        HF_NOISE_CANCEL_REF_DELAY_MIN_NS,
                        HF_NOISE_CANCEL_REF_DELAY_MAX_NS);
}

QString hfNoiseCancelRefDelayLabelText(double delayNs) {
    return QString("Ref delay: %1 ns").arg(static_cast<int>(std::lround(clampHfNoiseCancelRefDelayNs(delayNs))));
}

double clampHfNoiseCancelRefTiltDb(double tiltDb) {
    if (!std::isfinite(tiltDb)) {
        return 0.0;
    }
    return (std::clamp)(tiltDb,
                        HF_NOISE_CANCEL_REF_TILT_MIN / 10.0,
                        HF_NOISE_CANCEL_REF_TILT_MAX / 10.0);
}

int hfNoiseCancelRefTiltToSliderValue(double tiltDb) {
    return (std::clamp)(static_cast<int>(std::lround(clampHfNoiseCancelRefTiltDb(tiltDb) * 10.0)),
                        HF_NOISE_CANCEL_REF_TILT_MIN,
                        HF_NOISE_CANCEL_REF_TILT_MAX);
}

double hfNoiseCancelSliderValueToRefTiltDb(int value) {
    return (std::clamp)(value, HF_NOISE_CANCEL_REF_TILT_MIN, HF_NOISE_CANCEL_REF_TILT_MAX) / 10.0;
}

QString hfNoiseCancelRefTiltLabelText(double tiltDb) {
    return QString("Ref tilt: %1 dB").arg(clampHfNoiseCancelRefTiltDb(tiltDb), 0, 'f', 1);
}

double defaultBandwidthForModulation(int modulationType) {
    switch (modulationType) {
    case MOD_NFM:
        return 12500.0;
    case MOD_SAM:
        return 6000.0;
    case MOD_USB:
    case MOD_LSB:
        return 2700.0;
    case MOD_DSB:
        return 6000.0;
    case MOD_CW:
        return 500.0;
    case MOD_WFM:
        return 200000.0;
    case MOD_FT8:
        return 3000.0;
    case MOD_RTTY:
        return 2700.0;
    case MOD_PSK:
        return 2500.0;
    case MOD_FSK:
        return 12000.0;
    case MOD_DMR:
        return 12500.0;
    case MOD_ATV:
        return 5000000.0;
    case MOD_SSTV:
        return 3000.0;
    case MOD_APT:
        return 40000.0;
    case MOD_WEFAX:
        return 3000.0;
    case MOD_LRPT:
        return 140000.0;
    case MOD_AM:
    default:
        return 10000.0;
    }
}

QString formatBandwidthHz(double bandwidth) {
    return QString::number(bandwidth, 'f', 0);
}

double recommendedFpvDemodBandwidthHz(double detectedWidthHz) {
    if (!std::isfinite(detectedWidthHz) || detectedWidthHz <= 0.0) {
        return 5000000.0;
    }
    const double paddedWidthHz = detectedWidthHz * 1.18;
    if (paddedWidthHz <= 3200000.0) {
        return 3000000.0;
    }
    if (paddedWidthHz <= 5600000.0) {
        return 5000000.0;
    }
    if (paddedWidthHz <= 8500000.0) {
        return 8000000.0;
    }
    if (paddedWidthHz <= 11500000.0) {
        return 10000000.0;
    }
    return (std::clamp)(paddedWidthHz, 10000000.0, 20000000.0);
}

double recommendedDigitalVideoBandwidthHz(double detectedWidthHz) {
    if (!std::isfinite(detectedWidthHz) || detectedWidthHz <= 0.0) {
        return 5000000.0;
    }
    const double paddedWidthHz = detectedWidthHz * 1.15;
    if (paddedWidthHz <= 2500000.0) {
        return 2000000.0;
    }
    if (paddedWidthHz <= 5600000.0) {
        return 5000000.0;
    }
    if (paddedWidthHz <= 8500000.0) {
        return 8000000.0;
    }
    if (paddedWidthHz <= 11500000.0) {
        return 10000000.0;
    }
    return (std::clamp)(paddedWidthHz, 10000000.0, 20000000.0);
}

QImage createSstvTestPattern() {
    constexpr int width = 320;
    constexpr int height = 256;
    QImage image(width, height, QImage::Format_RGB32);
    static const QRgb bars[] = {
        qRgb(255, 255, 255),
        qRgb(255, 255, 0),
        qRgb(0, 255, 255),
        qRgb(0, 255, 0),
        qRgb(255, 0, 255),
        qRgb(255, 0, 0),
        qRgb(0, 0, 255),
        qRgb(16, 16, 16),
    };

    for (int y = 0; y < height; ++y) {
        QRgb *line = reinterpret_cast<QRgb *>(image.scanLine(y));
        for (int x = 0; x < width; ++x) {
            if (y < 86) {
                line[x] = bars[(x * 8) / width];
            } else if (y < 172) {
                const int value = (x * 255) / (width - 1);
                line[x] = qRgb(value, value, value);
            } else {
                const bool checker = (((x / 16) + (y / 16)) % 2) == 0;
                const int ramp = ((x + y) * 255) / (width + height - 2);
                line[x] = checker ? qRgb(ramp, 80, 255 - ramp)
                                  : qRgb(255 - ramp, ramp, 80);
            }
        }
    }
    return image;
}

void diagnosticMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &message) {
    const QString line = QString("%1 [%2] [tid 0x%3] %4%5")
                             .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz"))
                             .arg(messageTypeName(type))
                             .arg(reinterpret_cast<quintptr>(QThread::currentThreadId()), 0, 16)
                             .arg(message)
                             .arg(context.file ? QString(" (%1:%2)").arg(context.file).arg(context.line) : QString());

    {
        QMutexLocker lock(&gLogMutex);
        if (gLogFile.isOpen()) {
            if (gLogBytesWritten >= DIAGNOSTIC_LOG_MAX_BYTES) {
                const QString currentPath = gLogFile.fileName();
                gLogFile.close();
                QFile::remove(currentPath + QStringLiteral(".1"));
                QFile::rename(currentPath, currentPath + QStringLiteral(".1"));
                gLogFile.setFileName(currentPath);
                gLogFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append);
                gLogBytesWritten = 0;
                gLogLinesUntilFlush = 0;
            }

            const QByteArray encodedLine = line.toUtf8();
            gLogFile.write(encodedLine);
            gLogFile.write("\n");
            gLogBytesWritten += encodedLine.size() + 1;
            ++gLogLinesUntilFlush;
            if (type != QtDebugMsg || gLogLinesUntilFlush >= 64) {
                gLogFile.flush();
                gLogLinesUntilFlush = 0;
            }
        }
    }

    const QByteArray localLine = line.toLocal8Bit();
    std::fprintf(stderr, "%s\n", localLine.constData());
    std::fflush(stderr);

#ifdef _WIN32
    const std::wstring debugLine = (line + "\n").toStdWString();
    OutputDebugStringW(debugLine.c_str());
#endif

    if (type == QtFatalMsg) {
        std::abort();
    }
}

void installDiagnosticLogger() {
    const QString logPath = QDir(QCoreApplication::applicationDirPath()).filePath("FobosAPP_diagnostic.log");
    if (QFileInfo(logPath).size() >= DIAGNOSTIC_LOG_MAX_BYTES) {
        QFile::remove(logPath + QStringLiteral(".1"));
        QFile::rename(logPath, logPath + QStringLiteral(".1"));
    }
    gLogFile.setFileName(logPath);
    gLogFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append);
    gLogBytesWritten = gLogFile.isOpen() ? gLogFile.size() : 0;
    gLogLinesUntilFlush = 0;
    qInstallMessageHandler(diagnosticMessageHandler);
    qDebug() << "[Log] ===== Diagnostic session started =====";
    qDebug() << "[Log] Diagnostic log path:" << QDir::toNativeSeparators(logPath)
             << "fileOpen" << gLogFile.isOpen();
    qDebug() << "[Log] Verbose diagnostic logging"
             << (fobosVerboseLoggingEnabled() ? "enabled" : "disabled");
}

#ifdef _WIN32
QString modulePathForAddress(void *address) {
    if (!address) {
        return QString();
    }

    MEMORY_BASIC_INFORMATION memoryInfo;
    ZeroMemory(&memoryInfo, sizeof(memoryInfo));
    if (!VirtualQuery(address, &memoryInfo, sizeof(memoryInfo)) || !memoryInfo.AllocationBase) {
        return QString();
    }

    wchar_t modulePath[MAX_PATH] = {};
    const DWORD length = GetModuleFileNameW(static_cast<HMODULE>(memoryInfo.AllocationBase),
                                            modulePath,
                                            MAX_PATH);
    if (length == 0) {
        return QString();
    }
    return QString::fromWCharArray(modulePath, static_cast<int>(length));
}

LONG WINAPI diagnosticUnhandledExceptionFilter(EXCEPTION_POINTERS *exceptionInfo) {
    if (!exceptionInfo || !exceptionInfo->ExceptionRecord) {
        qCritical() << "[Crash] unhandled Windows exception without exception record";
        return EXCEPTION_EXECUTE_HANDLER;
    }

    EXCEPTION_RECORD *record = exceptionInfo->ExceptionRecord;
    void *address = record->ExceptionAddress;
    const QString modulePath = modulePathForAddress(address);
    qCritical() << "[Crash] unhandled Windows exception"
                << "code" << QString("0x%1").arg(static_cast<quint32>(record->ExceptionCode), 8, 16, QChar('0'))
                << "address" << address
                << "module" << (modulePath.isEmpty() ? QString("unknown") : QDir::toNativeSeparators(modulePath))
                << "parameters" << static_cast<quint32>(record->NumberParameters);
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif

void diagnosticTerminateHandler() {
    qCritical() << "[Crash] std::terminate called";
    std::abort();
}

void installCrashLogger() {
#ifdef _WIN32
    SetUnhandledExceptionFilter(diagnosticUnhandledExceptionFilter);
#endif
    std::set_terminate(diagnosticTerminateHandler);
    qDebug() << "[Log] Crash logger installed";
}

int getFobosStandardApiInfoSafely(char *libVersion, char *driverVersion) {
#ifdef _WIN32
    __try {
        return fobos_rx_get_api_info(libVersion, driverVersion);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_get_api_info(libVersion, driverVersion);
#endif
}

int getFobosAgileApiInfoSafely(char *libVersion, char *driverVersion) {
#ifdef _WIN32
    __try {
        return fobos_sdr_get_api_info(libVersion, driverVersion);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_get_api_info(libVersion, driverVersion);
#endif
}

int getFobosStandardDeviceCountSafely() {
#ifdef _WIN32
    __try {
        return fobos_rx_get_device_count();
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
#else
    return fobos_rx_get_device_count();
#endif
}

int getFobosAgileDeviceCountSafely() {
#ifdef _WIN32
    __try {
        return fobos_sdr_get_device_count();
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
#else
    return fobos_sdr_get_device_count();
#endif
}

int openFobosDeviceSafely(fobos_dev_t **dev, uint32_t index) {
    if (!dev) {
        return FOBOS_ERR_BAD_PARAM;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_open(dev, index);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        *dev = nullptr;
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_open(dev, index);
#endif
}

int openFobosAgileDeviceSafely(fobos_sdr_dev_t **dev, uint32_t index) {
    if (!dev) {
        return FOBOS_ERR_BAD_PARAM;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_open(dev, index);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        *dev = nullptr;
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_open(dev, index);
#endif
}

int getFobosBoardInfoSafely(fobos_dev_t *dev,
                            char *hwRevision,
                            char *firmwareVersion,
                            char *manufacturer,
                            char *product,
                            char *serial) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_get_board_info(dev, hwRevision, firmwareVersion, manufacturer, product, serial);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_get_board_info(dev, hwRevision, firmwareVersion, manufacturer, product, serial);
#endif
}

int getFobosAgileBoardInfoSafely(fobos_sdr_dev_t *dev,
                                 char *hwRevision,
                                 char *firmwareVersion,
                                 char *manufacturer,
                                 char *product,
                                 char *serial) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_get_board_info(dev, hwRevision, firmwareVersion, manufacturer, product, serial);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_get_board_info(dev, hwRevision, firmwareVersion, manufacturer, product, serial);
#endif
}

int getFobosSampleRatesSafely(fobos_dev_t *dev, double *values, unsigned int *count) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_get_samplerates(dev, values, count);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_get_samplerates(dev, values, count);
#endif
}

int getFobosAgileSampleRatesSafely(fobos_sdr_dev_t *dev, double *values, unsigned int *count) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_get_samplerates(dev, values, count);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_get_samplerates(dev, values, count);
#endif
}

void logFobosApiInfo() {
    char libVersion[256] = {};
    char driverVersion[256] = {};
    const int result = getFobosStandardApiInfoSafely(libVersion, driverVersion);
    if (result == FOBOS_ERR_OK) {
        qDebug() << "[FobosLifecycle] libfobos info"
                 << "library" << libVersion
                 << "driver" << driverVersion;
    } else {
        qDebug() << "[FobosLifecycle] libfobos info unavailable"
                 << "result" << result;
    }

    char agileLibVersion[256] = {};
    char agileDriverVersion[256] = {};
    const int agileResult = getFobosAgileApiInfoSafely(agileLibVersion, agileDriverVersion);
    if (agileResult == FOBOS_ERR_OK) {
        qDebug() << "[FobosLifecycle] libfobos agile info"
                 << "library" << agileLibVersion
                 << "driver" << agileDriverVersion;
    } else {
        qDebug() << "[FobosLifecycle] libfobos agile info unavailable"
                 << "result" << agileResult;
    }
}

int closeFobosDeviceSafely(fobos_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_close(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_close(dev);
#endif
}

int resetFobosDeviceSafely(fobos_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_close(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_reset(dev);
#endif
}

int setFobosClockSourceSafely(fobos_dev_t *dev, unsigned int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_clk_source(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_clk_source(dev, value);
#endif
}

int setFobosDirectSamplingSafely(fobos_dev_t *dev, unsigned int enabled) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_direct_sampling(dev, enabled);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_direct_sampling(dev, enabled);
#endif
}

int setFobosSampleRateSafely(fobos_dev_t *dev, double value, double *actual) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_samplerate(dev, value, actual);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_samplerate(dev, value, actual);
#endif
}

int setFobosFrequencySafely(fobos_dev_t *dev, double value, double *actual) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_frequency(dev, value, actual);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_frequency(dev, value, actual);
#endif
}

int setFobosLnaGainSafely(fobos_dev_t *dev, unsigned int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_lna_gain(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_lna_gain(dev, value);
#endif
}

int setFobosVgaGainSafely(fobos_dev_t *dev, unsigned int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_vga_gain(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_vga_gain(dev, value);
#endif
}

int setFobosGpoSafely(fobos_dev_t *dev, uint8_t value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_user_gpo(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_user_gpo(dev, value);
#endif
}

int closeFobosAgileDeviceSafely(fobos_sdr_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_close(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_close(dev);
#endif
}

int resetFobosAgileDeviceSafely(fobos_sdr_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_reset(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_reset(dev);
#endif
}

int setFobosAgileClockSourceSafely(fobos_sdr_dev_t *dev, int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_clk_source(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_clk_source(dev, value);
#endif
}

int setFobosAgileDirectSamplingSafely(fobos_sdr_dev_t *dev, unsigned int enabled) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_direct_sampling(dev, enabled);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_direct_sampling(dev, enabled);
#endif
}

int setFobosAgileSampleRateSafely(fobos_sdr_dev_t *dev, double value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_samplerate(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_samplerate(dev, value);
#endif
}

int setFobosAgileFrequencySafely(fobos_sdr_dev_t *dev, double value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_frequency(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_frequency(dev, value);
#endif
}

int setFobosAgileLnaGainSafely(fobos_sdr_dev_t *dev, unsigned int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_lna_gain(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_lna_gain(dev, value);
#endif
}

int setFobosAgileVgaGainSafely(fobos_sdr_dev_t *dev, unsigned int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_vga_gain(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_vga_gain(dev, value);
#endif
}

int setFobosAgileAutoBandwidthSafely(fobos_sdr_dev_t *dev, double value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_auto_bandwidth(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_auto_bandwidth(dev, value);
#endif
}

int setFobosAgileGpoSafely(fobos_sdr_dev_t *dev, uint8_t value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_user_gpo(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_user_gpo(dev, value);
#endif
}

int startFobosAgileScanSafely(fobos_sdr_dev_t *dev, double *frequencies, unsigned int count) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
    if (!frequencies ||
        count < static_cast<unsigned int>(AGILE_SCAN_MIN_POINTS) ||
        count > static_cast<unsigned int>(AGILE_SCAN_MAX_POINTS)) {
        return FOBOS_ERR_UNSUPPORTED;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_start_scan(dev, frequencies, count);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_start_scan(dev, frequencies, count);
#endif
}

int stopFobosAgileScanSafely(fobos_sdr_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_stop_scan(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_stop_scan(dev);
#endif
}

int isFobosAgileScanningSafely(fobos_sdr_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_is_scanning(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_is_scanning(dev);
#endif
}

int getFobosAgileScanIndexSafely(fobos_sdr_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_get_scan_index(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_get_scan_index(dev);
#endif
}

int setActiveClockSourceSafely(int value) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        return setFobosAgileClockSourceSafely(agileDevice, value);
    }
    return setFobosClockSourceSafely(device, static_cast<unsigned int>(value));
}

int setActiveDirectSamplingSafely(unsigned int enabled) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        return setFobosAgileDirectSamplingSafely(agileDevice, enabled);
    }
    return setFobosDirectSamplingSafely(device, enabled);
}

int setActiveSampleRateSafely(double value, double *actual) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        if (!agileDevice) {
            return FOBOS_ERR_NOT_OPEN;
        }
        const int result = setFobosAgileSampleRateSafely(agileDevice, value);
        if (actual) {
            *actual = value;
        }
        return result;
    }
    return setFobosSampleRateSafely(device, value, actual);
}

int setActiveFrequencySafely(double value, double *actual) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        if (!agileDevice) {
            return FOBOS_ERR_NOT_OPEN;
        }
        const int result = setFobosAgileFrequencySafely(agileDevice, value);
        if (actual) {
            *actual = value;
        }
        return result;
    }
    return setFobosFrequencySafely(device, value, actual);
}

int setActiveLnaGainSafely(unsigned int value) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        return setFobosAgileLnaGainSafely(agileDevice, value);
    }
    return setFobosLnaGainSafely(device, value);
}

int setActiveVgaGainSafely(unsigned int value) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        return setFobosAgileVgaGainSafely(agileDevice, value);
    }
    return setFobosVgaGainSafely(device, value);
}

int setActiveGpoSafely(uint8_t value) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        return setFobosAgileGpoSafely(agileDevice, value);
    }
    return setFobosGpoSafely(device, value);
}

void logMemorySnapshot(const char *tag) {
    if (!fobosVerboseLoggingEnabled()) {
        return;
    }

#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS_EX counters;
    ZeroMemory(&counters, sizeof(counters));
    counters.cb = sizeof(counters);

    MEMORYSTATUSEX memoryStatus;
    ZeroMemory(&memoryStatus, sizeof(memoryStatus));
    memoryStatus.dwLength = sizeof(memoryStatus);

    const bool processOk = GetProcessMemoryInfo(GetCurrentProcess(),
                                                reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
                                                sizeof(counters)) != 0;
    const bool systemOk = GlobalMemoryStatusEx(&memoryStatus) != 0;
    if (!processOk && !systemOk) {
        qDebug() << "[Memory]" << tag << "unavailable";
        return;
    }

    const auto toMb = [](quint64 bytes) {
        return static_cast<double>(bytes) / (1024.0 * 1024.0);
    };

    qDebug() << "[Memory]" << tag
             << "workingSetMB" << (processOk ? toMb(counters.WorkingSetSize) : -1.0)
             << "privateMB" << (processOk ? toMb(counters.PrivateUsage) : -1.0)
             << "availPhysMB" << (systemOk ? toMb(memoryStatus.ullAvailPhys) : -1.0)
             << "memoryLoadPct" << (systemOk ? static_cast<int>(memoryStatus.dwMemoryLoad) : -1);
#else
    qDebug() << "[Memory]" << tag << "snapshot unavailable on this platform";
#endif
}

} // namespace

QString formatSampleRate(double sampleRate);

YourClassName::YourClassName(QWidget *parent) 
    : QMainWindow(parent), deviceOpened(false)
    {

    loadUiTranslations();

    resize(1920, 1000);
    setMinimumSize(1180, 720);

    QStringList devices = getFobosDevices();

    centralWidget = new QWidget(this);
    QScrollArea *graphScrollArea = new QScrollArea(this);
    graphScrollArea->setWidgetResizable(true);
    graphScrollArea->setWidget(centralWidget);
    setCentralWidget(graphScrollArea);

    QWidget *controlsWidget = new QWidget(this);
    QScrollArea *controlsScrollArea = new QScrollArea(this);
    controlsScrollArea->setWidgetResizable(true);
    controlsScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    controlsScrollArea->setWidget(controlsWidget);
    controlsDock = new QDockWidget("Controls", this);
    controlsDock->setObjectName("controlsDock");
    markTranslatable(controlsDock, QStringLiteral("controls"), QStringLiteral("Controls"));
    controlsDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    controlsDock->setFeatures(QDockWidget::DockWidgetMovable |
                              QDockWidget::DockWidgetFloatable);
    controlsDock->setMinimumWidth(260);
    controlsDock->setWidget(controlsScrollArea);
    addDockWidget(Qt::LeftDockWidgetArea, controlsDock);

    modulationButtonGroup = new QButtonGroup(this);
    auto addModulationRadioButton = [this](QWidget *parent,
                                           QHBoxLayout *targetLayout,
                                           const QString &label,
                                           int modulationId,
                                           const QString &toolTip) -> QRadioButton * {
        QRadioButton *button = new QRadioButton(label, parent);
        if (!toolTip.isEmpty()) {
            button->setToolTip(toolTip);
        }
        modulationButtonGroup->addButton(button, modulationId);
        targetLayout->addWidget(button);
        return button;
    };

    QWidget *digitalWidget = new QWidget(this);
    QVBoxLayout *digitalLayout = new QVBoxLayout(digitalWidget);
    QHBoxLayout *digitalHeaderLayout = new QHBoxLayout();
    QHBoxLayout *digitalModeLayout = new QHBoxLayout();
    QGridLayout *dmrLabLayout = new QGridLayout();
    digitalDecodeCheckbox = new QCheckBox("Decode", digitalWidget);
    markTranslatable(digitalDecodeCheckbox, QStringLiteral("decode"), QStringLiteral("Decode"));
    digitalDecodeCheckbox->setChecked(digitalDecodeEnabled);
    dmrLabCaptureCheckbox = new QCheckBox("Lock DMR", digitalWidget);
    markTranslatable(dmrLabCaptureCheckbox, QStringLiteral("dmr_lock"), QStringLiteral("Lock DMR"));
    dmrLabCaptureCheckbox->setToolTip(uiText(QStringLiteral("dmr_lock_tooltip"),
                                             QStringLiteral("Unchecked: auto-fill DMR metadata from the signal. Checked: decode only the selected CC/slot/TG/SRC.")));
    dmrLabColorCodeCombo = new QComboBox(digitalWidget);
    dmrLabColorCodeCombo->addItem("?", -1);
    for (int cc = 0; cc <= 15; ++cc) {
        dmrLabColorCodeCombo->addItem(QString::number(cc), cc);
    }
    dmrLabSlotCombo = new QComboBox(digitalWidget);
    dmrLabSlotCombo->addItem("?", 0);
    dmrLabSlotCombo->addItem("TS1", 1);
    dmrLabSlotCombo->addItem("TS2", 2);
    dmrLabCallTypeCombo = new QComboBox(digitalWidget);
    dmrLabCallTypeCombo->addItem("?", QStringLiteral("unknown"));
    dmrLabCallTypeCombo->addItem(uiText(QStringLiteral("dmr_group"), QStringLiteral("Group")), QStringLiteral("group"));
    dmrLabCallTypeCombo->addItem(uiText(QStringLiteral("dmr_private"), QStringLiteral("Private")), QStringLiteral("private"));
    dmrLabCallTypeCombo->addItem(uiText(QStringLiteral("dmr_all"), QStringLiteral("All")), QStringLiteral("all_call"));
    dmrBasebandRateCombo = new QComboBox(digitalWidget);
    dmrBasebandRateCombo->addItem("24 kHz", 24000);
    dmrBasebandRateCombo->addItem("48 kHz", 48000);
    dmrBasebandRateCombo->addItem("96 kHz", 96000);
    dmrBasebandRateCombo->addItem("192 kHz", 192000);
    dmrBasebandRateCombo->addItem("384 kHz", 384000);
    dmrBasebandRateCombo->setToolTip(uiText(QStringLiteral("dmr_4fsk_rate_tooltip"),
                                            QStringLiteral("DMR 4FSK discriminator output sample rate before symbol slicing.")));
    dmrAmbeLayoutCombo = new QComboBox(digitalWidget);
    dmrAmbeLayoutCombo->addItem(uiText(QStringLiteral("auto"), QStringLiteral("Auto")), DMR_AMBE_LAYOUT_AUTO);
    dmrAmbeLayoutCombo->addItem("Linear72", DMR_AMBE_LAYOUT_LINEAR72);
    dmrAmbeLayoutCombo->addItem("Split36", DMR_AMBE_LAYOUT_SPLIT36);
    dmrAmbeLayoutCombo->addItem("Dibit stripe", DMR_AMBE_LAYOUT_DIBIT_STRIPE);
    dmrAmbeLayoutCombo->addItem("Bit stripe", DMR_AMBE_LAYOUT_BIT_STRIPE);
    dmrAmbeLayoutCombo->setToolTip(uiText(QStringLiteral("dmr_ambe_layout_tooltip"),
                                          QStringLiteral("How the 216 DMR voice burst bits are mapped into three 72-bit AMBE frames.")));
    dmrManualTimingCheckbox = new QCheckBox("Timing", digitalWidget);
    markTranslatable(dmrManualTimingCheckbox, QStringLiteral("dmr_timing"), QStringLiteral("Timing"));
    dmrManualTimingCheckbox->setToolTip(uiText(QStringLiteral("dmr_timing_tooltip"),
                                               QStringLiteral("Force the DMR symbol sampling offset instead of searching around the detected timing.")));
    dmrTimingOffsetSpin = new QSpinBox(digitalWidget);
    dmrTimingOffsetSpin->setRange(-80, 80);
    dmrTimingOffsetSpin->setSingleStep(1);
    dmrTimingOffsetSpin->setValue(0);
    dmrTimingOffsetSpin->setToolTip(uiText(QStringLiteral("dmr_timing_offset_tooltip"),
                                           QStringLiteral("Manual symbol timing offset in discriminator samples.")));
    dmrSlicerRatioSpin = new QDoubleSpinBox(digitalWidget);
    dmrSlicerRatioSpin->setRange(0.45, 0.80);
    dmrSlicerRatioSpin->setDecimals(3);
    dmrSlicerRatioSpin->setSingleStep(0.005);
    dmrSlicerRatioSpin->setValue(0.625);
    dmrSlicerRatioSpin->setToolTip(uiText(QStringLiteral("dmr_slicer_tooltip"),
                                          QStringLiteral("Manual 4FSK inner/outer threshold ratio when adaptive slicing is disabled.")));
    dmrAdaptiveSlicerCheckbox = new QCheckBox("Adaptive", digitalWidget);
    markTranslatable(dmrAdaptiveSlicerCheckbox, QStringLiteral("dmr_adaptive"), QStringLiteral("Adaptive"));
    dmrAdaptiveSlicerCheckbox->setChecked(true);
    dmrAdaptiveSlicerCheckbox->setToolTip(uiText(QStringLiteral("dmr_adaptive_tooltip"),
                                                 QStringLiteral("Use local 4-level clustering for voice payloads; disable to force the manual slicer ratio.")));
    dmrLabSourceIdEdit = new QLineEdit(digitalWidget);
    dmrLabSourceIdEdit->setPlaceholderText(uiText(QStringLiteral("dmr_src_id"), QStringLiteral("Src ID")));
    dmrLabTargetIdEdit = new QLineEdit(digitalWidget);
    dmrLabTargetIdEdit->setPlaceholderText(uiText(QStringLiteral("dmr_tg_target"), QStringLiteral("TG/Target")));
    dmrLabRadioEdit = new QLineEdit(digitalWidget);
    dmrLabRadioEdit->setPlaceholderText(uiText(QStringLiteral("dmr_radio"), QStringLiteral("Radio")));
    dmrLabNotesEdit = new QLineEdit(digitalWidget);
    dmrLabNotesEdit->setPlaceholderText(uiText(QStringLiteral("note"), QStringLiteral("Note")));
    dmrLabColorCodeCombo->setMaximumWidth(64);
    dmrLabSlotCombo->setMaximumWidth(72);
    dmrLabCallTypeCombo->setMaximumWidth(92);
    dmrBasebandRateCombo->setMaximumWidth(96);
    dmrAmbeLayoutCombo->setMaximumWidth(140);
    dmrTimingOffsetSpin->setMaximumWidth(70);
    dmrSlicerRatioSpin->setMaximumWidth(84);
    QPushButton *digitalClearButton = new QPushButton("Clear", digitalWidget);
    markTranslatable(digitalClearButton, QStringLiteral("clear"), QStringLiteral("Clear"));
    digitalStatusLabel = new QLabel(uiText(QStringLiteral("digital_audio_idle"), QStringLiteral("Digital audio decoder idle")), digitalWidget);
    digitalStatusLabel->setWordWrap(false);
    digitalStatusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    digitalStatusLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    digitalTextEdit = new QPlainTextEdit(digitalWidget);
    digitalTextEdit->setReadOnly(true);
    digitalTextEdit->setMaximumBlockCount(2000);
    digitalTextEdit->setMinimumHeight(120);
    digitalTextEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    digitalTextEdit->setPlaceholderText(uiText(QStringLiteral("decoded_digital_text_placeholder"),
                                               QStringLiteral("Decoded digital-audio text will appear here.")));
    QLabel *digitalModeLabel = new QLabel("Mode:", digitalWidget);
    markTranslatable(digitalModeLabel, QStringLiteral("mode"), QStringLiteral("Mode:"));
    digitalModeLayout->addWidget(digitalModeLabel);
    addModulationRadioButton(digitalWidget, digitalModeLayout, "FT8", MOD_FT8, "FT8 weak-signal decoder");
    addModulationRadioButton(digitalWidget, digitalModeLayout, "RTTY", MOD_RTTY, "AFSK RTTY decoder");
    addModulationRadioButton(digitalWidget, digitalModeLayout, "FSK", MOD_FSK, "Frequency-shift keying decoder");
    addModulationRadioButton(digitalWidget, digitalModeLayout, "PSK", MOD_PSK, "PSK audio mode placeholder");
    addModulationRadioButton(digitalWidget, digitalModeLayout, "DMR", MOD_DMR, "DMR 4FSK sync monitor");
    digitalModeLayout->addStretch();
    dmrLabLayout->addWidget(dmrLabCaptureCheckbox, 0, 0);
    QLabel *dmrCcLabel = new QLabel("CC:", digitalWidget);
    markTranslatable(dmrCcLabel, QStringLiteral("dmr_cc"), QStringLiteral("CC:"));
    dmrLabLayout->addWidget(dmrCcLabel, 0, 1);
    dmrLabLayout->addWidget(dmrLabColorCodeCombo, 0, 2);
    QLabel *dmrSlotLabel = new QLabel("Slot:", digitalWidget);
    markTranslatable(dmrSlotLabel, QStringLiteral("dmr_slot"), QStringLiteral("Slot:"));
    dmrLabLayout->addWidget(dmrSlotLabel, 0, 3);
    dmrLabLayout->addWidget(dmrLabSlotCombo, 0, 4);
    QLabel *dmrCallLabel = new QLabel("Call:", digitalWidget);
    markTranslatable(dmrCallLabel, QStringLiteral("dmr_call"), QStringLiteral("Call:"));
    dmrLabLayout->addWidget(dmrCallLabel, 0, 5);
    dmrLabLayout->addWidget(dmrLabCallTypeCombo, 0, 6);
    dmrLabLayout->addWidget(dmrLabSourceIdEdit, 1, 0, 1, 2);
    dmrLabLayout->addWidget(dmrLabTargetIdEdit, 1, 2, 1, 2);
    dmrLabLayout->addWidget(dmrLabRadioEdit, 1, 4, 1, 3);
    QLabel *dmr4fskLabel = new QLabel("4FSK:", digitalWidget);
    markTranslatable(dmr4fskLabel, QStringLiteral("dmr_4fsk"), QStringLiteral("4FSK:"));
    dmrLabLayout->addWidget(dmr4fskLabel, 2, 0);
    dmrLabLayout->addWidget(dmrBasebandRateCombo, 2, 1);
    QLabel *dmrAmbeLayoutLabel = new QLabel("AMBE layout:", digitalWidget);
    markTranslatable(dmrAmbeLayoutLabel, QStringLiteral("dmr_ambe_layout"), QStringLiteral("AMBE layout:"));
    dmrLabLayout->addWidget(dmrAmbeLayoutLabel, 2, 2);
    dmrLabLayout->addWidget(dmrAmbeLayoutCombo, 2, 3);
    dmrLabLayout->addWidget(dmrManualTimingCheckbox, 3, 0);
    dmrLabLayout->addWidget(dmrTimingOffsetSpin, 3, 1);
    QLabel *dmrSlicerLabel = new QLabel("Slicer:", digitalWidget);
    markTranslatable(dmrSlicerLabel, QStringLiteral("dmr_slicer"), QStringLiteral("Slicer:"));
    dmrLabLayout->addWidget(dmrSlicerLabel, 3, 2);
    dmrLabLayout->addWidget(dmrSlicerRatioSpin, 3, 3);
    dmrLabLayout->addWidget(dmrAdaptiveSlicerCheckbox, 3, 4, 1, 3);
    dmrLabLayout->addWidget(dmrLabNotesEdit, 4, 0, 1, 7);
    digitalHeaderLayout->addWidget(digitalDecodeCheckbox);
    digitalHeaderLayout->addStretch();
    digitalHeaderLayout->addWidget(digitalClearButton);
    digitalLayout->addLayout(digitalHeaderLayout);
    digitalLayout->addLayout(digitalModeLayout);
    digitalLayout->addLayout(dmrLabLayout);
    digitalLayout->addWidget(digitalStatusLabel);
    digitalLayout->addWidget(digitalTextEdit);
    digitalDock = new QDockWidget("Digital Audio", this);
    digitalDock->setObjectName("digitalDock");
    markTranslatable(digitalDock, QStringLiteral("digital_audio"), QStringLiteral("Digital Audio"));
    digitalDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
    digitalDock->setFeatures(QDockWidget::DockWidgetMovable |
                             QDockWidget::DockWidgetFloatable |
                             QDockWidget::DockWidgetClosable);
    digitalDock->setWidget(digitalWidget);
    addDockWidget(Qt::RightDockWidgetArea, digitalDock);
    digitalDock->hide();

    QWidget *videoPanel = new QWidget(this);
    QVBoxLayout *videoLayout = new QVBoxLayout(videoPanel);
    QHBoxLayout *videoHeaderLayout = new QHBoxLayout();
    QHBoxLayout *videoModeLayout = new QHBoxLayout();
    videoDecodeCheckbox = new QCheckBox("Decode", videoPanel);
    markTranslatable(videoDecodeCheckbox, QStringLiteral("decode"), QStringLiteral("Decode"));
    videoDecodeCheckbox->setChecked(videoDecodeEnabled);
    videoDemodCombo = new QComboBox(videoPanel);
    videoDemodCombo->addItem("FM video", VideoProcessor::FmVideo);
    videoDemodCombo->addItem("AM video", VideoProcessor::AmVideo);
    videoStandardCombo = new QComboBox(videoPanel);
    videoStandardCombo->addItem("PAL 15.625 kHz", 15625.0);
    videoStandardCombo->addItem("NTSC 15.734 kHz", 15734.2657);
    videoInvertCheckbox = new QCheckBox("Invert", videoPanel);
    markTranslatable(videoInvertCheckbox, QStringLiteral("invert"), QStringLiteral("Invert"));
    videoHSyncCheckbox = new QCheckBox("HSync", videoPanel);
    videoHSyncCheckbox->setChecked(true);
    videoHSyncCheckbox->setToolTip("Align video lines by the darkest horizontal sync pulse");
    videoVSyncCheckbox = new QCheckBox("VSync", videoPanel);
    videoVSyncCheckbox->setChecked(true);
    videoVSyncCheckbox->setToolTip("Reset analog video frame on broad vertical sync pulses");
    videoTestPatternCheckbox = new QCheckBox("Test", videoPanel);
    markTranslatable(videoTestPatternCheckbox, QStringLiteral("test"), QStringLiteral("Test"));
    videoTestPatternCheckbox->setToolTip("Generate an internal test pattern for the selected video mode");
    videoStatusLabel = new QLabel(uiText(QStringLiteral("video_decoder_disabled"),
                                         QStringLiteral("Video decoder disabled")), videoPanel);
    videoWidget = new VideoWidget(videoPanel);
    QLabel *videoModeLabel = new QLabel("Mode:", videoPanel);
    markTranslatable(videoModeLabel, QStringLiteral("mode"), QStringLiteral("Mode:"));
    videoModeLayout->addWidget(videoModeLabel);
    addModulationRadioButton(videoPanel, videoModeLayout, "ATV", MOD_ATV, "Analog television video demodulator");
    addModulationRadioButton(videoPanel, videoModeLayout, "SSTV", MOD_SSTV, "Slow-scan television image decoder");
    addModulationRadioButton(videoPanel, videoModeLayout, "APT", MOD_APT, "NOAA APT weather satellite image decoder");
    addModulationRadioButton(videoPanel, videoModeLayout, "WEFAX", MOD_WEFAX, "HF weather fax image decoder");
    addModulationRadioButton(videoPanel, videoModeLayout, "LRPT", MOD_LRPT, "Meteor LRPT beta IQ monitor");
    videoModeLayout->addStretch();
    videoHeaderLayout->addWidget(videoDecodeCheckbox);
    videoHeaderLayout->addWidget(videoDemodCombo);
    videoHeaderLayout->addWidget(videoStandardCombo);
    videoHeaderLayout->addWidget(videoInvertCheckbox);
    videoHeaderLayout->addWidget(videoHSyncCheckbox);
    videoHeaderLayout->addWidget(videoVSyncCheckbox);
    videoHeaderLayout->addWidget(videoTestPatternCheckbox);
    videoHeaderLayout->addStretch();
    videoLayout->addLayout(videoModeLayout);
    videoLayout->addLayout(videoHeaderLayout);
    videoLayout->addWidget(videoStatusLabel);
    videoLayout->addWidget(videoWidget, 1);
    videoDock = new QDockWidget("Video", this);
    markTranslatable(videoDock, QStringLiteral("video"), QStringLiteral("Video"));
    videoDock->setObjectName("videoDock");
    videoDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
    videoDock->setFeatures(QDockWidget::DockWidgetMovable |
                           QDockWidget::DockWidgetFloatable |
                           QDockWidget::DockWidgetClosable);
    videoDock->setWidget(videoPanel);
    addDockWidget(Qt::RightDockWidgetArea, videoDock);
    videoDock->hide();

    QHBoxLayout *scaleLayout = new QHBoxLayout();
    QVBoxLayout *contrastLayout = new QVBoxLayout();
    QVBoxLayout *sensLayout = new QVBoxLayout();
    QVBoxLayout *scaleControlLayout = new QVBoxLayout();
    QVBoxLayout *levelMinLayout = new QVBoxLayout();
    QVBoxLayout *levelMaxLayout = new QVBoxLayout();
    QVBoxLayout *layout = new QVBoxLayout();
    QGridLayout *checkboxLayout = new QGridLayout();
    QVBoxLayout *graphLayout = new QVBoxLayout();
    QHBoxLayout *graphToolLayout = new QHBoxLayout();
    
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
    fftComboBox->setCurrentIndex(4);
    
    lnaGainSlider = new QSlider(Qt::Horizontal, this);
    lnaGainSlider->setRange(1, 3);
    lnaGainSlider->setValue(1);
    
    vgaGainSlider = new QSlider(Qt::Horizontal, this);
    vgaGainSlider->setRange(0, 31);
    vgaGainSlider->setValue(3);

    volumeSlider = new QSlider(Qt::Horizontal, this);
    volumeSlider->setRange(0, 200);
    volumeSlider->setValue(100);

    audioLowPassSlider = new QSlider(Qt::Horizontal, this);
    audioLowPassSlider->setRange(0, AUDIO_LOW_PASS_SLIDER_MAX);
    audioLowPassSlider->setSingleStep(1);
    audioLowPassSlider->setPageStep(10);
    audioLowPassSlider->setValue(0);

    audioHighPassSlider = new QSlider(Qt::Horizontal, this);
    audioHighPassSlider->setRange(0, AUDIO_HIGH_PASS_SLIDER_MAX);
    audioHighPassSlider->setSingleStep(1);
    audioHighPassSlider->setPageStep(4);
    audioHighPassSlider->setValue(0);

    hfNoiseCancelDepthSlider = new QSlider(Qt::Horizontal, this);
    hfNoiseCancelDepthSlider->setRange(HF_NOISE_CANCEL_DEPTH_MIN, HF_NOISE_CANCEL_DEPTH_MAX);
    hfNoiseCancelDepthSlider->setSingleStep(5);
    hfNoiseCancelDepthSlider->setPageStep(25);
    hfNoiseCancelDepthSlider->setValue(hfNoiseCancelDepthToSliderValue(pendingSettings.hfNoiseCancelDepth));

    hfNoiseCancelRefGainSlider = new QSlider(Qt::Horizontal, this);
    hfNoiseCancelRefGainSlider->setRange(HF_NOISE_CANCEL_REF_GAIN_MIN, HF_NOISE_CANCEL_REF_GAIN_MAX);
    hfNoiseCancelRefGainSlider->setSingleStep(5);
    hfNoiseCancelRefGainSlider->setPageStep(30);
    hfNoiseCancelRefGainSlider->setValue(hfNoiseCancelRefGainToSliderValue(pendingSettings.hfNoiseCancelRefGainDb));

    hfNoiseCancelRefDelaySlider = new QSlider(Qt::Horizontal, this);
    hfNoiseCancelRefDelaySlider->setRange(HF_NOISE_CANCEL_REF_DELAY_MIN_NS, HF_NOISE_CANCEL_REF_DELAY_MAX_NS);
    hfNoiseCancelRefDelaySlider->setSingleStep(10);
    hfNoiseCancelRefDelaySlider->setPageStep(100);
    hfNoiseCancelRefDelaySlider->setValue(hfNoiseCancelRefDelayToSliderValue(pendingSettings.hfNoiseCancelRefDelayNs));

    hfNoiseCancelRefTiltSlider = new QSlider(Qt::Horizontal, this);
    hfNoiseCancelRefTiltSlider->setRange(HF_NOISE_CANCEL_REF_TILT_MIN, HF_NOISE_CANCEL_REF_TILT_MAX);
    hfNoiseCancelRefTiltSlider->setSingleStep(5);
    hfNoiseCancelRefTiltSlider->setPageStep(30);
    hfNoiseCancelRefTiltSlider->setValue(hfNoiseCancelRefTiltToSliderValue(pendingSettings.hfNoiseCancelRefTiltDb));

    hfNoiseCancelFreezeCheckbox = new QCheckBox("Freeze", this);
    markTranslatable(hfNoiseCancelFreezeCheckbox, QStringLiteral("freeze"), QStringLiteral("Freeze"));
    hfNoiseCancelFreezeCheckbox->setToolTip("Hold the small adaptive trim added on top of the manual HF2 reference");

    volumeLabel = new QLabel("Volume: 100%", this);
    audioLowPassLabel = new QLabel("Audio LPF: Auto", this);
    audioHighPassLabel = new QLabel("Audio HPF: Off", this);
    hfNoiseCancelDepthLabel = new QLabel(hfNoiseCancelDepthLabelText(pendingSettings.hfNoiseCancelDepth), this);
    hfNoiseCancelRefGainLabel = new QLabel(hfNoiseCancelRefGainLabelText(pendingSettings.hfNoiseCancelRefGainDb), this);
    hfNoiseCancelRefDelayLabel = new QLabel(hfNoiseCancelRefDelayLabelText(pendingSettings.hfNoiseCancelRefDelayNs), this);
    hfNoiseCancelRefTiltLabel = new QLabel(hfNoiseCancelRefTiltLabelText(pendingSettings.hfNoiseCancelRefTiltDb), this);
    lnaGainLabel = new QLabel("LNA Gain: 1", this);
    vgaGainLabel = new QLabel("VGA Gain: 3", this);
    
    scaleSlider = new QSlider(Qt::Horizontal, this); 
    scaleSlider->setRange(scalePercentToSliderValue(MIN_SCALE_PERCENT),
                          scalePercentToSliderValue(MAX_SCALE_PERCENT));
    scaleSlider->setSingleStep(1);
    scaleSlider->setPageStep(10);
    scaleSlider->setValue(scalePercentToSliderValue(currentScale));
    scaleLabel = new QLabel(scaleLabelText(currentScale), this);
    
    audioDeviceComboBox = new QComboBox(this);
    comboBox = new QComboBox(this);
    modeBox = new QComboBox(this);
    sampleBox = new QComboBox(this);
    clkBox = new QComboBox(this);
    languageComboBox = new QComboBox(this);
    languageComboBox->addItem("English", QStringLiteral("en"));
    languageComboBox->hide();
    languageComboBox->addItem(QString::fromUtf8("Українська"), QStringLiteral("uk"));
    
    audioCheckbox = new QCheckBox("Audio", this);
    markTranslatable(audioCheckbox, QStringLiteral("audio"), QStringLiteral("Audio"));
    syncCheckbox = new QCheckBox("Sync", this);
    markTranslatable(syncCheckbox, QStringLiteral("sync"), QStringLiteral("Sync"));
    syncCheckbox->setChecked(false);
    syncCheckbox->setEnabled(false);
    syncCheckbox->setToolTip("Async reader is forced for continuous streaming tests.");
    graphCheckbox = new QCheckBox("Spectr 2", this);
    markTranslatable(graphCheckbox, QStringLiteral("spectrum2"), QStringLiteral("Spectr 2"));
    colorCheckbox = new QCheckBox("Colorful", this);
    markTranslatable(colorCheckbox, QStringLiteral("colorful"), QStringLiteral("Colorful"));
    audioCheckbox->hide();
    syncCheckbox->hide();
    graphCheckbox->hide();
    colorCheckbox->hide();

    comboBox->addItems(getFobosDevices());
    for (int i = 0; i < comboBox->count(); ++i) {
        comboBox->setItemData(i, i);
    }
    modeBox->addItem("RF", INPUT_RF);
    modeBox->addItem("HF1 + HF2", INPUT_HF_COMBINED);
    modeBox->addItem("HF1", INPUT_HF1);
    modeBox->addItem("HF2", INPUT_HF2);
    modeBox->addItem("HF1 - HF2 cancel lab", INPUT_HF_NOISE_CANCEL);
    
    clkBox->addItem("Internal", 0);
    clkBox->addItem("External", 1);

    processor = new DataProcessor( this);
    audioProcessor = new AudioProcessor(this);
    digitalDecoderThread = new QThread(this);
    digitalDecoderThread->setObjectName(QStringLiteral("DigitalDecoderThread"));
    digitalDecoder = new DigitalDecoder();
    digitalDecoder->moveToThread(digitalDecoderThread);
    connect(digitalDecoderThread, &QThread::finished, digitalDecoder, &QObject::deleteLater);
    digitalDecoderThread->start();
    videoProcessorThread = new QThread(this);
    videoProcessorThread->setObjectName(QStringLiteral("VideoProcessorThread"));
    videoProcessor = new VideoProcessor();
    videoProcessor->moveToThread(videoProcessorThread);
    connect(videoProcessorThread, &QThread::finished, videoProcessor, &QObject::deleteLater);
    videoProcessorThread->start();
    playbackManager = new PlaybackManager(this);
    recordingManager = new RecordingManager(this);
    networkController = new NetworkController(this);
    remoteAudioPlayer = new RemoteAudioPlayer(this);
    audioRelaySocket = new QUdpSocket(this);
    connect(audioRelaySocket, &QUdpSocket::readyRead, this, &YourClassName::receiveAudioRelayDatagrams);
    audioHttpServer = new QTcpServer(this);
    connect(audioHttpServer, &QTcpServer::newConnection, this, &YourClassName::acceptAudioHttpClient);
    networkSettingsDebounceTimer = new QTimer(this);
    networkSettingsDebounceTimer->setSingleShot(true);
    connect(networkSettingsDebounceTimer, &QTimer::timeout, this, [this]() {
        sendRemoteControlCommand("settings");
    });
    connectDataProcessorSignals();
    
    graphWidget = new MyGraphWidget(this);
    graphWidget->setMinimumSize(760, 180);
    graphWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    scaleWidget = new ScaleWidget(this);
    scaleWidget->setMinimumWidth(760);
    scaleWidget->setFixedHeight(50);
    scaleWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    
    waterfallWidget = new MyWaterfallWidget(this);
    waterfallWidget->setMinimumSize(760, 300);
    waterfallWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    refreshButton = new QPushButton("Refresh USB Devices", this);
    markTranslatable(refreshButton, QStringLiteral("refresh_usb"), QStringLiteral("Refresh USB Devices"));
    fobosButton = new QPushButton("Show Fobos Details", this);
    markTranslatable(fobosButton, QStringLiteral("show_fobos_details"), QStringLiteral("Show Fobos Details"));
    networkButton = new QPushButton("Network", this);
    markTranslatable(networkButton, QStringLiteral("network"), QStringLiteral("Network"));
    appSettingsButton = new QPushButton("Settings...", this);
    markTranslatable(appSettingsButton, QStringLiteral("settings"), QStringLiteral("Settings..."));
    controlsToggleButton = new QPushButton("Cfg", this);
    markTranslatable(controlsToggleButton, QStringLiteral("settings_short"), QStringLiteral("Cfg"));
    controlsToggleButton->setCheckable(true);
    controlsToggleButton->setChecked(true);
    controlsToggleButton->setFixedWidth(44);
    controlsToggleButton->setToolTip("Show, hide, or redock the settings panel");
    controlsToggleButton->installEventFilter(this);
    digitalToggleButton = new QPushButton("Digital Audio", this);
    markTranslatable(digitalToggleButton, QStringLiteral("digital_audio"), QStringLiteral("Digital Audio"));
    digitalToggleButton->setCheckable(true);
    digitalToggleButton->setMaximumWidth(120);
    digitalToggleButton->setToolTip("Show or hide the digital audio decoder panel");
    videoToggleButton = new QPushButton("Video", this);
    markTranslatable(videoToggleButton, QStringLiteral("video"), QStringLiteral("Video"));
    videoToggleButton->setCheckable(true);
    videoToggleButton->setMaximumWidth(80);
    videoToggleButton->setToolTip("Show or hide the video/image decoder panel");
    recordingModeCombo = new QComboBox(this);
    recordingModeCombo->addItem("Audio WAV", static_cast<int>(RecordingManager::Mode::AudioWav));
    recordingModeCombo->addItem("Channel IQ WAV", static_cast<int>(RecordingManager::Mode::ChannelIqWav));
    recordButton = new QPushButton("Record", this);
    markTranslatable(recordButton, QStringLiteral("record"), QStringLiteral("Record"));
    recordButton->setCheckable(true);
    recordButton->setToolTip("Start/stop recording. Hold F9 for momentary recording.");
    recordingStatusLabel = new QLabel(localizedStatusText(QStringLiteral("Recording: idle")), this);
    recordingStatusLabel->setProperty("statusRawText", QStringLiteral("Recording: idle"));
    playbackStatusLabel = new QLabel(localizedStatusText(QStringLiteral("Playback: idle")), this);
    playbackStatusLabel->setProperty("statusRawText", QStringLiteral("Playback: idle"));
    playbackFileCombo = new QComboBox(this);
    playbackFileCombo->setMinimumContentsLength(24);
    playbackRefreshButton = new QPushButton("Refresh Playback", this);
    markTranslatable(playbackRefreshButton, QStringLiteral("refresh_playback"), QStringLiteral("Refresh Playback"));
    playbackButton = new QPushButton("Play", this);
    markTranslatable(playbackButton, QStringLiteral("play"), QStringLiteral("Play"));
    playbackButton->setCheckable(true);
    startButton = new QPushButton("Start", this);
    markTranslatable(startButton, QStringLiteral("start"), QStringLiteral("Start"));
    stopButton = new QPushButton("Stop", this);
    markTranslatable(stopButton, QStringLiteral("stop"), QStringLiteral("Stop"));
    
    contrastSlider = new QSlider(Qt::Horizontal, this);
    contrastSlider->setRange(1, 15);
    contrastSlider->setValue(10);
    contrastLabel = new QLabel(QString("Contrast: %1").arg(contrast), this);
    
    sensitivitySlider = new QSlider(Qt::Horizontal, this);
    sensitivitySlider->setRange(1, 30);
    sensitivitySlider->setValue(10);
    sensitivityLabel = new QLabel(QString("Sensitivity: %1").arg(sensitivity), this);

    levelMinSlider = new QSlider(Qt::Horizontal, this);
    levelMinSlider->setRange(MIN_LEVEL_SLIDER_VALUE, MAX_LEVEL_SLIDER_VALUE);
    levelMinSlider->setValue(levelToSliderValue(displayLevelMin));
    levelMinLabel = new QLabel(levelLabelText("Min", displayLevelMin), this);

    levelMaxSlider = new QSlider(Qt::Horizontal, this);
    levelMaxSlider->setRange(MIN_LEVEL_SLIDER_VALUE, MAX_LEVEL_SLIDER_VALUE);
    levelMaxSlider->setValue(levelToSliderValue(displayLevelMax));
    levelMaxLabel = new QLabel(levelLabelText("Max", displayLevelMax), this);

    auto prepareBottomScaleControl = [](QLabel *label, QSlider *slider, int maxWidth) {
        if (label) {
            label->setAlignment(Qt::AlignCenter);
            label->setWordWrap(false);
        }
        if (slider) {
            slider->setMinimumWidth((std::min)(maxWidth, 80));
            slider->setMaximumWidth(QWIDGETSIZE_MAX);
            slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        }
    };
    prepareBottomScaleControl(contrastLabel, contrastSlider, 120);
    prepareBottomScaleControl(sensitivityLabel, sensitivitySlider, 120);
    prepareBottomScaleControl(levelMinLabel, levelMinSlider, 150);
    prepareBottomScaleControl(levelMaxLabel, levelMaxSlider, 150);
    if (scaleLabel) {
        scaleLabel->setAlignment(Qt::AlignCenter);
        scaleLabel->setWordWrap(false);
    }
    if (scaleSlider) {
        scaleSlider->setMinimumWidth(160);
        scaleSlider->setMaximumWidth(QWIDGETSIZE_MAX);
        scaleSlider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    fineTuneLabel = new QLabel(this);
    fineTuneLabel->setAlignment(Qt::AlignCenter);
    fineTuneLabel->setWordWrap(false);
    fineTuneLabel->setToolTip("Relative fine tuning for the listening frequency");
    fineTuneDial = new QDial(this);
    fineTuneDial->setRange(FINE_TUNE_DIAL_MIN, FINE_TUNE_DIAL_MAX);
    fineTuneDial->setValue(0);
    fineTuneDial->setNotchesVisible(true);
    fineTuneDial->setWrapping(false);
    fineTuneDial->setFixedSize(74, 74);
    fineTuneDial->setToolTip("Drag or wheel to nudge the listening frequency. It returns to center on release.");
    fineTuneScaleWidget = new FineTuneScaleWidget(this);
    fineTuneScaleWidget->setMinimumWidth(180);
    fineTuneScaleModeButton = new QToolButton(this);
    fineTuneScaleModeButton->setCheckable(true);
    fineTuneScaleModeButton->setAutoRaise(true);
    fineTuneScaleModeButton->setFixedSize(18, 18);
    fineTuneScaleModeButton->setToolTip("Fine tune scale mode");
    fineTuneStack = new QStackedWidget(this);
    fineTuneStack->setMinimumWidth(180);
    fineTuneStack->setFixedHeight(58);
    updateFineTuneLabel();
    
    sensLayout->addWidget(sensitivityLabel);
    sensLayout->addWidget(sensitivitySlider);
    contrastLayout->addWidget(contrastLabel);
    contrastLayout->addWidget(contrastSlider);
    scaleControlLayout->addWidget(scaleLabel);
    scaleControlLayout->addWidget(scaleSlider);
    levelMinLayout->addWidget(levelMinLabel);
    levelMinLayout->addWidget(levelMinSlider);
    levelMaxLayout->addWidget(levelMaxLabel);
    levelMaxLayout->addWidget(levelMaxSlider);
    
    QLabel *centralFrequencyLabel = new QLabel("Central Frequency:", this);
    markTranslatable(centralFrequencyLabel, QStringLiteral("central_frequency"), QStringLiteral("Central Frequency:"));
    frequencyControl = new FrequencyControl(this);
    frequencyControl->setRangeHz(0.0, RF_EXPERIMENTAL_MAX_FREQUENCY);
    frequencyControl->setValueHz(100000000.0);
    
    QLabel *listeningFrequencyLabel = new QLabel("Listening Frequency:", this);
    markTranslatable(listeningFrequencyLabel, QStringLiteral("listening_frequency"), QStringLiteral("Listening Frequency:"));
    listeningFrequencyControl = new FrequencyControl(this);
    listeningFrequencyControl->setRangeHz(0.0, RF_EXPERIMENTAL_MAX_FREQUENCY);
    listeningFrequencyControl->setValueHz(100000000.0);

    presetManagerButton = new QPushButton("Presets...", this);
    markTranslatable(presetManagerButton, QStringLiteral("presets"), QStringLiteral("Presets..."));
    
    QLabel *clockSourceLabel = new QLabel("Clock:", this);
    markTranslatable(clockSourceLabel, QStringLiteral("clock"), QStringLiteral("Clock:"));
    clockSourceLabel->setToolTip("Receiver clock source");
    QLabel *inputModeLabel = new QLabel("Mode:", this);
    markTranslatable(inputModeLabel, QStringLiteral("mode"), QStringLiteral("Mode:"));
    inputModeLabel->setToolTip("Receiver input mode");
    QLabel *sampleRateLabel = new QLabel("Sample:", this);
    markTranslatable(sampleRateLabel, QStringLiteral("sample"), QStringLiteral("Sample:"));
    sampleRateLabel->setToolTip("ADC sample rate");
    QLabel *fftLabel = new QLabel("FFT:", this);
    markTranslatable(fftLabel, QStringLiteral("fft"), QStringLiteral("FFT:"));
    fftLabel->setToolTip("Spectrum FFT length");

    QGroupBox *agileScanBox = new QGroupBox("Agile scan", this);
    markTranslatable(agileScanBox, QStringLiteral("agile_scan"), QStringLiteral("Agile scan"));
    agileScanCheckbox = new QCheckBox("Enable scan", agileScanBox);
    markTranslatable(agileScanCheckbox, QStringLiteral("enable_scan"), QStringLiteral("Enable scan"));
    agileScanPresetCombo = new QComboBox(agileScanBox);
    agileScanPresetCombo->setEditable(true);
    agileScanPresetCombo->setMinimumContentsLength(8);
    agileScanPresetCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    agileScanPresetCombo->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    agileScanRangesEdit = new QLineEdit(agileScanRangesMhz, agileScanBox);
    agileScanRangesEdit->setPlaceholderText(QStringLiteral("430-470, 600-900\\1100-1300"));
    agileScanRangesEdit->setMinimumWidth(0);
    agileScanRangesEdit->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    agileScanStepSpin = new QDoubleSpinBox(agileScanBox);
    agileScanStepSpin->setRange(AGILE_SCAN_MIN_STEP_MHZ, AGILE_SCAN_MAX_STEP_MHZ);
    agileScanStepSpin->setDecimals(4);
    agileScanStepSpin->setSingleStep(0.0125);
    agileScanStepSpin->setSuffix(QStringLiteral(" MHz"));
    agileScanStepSpin->setValue(agileScanStepMhz);
    agileScanStepSpin->setMaximumWidth(104);
    scanMeasurementCheckbox = new QCheckBox("Measure", agileScanBox);
    markTranslatable(scanMeasurementCheckbox, QStringLiteral("measure"), QStringLiteral("Measure"));
    scanMeasurementCheckbox->setToolTip(uiText(
        QStringLiteral("measure_tooltip"),
        QStringLiteral("Collect current, peak-hold, baseline and delta values for scan coverage checks")));
    scanMeasurementCheckbox->setChecked(scanMeasurementEnabled);
    scanMeasurementBinSpin = new QDoubleSpinBox(agileScanBox);
    scanMeasurementBinSpin->setRange(SCAN_MEASUREMENT_MIN_BIN_MHZ, SCAN_MEASUREMENT_MAX_BIN_MHZ);
    scanMeasurementBinSpin->setDecimals(3);
    scanMeasurementBinSpin->setSingleStep(0.1);
    scanMeasurementBinSpin->setSuffix(QStringLiteral(" MHz"));
    scanMeasurementBinSpin->setValue(scanMeasurementBinMhz);
    scanMeasurementBaselineButton = new QPushButton("BG Rec", agileScanBox);
    markTranslatable(scanMeasurementBaselineButton, QStringLiteral("bg_rec"), QStringLiteral("BG Rec"));
    scanMeasurementBaselineButton->setCheckable(true);
    scanMeasurementBaselineButton->setToolTip(uiText(
        QStringLiteral("bg_rec_tooltip"),
        QStringLiteral("Record baseline while the source under test is off")));
    scanMeasurementResetPeakButton = new QPushButton("Reset Peak", agileScanBox);
    markTranslatable(scanMeasurementResetPeakButton, QStringLiteral("reset_peak"), QStringLiteral("Reset Peak"));
    scanMeasurementResetPeakButton->setToolTip(uiText(
        QStringLiteral("reset_peak_tooltip"),
        QStringLiteral("Clear peak-hold values without clearing baseline")));
    scanMeasurementExportButton = new QPushButton("CSV", agileScanBox);
    scanMeasurementExportButton->setToolTip(uiText(QStringLiteral("csv_tooltip"),
                                                   QStringLiteral("Export scan measurement bins to CSV")));
    spurSuppressionCheckbox = new QCheckBox("Spur", agileScanBox);
    markTranslatable(spurSuppressionCheckbox, QStringLiteral("spur"), QStringLiteral("Spur"));
    spurSuppressionCheckbox->setToolTip(uiText(
        QStringLiteral("spur_tooltip"),
        QStringLiteral("Suppress calibrated internal receiver spurs in spectrum, waterfall and scan measurements")));
    spurSuppressionCheckbox->setChecked(spurSuppressionEnabled);
    spurCalibrateButton = new QPushButton("Cal", agileScanBox);
    markTranslatable(spurCalibrateButton, QStringLiteral("cal"), QStringLiteral("Cal"));
    spurCalibrateButton->setToolTip(uiText(
        QStringLiteral("cal_tooltip"),
        QStringLiteral("Calibrate stable narrow spurs with a 50 ohm load connected")));
    spurClearButton = new QPushButton("Clear", agileScanBox);
    markTranslatable(spurClearButton, QStringLiteral("clear"), QStringLiteral("Clear"));
    spurClearButton->setToolTip(uiText(QStringLiteral("clear_spur_tooltip"),
                                       QStringLiteral("Clear calibrated spur mask")));
    agileScanSavePresetButton = new QPushButton("Save", agileScanBox);
    markTranslatable(agileScanSavePresetButton, QStringLiteral("save"), QStringLiteral("Save"));
    agileScanDeletePresetButton = new QPushButton("Del", agileScanBox);
    markTranslatable(agileScanDeletePresetButton, QStringLiteral("delete_short"), QStringLiteral("Del"));
    agileScanStatusLabel = new QLabel(uiText(QStringLiteral("agile_scan_off"),
                                             QStringLiteral("Agile scan: off")), agileScanBox);
    agileScanStatusLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    agileScanStatusLabel->setMinimumWidth(0);
    agileScanStatusLabel->setWordWrap(false);
    agileScanStatusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    scanMeasurementStatusLabel = new QLabel(uiText(QStringLiteral("scan_measurement_idle"),
                                                   QStringLiteral("Scan measurement: idle")), agileScanBox);
    scanMeasurementStatusLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    scanMeasurementStatusLabel->setMinimumWidth(0);
    scanMeasurementStatusLabel->setWordWrap(false);
    scanMeasurementStatusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    spurSuppressionStatusLabel = new QLabel(uiText(QStringLiteral("spur_mask_off"),
                                                   QStringLiteral("Spur mask: off")), agileScanBox);
    spurSuppressionStatusLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    spurSuppressionStatusLabel->setMinimumWidth(0);
    spurSuppressionStatusLabel->setWordWrap(false);
    spurSuppressionStatusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    QVBoxLayout *agileScanLayout = new QVBoxLayout(agileScanBox);
    QHBoxLayout *agileScanPresetLayout = new QHBoxLayout();
    QHBoxLayout *agileScanPresetButtonLayout = new QHBoxLayout();
    QHBoxLayout *agileScanRangeLayout = new QHBoxLayout();
    QHBoxLayout *scanToolsLayout = new QHBoxLayout();
    QVBoxLayout *scanMeasurementPanelLayout = new QVBoxLayout();
    QHBoxLayout *scanMeasurementTopLayout = new QHBoxLayout();
    QHBoxLayout *scanMeasurementButtonLayout = new QHBoxLayout();
    QVBoxLayout *spurSuppressionPanelLayout = new QVBoxLayout();
    QHBoxLayout *spurSuppressionLayout = new QHBoxLayout();
    agileScanPresetLayout->addWidget(agileScanCheckbox);
    agileScanPresetLayout->addWidget(agileScanPresetCombo, 1);
    QLabel *agileScanRangeLabel = new QLabel("Ranges MHz:", agileScanBox);
    markTranslatable(agileScanRangeLabel, QStringLiteral("ranges_mhz"), QStringLiteral("Ranges MHz:"));
    QLabel *agileScanStepLabel = new QLabel("Step:", agileScanBox);
    markTranslatable(agileScanStepLabel, QStringLiteral("step"), QStringLiteral("Step:"));
    agileScanPresetButtonLayout->setContentsMargins(0, 0, 0, 0);
    agileScanPresetButtonLayout->setSpacing(4);
    agileScanPresetButtonLayout->addWidget(agileScanSavePresetButton);
    agileScanPresetButtonLayout->addWidget(agileScanDeletePresetButton);
    agileScanPresetButtonLayout->addStretch(1);
    agileScanPresetButtonLayout->addWidget(agileScanStepLabel);
    agileScanPresetButtonLayout->addWidget(agileScanStepSpin);
    agileScanRangeLayout->addWidget(agileScanRangeLabel);
    agileScanRangeLayout->addWidget(agileScanRangesEdit, 1);
    QLabel *scanMeasurementBinLabel = new QLabel("Bin:", agileScanBox);
    markTranslatable(scanMeasurementBinLabel, QStringLiteral("bin"), QStringLiteral("Bin:"));
    scanMeasurementBinSpin->setMaximumWidth(92);
    scanMeasurementBaselineButton->setMaximumWidth(68);
    scanMeasurementResetPeakButton->setMaximumWidth(86);
    scanMeasurementExportButton->setMaximumWidth(44);
    spurCalibrateButton->setMaximumWidth(44);
    spurClearButton->setMaximumWidth(54);

    scanMeasurementTopLayout->setContentsMargins(0, 0, 0, 0);
    scanMeasurementTopLayout->setSpacing(4);
    scanMeasurementTopLayout->addWidget(scanMeasurementCheckbox);
    scanMeasurementTopLayout->addWidget(scanMeasurementBinLabel);
    scanMeasurementTopLayout->addWidget(scanMeasurementBinSpin);
    scanMeasurementTopLayout->addStretch(1);
    scanMeasurementButtonLayout->setContentsMargins(0, 0, 0, 0);
    scanMeasurementButtonLayout->setSpacing(4);
    scanMeasurementButtonLayout->addWidget(scanMeasurementBaselineButton);
    scanMeasurementButtonLayout->addWidget(scanMeasurementResetPeakButton);
    scanMeasurementButtonLayout->addWidget(scanMeasurementExportButton);
    scanMeasurementButtonLayout->addStretch(1);
    scanMeasurementPanelLayout->setContentsMargins(0, 0, 0, 0);
    scanMeasurementPanelLayout->setSpacing(2);
    scanMeasurementPanelLayout->addLayout(scanMeasurementTopLayout);
    scanMeasurementPanelLayout->addLayout(scanMeasurementButtonLayout);
    scanMeasurementPanelLayout->addWidget(agileScanStatusLabel);
    scanMeasurementPanelLayout->addWidget(scanMeasurementStatusLabel);

    spurSuppressionLayout->setContentsMargins(0, 0, 0, 0);
    spurSuppressionLayout->setSpacing(4);
    spurSuppressionLayout->addWidget(spurSuppressionCheckbox);
    spurSuppressionLayout->addWidget(spurCalibrateButton);
    spurSuppressionLayout->addWidget(spurClearButton);
    spurSuppressionLayout->addStretch(1);
    spurSuppressionPanelLayout->setContentsMargins(0, 0, 0, 0);
    spurSuppressionPanelLayout->setSpacing(2);
    spurSuppressionPanelLayout->addLayout(spurSuppressionLayout);
    spurSuppressionPanelLayout->addWidget(spurSuppressionStatusLabel);

    scanToolsLayout->setContentsMargins(0, 0, 0, 0);
    scanToolsLayout->setSpacing(8);
    scanToolsLayout->addLayout(scanMeasurementPanelLayout, 3);
    scanToolsLayout->addLayout(spurSuppressionPanelLayout, 2);
    agileScanLayout->addLayout(agileScanPresetLayout);
    agileScanLayout->addLayout(agileScanPresetButtonLayout);
    agileScanLayout->addLayout(agileScanRangeLayout);
    agileScanLayout->addLayout(scanToolsLayout);

    QGroupBox *standardScanBox = new QGroupBox("Standard scan", this);
    markTranslatable(standardScanBox, QStringLiteral("standard_scan"), QStringLiteral("Standard scan"));
    standardScanCheckbox = new QCheckBox("Enable standard scan", standardScanBox);
    markTranslatable(standardScanCheckbox,
                     QStringLiteral("enable_standard_scan"),
                     QStringLiteral("Enable standard scan"));
    standardScanCheckbox->setToolTip(uiText(
        QStringLiteral("standard_scan_tooltip"),
        QStringLiteral("Slow manual retune scan by cycling through listed center frequencies")));
    standardScanCheckbox->setChecked(standardScanEnabled);
    scanListeningLockCheckbox = new QCheckBox("Lock listening frequency", standardScanBox);
    markTranslatable(scanListeningLockCheckbox,
                     QStringLiteral("scan_lock_listen"),
                     QStringLiteral("Lock listening frequency"));
    scanListeningLockCheckbox->setToolTip(uiText(
        QStringLiteral("scan_lock_listen_tooltip"),
        QStringLiteral("Keep listening frequency and marker fixed while scan retunes between centers")));
    scanListeningLockCheckbox->setChecked(scanListeningLockEnabled);
    standardScanPresetCombo = new QComboBox(standardScanBox);
    standardScanPresetCombo->setEditable(true);
    standardScanPresetCombo->setInsertPolicy(QComboBox::NoInsert);
    standardScanPresetCombo->setMinimumWidth(150);
    standardScanPresetCombo->setToolTip(uiText(
        QStringLiteral("standard_scan_preset_tooltip"),
        QStringLiteral("Saved standard-scan presets with centers, dwell and settle time")));
    standardScanSavePresetButton = new QPushButton(uiText(QStringLiteral("save"), QStringLiteral("Save")), standardScanBox);
    standardScanSavePresetButton->setMaximumWidth(58);
    standardScanDeletePresetButton = new QPushButton(uiText(QStringLiteral("delete_short"), QStringLiteral("Del")), standardScanBox);
    standardScanDeletePresetButton->setMaximumWidth(48);
    standardScanDwellSpin = new QSpinBox(standardScanBox);
    standardScanDwellSpin->setRange(STANDARD_SCAN_MIN_DWELL_MS, STANDARD_SCAN_MAX_DWELL_MS);
    standardScanDwellSpin->setSingleStep(50);
    standardScanDwellSpin->setSuffix(QStringLiteral(" ms"));
    standardScanDwellSpin->setValue(standardScanDwellMs);
    standardScanDwellSpin->setMaximumWidth(92);
    standardScanDwellSpin->setToolTip(uiText(
        QStringLiteral("standard_scan_dwell_tooltip"),
        QStringLiteral("How long to stay on each standard-scan center before retuning")));
    standardScanSettleSpin = new QSpinBox(standardScanBox);
    standardScanSettleSpin->setRange(STANDARD_SCAN_MIN_SETTLE_MS, STANDARD_SCAN_MAX_SETTLE_MS);
    standardScanSettleSpin->setSingleStep(10);
    standardScanSettleSpin->setSuffix(QStringLiteral(" ms"));
    standardScanSettleSpin->setValue(standardScanSettleMs);
    standardScanSettleSpin->setMaximumWidth(92);
    standardScanSettleSpin->setToolTip(uiText(
        QStringLiteral("standard_scan_settle_tooltip"),
        QStringLiteral("How long to discard IQ after each retune before drawing the next scan center")));
    standardScanCentersEdit = new QLineEdit(standardScanCentersMhz, standardScanBox);
    standardScanCentersEdit->setPlaceholderText(QStringLiteral("430, 480, 530"));
    standardScanCentersEdit->setToolTip(uiText(
        QStringLiteral("standard_scan_centers_tooltip"),
        QStringLiteral("Comma-separated center frequencies in MHz. Adjacent centers must be at least one sample rate apart.")));
    standardScanRangeStartEdit = new QLineEdit(standardScanRangeStartMhz, standardScanBox);
    standardScanRangeStartEdit->setPlaceholderText(QStringLiteral("100"));
    standardScanRangeStartEdit->setMaximumWidth(80);
    standardScanRangeStartEdit->setToolTip(uiText(
        QStringLiteral("standard_scan_range_start_tooltip"),
        QStringLiteral("First scan center in MHz for auto-filled center list")));
    standardScanRangeEndEdit = new QLineEdit(standardScanRangeEndMhz, standardScanBox);
    standardScanRangeEndEdit->setPlaceholderText(QStringLiteral("300"));
    standardScanRangeEndEdit->setMaximumWidth(80);
    standardScanRangeEndEdit->setToolTip(uiText(
        QStringLiteral("standard_scan_range_end_tooltip"),
        QStringLiteral("Last scan center in MHz for auto-filled center list")));
    standardScanRemoveLowerButton = new QPushButton("Rm-", standardScanBox);
    markTranslatable(standardScanRemoveLowerButton,
                     QStringLiteral("standard_scan_remove_lower"),
                     QStringLiteral("Rm-"));
    standardScanRemoveLowerButton->setToolTip(uiText(
        QStringLiteral("standard_scan_remove_lower_tooltip"),
        QStringLiteral("Remove the lowest center from the scan list")));
    standardScanRemoveLowerButton->setMaximumWidth(48);
    standardScanAddLowerButton = new QPushButton("-SR", standardScanBox);
    markTranslatable(standardScanAddLowerButton,
                     QStringLiteral("standard_scan_add_lower"),
                     QStringLiteral("-SR"));
    standardScanAddLowerButton->setToolTip(uiText(
        QStringLiteral("standard_scan_add_lower_tooltip"),
        QStringLiteral("Add one lower center using the current sample rate step")));
    standardScanAddLowerButton->setMaximumWidth(48);
    standardScanAddUpperButton = new QPushButton("+SR", standardScanBox);
    markTranslatable(standardScanAddUpperButton,
                     QStringLiteral("standard_scan_add_upper"),
                     QStringLiteral("+SR"));
    standardScanAddUpperButton->setToolTip(uiText(
        QStringLiteral("standard_scan_add_upper_tooltip"),
        QStringLiteral("Add one higher center using the current sample rate step")));
    standardScanAddUpperButton->setMaximumWidth(48);
    standardScanRemoveUpperButton = new QPushButton("Rm+", standardScanBox);
    markTranslatable(standardScanRemoveUpperButton,
                     QStringLiteral("standard_scan_remove_upper"),
                     QStringLiteral("Rm+"));
    standardScanRemoveUpperButton->setToolTip(uiText(
        QStringLiteral("standard_scan_remove_upper_tooltip"),
        QStringLiteral("Remove the highest center from the scan list")));
    standardScanRemoveUpperButton->setMaximumWidth(48);
    standardScanFillRangeButton = new QPushButton("Fill", standardScanBox);
    markTranslatable(standardScanFillRangeButton,
                     QStringLiteral("standard_scan_fill_range"),
                     QStringLiteral("Fill"));
    standardScanFillRangeButton->setToolTip(uiText(
        QStringLiteral("standard_scan_fill_range_tooltip"),
        QStringLiteral("Replace centers with a sample-rate-spaced list from start to end")));
    standardScanFillRangeButton->setMaximumWidth(56);
    standardScanStatusLabel = new QLabel(
        uiText(QStringLiteral("standard_scan_off"), QStringLiteral("Standard scan: off")),
        standardScanBox);
    standardScanStatusLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    standardScanStatusLabel->setMinimumWidth(0);
    standardScanStatusLabel->setWordWrap(false);
    standardScanStatusLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    QLabel *standardScanCentersLabel = new QLabel("Centers MHz:", standardScanBox);
    markTranslatable(standardScanCentersLabel,
                     QStringLiteral("centers_mhz"),
                     QStringLiteral("Centers MHz:"));
    standardScanCentersLabel->setAlignment(Qt::AlignCenter);
    standardScanCentersLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    QLabel *standardScanDwellLabel = new QLabel("Dwell:", standardScanBox);
    markTranslatable(standardScanDwellLabel,
                     QStringLiteral("dwell"),
                     QStringLiteral("Dwell:"));
    QLabel *standardScanSettleLabel = new QLabel("Settle:", standardScanBox);
    markTranslatable(standardScanSettleLabel,
                     QStringLiteral("settle"),
                     QStringLiteral("Settle:"));
    QLabel *standardScanRangeStartLabel = new QLabel("Start MHz:", standardScanBox);
    markTranslatable(standardScanRangeStartLabel,
                     QStringLiteral("range_start_mhz"),
                     QStringLiteral("Start MHz:"));
    QLabel *standardScanRangeEndLabel = new QLabel("End MHz:", standardScanBox);
    markTranslatable(standardScanRangeEndLabel,
                     QStringLiteral("range_end_mhz"),
                     QStringLiteral("End MHz:"));
    QVBoxLayout *standardScanLayout = new QVBoxLayout(standardScanBox);
    QHBoxLayout *standardScanTopLayout = new QHBoxLayout();
    QHBoxLayout *standardScanPresetLayout = new QHBoxLayout();
    QHBoxLayout *standardScanTimingLayout = new QHBoxLayout();
    QHBoxLayout *standardScanLockLayout = new QHBoxLayout();
    QHBoxLayout *standardScanCentersHeaderLayout = new QHBoxLayout();
    QHBoxLayout *standardScanCentersEditLayout = new QHBoxLayout();
    QHBoxLayout *standardScanRangeLayout = new QHBoxLayout();
    standardScanTopLayout->setContentsMargins(0, 0, 0, 0);
    standardScanTopLayout->setSpacing(4);
    standardScanTopLayout->addWidget(standardScanCheckbox);
    standardScanTopLayout->addStretch(1);
    standardScanPresetLayout->setContentsMargins(0, 0, 0, 0);
    standardScanPresetLayout->setSpacing(4);
    standardScanPresetLayout->addWidget(new QLabel(uiText(QStringLiteral("presets"), QStringLiteral("Presets...")), standardScanBox));
    standardScanPresetLayout->addWidget(standardScanPresetCombo, 1);
    standardScanPresetLayout->addWidget(standardScanSavePresetButton);
    standardScanPresetLayout->addWidget(standardScanDeletePresetButton);
    standardScanTimingLayout->setContentsMargins(0, 0, 0, 0);
    standardScanTimingLayout->setSpacing(4);
    standardScanTimingLayout->addWidget(standardScanDwellLabel);
    standardScanTimingLayout->addWidget(standardScanDwellSpin);
    standardScanTimingLayout->addWidget(standardScanSettleLabel);
    standardScanTimingLayout->addWidget(standardScanSettleSpin);
    standardScanTimingLayout->addStretch(1);
    standardScanLockLayout->setContentsMargins(0, 0, 0, 0);
    standardScanLockLayout->setSpacing(4);
    standardScanLockLayout->addWidget(scanListeningLockCheckbox);
    standardScanLockLayout->addStretch(1);
    standardScanCentersHeaderLayout->setContentsMargins(0, 0, 0, 0);
    standardScanCentersHeaderLayout->setSpacing(4);
    standardScanCentersHeaderLayout->addWidget(standardScanRemoveLowerButton);
    standardScanCentersHeaderLayout->addWidget(standardScanCentersLabel, 1);
    standardScanCentersHeaderLayout->addWidget(standardScanRemoveUpperButton);
    standardScanCentersEditLayout->setContentsMargins(0, 0, 0, 0);
    standardScanCentersEditLayout->setSpacing(4);
    standardScanCentersEditLayout->addWidget(standardScanAddLowerButton);
    standardScanCentersEditLayout->addWidget(standardScanCentersEdit, 1);
    standardScanCentersEditLayout->addWidget(standardScanAddUpperButton);
    standardScanRangeLayout->setContentsMargins(0, 0, 0, 0);
    standardScanRangeLayout->setSpacing(4);
    standardScanRangeLayout->addWidget(standardScanRangeStartLabel);
    standardScanRangeLayout->addWidget(standardScanRangeStartEdit);
    standardScanRangeLayout->addWidget(standardScanRangeEndLabel);
    standardScanRangeLayout->addWidget(standardScanRangeEndEdit);
    standardScanRangeLayout->addWidget(standardScanFillRangeButton);
    standardScanRangeLayout->addStretch(1);
    standardScanLayout->addLayout(standardScanTopLayout);
    standardScanLayout->addLayout(standardScanPresetLayout);
    standardScanLayout->addLayout(standardScanTimingLayout);
    standardScanLayout->addLayout(standardScanLockLayout);
    standardScanLayout->addLayout(standardScanCentersHeaderLayout);
    standardScanLayout->addLayout(standardScanCentersEditLayout);
    standardScanLayout->addLayout(standardScanRangeLayout);
    standardScanLayout->addWidget(standardScanStatusLabel);

    dmrHunterControls = new SpectrumHunterControls(
        QStringLiteral("DMR Hunter"),
        QStringLiteral("Detect narrow DMR-like 4FSK channel candidates in the current spectrum"),
        QStringLiteral(" kHz"),
        DmrHunterDetector::MinWidthKhz,
        DmrHunterDetector::MaxWidthKhz,
        dmrHunterSettings.minWidthKhz,
        dmrHunterSettings.maxWidthKhz,
        DmrHunterDetector::MinThresholdDb,
        DmrHunterDetector::MaxThresholdDb,
        dmrHunterSettings.thresholdDb,
        this);
    dmrHunterControls->addPreset(QStringLiteral("Narrow DMR example"), QStringLiteral("430-432\t0.012500"));
    dmrHunterControls->addPreset(QStringLiteral("VHF DMR 160-174"), QStringLiteral("160-174\t0.012500"));
    dmrHunterControls->addPreset(QStringLiteral("UHF DMR 400-470"), QStringLiteral("400-470\t0.012500"));
    dmrHunterControls->addPreset(QStringLiteral("DMR 2m/70cm sparse"), QStringLiteral("160-174\\400-470\t0.012500"));
    dmrHunterControls->setCandidateNavigationVisible(true);

    fpvHunterControls = new SpectrumHunterControls(
        QStringLiteral("FPV Hunter"),
        QStringLiteral("Detect wide FPV-like video carriers in the current spectrum"),
        QStringLiteral(" MHz"),
        FpvHunterDetector::MinWidthMhz,
        FpvHunterDetector::MaxWidthMhz,
        fpvHunterSettings.minWidthMhz,
        fpvHunterSettings.maxWidthMhz,
        FpvHunterDetector::MinThresholdDb,
        FpvHunterDetector::MaxThresholdDb,
        fpvHunterSettings.thresholdDb,
        this);
    fpvHunterControls->addPreset(QStringLiteral("FPV 5.8 analog"), QStringLiteral("5650-5925\t5.000000"));
    fpvHunterControls->addPreset(QStringLiteral("FPV 1.2/1.3"), QStringLiteral("1080-1360\t5.000000"));
    fpvHunterControls->addPreset(QStringLiteral("FPV 2.4"), QStringLiteral("2300-2500\t5.000000"));
    fpvHunterControls->addPreset(QStringLiteral("FPV 3.3"), QStringLiteral("3200-3500\t5.000000"));
    fpvHunterControls->addPreset(QStringLiteral("All FPV sparse"), QStringLiteral("1080-1360\\2300-2500\\3200-3500\\5650-5925\t5.000000"));
    fpvHunterControls->setCandidateNavigationVisible(true);
    fpvHunterControls->setFollowVisible(true);

    fpvHunterHistoryLabel = new QLabel(QStringLiteral("Recent FPV:"), this);
    markTranslatable(fpvHunterHistoryLabel, QStringLiteral("recent_fpv"), QStringLiteral("Recent FPV:"));
    fpvHunterHistoryCombo = new QComboBox(this);
    fpvHunterHistoryCombo->setMinimumContentsLength(10);
    fpvHunterHistoryCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    fpvHunterHistoryCombo->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    fpvHunterHistoryTuneButton = new QPushButton(QStringLiteral("Tune"), this);
    markTranslatable(fpvHunterHistoryTuneButton, QStringLiteral("tune"), QStringLiteral("Tune"));
    fpvHunterHistoryTuneButton->setMaximumWidth(58);
    fpvHunterHistoryTuneButton->setToolTip(uiText(QStringLiteral("tune_fpv_event_tooltip"),
                                                  QStringLiteral("Tune to the selected remembered FPV event")));
    fpvHunterHistoryClearButton = new QPushButton(QStringLiteral("Clear"), this);
    markTranslatable(fpvHunterHistoryClearButton, QStringLiteral("clear"), QStringLiteral("Clear"));
    fpvHunterHistoryClearButton->setMaximumWidth(58);
    fpvHunterHistoryClearButton->setToolTip(uiText(QStringLiteral("clear_fpv_events_tooltip"),
                                                   QStringLiteral("Clear remembered FPV events")));
    updateFpvHunterHistoryControls();

    digitalVideoHunterControls = new SpectrumHunterControls(
        QStringLiteral("Digital Video Hunter"),
        QStringLiteral("Detect wide digital video / OFDM-like carriers in the current spectrum"),
        QStringLiteral(" MHz"),
        DigitalVideoHunterDetector::MinWidthMhz,
        DigitalVideoHunterDetector::MaxWidthMhz,
        digitalVideoHunterSettings.minWidthMhz,
        digitalVideoHunterSettings.maxWidthMhz,
        DigitalVideoHunterDetector::MinThresholdDb,
        DigitalVideoHunterDetector::MaxThresholdDb,
        digitalVideoHunterSettings.thresholdDb,
        this);
    digitalVideoHunterControls->addPreset(QStringLiteral("DATV/DVB 1.2/1.3"), QStringLiteral("1080-1360\t2.000000"));
    digitalVideoHunterControls->addPreset(QStringLiteral("DATV/DVB 2.4"), QStringLiteral("2300-2500\t2.000000"));
    digitalVideoHunterControls->addPreset(QStringLiteral("DATV/DVB 3.3"), QStringLiteral("3200-3500\t5.000000"));
    digitalVideoHunterControls->addPreset(QStringLiteral("DATV/DVB 5.8"), QStringLiteral("4900-5925\t5.000000"));
    digitalVideoHunterControls->addPreset(QStringLiteral("Digital video sparse"), QStringLiteral("1080-1360\\2300-2500\\3200-3500\\4900-5925\t5.000000"));
    digitalVideoHunterControls->setCandidateNavigationVisible(true);

    QLabel *bandwidthLabel = new QLabel("Audio Bandwidth:", this);
    markTranslatable(bandwidthLabel, QStringLiteral("audio_bandwidth"), QStringLiteral("Audio Bandwidth:"));
    bandwidthControl = new FrequencyControl(this);
    bandwidthControl->setRangeHz(10.0, 20000000.0);
    bandwidthControl->setStepPresets({
        {"10 Hz", 10.0},
        {"100 Hz", 100.0},
        {"500 Hz", 500.0},
        {"1 kHz", 1000.0},
        {"2.5 kHz", 2500.0},
        {"5 kHz", 5000.0},
        {"10 kHz", 10000.0},
        {"25 kHz", 25000.0},
        {"100 kHz", 100000.0},
        {"1 MHz", 1000000.0},
        {"5 MHz", 5000000.0},
        {"10 MHz", 10000000.0},
    });
    ensureDefaultFrequencyPresets();
    updateFrequencyPresetControls();
    bandwidthControl->setValueHz(defaultBandwidthForModulation(MOD_AM));
   
    QStringList modulationNames = {"AM", "NFM", "SAM", "USB", "LSB", "DSB", "CW", "WFM"};
    QVector<int> modulationIds = {MOD_AM, MOD_NFM, MOD_SAM, MOD_USB, MOD_LSB, MOD_DSB,
                                  MOD_CW, MOD_WFM};
    
    QHBoxLayout* row1 = new QHBoxLayout();
    QHBoxLayout* row2 = new QHBoxLayout();
    
    QVBoxLayout *controlsToggleLayout = new QVBoxLayout();
    controlsToggleLayout->addStretch();
    controlsToggleLayout->addWidget(controlsToggleButton);

    scaleLayout->addLayout(controlsToggleLayout, 0);
    scaleLayout->addLayout(contrastLayout, 1);
    scaleLayout->addLayout(sensLayout, 1);
    scaleLayout->addLayout(scaleControlLayout, 3);
    scaleLayout->addLayout(levelMinLayout, 2);
    scaleLayout->addLayout(levelMaxLayout, 2);
    scaleLayout->addLayout(graphToolLayout, 0);

    graphToolLayout->addWidget(digitalToggleButton);
    graphToolLayout->addWidget(videoToggleButton);

    graphLayout->addWidget(graphWidget);
    graphLayout->addWidget(scaleWidget); 
    graphLayout->addWidget(waterfallWidget);
    graphLayout->addLayout(scaleLayout);
    
    for (int i = 0; i < modulationNames.size(); ++i) {
        QRadioButton* radioButton = addModulationRadioButton(controlsWidget,
                                                             i < 4 ? row1 : row2,
                                                             modulationNames[i],
                                                             modulationIds[i],
                                                             QString());
        if (i == 0) {
            radioButton->setChecked(true); 
        }
    }
    
    auto prepareCompactPanelButton = [](QPushButton *button) {
        if (!button) {
            return;
        }
        button->setMinimumWidth(0);
        button->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    };
    prepareCompactPanelButton(refreshButton);
    prepareCompactPanelButton(fobosButton);
    prepareCompactPanelButton(networkButton);
    prepareCompactPanelButton(appSettingsButton);

    QGridLayout *deviceButtonLayout = new QGridLayout();
    deviceButtonLayout->setContentsMargins(0, 0, 0, 0);
    deviceButtonLayout->setHorizontalSpacing(4);
    deviceButtonLayout->setVerticalSpacing(2);
    deviceButtonLayout->addWidget(refreshButton, 0, 0);
    deviceButtonLayout->addWidget(fobosButton, 0, 1);
    deviceButtonLayout->addWidget(networkButton, 1, 0);
    deviceButtonLayout->addWidget(appSettingsButton, 1, 1);
    deviceButtonLayout->setColumnStretch(0, 1);
    deviceButtonLayout->setColumnStretch(1, 1);

    QHBoxLayout *gainLabelLayout = new QHBoxLayout();
    lnaGainLabel->setAlignment(Qt::AlignCenter);
    vgaGainLabel->setAlignment(Qt::AlignCenter);
    gainLabelLayout->addWidget(lnaGainLabel, 1);
    gainLabelLayout->addWidget(vgaGainLabel, 4);

    QHBoxLayout *gainSliderLayout = new QHBoxLayout();
    gainSliderLayout->addWidget(lnaGainSlider, 1);
    gainSliderLayout->addWidget(vgaGainSlider, 4);

    QGridLayout *hfNoiseCancelLayout = new QGridLayout();
    hfNoiseCancelLayout->addWidget(hfNoiseCancelDepthLabel, 0, 0);
    hfNoiseCancelLayout->addWidget(hfNoiseCancelDepthSlider, 0, 1);
    hfNoiseCancelLayout->addWidget(hfNoiseCancelFreezeCheckbox, 0, 2);
    hfNoiseCancelLayout->addWidget(hfNoiseCancelRefGainLabel, 1, 0);
    hfNoiseCancelLayout->addWidget(hfNoiseCancelRefGainSlider, 1, 1, 1, 2);
    hfNoiseCancelLayout->addWidget(hfNoiseCancelRefDelayLabel, 2, 0);
    hfNoiseCancelLayout->addWidget(hfNoiseCancelRefDelaySlider, 2, 1, 1, 2);
    hfNoiseCancelLayout->addWidget(hfNoiseCancelRefTiltLabel, 3, 0);
    hfNoiseCancelLayout->addWidget(hfNoiseCancelRefTiltSlider, 3, 1, 1, 2);

    QHBoxLayout *recordingLayout = new QHBoxLayout();
    recordingLayout->addWidget(recordingModeCombo);
    recordingLayout->addWidget(recordButton);

    QHBoxLayout *playbackButtonLayout = new QHBoxLayout();
    playbackButtonLayout->addWidget(playbackRefreshButton);
    playbackButtonLayout->addWidget(playbackButton);

    auto prepareReceiverCombo = [](QComboBox *combo) {
        if (!combo) {
            return;
        }
        combo->setMinimumContentsLength(8);
        combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        combo->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    };
    prepareReceiverCombo(clkBox);
    prepareReceiverCombo(modeBox);
    prepareReceiverCombo(fftComboBox);
    prepareReceiverCombo(sampleBox);

    auto makeReceiverComboRow = [this](QLabel *label, QWidget *control) {
        QHBoxLayout *row = new QHBoxLayout();
        row->setContentsMargins(0, 0, 0, 0);
        row->setSpacing(4);
        if (label) {
            label->setMinimumWidth(48);
            row->addWidget(label, 0);
        }
        if (control) {
            row->addWidget(control, 1);
        }
        return row;
    };

    QVBoxLayout *receiverRowsLayout = new QVBoxLayout();
    receiverRowsLayout->setSpacing(2);
    receiverRowsLayout->addLayout(makeReceiverComboRow(clockSourceLabel, clkBox));
    receiverRowsLayout->addLayout(makeReceiverComboRow(inputModeLabel, modeBox));
    receiverRowsLayout->addLayout(makeReceiverComboRow(fftLabel, fftComboBox));
    receiverRowsLayout->addLayout(makeReceiverComboRow(sampleRateLabel, sampleBox));

    QVBoxLayout *fineTuneLayout = new QVBoxLayout();
    fineTuneLayout->setSpacing(2);
    QWidget *fineTuneDialPage = new QWidget(this);
    QVBoxLayout *fineTuneDialPageLayout = new QVBoxLayout(fineTuneDialPage);
    fineTuneDialPageLayout->setContentsMargins(0, 0, 0, 0);
    fineTuneDialPageLayout->addWidget(fineTuneDial, 0, Qt::AlignHCenter);
    QWidget *fineTuneScalePage = new QWidget(this);
    QVBoxLayout *fineTuneScalePageLayout = new QVBoxLayout(fineTuneScalePage);
    fineTuneScalePageLayout->setContentsMargins(0, 0, 0, 0);
    fineTuneScalePageLayout->addWidget(fineTuneScaleWidget);
    fineTuneStack->addWidget(fineTuneScalePage);
    fineTuneStack->addWidget(fineTuneDialPage);
    QHBoxLayout *fineTuneHeaderLayout = new QHBoxLayout();
    fineTuneHeaderLayout->setContentsMargins(0, 0, 0, 0);
    fineTuneHeaderLayout->setSpacing(4);
    fineTuneHeaderLayout->addWidget(fineTuneLabel, 1);
    fineTuneHeaderLayout->addWidget(fineTuneScaleModeButton, 0, Qt::AlignRight | Qt::AlignVCenter);
    fineTuneLayout->addLayout(fineTuneHeaderLayout);
    fineTuneLayout->addWidget(fineTuneStack);
    fineTuneLayout->addWidget(presetManagerButton);

    QHBoxLayout *receiverControlLayout = new QHBoxLayout();
    receiverControlLayout->addLayout(receiverRowsLayout, 1);
    receiverControlLayout->addLayout(fineTuneLayout, 0);

    QHBoxLayout *startStopLayout = new QHBoxLayout();
    startStopLayout->addWidget(startButton, 2);
    startStopLayout->addWidget(stopButton, 1);

    QHBoxLayout *centralFrequencyHeaderLayout = new QHBoxLayout();
    centralFrequencyHeaderLayout->addWidget(centralFrequencyLabel);
    centralFrequencyHeaderLayout->addStretch();

    QHBoxLayout *listeningFrequencyHeaderLayout = new QHBoxLayout();
    listeningFrequencyHeaderLayout->addWidget(listeningFrequencyLabel);
    listeningFrequencyHeaderLayout->addStretch();

    struct CollapsibleSection {
        QWidget *widget = nullptr;
        QVBoxLayout *contentLayout = nullptr;
    };
    auto createCollapsibleSection = [this](const QString &key, const QString &fallback, bool expanded) -> CollapsibleSection {
        const QString settingsKey = QStringLiteral("uiSections/%1Expanded").arg(key);
        QSettings sectionSettings(persistentSettingsFilePath(), QSettings::IniFormat);
        const bool isExpanded = sectionSettings.value(settingsKey, expanded).toBool();
        QWidget *section = new QWidget(this);
        QVBoxLayout *sectionLayout = new QVBoxLayout(section);
        sectionLayout->setContentsMargins(0, 0, 0, 0);
        sectionLayout->setSpacing(2);

        QToolButton *header = new QToolButton(section);
        header->setCheckable(true);
        header->setChecked(isExpanded);
        header->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        header->setArrowType(isExpanded ? Qt::DownArrow : Qt::RightArrow);
        header->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        markTranslatable(header, key, fallback);

        QWidget *content = new QWidget(section);
        QVBoxLayout *contentLayout = new QVBoxLayout(content);
        contentLayout->setContentsMargins(12, 0, 0, 4);
        contentLayout->setSpacing(4);
        content->setVisible(isExpanded);

        connect(header, &QToolButton::toggled, section, [header, content, settingsKey](bool checked) {
            header->setArrowType(checked ? Qt::DownArrow : Qt::RightArrow);
            content->setVisible(checked);
            QSettings sectionSettings(persistentSettingsFilePath(), QSettings::IniFormat);
            sectionSettings.setValue(settingsKey, checked);
        });

        sectionLayout->addWidget(header);
        sectionLayout->addWidget(content);
        return {section, contentLayout};
    };

    CollapsibleSection deviceSection = createCollapsibleSection(QStringLiteral("device"), QStringLiteral("Device"), true);
    deviceSection.contentLayout->addLayout(deviceButtonLayout);
    deviceSection.contentLayout->addWidget(comboBox);

    CollapsibleSection receiverSection = createCollapsibleSection(QStringLiteral("receiver"), QStringLiteral("Receiver"), true);
    receiverSection.contentLayout->addLayout(receiverControlLayout);
    receiverSection.contentLayout->addLayout(centralFrequencyHeaderLayout);
    receiverSection.contentLayout->addWidget(frequencyControl);
    receiverSection.contentLayout->addLayout(listeningFrequencyHeaderLayout);
    receiverSection.contentLayout->addWidget(listeningFrequencyControl);
    receiverSection.contentLayout->addLayout(gainLabelLayout);
    receiverSection.contentLayout->addLayout(gainSliderLayout);
    receiverSection.contentLayout->addLayout(startStopLayout);

    CollapsibleSection hfCancelSection = createCollapsibleSection(QStringLiteral("hf_cancel_lab_section"), QStringLiteral("HF cancel lab"), false);
    hfCancelSection.contentLayout->addLayout(hfNoiseCancelLayout);

    CollapsibleSection scanSection = createCollapsibleSection(QStringLiteral("scan_measurement"), QStringLiteral("Scan / measurement"), false);
    scanSection.contentLayout->addWidget(agileScanBox);
    scanSection.contentLayout->addWidget(standardScanBox);

    CollapsibleSection dmrHunterSection = createCollapsibleSection(QStringLiteral("dmr_hunter"), QStringLiteral("DMR Hunter"), false);
    dmrHunterSection.contentLayout->addWidget(dmrHunterControls);

    CollapsibleSection fpvHunterSection = createCollapsibleSection(QStringLiteral("fpv_hunter"), QStringLiteral("FPV Hunter"), false);
    fpvHunterSection.contentLayout->addWidget(fpvHunterControls);
    QHBoxLayout *fpvHistoryRow = new QHBoxLayout();
    fpvHistoryRow->setContentsMargins(0, 0, 0, 0);
    fpvHistoryRow->setSpacing(4);
    fpvHistoryRow->addWidget(fpvHunterHistoryLabel);
    fpvHistoryRow->addWidget(fpvHunterHistoryCombo, 1);
    QHBoxLayout *fpvHistoryButtonRow = new QHBoxLayout();
    fpvHistoryButtonRow->setContentsMargins(0, 0, 0, 0);
    fpvHistoryButtonRow->setSpacing(4);
    fpvHistoryButtonRow->addStretch(1);
    fpvHistoryButtonRow->addWidget(fpvHunterHistoryTuneButton);
    fpvHistoryButtonRow->addWidget(fpvHunterHistoryClearButton);
    fpvHunterSection.contentLayout->addLayout(fpvHistoryRow);
    fpvHunterSection.contentLayout->addLayout(fpvHistoryButtonRow);

    CollapsibleSection digitalVideoHunterSection = createCollapsibleSection(QStringLiteral("digital_video_hunter"), QStringLiteral("Digital Video Hunter"), false);
    digitalVideoHunterSection.contentLayout->addWidget(digitalVideoHunterControls);

    CollapsibleSection gpioSection = createCollapsibleSection(QStringLiteral("gpio"), QStringLiteral("GPIO"), false);
    gpioSection.contentLayout->addLayout(checkboxLayout);

    CollapsibleSection audioSection = createCollapsibleSection(QStringLiteral("audio_demod"), QStringLiteral("Audio / demod"), false);
    audioSection.contentLayout->addWidget(volumeLabel);
    audioSection.contentLayout->addWidget(volumeSlider);
    audioSection.contentLayout->addWidget(audioLowPassLabel);
    audioSection.contentLayout->addWidget(audioLowPassSlider);
    audioSection.contentLayout->addWidget(audioHighPassLabel);
    audioSection.contentLayout->addWidget(audioHighPassSlider);
    audioSection.contentLayout->addWidget(bandwidthLabel);
    audioSection.contentLayout->addWidget(bandwidthControl);
    audioSection.contentLayout->addLayout(row1);
    audioSection.contentLayout->addLayout(row2);
    audioSection.contentLayout->addWidget(audioDeviceComboBox);

    CollapsibleSection recordingSection = createCollapsibleSection(QStringLiteral("recording_playback"), QStringLiteral("Recording / playback"), false);
    recordingSection.contentLayout->addWidget(recordingStatusLabel);
    recordingSection.contentLayout->addLayout(recordingLayout);
    recordingSection.contentLayout->addWidget(playbackStatusLabel);
    recordingSection.contentLayout->addWidget(playbackFileCombo);
    recordingSection.contentLayout->addLayout(playbackButtonLayout);

    layout->addWidget(deviceSection.widget);
    layout->addWidget(receiverSection.widget);
    layout->addWidget(hfCancelSection.widget);
    layout->addWidget(scanSection.widget);
    layout->addWidget(dmrHunterSection.widget);
    layout->addWidget(fpvHunterSection.widget);
    layout->addWidget(digitalVideoHunterSection.widget);
    layout->addWidget(gpioSection.widget);
    layout->addWidget(audioSection.widget);
    layout->addWidget(recordingSection.widget);
    layout->addStretch(1);

    controlsWidget->setLayout(layout);
    centralWidget->setLayout(graphLayout);
    graphLayout->setStretch(0, 2);
    graphLayout->setStretch(1, 0);
    graphLayout->setStretch(2, 5);
    graphLayout->setStretch(3, 0);
    
    scaleWidget->setTuning(listeningFrequency, globalFrequency, globalBandwidth, globalModulationType);
    scaleWidget->setMarkerPosition(0.5);
    scaleWidget->setRange(minFrequency, maxFrequency);

    updateTimer = new QTimer(this);
    updateSpectrumTimerInterval();
    stopPollTimer = new QTimer(this);
    stopPollTimer->setInterval(100);
    streamWatchdogTimer = new QTimer(this);
    streamWatchdogTimer->setInterval(250);
    standardScanAdvanceTimer = new QTimer(this);
    standardScanAdvanceTimer->setTimerType(Qt::PreciseTimer);
    standardScanAdvanceTimer->setInterval(10);
    videoSnapshotTimer = new QTimer(this);
    videoSnapshotTimer->setInterval(VIDEO_SNAPSHOT_INTERVAL_MS);
    
    connect(updateTimer, &QTimer::timeout, this, &YourClassName::updateSpectrum);
    connect(stopPollTimer, &QTimer::timeout, this, &YourClassName::pollStopCompletion);
    connect(streamWatchdogTimer, &QTimer::timeout, this, &YourClassName::checkStreamStartup);
    connect(standardScanAdvanceTimer, &QTimer::timeout, this, &YourClassName::advanceStandardScanIfNeeded);
    connect(videoSnapshotTimer, &QTimer::timeout, this, &YourClassName::processVideoSnapshotFrame);
    connect(graphCheckbox, &QCheckBox::toggled, this, &YourClassName::doubleGraphEnable);
    connect(colorCheckbox, &QCheckBox::toggled, this, &YourClassName::colorGraphEnable);
    connect(syncCheckbox, &QCheckBox::toggled, this, &YourClassName::syncEnable);
    connect(audioDeviceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onAudioDeviceChanged(int)));
    connect(modulationButtonGroup, QOverload<int>::of(&QButtonGroup::idClicked), this, &YourClassName::onModulationChanged);
    connect(scaleSlider, &QSlider::valueChanged, this, &YourClassName::onScaleChanged);
    connect(frequencyControl, &FrequencyControl::valueCommitted, this, [this](double) {
        onFrequencyEntered();
    });
    connect(fftComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onfftLengthEntered()));
    connect(listeningFrequencyControl, &FrequencyControl::valueCommitted, this, [this](double) {
        onListeningFrequencyEntered();
    });
    connect(lnaGainSlider, &QSlider::valueChanged, this, &YourClassName::onLnaGainChanged);
    connect(vgaGainSlider, &QSlider::valueChanged, this, &YourClassName::onVgaGainChanged);
    connect(contrastSlider, &QSlider::valueChanged, this, &YourClassName::onContrastChanged);
    connect(sensitivitySlider, &QSlider::valueChanged, this, &YourClassName::onSensitivityChanged);
    connect(levelMinSlider, &QSlider::valueChanged, this, &YourClassName::onLevelMinChanged);
    connect(levelMaxSlider, &QSlider::valueChanged, this, &YourClassName::onLevelMaxChanged);
    connect(fineTuneDial, &QDial::valueChanged, this, &YourClassName::onFineTuneDialChanged);
    connect(fineTuneDial, &QDial::sliderReleased, this, &YourClassName::onFineTuneDialReleased);
    connect(fineTuneScaleWidget, &FineTuneScaleWidget::fineTuneDelta, this, [this](double deltaHz) {
        applyListeningFrequencyDelta(deltaHz, 60);
    });
    connect(fineTuneScaleWidget, &FineTuneScaleWidget::holdOffsetModeChanged, this, [this](bool enabled) {
        fineTuneScaleHoldMode = enabled;
        updateFineTuneScaleModeButton();
        savePersistentSettings();
    });
    connect(fineTuneScaleModeButton, &QToolButton::toggled, this, [this](bool checked) {
        fineTuneScaleHoldMode = checked;
        if (fineTuneScaleWidget) {
            fineTuneScaleWidget->setHoldOffsetMode(checked);
        }
        updateFineTuneScaleModeButton();
        savePersistentSettings();
    });
    connect(scaleSlider, &QSlider::sliderReleased, this, [this]() {
        if (isNetworkClientMode() && !isFullIqProcessingMode()) {
            scheduleRemoteSettingsCommand();
        }
    });
    connect(volumeSlider, &QSlider::valueChanged, this, [this](int value) {
        volumePercent = value;
        volumeLabel->setText(QStringLiteral("%1: %2%").arg(uiText(QStringLiteral("volume"), QStringLiteral("Volume"))).arg(value));

        const float volume = value / 100.0f;

        if (audioProcessor) {
            audioProcessor->setVolume(volume);
        }

        if (remoteAudioPlayer) {
            remoteAudioPlayer->setVolume(volume);
        }
    });
    connect(audioLowPassSlider, &QSlider::valueChanged, this, [this](int value) {
        pendingSettings.audioLowPassHz = audioLowPassSliderValueToHz(value);
        updateAudioFilterLabels();
        publishSettingsToGlobals();
        if (isNetworkClientMode()) {
            scheduleRemoteSettingsCommand();
        }
    });
    connect(audioHighPassSlider, &QSlider::valueChanged, this, [this](int value) {
        pendingSettings.audioHighPassHz = audioHighPassSliderValueToHz(value);
        updateAudioFilterLabels();
        publishSettingsToGlobals();
        if (isNetworkClientMode()) {
            scheduleRemoteSettingsCommand();
        }
    });
    auto applyHfNoiseCancelControls = [this]() {
        if (hfNoiseCancelDepthSlider) {
            pendingSettings.hfNoiseCancelDepth =
                hfNoiseCancelSliderValueToDepth(hfNoiseCancelDepthSlider->value());
        }
        if (hfNoiseCancelRefGainSlider) {
            pendingSettings.hfNoiseCancelRefGainDb =
                hfNoiseCancelSliderValueToRefGainDb(hfNoiseCancelRefGainSlider->value());
        }
        if (hfNoiseCancelRefDelaySlider) {
            pendingSettings.hfNoiseCancelRefDelayNs =
                hfNoiseCancelSliderValueToRefDelayNs(hfNoiseCancelRefDelaySlider->value());
        }
        if (hfNoiseCancelRefTiltSlider) {
            pendingSettings.hfNoiseCancelRefTiltDb =
                hfNoiseCancelSliderValueToRefTiltDb(hfNoiseCancelRefTiltSlider->value());
        }
        if (hfNoiseCancelFreezeCheckbox) {
            pendingSettings.hfNoiseCancelFreeze = hfNoiseCancelFreezeCheckbox->isChecked();
        }
        updateHfNoiseCancelControls();
        if (fftResult) {
            fftResult->resetHfNoiseCancelState();
        }
        if (audioProcessor) {
            audioProcessor->resetHfNoiseCancelState();
        }
        publishSettingsToGlobals();
        if (audioProcessor) {
            audioProcessor->configure(audioProcessorSettings());
        }
        if (processor && processor->isRunning()) {
            updateIqFrameProducerSettings();
        }
        if (isNetworkClientMode()) {
            scheduleRemoteSettingsCommand();
        }
    };
    connect(hfNoiseCancelDepthSlider, &QSlider::valueChanged, this, applyHfNoiseCancelControls);
    connect(hfNoiseCancelRefGainSlider, &QSlider::valueChanged, this, applyHfNoiseCancelControls);
    connect(hfNoiseCancelRefDelaySlider, &QSlider::valueChanged, this, applyHfNoiseCancelControls);
    connect(hfNoiseCancelRefTiltSlider, &QSlider::valueChanged, this, applyHfNoiseCancelControls);
    connect(hfNoiseCancelFreezeCheckbox, &QCheckBox::toggled, this, applyHfNoiseCancelControls);
    connect(startButton, &QPushButton::clicked, this, &YourClassName::startFobosProcessing);
    connect(stopButton, &QPushButton::clicked, this, &YourClassName::stopFobosProcessing);
    connect(modeBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onDirectSamplingChanged);
    connect(clkBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onClkChanged);
    connect(refreshButton, &QPushButton::clicked, [this]() {
        refreshFobosDeviceList(true);

        comboBox->blockSignals(true);
        comboBox->clear();
        QStringList deviceLabels;
        for (const FobosDeviceInfo &info : std::as_const(availableFobosDevices)) {
            deviceLabels << info.label;
        }
        if (deviceLabels.isEmpty()) {
            deviceLabels << uiText(QStringLiteral("no_fobos_devices_detected"),
                                   QStringLiteral("No Fobos devices detected"));
        }
        comboBox->addItems(deviceLabels);
        for (int i = 0; i < availableFobosDevices.size() && i < comboBox->count(); ++i) {
            comboBox->setItemData(i, i);
        }
        if (!availableFobosDevices.isEmpty()) {
            pendingSettings.deviceIndex = comboBox->currentData().toInt();
        }
        comboBox->blockSignals(false);
        if (sampleBox) {
            sampleBox->clear();
            populateSampleRates();
        }
    });
    connect(comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (index < 0 || isRunningOrTransitioning()) {
            return;
        }
        bool ok = false;
        const int selectedIndex = comboBox->currentData().toInt(&ok);
        pendingSettings.deviceIndex = ok ? selectedIndex : index;
        if (sampleBox) {
            sampleBox->clear();
            populateSampleRates();
        }
        qDebug() << "[FobosDevices] selected logical device" << pendingSettings.deviceIndex;
    });
    connect(fobosButton, &QPushButton::clicked, this, &YourClassName::listFobosDevices);
    connect(networkButton, &QPushButton::clicked, this, &YourClassName::openNetworkSettingsDialog);
    connect(networkController, &NetworkController::statusChanged, this, &YourClassName::onNetworkStatusChanged);
    connect(networkController, &NetworkController::channelReady, this, [this](const QString &status) {
        Q_UNUSED(status);
        if (isNetworkClientMode()) {
            networkClientReconnectPending = false;
            if (runState == RadioRunState::Running) {
                if (isClientIqProcessingMode()) {
                    startNetworkClientProcessing();
                } else {
                    stopNetworkClientProcessing();
                }
            }
            QTimer::singleShot(0, this, [this]() {
                sendRemoteControlCommand(runState == RadioRunState::Running ? "start" : "settings");
            });
        } else if (networkMode == NetworkMode::Server && runState == RadioRunState::Running) {
            applyServerLocalOutputPolicy();
        }
    });
    connect(networkController, &NetworkController::channelError, this, [this](const QString &message) {
        qDebug() << "[Network]" << message;
        if (networkMode == NetworkMode::Server && runState == RadioRunState::Running) {
            applyServerLocalOutputPolicy();
        }
        if (isNetworkClientMode() && runState == RadioRunState::Running) {
            if (isClientIqProcessingMode()) {
                stopNetworkClientProcessing();
            } else if (remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
            updateUiForRunState();
        }
        if (!isNetworkClientMode() ||
            runState != RadioRunState::Running ||
            networkClientReconnectPending) {
            return;
        }

        networkClientReconnectPending = true;
        QTimer::singleShot(1500, this, [this]() {
            networkClientReconnectPending = false;
            if (isNetworkClientMode() &&
                runState == RadioRunState::Running &&
                networkController &&
                !networkController->isControlReady()) {
                qDebug() << "[Network] reconnecting client after channel loss";
                networkController->testClientConnection(networkServerAddress, networkControlPort);
            }
        });
    });
    connect(networkController, &NetworkController::controlCommandReceived, this, &YourClassName::onNetworkControlCommandReceived);
    connect(networkController, &NetworkController::binaryCommandReceived, this, [this](const QJsonObject &command, const QByteArray &payload) {
        if (networkMode != NetworkMode::Client) {
            return;
        }
        const QString type = command.value("type").toString();
        if (type == QStringLiteral("iqbin")) {
            receiveNetworkIqFrameBinary(command, payload);
        } else if (type == QStringLiteral("spectrumbin")) {
            displayNetworkSpectrumFrameBinary(command, payload);
        }
    });
    connect(controlsToggleButton, &QPushButton::toggled, this, [this](bool checked) {
        setControlsPanelVisible(checked);
    });
    connect(controlsDock, &QDockWidget::visibilityChanged, this, [this](bool visible) {
        if (controlsToggleButton && controlsToggleButton->isChecked() != visible) {
            QSignalBlocker blocker(controlsToggleButton);
            controlsToggleButton->setChecked(visible);
        }
    });
    connect(digitalClearButton, &QPushButton::clicked, this, [this]() {
        if (digitalTextEdit) {
            digitalTextEdit->clear();
        }
    });
    connect(digitalToggleButton, &QPushButton::toggled, this, [this](bool checked) {
        if (digitalDock) {
            digitalDock->setVisible(checked);
        }
    });
    connect(digitalDock, &QDockWidget::visibilityChanged, this, [this](bool visible) {
        if (digitalToggleButton && digitalToggleButton->isChecked() != visible) {
            QSignalBlocker blocker(digitalToggleButton);
            digitalToggleButton->setChecked(visible);
        }
    });
    connect(digitalDecodeCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
        digitalDecodeEnabled = checked;
        updateDigitalDecoderMode();
    });
    auto updateDmrLabControls = [this](bool locked) {
        const QList<QWidget *> controls = {
            dmrLabColorCodeCombo,
            dmrLabSlotCombo,
            dmrLabCallTypeCombo,
            dmrLabSourceIdEdit,
            dmrLabTargetIdEdit,
            dmrLabRadioEdit,
            dmrLabNotesEdit,
            dmrBasebandRateCombo,
            dmrAmbeLayoutCombo,
            dmrManualTimingCheckbox,
            dmrTimingOffsetSpin,
            dmrSlicerRatioSpin,
            dmrAdaptiveSlicerCheckbox,
        };
        for (QWidget *control : controls) {
            if (control) {
                control->setEnabled(true);
            }
        }
        if (dmrTimingOffsetSpin && dmrManualTimingCheckbox) {
            dmrTimingOffsetSpin->setEnabled(dmrManualTimingCheckbox->isChecked());
        }
        if (dmrLabCaptureCheckbox) {
            dmrLabCaptureCheckbox->setToolTip(
                locked
                    ? QStringLiteral("Locked: DMR decode is filtered by the selected CC/slot/TG/SRC.")
                    : QStringLiteral("Auto: DMR metadata is learned from the signal and written into the fields."));
        }
    };
    connect(dmrLabCaptureCheckbox, &QCheckBox::toggled, this, updateDmrLabControls);
    const auto resetDmrLabDecoderState = [this](const char *reason) {
        pendingDmrDecoderPcm.clear();
        pendingDmrDecoderSampleRate =
            dmrBasebandRateCombo
                ? normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt())
                : normalizedDmrBasebandSampleRate(pendingSettings.dmrBasebandSampleRate);
        digitalDecoderGeneration.fetch_add(1, std::memory_order_relaxed);
        if (digitalDecoder) {
            QMetaObject::invokeMethod(digitalDecoder,
                                      [decoder = digitalDecoder]() {
                                          decoder->reset();
                                      },
                                      Qt::QueuedConnection);
        }
        qDebug() << "[DMR lab] decoder reset after control change"
                 << "reason" << reason
                 << "manualTiming" << (dmrManualTimingCheckbox && dmrManualTimingCheckbox->isChecked())
                 << "timingOffset" << (dmrTimingOffsetSpin ? dmrTimingOffsetSpin->value() : 0)
                 << "slicerRatio" << (dmrSlicerRatioSpin ? dmrSlicerRatioSpin->value() : 0.625)
                 << "adaptiveSlicer" << (!dmrAdaptiveSlicerCheckbox || dmrAdaptiveSlicerCheckbox->isChecked())
                 << "ambeLayout"
                 << (dmrAmbeLayoutCombo
                         ? dmrAmbeLayoutName(dmrAmbeLayoutCombo->currentData().toInt())
                         : dmrAmbeLayoutName(pendingSettings.dmrAmbeLayout));
    };
    connect(dmrBasebandRateCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this, resetDmrLabDecoderState]() {
                if (!dmrBasebandRateCombo) {
                    return;
                }
                const int selectedRate =
                    normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt());
                if (pendingSettings.dmrBasebandSampleRate == selectedRate) {
                    return;
                }
                const int previousRate = pendingSettings.dmrBasebandSampleRate;
                pendingSettings.dmrBasebandSampleRate = selectedRate;
                resetDmrLabDecoderState("basebandRate");
                qDebug() << "[DMR lab] 4FSK baseband sample-rate selected"
                         << "oldRate" << previousRate
                         << "newRate" << selectedRate
                         << "samplesPerSymbol"
                         << (static_cast<double>(selectedRate) / 4800.0);
                if (audioProcessor) {
                    audioProcessor->configure(audioProcessorSettings());
                }
                updateDigitalDecoderMode();
                updateIqFrameProducerSettings();
            });
    connect(dmrAmbeLayoutCombo,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this, resetDmrLabDecoderState]() {
                if (!dmrAmbeLayoutCombo) {
                    return;
                }
                const int selectedLayout =
                    normalizedDmrAmbeLayout(dmrAmbeLayoutCombo->currentData().toInt());
                if (pendingSettings.dmrAmbeLayout == selectedLayout) {
                    return;
                }
                const int previousLayout = pendingSettings.dmrAmbeLayout;
                pendingSettings.dmrAmbeLayout = selectedLayout;
                resetDmrLabDecoderState("ambeLayout");
                qDebug() << "[DMR lab] AMBE layout selected"
                         << "old" << dmrAmbeLayoutName(previousLayout)
                         << "new" << dmrAmbeLayoutName(selectedLayout);
                updateDigitalDecoderMode();
            });
    connect(dmrManualTimingCheckbox, &QCheckBox::toggled, this, [this, updateDmrLabControls, resetDmrLabDecoderState](bool) {
        pendingSettings.dmrManualTimingEnabled =
            dmrManualTimingCheckbox && dmrManualTimingCheckbox->isChecked();
        if (dmrTimingOffsetSpin) {
            pendingSettings.dmrManualTimingOffset = dmrTimingOffsetSpin->value();
        }
        updateDmrLabControls(dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked());
        resetDmrLabDecoderState("manualTiming");
        updateDigitalDecoderMode();
    });
    connect(dmrTimingOffsetSpin,
            QOverload<int>::of(&QSpinBox::valueChanged),
            this,
            [this, resetDmrLabDecoderState](int value) {
                pendingSettings.dmrManualTimingOffset = value;
                if (dmrManualTimingCheckbox && dmrManualTimingCheckbox->isChecked()) {
                    resetDmrLabDecoderState("timingOffset");
                    updateDigitalDecoderMode();
                }
            });
    connect(dmrSlicerRatioSpin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this, resetDmrLabDecoderState](double value) {
                pendingSettings.dmrSlicerRatio = value;
                resetDmrLabDecoderState("slicerRatio");
                updateDigitalDecoderMode();
            });
    connect(dmrAdaptiveSlicerCheckbox, &QCheckBox::toggled, this, [this, resetDmrLabDecoderState](bool checked) {
        pendingSettings.dmrAdaptiveSlicer = checked;
        resetDmrLabDecoderState("adaptiveSlicer");
        updateDigitalDecoderMode();
    });
    updateDmrLabControls(dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked());
    connect(digitalDecoder, &DigitalDecoder::textDecoded, this, &YourClassName::onDigitalTextDecoded);
    connect(digitalDecoder, &DigitalDecoder::statusChanged, this, &YourClassName::onDigitalDecoderStatusChanged);
    connect(digitalDecoder, &DigitalDecoder::dmrMetadataDetected, this, &YourClassName::onDmrMetadataDetected);
    connect(digitalDecoder,
            &DigitalDecoder::voicePcmReady,
            audioProcessor,
            &AudioProcessor::enqueueExternalPcm,
            Qt::QueuedConnection);
    connect(videoToggleButton, &QPushButton::toggled, this, [this](bool checked) {
        if (videoDock) {
            videoDock->setVisible(checked);
        }
    });
    connect(videoDock, &QDockWidget::visibilityChanged, this, [this](bool visible) {
        if (videoToggleButton && videoToggleButton->isChecked() != visible) {
            QSignalBlocker blocker(videoToggleButton);
            videoToggleButton->setChecked(visible);
        }
        updateVideoProcessorMode();
        updateIqFrameProducerSettings();
    });
    connect(videoDecodeCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
        videoDecodeEnabled = checked;
        updateVideoProcessorMode();
        updateIqFrameProducerSettings();
    });
    connect(videoDemodCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
        updateVideoProcessorMode();
    });
    connect(videoStandardCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
        updateVideoProcessorMode();
    });
    connect(videoInvertCheckbox, &QCheckBox::toggled, this, [this]() {
        updateVideoProcessorMode();
    });
    connect(videoHSyncCheckbox, &QCheckBox::toggled, this, [this]() {
        updateVideoProcessorMode();
    });
    connect(videoVSyncCheckbox, &QCheckBox::toggled, this, [this]() {
        updateVideoProcessorMode();
    });
    connect(videoTestPatternCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
        const bool analogVideoTest = checked && pendingSettings.modulationType == MOD_ATV;
        const bool sstvTest = checked && pendingSettings.modulationType == MOD_SSTV;
        const bool aptTest = checked && pendingSettings.modulationType == MOD_APT;
        const bool wefaxTest = checked && pendingSettings.modulationType == MOD_WEFAX;
        if (videoProcessor && videoProcessorThread) {
            QMetaObject::invokeMethod(videoProcessor,
                                      [processor = videoProcessor, analogVideoTest]() {
                                          processor->setTestPatternEnabled(analogVideoTest);
                                      },
                                      Qt::QueuedConnection);
            QMetaObject::invokeMethod(videoProcessor,
                                      [processor = videoProcessor, sstvTest]() {
                                          processor->setSstvTestPatternEnabled(sstvTest);
                                      },
                                      Qt::QueuedConnection);
            QMetaObject::invokeMethod(videoProcessor,
                                      [processor = videoProcessor, aptTest]() {
                                          processor->setAptTestPatternEnabled(aptTest);
                                      },
                                      Qt::QueuedConnection);
            QMetaObject::invokeMethod(videoProcessor,
                                      [processor = videoProcessor, wefaxTest]() {
                                          processor->setWefaxTestPatternEnabled(wefaxTest);
                                      },
                                      Qt::QueuedConnection);
        }
        updateVideoProcessorMode();
        updateIqFrameProducerSettings();
    });
    connect(videoProcessor, &VideoProcessor::frameReady, videoWidget, &VideoWidget::setFrame);
    connect(videoProcessor, &VideoProcessor::statusChanged, this, &YourClassName::onVideoStatusChanged);
    connect(recordingManager, &RecordingManager::statusChanged, this, &YourClassName::updateRecordingStatus);
    connect(recordButton, &QPushButton::toggled, this, [this](bool checked) {
        if (checked) {
            startRecording(false);
        } else {
            stopRecording(false);
        }
    });
    connect(playbackRefreshButton, &QPushButton::clicked, this, &YourClassName::refreshPlaybackFiles);
    connect(playbackButton, &QPushButton::toggled, this, [this](bool checked) {
        if (checked) {
            startPlayback();
        } else {
            stopPlayback();
        }
    });
    connect(playbackManager, &PlaybackManager::audioFrameReady, this, &YourClassName::handlePlaybackAudioFrame);
    connect(playbackManager, &PlaybackManager::iqFrameReady, this, &YourClassName::handlePlaybackIqFrame);
    connect(playbackManager, &PlaybackManager::started, this, &YourClassName::onPlaybackStarted);
    connect(playbackManager, &PlaybackManager::stopped, this, &YourClassName::onPlaybackStopped);
    connect(playbackManager, &PlaybackManager::statusChanged, this, &YourClassName::onPlaybackStatusChanged);
    connect(audioProcessor,
            &AudioProcessor::demodulatorFrameReady,
            this,
            [this](const QByteArray &pcmData) {
                if (pendingSettings.modulationType != MOD_DMR) {
                    processDigitalAudioFrame(pcmData);
                }
                processSstvAudioFrame(pcmData);
                processAptAudioFrame(pcmData);
                processWefaxAudioFrame(pcmData);
            },
            Qt::QueuedConnection);
    connect(audioProcessor,
            &AudioProcessor::dmrBasebandFrameReady,
            this,
            [this](const QByteArray &pcmData, int sampleRate) {
                if (pendingSettings.modulationType == MOD_DMR) {
                    processDigitalAudioFrame(pcmData, sampleRate);
                }
            },
            Qt::QueuedConnection);
    connect(audioProcessor,
            &AudioProcessor::audioFrameReady,
            this,
            [this](const QByteArray &pcmData) {
                if (recordingManager &&
                    recordingManager->isRecording() &&
                    recordingManager->mode() == RecordingManager::Mode::AudioWav) {
                    recordingManager->appendAudioFrame(pcmData);
                }
                sendNetworkAudioFrame(pcmData);
                sendAudioRelayFrame(pcmData);
                sendAudioHttpFrame(pcmData);
#if !defined(_WIN32) && defined(FOBOSAPP_HAS_QT_AUDIO)
                const bool suppressServerLocalOutput =
                    networkMode == NetworkMode::Server &&
                    serverDisableLocalVisualAudio &&
                    networkController &&
                    networkController->isControlReady();
                const bool localQtPlayback =
                    remoteAudioPlayer &&
                    audioCheckbox &&
                    audioCheckbox->isChecked() &&
                    !suppressServerLocalOutput &&
                    (networkMode != NetworkMode::Client || isClientIqProcessingMode());
                if (localQtPlayback) {
                    remoteAudioPlayer->playPcmFrame(pcmData);
                }
#endif
            },
            Qt::QueuedConnection);
    connect(audioCheckbox, &QCheckBox::toggled,
            this, &YourClassName::onAudioEnabledChanged);
    connect(languageComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (!languageComboBox || index < 0) {
            return;
        }
        const QString nextLanguage = languageComboBox->itemData(index).toString();
        if (nextLanguage.isEmpty() || nextLanguage == uiLanguage) {
            return;
        }
        uiLanguage = nextLanguage == QStringLiteral("uk") ? QStringLiteral("uk") : QStringLiteral("en");
        applyUiLanguage();
        savePersistentSettings();
    });
    connect(sampleBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onSampleRateChanged);
    auto applyScanUiChange = [this](bool applyNow) {
        const bool previousAgileScanEnabled = agileScanEnabled;
        const bool previousStandardScanEnabled = standardScanEnabled;
        refreshSettingsFromUi();
        updateAgileScanControls();
        publishSettingsToGlobals();
        if (isNetworkClientMode()) {
            scheduleRemoteSettingsCommand();
            return;
        }
        if (applyNow && !isIdle() && hasActiveFobosDevice()) {
            if (activeFobosApiKind == FobosApiKind::Agile &&
                previousAgileScanEnabled != agileScanEnabled) {
                restartStreamForHardwareChange();
            } else if (activeFobosApiKind == FobosApiKind::Standard &&
                       previousStandardScanEnabled != standardScanEnabled) {
                applyStandardScanSettings(false);
            } else {
                applyAgileScanSettings(false);
                applyStandardScanSettings(false);
            }
        }
    };
    connect(agileScanCheckbox, &QCheckBox::toggled, this, [this, applyScanUiChange](bool checked) {
        if (checked && standardScanCheckbox && standardScanCheckbox->isChecked()) {
            QSignalBlocker blocker(standardScanCheckbox);
            standardScanCheckbox->setChecked(false);
            standardScanEnabled = false;
            resetStandardScanState(true);
        }
        applyScanUiChange(true);
    });
    connect(agileScanRangesEdit, &QLineEdit::editingFinished, this, [applyScanUiChange]() {
        applyScanUiChange(true);
    });
    connect(agileScanStepSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [applyScanUiChange](double) {
        applyScanUiChange(false);
    });
    connect(agileScanStepSpin, &QDoubleSpinBox::editingFinished, this, [applyScanUiChange]() {
        applyScanUiChange(true);
    });
    connect(agileScanPresetCombo, QOverload<int>::of(&QComboBox::activated), this, [this, applyScanUiChange](int index) {
        const QString name = agileScanPresetCombo ? agileScanPresetCombo->itemData(index).toString() : QString();
        if (name.isEmpty() || !agileScanPresets.contains(name)) {
            return;
        }
        const QString spec = agileScanPresets.value(name);
        if (agileScanRangesEdit) {
            agileScanRangesEdit->setText(agileScanPresetRanges(spec, agileScanRangesMhz));
        }
        if (agileScanStepSpin) {
            agileScanStepSpin->setValue(agileScanPresetStepMhz(spec, agileScanStepMhz));
        }
        applyScanUiChange(true);
    });
    connect(standardScanCheckbox, &QCheckBox::toggled, this, [this, applyScanUiChange](bool checked) {
        if (checked && agileScanCheckbox && agileScanCheckbox->isChecked()) {
            QSignalBlocker blocker(agileScanCheckbox);
            agileScanCheckbox->setChecked(false);
            agileScanEnabled = false;
        }
        applyScanUiChange(true);
    });
    connect(scanListeningLockCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
        scanListeningLockEnabled = checked;
        settingRange();
        if (persistentSettingsReady) {
            savePersistentSettings();
        }
    });
    connect(standardScanPresetCombo, QOverload<int>::of(&QComboBox::activated), this, [this, applyScanUiChange](int index) {
        const QString name = standardScanPresetCombo ? standardScanPresetCombo->itemData(index).toString() : QString();
        if (name.isEmpty() || !standardScanPresets.contains(name)) {
            return;
        }
        const QString spec = standardScanPresets.value(name);
        standardScanCentersMhz = standardScanPresetCenters(spec, standardScanCentersMhz);
        standardScanDwellMs = standardScanPresetDwellMs(spec, standardScanDwellMs);
        standardScanSettleMs = standardScanPresetSettleMs(spec, standardScanSettleMs);
        if (standardScanCentersEdit) {
            standardScanCentersEdit->setText(standardScanCentersMhz);
        }
        if (standardScanDwellSpin) {
            standardScanDwellSpin->setValue(standardScanDwellMs);
        }
        if (standardScanSettleSpin) {
            standardScanSettleSpin->setValue(standardScanSettleMs);
        }
        applyScanUiChange(true);
    });
    connect(standardScanSavePresetButton, &QPushButton::clicked, this, [this]() {
        refreshSettingsFromUi();
        QString name = standardScanPresetCombo ? standardScanPresetCombo->currentText().trimmed() : QString();
        if (name.isEmpty()) {
            name = QStringLiteral("Standard scan %1").arg(standardScanPresets.size() + 1);
        }
        QString error;
        parseStandardScanCentersMhz(standardScanCentersMhz,
                                    pendingSettings.sampleRate,
                                    AGILE_SCAN_MIN_POINTS,
                                    &error,
                                    nullptr);
        if (!error.isEmpty()) {
            if (standardScanStatusLabel) {
                standardScanStatusLabel->setText(error);
            }
            return;
        }
        standardScanPresets[name] =
            standardScanPresetSpec(standardScanCentersMhz, standardScanDwellMs, standardScanSettleMs);
        savePersistentSettings();
        updateAgileScanControls();
    });
    connect(standardScanDeletePresetButton, &QPushButton::clicked, this, [this]() {
        const QString name = standardScanPresetCombo ? standardScanPresetCombo->currentText().trimmed() : QString();
        if (name.isEmpty()) {
            return;
        }
        standardScanPresets.remove(name);
        savePersistentSettings();
        updateAgileScanControls();
    });
    connect(standardScanCentersEdit, &QLineEdit::editingFinished, this, [applyScanUiChange]() {
        applyScanUiChange(true);
    });
    auto addStandardScanCenter = [this, applyScanUiChange](int direction) {
        refreshSettingsFromUi();
        const double stepHz = pendingSettings.sampleRate;
        if (!std::isfinite(stepHz) || stepHz <= 0.0) {
            if (standardScanStatusLabel) {
                standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_bad_sample_rate"),
                                                        QStringLiteral("Standard scan: bad sample rate")));
            }
            return;
        }

        QString error;
        bool adjusted = false;
        QVector<double> centers = parseStandardScanCentersMhz(standardScanCentersMhz,
                                                              stepHz,
                                                              0,
                                                              &error,
                                                              &adjusted);
        if (!error.isEmpty()) {
            if (standardScanStatusLabel) {
                standardScanStatusLabel->setText(error);
            }
            return;
        }
        if (centers.isEmpty()) {
            const double seed = pendingSettings.inputMode == INPUT_RF && std::isfinite(pendingSettings.centerFrequency)
                                    ? pendingSettings.centerFrequency
                                    : RF_MIN_CENTER_FREQUENCY;
            centers.push_back(seed);
        }

        std::sort(centers.begin(), centers.end());
        const double nextCenter = direction < 0 ? centers.first() - stepHz
                                                : centers.last() + stepHz;
        if (!std::isfinite(nextCenter) ||
            nextCenter < RF_MIN_CENTER_FREQUENCY ||
            nextCenter > RF_EXPERIMENTAL_MAX_FREQUENCY) {
            if (standardScanStatusLabel) {
                standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_add_out_of_range"),
                                                        QStringLiteral("Standard scan: new center is out of range")));
            }
            return;
        }
        centers.push_back(nextCenter);
        std::sort(centers.begin(), centers.end());
        standardScanCentersMhz = formatMhzList(centers);
        if (standardScanCentersEdit) {
            QSignalBlocker blocker(standardScanCentersEdit);
            standardScanCentersEdit->setText(standardScanCentersMhz);
        }
        applyScanUiChange(true);
    };
    connect(standardScanAddLowerButton, &QPushButton::clicked, this, [addStandardScanCenter]() {
        addStandardScanCenter(-1);
    });
    connect(standardScanAddUpperButton, &QPushButton::clicked, this, [addStandardScanCenter]() {
        addStandardScanCenter(1);
    });
    auto removeStandardScanCenter = [this, applyScanUiChange](int direction) {
        refreshSettingsFromUi();
        QString error;
        bool adjusted = false;
        QVector<double> centers = parseStandardScanCentersMhz(standardScanCentersMhz,
                                                              pendingSettings.sampleRate,
                                                              0,
                                                              &error,
                                                              &adjusted);
        if (!error.isEmpty()) {
            if (standardScanStatusLabel) {
                standardScanStatusLabel->setText(error);
            }
            return;
        }
        if (centers.isEmpty()) {
            if (standardScanStatusLabel) {
                standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_empty_list"),
                                                        QStringLiteral("Standard scan centers are empty")));
            }
            return;
        }
        std::sort(centers.begin(), centers.end());
        if (direction < 0) {
            centers.removeFirst();
        } else {
            centers.removeLast();
        }
        standardScanCentersMhz = formatMhzList(centers);
        if (standardScanCentersEdit) {
            QSignalBlocker blocker(standardScanCentersEdit);
            standardScanCentersEdit->setText(standardScanCentersMhz);
        }
        applyScanUiChange(true);
    };
    connect(standardScanRemoveLowerButton, &QPushButton::clicked, this, [removeStandardScanCenter]() {
        removeStandardScanCenter(-1);
    });
    connect(standardScanRemoveUpperButton, &QPushButton::clicked, this, [removeStandardScanCenter]() {
        removeStandardScanCenter(1);
    });
    auto fillStandardScanRange = [this, applyScanUiChange]() {
        refreshSettingsFromUi();
        applyStandardScanRangeToCenters();
        applyScanUiChange(true);
    };
    connect(standardScanFillRangeButton, &QPushButton::clicked, this, fillStandardScanRange);
    connect(standardScanRangeStartEdit, &QLineEdit::returnPressed, this, fillStandardScanRange);
    connect(standardScanRangeEndEdit, &QLineEdit::returnPressed, this, fillStandardScanRange);
    connect(standardScanDwellSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
        standardScanDwellMs = (std::clamp)(value,
                                           STANDARD_SCAN_MIN_DWELL_MS,
                                           STANDARD_SCAN_MAX_DWELL_MS);
        if (persistentSettingsReady) {
            savePersistentSettings();
        }
    });
    connect(standardScanSettleSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int value) {
        standardScanSettleMs = (std::clamp)(value,
                                            STANDARD_SCAN_MIN_SETTLE_MS,
                                            STANDARD_SCAN_MAX_SETTLE_MS);
        if (persistentSettingsReady) {
            savePersistentSettings();
        }
    });
    connect(agileScanSavePresetButton, &QPushButton::clicked, this, &YourClassName::saveAgileScanPreset);
    connect(agileScanDeletePresetButton, &QPushButton::clicked, this, &YourClassName::deleteAgileScanPreset);
    connect(scanMeasurementCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
        scanMeasurementEnabled = checked;
        if (!checked && scanMeasurementBaselineButton) {
            QSignalBlocker blocker(scanMeasurementBaselineButton);
            scanMeasurementBaselineButton->setChecked(false);
            scanMeasurementBaselineRecording = false;
            scanMeasurementBaselineButton->setText(uiText(QStringLiteral("bg_rec"), QStringLiteral("BG Rec")));
        }
        updateAgileScanControls();
        updateScanMeasurementStatus();
    });
    connect(scanMeasurementBinSpin,
            QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this,
            [this](double value) {
                scanMeasurementBinMhz = (std::clamp)(value,
                                                     SCAN_MEASUREMENT_MIN_BIN_MHZ,
                                                     SCAN_MEASUREMENT_MAX_BIN_MHZ);
                clearScanMeasurement();
            });
    connect(scanMeasurementBaselineButton, &QPushButton::toggled, this, [this](bool checked) {
        scanMeasurementBaselineRecording = checked;
        if (scanMeasurementBaselineButton) {
            scanMeasurementBaselineButton->setText(checked
                                                       ? uiText(QStringLiteral("stop_bg"), QStringLiteral("Stop BG"))
                                                       : uiText(QStringLiteral("bg_rec"), QStringLiteral("BG Rec")));
        }
        if (checked) {
            scanMeasurementEnabled = true;
            if (scanMeasurementCheckbox) {
                QSignalBlocker blocker(scanMeasurementCheckbox);
                scanMeasurementCheckbox->setChecked(true);
            }
            for (auto &bin : scanMeasurementBins) {
                bin.baselineDb = -160.0f;
                bin.baselineCount = 0;
            }
        }
        updateScanMeasurementStatus();
    });
    connect(scanMeasurementResetPeakButton, &QPushButton::clicked, this, &YourClassName::resetScanMeasurementPeaks);
    connect(scanMeasurementExportButton, &QPushButton::clicked, this, &YourClassName::exportScanMeasurementCsv);
    connect(dmrHunterControls, &SpectrumHunterControls::detectToggled, this, [this](bool checked) {
        dmrHunterSettings.enabled = checked;
        if (!checked) {
            dmrHunterCandidates.clear();
            dmrHunterCandidateIndex = -1;
        }
        updateDmrHunterControls();
        savePersistentSettings();
    });
    connect(dmrHunterControls,
            &SpectrumHunterControls::minWidthChanged,
            this,
            [this](double value) {
                dmrHunterSettings.minWidthKhz = (std::clamp)(value,
                                                             DmrHunterDetector::MinWidthKhz,
                                                             DmrHunterDetector::MaxWidthKhz);
                dmrHunterSettings = DmrHunterDetector::normalizedSettings(dmrHunterSettings);
                dmrHunterCandidates.clear();
                dmrHunterCandidateIndex = -1;
                updateDmrHunterControls();
                savePersistentSettings();
            });
    connect(dmrHunterControls,
            &SpectrumHunterControls::maxWidthChanged,
            this,
            [this](double value) {
                dmrHunterSettings.maxWidthKhz = (std::clamp)(value,
                                                             DmrHunterDetector::MinWidthKhz,
                                                             DmrHunterDetector::MaxWidthKhz);
                dmrHunterSettings = DmrHunterDetector::normalizedSettings(dmrHunterSettings);
                dmrHunterCandidates.clear();
                dmrHunterCandidateIndex = -1;
                updateDmrHunterControls();
                savePersistentSettings();
            });
    connect(dmrHunterControls,
            &SpectrumHunterControls::thresholdChanged,
            this,
            [this](double value) {
                dmrHunterSettings.thresholdDb = (std::clamp)(value,
                                                             DmrHunterDetector::MinThresholdDb,
                                                             DmrHunterDetector::MaxThresholdDb);
                dmrHunterSettings = DmrHunterDetector::normalizedSettings(dmrHunterSettings);
                dmrHunterCandidates.clear();
                dmrHunterCandidateIndex = -1;
                updateDmrHunterControls();
                savePersistentSettings();
            });
    connect(dmrHunterControls,
            &SpectrumHunterControls::applyPresetRequested,
            this,
            &YourClassName::applyDmrHunterPresetToScan);
    connect(dmrHunterControls,
            &SpectrumHunterControls::tuneRequested,
            this,
            &YourClassName::tuneDmrHunterCandidate);
    connect(dmrHunterControls,
            &SpectrumHunterControls::previousCandidateRequested,
            this,
            [this]() {
                selectDmrHunterCandidate(-1);
            });
    connect(dmrHunterControls,
            &SpectrumHunterControls::nextCandidateRequested,
            this,
            [this]() {
                selectDmrHunterCandidate(1);
            });
    connect(fpvHunterControls, &SpectrumHunterControls::detectToggled, this, [this](bool checked) {
        fpvHunterSettings.enabled = checked;
        if (!checked) {
            fpvHunterTrack = {};
            fpvHunterCandidates.clear();
            fpvHunterCandidateIndex = -1;
            fpvHunterLastFollowCenterHz = std::numeric_limits<double>::quiet_NaN();
            fpvHunterLastFollowBandwidthHz = std::numeric_limits<double>::quiet_NaN();
        }
        updateFpvHunterControls();
        savePersistentSettings();
    });
    connect(fpvHunterControls,
            &SpectrumHunterControls::minWidthChanged,
            this,
            [this](double value) {
                fpvHunterSettings.minWidthMhz = (std::clamp)(value,
                                                             FpvHunterDetector::MinWidthMhz,
                                                             FpvHunterDetector::MaxWidthMhz);
                fpvHunterSettings = FpvHunterDetector::normalizedSettings(fpvHunterSettings);
                fpvHunterTrack = {};
                fpvHunterCandidates.clear();
                fpvHunterCandidateIndex = -1;
                fpvHunterLastFollowCenterHz = std::numeric_limits<double>::quiet_NaN();
                fpvHunterLastFollowBandwidthHz = std::numeric_limits<double>::quiet_NaN();
                updateFpvHunterControls();
                savePersistentSettings();
            });
    connect(fpvHunterControls,
            &SpectrumHunterControls::maxWidthChanged,
            this,
            [this](double value) {
                fpvHunterSettings.maxWidthMhz = (std::clamp)(value,
                                                             FpvHunterDetector::MinWidthMhz,
                                                             FpvHunterDetector::MaxWidthMhz);
                fpvHunterSettings = FpvHunterDetector::normalizedSettings(fpvHunterSettings);
                fpvHunterTrack = {};
                fpvHunterCandidates.clear();
                fpvHunterCandidateIndex = -1;
                fpvHunterLastFollowCenterHz = std::numeric_limits<double>::quiet_NaN();
                fpvHunterLastFollowBandwidthHz = std::numeric_limits<double>::quiet_NaN();
                updateFpvHunterControls();
                savePersistentSettings();
            });
    connect(fpvHunterControls,
            &SpectrumHunterControls::thresholdChanged,
            this,
            [this](double value) {
                fpvHunterSettings.thresholdDb = (std::clamp)(value,
                                                             FpvHunterDetector::MinThresholdDb,
                                                             FpvHunterDetector::MaxThresholdDb);
                fpvHunterSettings = FpvHunterDetector::normalizedSettings(fpvHunterSettings);
                fpvHunterTrack = {};
                fpvHunterCandidates.clear();
                fpvHunterCandidateIndex = -1;
                fpvHunterLastFollowCenterHz = std::numeric_limits<double>::quiet_NaN();
                fpvHunterLastFollowBandwidthHz = std::numeric_limits<double>::quiet_NaN();
                updateFpvHunterControls();
                savePersistentSettings();
            });
    connect(fpvHunterControls,
            &SpectrumHunterControls::applyPresetRequested,
            this,
            &YourClassName::applyFpvHunterPresetToScan);
    connect(fpvHunterControls,
            &SpectrumHunterControls::tuneRequested,
            this,
            &YourClassName::tuneFpvHunterCandidate);
    connect(fpvHunterControls,
            &SpectrumHunterControls::followToggled,
            this,
            [this](bool checked) {
                fpvHunterFollowEnabled = checked;
                fpvHunterLastFollowCenterHz = std::numeric_limits<double>::quiet_NaN();
                fpvHunterLastFollowBandwidthHz = std::numeric_limits<double>::quiet_NaN();
                updateFpvHunterControls();
                if (checked &&
                    fpvHunterCandidateIndex >= 0 &&
                    fpvHunterCandidateIndex < static_cast<int>(fpvHunterCandidates.size())) {
                    tuneFpvHunterCandidateIndex(fpvHunterCandidateIndex);
                }
                savePersistentSettings();
            });
    connect(fpvHunterControls,
            &SpectrumHunterControls::previousCandidateRequested,
            this,
            [this]() {
                selectFpvHunterCandidate(-1);
            });
    connect(fpvHunterControls,
            &SpectrumHunterControls::nextCandidateRequested,
            this,
            [this]() {
                selectFpvHunterCandidate(1);
            });
    connect(fpvHunterHistoryTuneButton,
            &QPushButton::clicked,
            this,
            &YourClassName::tuneFpvHunterHistorySelection);
    connect(fpvHunterHistoryClearButton,
            &QPushButton::clicked,
            this,
            &YourClassName::clearFpvHunterHistory);
    connect(digitalVideoHunterControls, &SpectrumHunterControls::detectToggled, this, [this](bool checked) {
        digitalVideoHunterSettings.enabled = checked;
        if (!checked) {
            digitalVideoHunterCandidates.clear();
            digitalVideoHunterCandidateIndex = -1;
        }
        updateDigitalVideoHunterControls();
        savePersistentSettings();
    });
    connect(digitalVideoHunterControls,
            &SpectrumHunterControls::minWidthChanged,
            this,
            [this](double value) {
                digitalVideoHunterSettings.minWidthMhz = (std::clamp)(value,
                                                                      DigitalVideoHunterDetector::MinWidthMhz,
                                                                      DigitalVideoHunterDetector::MaxWidthMhz);
                digitalVideoHunterSettings = DigitalVideoHunterDetector::normalizedSettings(digitalVideoHunterSettings);
                digitalVideoHunterCandidates.clear();
                digitalVideoHunterCandidateIndex = -1;
                updateDigitalVideoHunterControls();
                savePersistentSettings();
            });
    connect(digitalVideoHunterControls,
            &SpectrumHunterControls::maxWidthChanged,
            this,
            [this](double value) {
                digitalVideoHunterSettings.maxWidthMhz = (std::clamp)(value,
                                                                      DigitalVideoHunterDetector::MinWidthMhz,
                                                                      DigitalVideoHunterDetector::MaxWidthMhz);
                digitalVideoHunterSettings = DigitalVideoHunterDetector::normalizedSettings(digitalVideoHunterSettings);
                digitalVideoHunterCandidates.clear();
                digitalVideoHunterCandidateIndex = -1;
                updateDigitalVideoHunterControls();
                savePersistentSettings();
            });
    connect(digitalVideoHunterControls,
            &SpectrumHunterControls::thresholdChanged,
            this,
            [this](double value) {
                digitalVideoHunterSettings.thresholdDb = (std::clamp)(value,
                                                                      DigitalVideoHunterDetector::MinThresholdDb,
                                                                      DigitalVideoHunterDetector::MaxThresholdDb);
                digitalVideoHunterSettings = DigitalVideoHunterDetector::normalizedSettings(digitalVideoHunterSettings);
                digitalVideoHunterCandidates.clear();
                digitalVideoHunterCandidateIndex = -1;
                updateDigitalVideoHunterControls();
                savePersistentSettings();
            });
    connect(digitalVideoHunterControls,
            &SpectrumHunterControls::applyPresetRequested,
            this,
            &YourClassName::applyDigitalVideoHunterPresetToScan);
    connect(digitalVideoHunterControls,
            &SpectrumHunterControls::tuneRequested,
            this,
            &YourClassName::tuneDigitalVideoHunterCandidate);
    connect(digitalVideoHunterControls,
            &SpectrumHunterControls::previousCandidateRequested,
            this,
            [this]() {
                selectDigitalVideoHunterCandidate(-1);
            });
    connect(digitalVideoHunterControls,
            &SpectrumHunterControls::nextCandidateRequested,
            this,
            [this]() {
                selectDigitalVideoHunterCandidate(1);
            });
    connect(spurSuppressionCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
        spurSuppressionEnabled = checked;
        updateSpurSuppressionStatus();
        savePersistentSettings();
    });
    connect(spurCalibrateButton, &QPushButton::clicked, this, &YourClassName::startSpurCalibration);
    connect(spurClearButton, &QPushButton::clicked, this, &YourClassName::clearSpurMask);
    connect(presetManagerButton, &QPushButton::clicked, this, &YourClassName::openPresetManager);
    connect(appSettingsButton, &QPushButton::clicked, this, &YourClassName::openApplicationSettings);
    connect(bandwidthControl, &FrequencyControl::valueCommitted, this, [this](double) {
        onBandwidthChanged();
    });
    connect(scaleWidget, SIGNAL(frequencyChanged()), this, SLOT(updateFrequency()));
    connect(scaleWidget, SIGNAL(centralFrequencyChanged()), this, SLOT(updateCentralFrequency()));
    connect(scaleWidget, &ScaleWidget::tuningChanged, this, &YourClassName::updateTuningFromScale);
    //connect(scaleWidget, &ScaleWidget::frequencyChanged, this, &YourClassName::updateFrequency);
    //connect(scaleWidget, &ScaleWidget::centralFrequencyChanged, this, &YourClassName::updateCentralFrequency);
    //connect(waterfallWidget, SIGNAL(scaleChanged(int delta)), this, SLOT(onWaterfallScaleChanged(int delta)));
    //connect(graphWidget, SIGNAL(scaleChanged(int delta)), this, SLOT(onWaterfallScaleChanged(int delta)));
    connect(waterfallWidget, &MyWaterfallWidget::scaleChanged, this, &YourClassName::onWaterfallScaleChanged);
    connect(graphWidget, &MyGraphWidget::scaleChanged, this, &YourClassName::onWaterfallScaleChanged);
    connect(graphWidget, &MyGraphWidget::tuneContextRequested, this, &YourClassName::showTuneContextMenu);
    connect(waterfallWidget, &MyWaterfallWidget::tuneContextRequested, this, &YourClassName::showTuneContextMenu);
    connect(graphWidget, &MyGraphWidget::autoTuneRequested, this, &YourClassName::tuneSignalCenterAt);
    connect(waterfallWidget, &MyWaterfallWidget::autoTuneRequested, this, &YourClassName::tuneSignalCenterAt);
    onVgaGainChanged(3);
    onLnaGainChanged(1);
    populateSampleRates();
    populateAudioDevices();
    refreshSettingsFromUi();
    loadPersistentSettings();
    publishSettingsToGlobals();
    updateFineTuneControlMode();
    updateUiFromPendingSettings();
    settingRange();
    updateGraphBandMarkers();
    updateAgileScanControls();
    updateScanMeasurementStatus();
    updateSpurSuppressionStatus();
    updateDmrHunterControls();
    updateFpvHunterControls();
    updateDigitalVideoHunterControls();
    updateDigitalDecoderMode();
    updateVideoProcessorMode();
    updateAudioRelaySocket();
    updateAudioHttpStreamServer();
    updateUiForRunState();
    refreshPlaybackFiles();
    persistentSettingsReady = true;
    qApp->installEventFilter(this);
}

YourClassName::~YourClassName() {
    qApp->removeEventFilter(this);
    refreshSettingsFromUi();
    savePersistentSettings();
    stopPlayback();
    stopRecording(false);

    if (stopPollTimer) {
        stopPollTimer->stop();
    }
    if (streamWatchdogTimer) {
        streamWatchdogTimer->stop();
    }
    if (videoSnapshotTimer) {
        videoSnapshotTimer->stop();
    }
    pendingAudioStartAfterStreamReady = false;
    pendingNetworkAudioStartAfterIqPrebuffer = false;
    pendingPlaybackAudioStartAfterIqPrebuffer = false;
    if (updateTimer) {
        updateTimer->stop();
    }
    if (remoteAudioPlayer) {
        remoteAudioPlayer->stop();
    }
    if (audioHttpServer) {
        audioHttpServer->close();
    }
    for (QTcpSocket *client : std::as_const(audioHttpClients)) {
        if (client) {
            client->disconnectFromHost();
            client->deleteLater();
        }
    }
    audioHttpClients.clear();
    if (networkController) {
        networkController->stop();
    }
    if (audioProcessor) {
        audioProcessor->stopDemodulation();
        audioProcessor->setLocalPlaybackEnabled(true);
    }
    if (processor) {
        processor->requestStop();
        if (processor->isRunning() && !processor->wait(1500)) {
            processor->forceStop(1000);
        }
        processor->finalizeStopped();
    }

    if (digitalDecoderThread) {
        digitalDecoderThread->quit();
        if (!digitalDecoderThread->wait(3000)) {
            qDebug() << "[Digital] decoder thread did not stop in time; terminating";
            digitalDecoderThread->terminate();
            digitalDecoderThread->wait();
        }
        digitalDecoder = nullptr;
        digitalDecoderThread = nullptr;
    }

    if (videoProcessorThread) {
        videoProcessorThread->quit();
        if (!videoProcessorThread->wait(3000)) {
            qDebug() << "[Video] processor thread did not stop in time; terminating";
            videoProcessorThread->terminate();
            videoProcessorThread->wait();
        }
        videoProcessor = nullptr;
        videoProcessorThread = nullptr;
    }

    if (audioProcessor) { 
    delete audioProcessor;
    audioProcessor = nullptr;
    }
    if (processor) {
        delete processor;
        processor = nullptr;
    }

    if (device) {
        if (fobosCloseKnownUnsafe) {
            qDebug() << "[FobosLifecycle] destructor: skipping fobos_rx_close because a previous close failed or hung";
        } else {
            closeFobosDeviceSafely(device);
        }
        device = nullptr;
    }
    if (agileDevice) {
        if (agileScanRunning) {
            stopFobosAgileScanSafely(agileDevice);
            agileScanRunning = false;
            activeAgileScanFrequencies.clear();
        }
        closeFobosAgileDeviceSafely(agileDevice);
        agileDevice = nullptr;
    }
    if (iqData) {
        iqData = nullptr;
    }
}

bool YourClassName::eventFilter(QObject *watched, QEvent *event) {
    if (watched == controlsToggleButton && event->type() == QEvent::MouseButtonDblClick) {
        event->accept();
        return true;
    }
    if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
        auto *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent && keyEvent->key() == Qt::Key_F9 && !keyEvent->isAutoRepeat()) {
            if (event->type() == QEvent::KeyPress) {
                startRecording(true);
            } else {
                stopRecording(true);
            }
            return true;
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

bool YourClassName::restartStreamForHardwareChange() {
    if (isChannelIqRecordingActive() &&
        networkMode != NetworkMode::Disabled &&
        isFullIqProcessingMode()) {
        stopRecording(false);
        updateRecordingStatus(QStringLiteral("Recording stopped: Channel IQ cannot run during Full IQ streaming"));
    }

    if (isIdle()) {
        if (!hasActiveFobosDevice()) {
            qDebug() << "[LiveHardware] settings changed while idle; no open Fobos session to restart";
            return true;
        }
        return applyFobosSettings();
    }

    if (!hasActiveFobosDevice()) {
        qDebug() << "[LiveHardware] cannot restart stream because Fobos session is missing";
        deviceOpened = false;
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return false;
    }

    qDebug() << "[LiveHardware] restarting stream for hardware change";

    runState = RadioRunState::Stopping;
    updateUiForRunState();
    if (streamWatchdogTimer) streamWatchdogTimer->stop();
    if (stopPollTimer) stopPollTimer->stop();
    if (updateTimer) updateTimer->stop();
    pendingAudioStartAfterStreamReady = false;
    if (audioProcessor) audioProcessor->stopDemodulation();
    digitalDecoderGeneration.fetch_add(1, std::memory_order_relaxed);
    pendingDmrDecoderPcm.clear();
    pendingDmrDecoderSampleRate = 48000;
    droppedDigitalDecoderFramesSinceLog.store(0);
    if (digitalDecoder) {
        QMetaObject::invokeMethod(digitalDecoder,
                                  [decoder = digitalDecoder]() {
                                      decoder->reset();
                                  },
                                  Qt::QueuedConnection);
    }

    if (processor) {
        processor->requestStop();
        if (processor->isRunning() && !processor->wait(1500)) {
            const bool forced = processor->forceStop(1000);
            qDebug() << "[LiveHardware] forced DataProcessor stop during live restart" << forced;
            if (!forced || processor->isRunning()) {
                qDebug() << "[LiveHardware] DataProcessor is still running; live restart is deferred to stop recovery";
                stopCancelRetryCount = 0;
                stopElapsedTimer.restart();
                if (stopPollTimer) stopPollTimer->start();
                updateUiForRunState();
                return false;
            }
        }
        processor->finalizeStopped();
    }

    clearLiveSpectrumSnapshot(false);

    runState = RadioRunState::Starting;
    updateUiForRunState();
    if (!applyFobosSettings()) {
        qDebug() << "[LiveHardware] applyFobosSettings failed";
        closeFobosSession(true);
        clearLiveSpectrumSnapshot();
        deviceOpened = false;
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return false;
    }

    if (!stabilizeAgileFrequencyBeforeStreaming("live hardware restart")) {
        qDebug() << "[LiveHardware] Agile frequency settle failed before reader restart";
        closeFobosSession(true);
        clearLiveSpectrumSnapshot();
        deviceOpened = false;
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return false;
    }

    updateSpectrumTimerInterval();
    settingRange();
    spectrumTuningDebugFramesRemaining = 32;

    const bool serverIqStreaming = networkMode == NetworkMode::Server && isClientIqProcessingMode();
    const bool serverFullIqStreaming = networkMode == NetworkMode::Server && isFullIqProcessingMode();
    const bool serverChannelIqStreaming = networkMode == NetworkMode::Server && isChannelIqProcessingMode();
    const bool channelIqRecording = isChannelIqRecordingActive();
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    const bool serverAudioStreamingForFullIq = serverFullIqStreaming && pendingSettings.audioEnabled;
    const bool channelIqRecordingOnly = channelIqRecording && !serverIqStreaming;
    const bool serverLocalAudioEnabled =
        pendingSettings.audioEnabled &&
        !channelIqRecordingOnly &&
        (!serverIqStreaming || serverAudioStreamingForFullIq);
    const bool queueAudioBlocks =
        !channelIqRecordingOnly &&
        (!serverIqStreaming || serverAudioStreamingForFullIq);
    const bool publishIqSnapshot = !channelIqRecordingOnly;
    if (audioProcessor) {
        audioProcessor->setLocalPlaybackEnabled(!suppressServerLocalOutput);
    }

    if ((serverIqStreaming || channelIqRecording) && processor) {
        processor->configureNetworkIqStreaming(pendingSettings,
                                               true,
                                               serverChannelIqStreaming || channelIqRecording);
    }

    processor->startProcessing(activeFobosDevice(),
                               activeFobosApiKind,
                               pendingSettings.syncEnabled,
                               pendingSettings.sampleRate,
                               queueAudioBlocks,
                               publishIqSnapshot,
                               serverIqStreaming || channelIqRecording,
                               agileScanEnabled &&
                                   !standardScanEnabled &&
                                   activeFobosApiKind == FobosApiKind::Agile);

    if (activeFobosApiKind == FobosApiKind::Agile &&
        pendingSettings.inputMode == INPUT_RF &&
        !agileScanEnabled) {
        liveRetuneSettleDurationMs = agileRfLiveSettleMs(pendingSettings.sampleRate, false);
        clearLiveSpectrumSnapshot(false);
        liveRetuneSettleTimer.start();
        qDebug() << "[LiveHardware] Agile reader restart settle armed"
                 << "settleMs" << liveRetuneSettleDurationMs;
    }

    deviceOpened = true;

    if (updateTimer && !(serverFullIqStreaming && suppressServerLocalOutput)) {
        updateTimer->start();
    }

    pendingAudioStartAfterStreamReady = serverLocalAudioEnabled;
    streamStartCallbackCount = processor ? processor->callbackCount() : 0;
    streamStartElapsedTimer.restart();
    if (streamWatchdogTimer) streamWatchdogTimer->start();

    runState = RadioRunState::Running;
    updateUiForRunState();
    applyServerLocalOutputPolicy();
    return true;
}


uint8_t YourClassName::currentGpoValue() const {
    uint8_t value = 0;
    for (int i = 0; i < 8; ++i) {
        if (checkBoxes[i] && checkBoxes[i]->isChecked()) {
            value |= (1 << i);
        }
    }
    return value;
}

QVector<double> YourClassName::agileScanFrequencyList(QString *error) const {
    const QString ranges = agileScanRangesEdit
                               ? agileScanRangesEdit->text().trimmed()
                               : agileScanRangesMhz.trimmed();
    const double step = agileScanStepSpin
                            ? agileScanStepSpin->value()
                            : agileScanStepMhz;
    return parseAgileScanFrequenciesMhz(ranges, step, error);
}

QVector<double> YourClassName::standardScanFrequencyList(QString *error) const {
    const QString centers = standardScanCentersEdit
                                ? standardScanCentersEdit->text().trimmed()
                                : standardScanCentersMhz.trimmed();
    return parseStandardScanCentersMhz(centers,
                                       pendingSettings.sampleRate,
                                       AGILE_SCAN_MIN_POINTS,
                                       error,
                                       nullptr);
}

void YourClassName::normalizeStandardScanCentersUi(bool requireTwoCenters) {
    const QString centers = standardScanCentersEdit
                                ? standardScanCentersEdit->text().trimmed()
                                : standardScanCentersMhz.trimmed();
    QString error;
    bool adjusted = false;
    const QVector<double> normalized =
        parseStandardScanCentersMhz(centers,
                                    pendingSettings.sampleRate,
                                    requireTwoCenters ? AGILE_SCAN_MIN_POINTS : 0,
                                    &error,
                                    &adjusted);
    if (!error.isEmpty() || normalized.isEmpty() || !adjusted) {
        return;
    }

    standardScanCentersMhz = formatMhzList(normalized);
    if (standardScanCentersEdit) {
        QSignalBlocker blocker(standardScanCentersEdit);
        standardScanCentersEdit->setText(standardScanCentersMhz);
    }
    if (standardScanStatusLabel) {
        standardScanStatusLabel->setText(
            uiText(QStringLiteral("standard_scan_adjusted"),
                   QStringLiteral("Standard scan: centers adjusted to sample-rate spacing")));
    }
    qDebug() << "[StandardScan] centers adjusted to sample-rate spacing"
             << "sampleRate" << pendingSettings.sampleRate
             << "centers" << standardScanCentersMhz;
}

void YourClassName::applyStandardScanRangeToCenters() {
    auto parseRangeMhz = [](QString text, double *frequencyHz) -> bool {
        if (!frequencyHz) {
            return false;
        }
        text = text.trimmed();
        text.remove(QRegularExpression(QStringLiteral("mhz"), QRegularExpression::CaseInsensitiveOption));
        text.replace(QLatin1Char(','), QLatin1Char('.'));
        bool ok = false;
        const double mhz = text.toDouble(&ok);
        const double hz = mhz * 1000000.0;
        if (!ok ||
            !std::isfinite(hz) ||
            hz < RF_MIN_CENTER_FREQUENCY ||
            hz > RF_EXPERIMENTAL_MAX_FREQUENCY) {
            return false;
        }
        *frequencyHz = hz;
        return true;
    };

    if (standardScanRangeStartEdit) {
        standardScanRangeStartMhz = standardScanRangeStartEdit->text().trimmed();
    }
    if (standardScanRangeEndEdit) {
        standardScanRangeEndMhz = standardScanRangeEndEdit->text().trimmed();
    }

    double startHz = 0.0;
    double endHz = 0.0;
    if (!parseRangeMhz(standardScanRangeStartMhz, &startHz)) {
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_bad_range_start"),
                                                    QStringLiteral("Bad scan range start")));
        }
        return;
    }
    if (!parseRangeMhz(standardScanRangeEndMhz, &endHz)) {
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_bad_range_end"),
                                                    QStringLiteral("Bad scan range end")));
        }
        return;
    }

    if (endHz < startHz) {
        std::swap(startHz, endHz);
    }
    const double stepHz = pendingSettings.sampleRate;
    if (!std::isfinite(stepHz) || stepHz <= 0.0) {
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_bad_sample_rate"),
                                                    QStringLiteral("Standard scan: bad sample rate")));
        }
        return;
    }
    if (endHz - startHz < stepHz - 0.5) {
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_range_too_small"),
                                                    QStringLiteral("Scan range is smaller than one sample-rate step")));
        }
        return;
    }

    QVector<double> centers;
    constexpr int maxGeneratedCenters = 512;
    for (double centerHz = startHz;
         centerHz <= endHz + 0.5 && centers.size() < maxGeneratedCenters;
         centerHz += stepHz) {
        centers.push_back(centerHz);
    }
    if (centers.size() >= maxGeneratedCenters && centers.last() + stepHz <= endHz + 0.5) {
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_range_too_many"),
                                                    QStringLiteral("Scan range generated too many centers")));
        }
        return;
    }
    if (centers.size() < AGILE_SCAN_MIN_POINTS) {
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_range_too_small"),
                                                    QStringLiteral("Scan range is smaller than one sample-rate step")));
        }
        return;
    }

    standardScanCentersMhz = formatMhzList(centers);
    if (standardScanCentersEdit) {
        QSignalBlocker blocker(standardScanCentersEdit);
        standardScanCentersEdit->setText(standardScanCentersMhz);
    }
    if (standardScanStatusLabel) {
        standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_range_filled"),
                                                QStringLiteral("Standard scan: %1 centers from range"))
                                         .arg(centers.size()));
    }
}

double YourClassName::currentAgileScanCenterFrequencyHz() const {
    if (!agileScanRunning ||
        activeFobosApiKind != FobosApiKind::Agile ||
        !agileDevice ||
        activeAgileScanFrequencies.isEmpty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const int index = getFobosAgileScanIndexSafely(agileDevice);
    if (index < 0 || index >= activeAgileScanFrequencies.size()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return activeAgileScanFrequencies.at(index);
}

double YourClassName::currentStandardScanCenterFrequencyHz() const {
    if (!standardScanRunning ||
        !hasActiveFobosDevice() ||
        activeStandardScanFrequencies.isEmpty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const int index = (std::clamp)(standardScanIndex, 0, activeStandardScanFrequencies.size() - 1);
    return activeStandardScanFrequencies.at(index);
}

void YourClassName::updateAgileScanControls() {
    if (agileScanPresetCombo) {
        const QString currentText = agileScanPresetCombo->currentText();
        QSignalBlocker blocker(agileScanPresetCombo);
        agileScanPresetCombo->clear();
        for (auto it = agileScanPresets.constBegin(); it != agileScanPresets.constEnd(); ++it) {
            agileScanPresetCombo->addItem(it.key(), it.key());
        }
        agileScanPresetCombo->setEditText(currentText);
    }
    if (standardScanPresetCombo) {
        const QString currentText = standardScanPresetCombo->currentText();
        QSignalBlocker blocker(standardScanPresetCombo);
        standardScanPresetCombo->clear();
        for (auto it = standardScanPresets.constBegin(); it != standardScanPresets.constEnd(); ++it) {
            standardScanPresetCombo->addItem(it.key(), it.key());
        }
        standardScanPresetCombo->setEditText(currentText);
    }

    const FobosDeviceInfo selectedInfo = selectedFobosDeviceInfo();
    const bool agileScanSupported = selectedInfo.apiKind == FobosApiKind::Agile;
    const bool standardScanSupported =
        selectedInfo.apiKind == FobosApiKind::Standard ||
        selectedInfo.apiKind == FobosApiKind::Agile;
    if (agileScanCheckbox) {
        agileScanCheckbox->setEnabled(agileScanSupported);
        agileScanCheckbox->setToolTip(agileScanSupported
                                          ? uiText(QStringLiteral("agile_scan_tooltip"),
                                                   QStringLiteral("Use Agile firmware scan mode"))
                                          : uiText(QStringLiteral("agile_receiver_required"),
                                                   QStringLiteral("Agile firmware receiver required")));
        if (!agileScanSupported && agileScanCheckbox->isChecked()) {
            QSignalBlocker blocker(agileScanCheckbox);
            agileScanCheckbox->setChecked(false);
            agileScanEnabled = false;
        }
    }
    if (standardScanCheckbox) {
        standardScanCheckbox->setEnabled(standardScanSupported);
        standardScanCheckbox->setToolTip(standardScanSupported
                                             ? uiText(QStringLiteral("standard_scan_tooltip"),
                                                      QStringLiteral("Slow manual retune scan by cycling through listed center frequencies"))
                                             : uiText(QStringLiteral("standard_receiver_required"),
                                                     QStringLiteral("Fobos receiver required")));
        if (!standardScanSupported && standardScanCheckbox->isChecked()) {
            QSignalBlocker blocker(standardScanCheckbox);
            standardScanCheckbox->setChecked(false);
            standardScanEnabled = false;
            resetStandardScanState(true);
        }
    }
    if (scanListeningLockCheckbox) {
        scanListeningLockCheckbox->setEnabled(standardScanSupported || agileScanSupported);
    }

    const bool scanChecked = agileScanSupported && agileScanCheckbox && agileScanCheckbox->isChecked();
    if (agileScanRangesEdit) {
        agileScanRangesEdit->setEnabled(scanChecked);
    }
    if (agileScanStepSpin) {
        agileScanStepSpin->setEnabled(scanChecked);
    }
    if (agileScanPresetCombo) {
        agileScanPresetCombo->setEnabled(agileScanSupported);
    }
    if (agileScanSavePresetButton) {
        agileScanSavePresetButton->setEnabled(agileScanSupported);
    }
    if (agileScanDeletePresetButton) {
        agileScanDeletePresetButton->setEnabled(agileScanSupported);
    }
    const bool standardScanChecked =
        standardScanSupported && standardScanCheckbox && standardScanCheckbox->isChecked();
    if (standardScanCentersEdit) {
        standardScanCentersEdit->setEnabled(standardScanChecked);
    }
    if (standardScanPresetCombo) {
        standardScanPresetCombo->setEnabled(standardScanSupported);
    }
    if (standardScanSavePresetButton) {
        standardScanSavePresetButton->setEnabled(standardScanSupported);
    }
    if (standardScanDeletePresetButton) {
        standardScanDeletePresetButton->setEnabled(standardScanSupported);
    }
    if (standardScanRangeStartEdit) {
        standardScanRangeStartEdit->setEnabled(standardScanSupported);
    }
    if (standardScanRangeEndEdit) {
        standardScanRangeEndEdit->setEnabled(standardScanSupported);
    }
    if (standardScanDwellSpin) {
        standardScanDwellSpin->setEnabled(standardScanSupported);
    }
    if (standardScanSettleSpin) {
        standardScanSettleSpin->setEnabled(standardScanSupported);
    }
    if (standardScanAddLowerButton) {
        standardScanAddLowerButton->setEnabled(standardScanSupported);
    }
    if (standardScanAddUpperButton) {
        standardScanAddUpperButton->setEnabled(standardScanSupported);
    }
    if (standardScanRemoveLowerButton) {
        standardScanRemoveLowerButton->setEnabled(standardScanSupported);
    }
    if (standardScanRemoveUpperButton) {
        standardScanRemoveUpperButton->setEnabled(standardScanSupported);
    }
    if (standardScanFillRangeButton) {
        standardScanFillRangeButton->setEnabled(standardScanSupported);
    }
    if (scanMeasurementBinSpin) {
        scanMeasurementBinSpin->setEnabled(scanMeasurementCheckbox && scanMeasurementCheckbox->isChecked());
    }
    if (scanMeasurementBaselineButton) {
        scanMeasurementBaselineButton->setEnabled(scanMeasurementCheckbox && scanMeasurementCheckbox->isChecked());
    }
    if (scanMeasurementResetPeakButton) {
        scanMeasurementResetPeakButton->setEnabled(scanMeasurementCheckbox && scanMeasurementCheckbox->isChecked());
    }
    if (scanMeasurementExportButton) {
        scanMeasurementExportButton->setEnabled(scanMeasurementCheckbox && scanMeasurementCheckbox->isChecked());
    }

    QString error;
    const QVector<double> frequencies = agileScanFrequencyList(&error);
    if (agileScanStatusLabel) {
        if (!agileScanSupported) {
            agileScanStatusLabel->setText(uiText(QStringLiteral("agile_firmware_required"),
                                                 QStringLiteral("Agile firmware required")));
        } else if (!error.isEmpty()) {
            agileScanStatusLabel->setText(error);
        } else if (scanChecked) {
            agileScanStatusLabel->setText(uiText(QStringLiteral("scan_list_points"),
                                                 QStringLiteral("Scan list: %1 points"))
                                          .arg(frequencies.size()));
        } else {
            agileScanStatusLabel->setText(uiText(QStringLiteral("agile_scan_off"),
                                                 QStringLiteral("Agile scan: off")));
        }
    }

    if (standardScanStatusLabel) {
        normalizeStandardScanCentersUi(false);
        QString standardError;
        const QVector<double> centers =
            parseStandardScanCentersMhz(standardScanCentersMhz,
                                        pendingSettings.sampleRate,
                                        standardScanChecked ? AGILE_SCAN_MIN_POINTS : 0,
                                        &standardError,
                                        nullptr);
        if (!standardScanSupported) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_firmware_required"),
                                                    QStringLiteral("Fobos receiver required")));
        } else if (!standardError.isEmpty()) {
            standardScanStatusLabel->setText(standardError);
        } else if (standardScanChecked) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_list_points"),
                                                    QStringLiteral("Standard scan: %1 centers"))
                                             .arg(centers.size()));
        } else {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_off"),
                                                    QStringLiteral("Standard scan: off")));
        }
    }
}

void YourClassName::updateScanMeasurement(const std::vector<float> &frequencies,
                                          const std::vector<float> &magnitudes) {
    if (!scanMeasurementEnabled || frequencies.empty() || magnitudes.empty()) {
        return;
    }

    const int dataCount = std::min(static_cast<int>(frequencies.size()),
                                   static_cast<int>(magnitudes.size()));
    if (dataCount <= 0) {
        return;
    }

    const double binHz = (std::clamp)(scanMeasurementBinMhz,
                                      SCAN_MEASUREMENT_MIN_BIN_MHZ,
                                      SCAN_MEASUREMENT_MAX_BIN_MHZ) * 1000000.0;
    if (!std::isfinite(binHz) || binHz <= 0.0) {
        return;
    }

    ++scanMeasurementSequence;
    QMap<qint64, float> framePeakByBin;
    for (int i = 0; i < dataCount; ++i) {
        const double frequency = frequencies[static_cast<std::size_t>(i)];
        const float level = magnitudes[static_cast<std::size_t>((i + dataCount / 2) % dataCount)];
        if (!std::isfinite(frequency) || !std::isfinite(level)) {
            continue;
        }
        const qint64 key = static_cast<qint64>(std::llround(frequency / binHz));
        auto it = framePeakByBin.find(key);
        if (it == framePeakByBin.end() || level > it.value()) {
            framePeakByBin[key] = level;
        }
    }

    for (auto it = framePeakByBin.constBegin(); it != framePeakByBin.constEnd(); ++it) {
        ScanMeasurementBin &bin = scanMeasurementBins[it.key()];
        bin.frequencyHz = static_cast<double>(it.key()) * binHz;
        bin.currentDb = it.value();
        bin.peakDb = (std::max)(bin.peakDb, it.value());
        bin.seenCount += 1;
        bin.lastSequence = scanMeasurementSequence;
        if (scanMeasurementBaselineRecording) {
            if (bin.baselineCount <= 0) {
                bin.baselineDb = it.value();
                bin.baselineCount = 1;
            } else {
                const float alpha = bin.baselineCount < 8 ? 0.35f : 0.12f;
                bin.baselineDb += alpha * (it.value() - bin.baselineDb);
                ++bin.baselineCount;
            }
        }
    }

    updateScanMeasurementStatus();
}

std::vector<float> YourClassName::scanMeasurementOverlay(const std::vector<float> &frequencies,
                                                         int dataCount) const {
    std::vector<float> overlay;
    if (!scanMeasurementEnabled || scanMeasurementBins.isEmpty() || dataCount <= 0) {
        return overlay;
    }

    const double binHz = (std::clamp)(scanMeasurementBinMhz,
                                      SCAN_MEASUREMENT_MIN_BIN_MHZ,
                                      SCAN_MEASUREMENT_MAX_BIN_MHZ) * 1000000.0;
    if (!std::isfinite(binHz) || binHz <= 0.0) {
        return overlay;
    }

    overlay.assign(static_cast<std::size_t>(dataCount), -160.0f);
    for (int i = 0; i < dataCount && i < static_cast<int>(frequencies.size()); ++i) {
        const double frequency = frequencies[static_cast<std::size_t>(i)];
        if (!std::isfinite(frequency)) {
            continue;
        }
        const qint64 key = static_cast<qint64>(std::llround(frequency / binHz));
        const auto it = scanMeasurementBins.constFind(key);
        if (it == scanMeasurementBins.constEnd()) {
            continue;
        }
        overlay[static_cast<std::size_t>((i + dataCount / 2) % dataCount)] = it.value().peakDb;
    }
    return overlay;
}

void YourClassName::updateScanMeasurementStatus() {
    if (!scanMeasurementStatusLabel) {
        return;
    }

    auto setScanStatus = [this](const QString &text) {
        scanMeasurementStatusLabel->setToolTip(text);
        scanMeasurementStatusLabel->setText(text);
    };

    if (!scanMeasurementEnabled) {
        setScanStatus(uiText(QStringLiteral("scan_measurement_off"),
                             QStringLiteral("Scan measurement: off")));
        return;
    }

    if (scanMeasurementBins.isEmpty()) {
        setScanStatus(scanMeasurementBaselineRecording
                          ? uiText(QStringLiteral("scan_measurement_recording_baseline"),
                                   QStringLiteral("Scan measurement: recording baseline..."))
                          : uiText(QStringLiteral("scan_measurement_waiting"),
                                   QStringLiteral("Scan measurement: waiting for spectrum")));
        return;
    }

    int currentBins = 0;
    int baselineBins = 0;
    int coveredBins = 0;
    double peakSum = 0.0;
    double deltaSum = 0.0;
    float maxPeak = -160.0f;
    for (const ScanMeasurementBin &bin : scanMeasurementBins) {
        if (bin.seenCount <= 0) {
            continue;
        }
        ++currentBins;
        peakSum += bin.peakDb;
        maxPeak = (std::max)(maxPeak, bin.peakDb);
        if (bin.baselineCount > 0) {
            ++baselineBins;
            const float delta = bin.peakDb - bin.baselineDb;
            deltaSum += delta;
            if (delta >= SCAN_MEASUREMENT_COVERAGE_DELTA_DB) {
                ++coveredBins;
            }
        }
    }

    const double avgPeak = currentBins > 0 ? peakSum / currentBins : -160.0;
    const double avgDelta = baselineBins > 0 ? deltaSum / baselineBins : 0.0;
    const double coverage = baselineBins > 0
                                ? (100.0 * static_cast<double>(coveredBins) / baselineBins)
                                : 0.0;
    setScanStatus(
        baselineBins > 0
            ? uiText(QStringLiteral("scan_measurement_with_baseline"),
                     QStringLiteral("Scan measurement: %1 bins, peak %2 dB, avg %3 dB, delta %4 dB, coverage %5% >+%6 dB%7"))
                  .arg(currentBins)
                  .arg(maxPeak, 0, 'f', 1)
                  .arg(avgPeak, 0, 'f', 1)
                  .arg(avgDelta, 0, 'f', 1)
                  .arg(coverage, 0, 'f', 0)
                  .arg(SCAN_MEASUREMENT_COVERAGE_DELTA_DB, 0, 'f', 0)
                  .arg(scanMeasurementBaselineRecording
                           ? uiText(QStringLiteral("bg_rec_suffix"), QStringLiteral(" (BG rec)"))
                           : QString())
            : uiText(QStringLiteral("scan_measurement_without_baseline"),
                     QStringLiteral("Scan measurement: %1 bins, peak %2 dB, avg %3 dB%4"))
                  .arg(currentBins)
                  .arg(maxPeak, 0, 'f', 1)
                  .arg(avgPeak, 0, 'f', 1)
                  .arg(scanMeasurementBaselineRecording
                           ? uiText(QStringLiteral("bg_rec_suffix"), QStringLiteral(" (BG rec)"))
                           : QString()));
}

void YourClassName::updateDmrHunter(const std::vector<float> &frequencies,
                                    const std::vector<float> &magnitudes) {
    if (!dmrHunterControls) {
        return;
    }

    double previousSelectedCenterHz = std::numeric_limits<double>::quiet_NaN();
    if (dmrHunterCandidateIndex >= 0 &&
        dmrHunterCandidateIndex < static_cast<int>(dmrHunterCandidates.size())) {
        previousSelectedCenterHz = dmrHunterCandidates[static_cast<std::size_t>(dmrHunterCandidateIndex)].centerHz;
    }

    dmrHunterSettings = DmrHunterDetector::normalizedSettings(dmrHunterSettings);
    dmrHunterLastResult = DmrHunterDetector::analyze(frequencies,
                                                     magnitudes,
                                                     dmrHunterSettings);
    dmrHunterCandidates = dmrHunterLastResult.candidateList;
    dmrHunterCandidateIndex = -1;
    if (!dmrHunterCandidates.empty()) {
        int bestIndex = 0;
        if (std::isfinite(previousSelectedCenterHz)) {
            double bestDeltaHz = std::numeric_limits<double>::max();
            for (int i = 0; i < static_cast<int>(dmrHunterCandidates.size()); ++i) {
                const double centerHz = dmrHunterCandidates[static_cast<std::size_t>(i)].centerHz;
                if (!std::isfinite(centerHz)) {
                    continue;
                }
                const double deltaHz = std::abs(centerHz - previousSelectedCenterHz);
                if (deltaHz < bestDeltaHz) {
                    bestDeltaHz = deltaHz;
                    bestIndex = i;
                }
            }
        }
        dmrHunterCandidateIndex = bestIndex;
    }

    QString statusText = dmrHunterLastResult.statusText;
    if (dmrHunterCandidateIndex >= 0 &&
        dmrHunterCandidateIndex < static_cast<int>(dmrHunterCandidates.size())) {
        const DmrHunterCandidate &candidate =
            dmrHunterCandidates[static_cast<std::size_t>(dmrHunterCandidateIndex)];
        statusText += QStringLiteral("\nSelected %1/%2: %3 MHz, width %4 kHz, peak %5 dB, +%6 dB")
                          .arg(dmrHunterCandidateIndex + 1)
                          .arg(static_cast<int>(dmrHunterCandidates.size()))
                          .arg(candidate.centerHz / 1000000.0, 0, 'f', 6)
                          .arg(candidate.widthHz / 1000.0, 0, 'f', 1)
                          .arg(candidate.peakDb, 0, 'f', 1)
                          .arg(candidate.excessDb, 0, 'f', 1);
    }
    dmrHunterControls->setStatusText(statusText);
    updateDmrHunterControls();
}

void YourClassName::updateDmrHunterControls() {
    dmrHunterSettings = DmrHunterDetector::normalizedSettings(dmrHunterSettings);
    const bool enabled = dmrHunterSettings.enabled;
    if (dmrHunterControls) {
        dmrHunterControls->setDetectChecked(enabled);
        dmrHunterControls->setWidthValues(dmrHunterSettings.minWidthKhz,
                                          dmrHunterSettings.maxWidthKhz,
                                          dmrHunterSettings.thresholdDb);
        dmrHunterControls->setControlsEnabled(enabled);
        const bool hasCandidate = !dmrHunterCandidates.empty();
        dmrHunterControls->setCandidateNavigationEnabled(enabled && hasCandidate);
        dmrHunterControls->setCandidateIndex(enabled ? dmrHunterCandidateIndex : -1,
                                             enabled ? static_cast<int>(dmrHunterCandidates.size()) : 0);
        dmrHunterControls->setTuneEnabled(enabled && (hasCandidate || dmrHunterLastResult.best.valid));
        if (!enabled) {
            dmrHunterCandidates.clear();
            dmrHunterCandidateIndex = -1;
            dmrHunterControls->setCandidateNavigationEnabled(false);
            dmrHunterControls->setCandidateIndex(-1, 0);
            dmrHunterControls->setStatusText(uiText(QStringLiteral("dmr_hunter_off"),
                                                    QStringLiteral("DMR Hunter: off")));
        }
    }
}

void YourClassName::applyDmrHunterPresetToScan() {
    if (!dmrHunterControls || !agileScanRangesEdit || !agileScanStepSpin) {
        return;
    }

    const QString spec = dmrHunterControls->currentPresetSpec();
    const QString ranges = agileScanPresetRanges(spec);
    const double step = agileScanPresetStepMhz(spec, 0.0125);
    if (ranges.isEmpty()) {
        return;
    }

    agileScanRangesMhz = ranges;
    agileScanStepMhz = step;
    agileScanEnabled = true;
    if (agileScanCheckbox) {
        QSignalBlocker blocker(agileScanCheckbox);
        agileScanCheckbox->setChecked(true);
    }
    {
        QSignalBlocker blocker(agileScanRangesEdit);
        agileScanRangesEdit->setText(agileScanRangesMhz);
    }
    {
        QSignalBlocker blocker(agileScanStepSpin);
        agileScanStepSpin->setValue(agileScanStepMhz);
    }
    updateAgileScanControls();
    savePersistentSettings();
}

void YourClassName::tuneDmrHunterCandidate() {
    if (dmrHunterCandidateIndex >= 0 &&
        dmrHunterCandidateIndex < static_cast<int>(dmrHunterCandidates.size())) {
        tuneDmrHunterCandidateIndex(dmrHunterCandidateIndex);
        return;
    }

    if (!dmrHunterLastResult.best.valid || !std::isfinite(dmrHunterLastResult.best.centerHz)) {
        return;
    }

    DmrHunterCandidate candidate = dmrHunterLastResult.best;
    dmrHunterCandidates = {candidate};
    dmrHunterCandidateIndex = 0;
    tuneDmrHunterCandidateIndex(0);
}

void YourClassName::selectDmrHunterCandidate(int direction) {
    if (dmrHunterCandidates.empty()) {
        updateDmrHunterControls();
        return;
    }

    const int count = static_cast<int>(dmrHunterCandidates.size());
    int nextIndex = dmrHunterCandidateIndex;
    if (nextIndex < 0 || nextIndex >= count) {
        nextIndex = direction < 0 ? count - 1 : 0;
    } else {
        nextIndex = (nextIndex + direction) % count;
        if (nextIndex < 0) {
            nextIndex += count;
        }
    }

    dmrHunterCandidateIndex = nextIndex;
    updateDmrHunterControls();
    tuneDmrHunterCandidateIndex(dmrHunterCandidateIndex);
}

void YourClassName::tuneDmrHunterCandidateIndex(int index) {
    if (index < 0 || index >= static_cast<int>(dmrHunterCandidates.size())) {
        return;
    }

    const DmrHunterCandidate &candidate = dmrHunterCandidates[static_cast<std::size_t>(index)];
    if (!candidate.valid || !std::isfinite(candidate.centerHz)) {
        return;
    }

    if (pendingSettings.modulationType != MOD_DMR) {
        if (modulationButtonGroup) {
            if (QAbstractButton *button = modulationButtonGroup->button(MOD_DMR)) {
                modulationButtonGroup->blockSignals(true);
                button->setChecked(true);
                modulationButtonGroup->blockSignals(false);
            }
        }
        onModulationChanged(MOD_DMR);
    }

    pendingSettings.bandwidth = 12500.0;
    if (bandwidthControl) {
        QSignalBlocker blocker(bandwidthControl);
        bandwidthControl->setValueHz(pendingSettings.bandwidth);
    }
    publishSettingsToGlobals();
    updateIqFrameProducerSettings();
    settingRange();
    updateTuningFromScale(candidate.centerHz, candidate.centerHz);
    savePersistentSettings();
}

void YourClassName::updateFpvHunter(const std::vector<float> &frequencies,
                                    const std::vector<float> &magnitudes) {
    if (!fpvHunterControls) {
        return;
    }
    if (!fpvHunterClock.isValid()) {
        fpvHunterClock.start();
    }
    const qint64 fpvHunterNowMs = fpvHunterClock.elapsed();

    double previousSelectedCenterHz = std::numeric_limits<double>::quiet_NaN();
    if (fpvHunterCandidateIndex >= 0 &&
        fpvHunterCandidateIndex < static_cast<int>(fpvHunterCandidates.size())) {
        previousSelectedCenterHz = fpvHunterCandidates[static_cast<std::size_t>(fpvHunterCandidateIndex)].centerHz;
    }

    fpvHunterSettings = FpvHunterDetector::normalizedSettings(fpvHunterSettings);
    fpvHunterLastResult = FpvHunterDetector::analyze(frequencies,
                                                     magnitudes,
                                                     fpvHunterSettings);
    fpvHunterCandidates = fpvHunterLastResult.candidateList;
    fpvHunterCandidateIndex = -1;
    if (!fpvHunterCandidates.empty()) {
        int bestIndex = 0;
        if (std::isfinite(previousSelectedCenterHz)) {
            double bestDeltaHz = std::numeric_limits<double>::max();
            for (int i = 0; i < static_cast<int>(fpvHunterCandidates.size()); ++i) {
                const double centerHz = fpvHunterCandidates[static_cast<std::size_t>(i)].centerHz;
                if (!std::isfinite(centerHz)) {
                    continue;
                }
                const double deltaHz = std::abs(centerHz - previousSelectedCenterHz);
                if (deltaHz < bestDeltaHz) {
                    bestDeltaHz = deltaHz;
                    bestIndex = i;
                }
            }
        }
        fpvHunterCandidateIndex = bestIndex;
    }

    ++fpvHunterFrameSequence;
    if (!fpvHunterSettings.enabled) {
        fpvHunterTrack = {};
    } else if (fpvHunterLastResult.best.valid &&
               std::isfinite(fpvHunterLastResult.best.centerHz)) {
        const FpvHunterCandidate &candidate = fpvHunterLastResult.best;
        const double matchWindowHz =
            (std::max)(2500000.0, (std::max)(candidate.widthHz, fpvHunterTrack.widthHz) * 0.65);
        const bool sameTrack =
            fpvHunterTrack.valid &&
            std::isfinite(fpvHunterTrack.centerHz) &&
            std::abs(candidate.centerHz - fpvHunterTrack.centerHz) <= matchWindowHz;

        if (!sameTrack) {
            fpvHunterTrack = {};
            fpvHunterTrack.valid = true;
            fpvHunterTrack.centerHz = candidate.centerHz;
            fpvHunterTrack.widthHz = candidate.widthHz;
            fpvHunterTrack.peakDb = candidate.peakDb;
            fpvHunterTrack.averageDb = candidate.averageDb;
            fpvHunterTrack.excessDb = candidate.excessDb;
            fpvHunterTrack.score = candidate.score;
            fpvHunterTrack.hits = 1;
            fpvHunterTrack.type = candidate.type;
            fpvHunterTrack.firstSeenMsec = fpvHunterNowMs;
        } else {
            constexpr double alpha = 0.35;
            fpvHunterTrack.centerHz =
                fpvHunterTrack.centerHz * (1.0 - alpha) + candidate.centerHz * alpha;
            fpvHunterTrack.widthHz =
                fpvHunterTrack.widthHz * (1.0 - alpha) + candidate.widthHz * alpha;
            fpvHunterTrack.peakDb = (std::max)(fpvHunterTrack.peakDb, candidate.peakDb);
            fpvHunterTrack.averageDb =
                static_cast<float>(fpvHunterTrack.averageDb * (1.0 - alpha) +
                                   candidate.averageDb * alpha);
            fpvHunterTrack.excessDb =
                static_cast<float>(fpvHunterTrack.excessDb * (1.0 - alpha) +
                                   candidate.excessDb * alpha);
            fpvHunterTrack.score =
                static_cast<float>(fpvHunterTrack.score * 0.75f + candidate.score * 0.25f);
            fpvHunterTrack.hits = (std::min)(fpvHunterTrack.hits + 1, 99);
            fpvHunterTrack.type = candidate.type.isEmpty() ? fpvHunterTrack.type : candidate.type;
        }
        rememberFpvHunterCandidate(candidate, !sameTrack, fpvHunterNowMs);
        fpvHunterTrack.misses = 0;
        fpvHunterTrack.lastSeenSequence = fpvHunterFrameSequence;
        fpvHunterTrack.lastSeenMsec = fpvHunterNowMs;
        if (fpvHunterTrack.firstSeenMsec < 0) {
            fpvHunterTrack.firstSeenMsec = fpvHunterNowMs;
        }
        fpvHunterTrack.stable = fpvHunterTrack.hits >= 3;
    } else if (fpvHunterTrack.valid) {
        ++fpvHunterTrack.misses;
        const bool staleByTime =
            fpvHunterTrack.lastSeenMsec >= 0 &&
            fpvHunterNowMs - fpvHunterTrack.lastSeenMsec > FPV_HUNTER_TRACK_HOLD_MS;
        if (staleByTime || fpvHunterTrack.misses > FPV_HUNTER_TRACK_HOLD_FRAMES) {
            fpvHunterTrack = {};
        } else {
            fpvHunterTrack.stable =
                fpvHunterTrack.hits >= 3 &&
                fpvHunterTrack.misses <= FPV_HUNTER_TRACK_STABLE_MISS_FRAMES;
        }
    }

    QString statusText = fpvHunterLastResult.statusText;
    if (fpvHunterTrack.valid) {
        FpvHunterCandidate tracked;
        tracked.valid = true;
        tracked.centerHz = fpvHunterTrack.centerHz;
        tracked.widthHz = fpvHunterTrack.widthHz;
        tracked.peakDb = fpvHunterTrack.peakDb;
        tracked.averageDb = fpvHunterTrack.averageDb;
        tracked.excessDb = fpvHunterTrack.excessDb;
        tracked.score = fpvHunterTrack.score;
        tracked.type = fpvHunterTrack.type;
        fpvHunterLastResult.best = tracked;

        const int confidence = (std::clamp)(fpvHunterTrack.hits * 18 - fpvHunterTrack.misses * 4,
                                            5,
                                            100);
        const QString state =
            fpvHunterTrack.misses > 0
                ? QStringLiteral("hold")
                : (fpvHunterTrack.stable ? QStringLiteral("stable") : QStringLiteral("tracking"));
        const qint64 ageMs =
            fpvHunterTrack.lastSeenMsec >= 0 ? fpvHunterNowMs - fpvHunterTrack.lastSeenMsec : -1;
        const qint64 durationMs =
            fpvHunterTrack.firstSeenMsec >= 0 && fpvHunterTrack.lastSeenMsec >= fpvHunterTrack.firstSeenMsec
                ? fpvHunterTrack.lastSeenMsec - fpvHunterTrack.firstSeenMsec
                : -1;
        const QString holdText =
            fpvHunterTrack.misses > 0
                ? QStringLiteral(", last seen %1 s ago")
                      .arg(ageMs >= 0 ? ageMs / 1000.0 : 0.0, 0, 'f', 1)
                : QString();
        const QString durationText =
            durationMs >= 1500
                ? QStringLiteral(", seen %1 s").arg(durationMs / 1000.0, 0, 'f', 1)
                : QString();
        statusText =
            QStringLiteral("%1\nFPV Hunter %2: %3 at %4 MHz, width %5 MHz, demod %6 MHz, confidence %7%, hits %8%9%10")
                .arg(fpvHunterLastResult.statusText)
                .arg(state)
                .arg(fpvHunterTrack.type.isEmpty() ? QStringLiteral("wide video") : fpvHunterTrack.type)
                .arg(fpvHunterTrack.centerHz / 1000000.0, 0, 'f', 3)
                .arg(fpvHunterTrack.widthHz / 1000000.0, 0, 'f', 2)
                .arg(recommendedFpvDemodBandwidthHz(fpvHunterTrack.widthHz) / 1000000.0, 0, 'f', 1)
                .arg(confidence)
                .arg(fpvHunterTrack.hits)
                .arg(durationText)
                .arg(holdText);
    }

    if (fpvHunterCandidateIndex >= 0 &&
        fpvHunterCandidateIndex < static_cast<int>(fpvHunterCandidates.size())) {
        const FpvHunterCandidate &candidate =
            fpvHunterCandidates[static_cast<std::size_t>(fpvHunterCandidateIndex)];
        statusText += QStringLiteral("\nSelected %1/%2: %3 at %4 MHz, width %5 MHz, demod %6 MHz, peak %7 dB")
                          .arg(fpvHunterCandidateIndex + 1)
                          .arg(static_cast<int>(fpvHunterCandidates.size()))
                          .arg(candidate.type.isEmpty() ? QStringLiteral("wide video") : candidate.type)
                          .arg(candidate.centerHz / 1000000.0, 0, 'f', 3)
                          .arg(candidate.widthHz / 1000000.0, 0, 'f', 2)
                          .arg(recommendedFpvDemodBandwidthHz(candidate.widthHz) / 1000000.0, 0, 'f', 1)
                          .arg(candidate.peakDb, 0, 'f', 1);
    }

    fpvHunterControls->setStatusText(statusText);
    updateFpvHunterControls();

    if (fpvHunterFollowEnabled &&
        fpvHunterCandidateIndex >= 0 &&
        fpvHunterCandidateIndex < static_cast<int>(fpvHunterCandidates.size())) {
        const FpvHunterCandidate &candidate =
            fpvHunterCandidates[static_cast<std::size_t>(fpvHunterCandidateIndex)];
        const double targetBandwidthHz = recommendedFpvDemodBandwidthHz(candidate.widthHz);
        const double centerThresholdHz =
            fpvHunterTrack.stable
                ? (std::max)(75000.0, targetBandwidthHz * 0.025)
                : (std::max)(150000.0, targetBandwidthHz * 0.05);
        const bool centerChanged =
            !std::isfinite(fpvHunterLastFollowCenterHz) ||
            std::abs(candidate.centerHz - fpvHunterLastFollowCenterHz) > centerThresholdHz;
        const bool bandwidthChanged =
            !std::isfinite(fpvHunterLastFollowBandwidthHz) ||
            std::abs(targetBandwidthHz - fpvHunterLastFollowBandwidthHz) > 500000.0;
        if (centerChanged || bandwidthChanged) {
            tuneFpvHunterCandidateValue(candidate, false);
        }
    }
}

void YourClassName::updateFpvHunterControls() {
    fpvHunterSettings = FpvHunterDetector::normalizedSettings(fpvHunterSettings);
    const bool enabled = fpvHunterSettings.enabled;
    if (fpvHunterControls) {
        fpvHunterControls->setDetectChecked(enabled);
        fpvHunterControls->setWidthValues(fpvHunterSettings.minWidthMhz,
                                          fpvHunterSettings.maxWidthMhz,
                                          fpvHunterSettings.thresholdDb);
        fpvHunterControls->setControlsEnabled(enabled);
        const bool hasCandidate = !fpvHunterCandidates.empty();
        fpvHunterControls->setCandidateNavigationEnabled(enabled && hasCandidate);
        fpvHunterControls->setCandidateIndex(enabled ? fpvHunterCandidateIndex : -1,
                                             enabled ? static_cast<int>(fpvHunterCandidates.size()) : 0);
        fpvHunterControls->setFollowChecked(fpvHunterFollowEnabled);
        fpvHunterControls->setFollowEnabled(enabled && hasCandidate);
        fpvHunterControls->setTuneEnabled(enabled && (hasCandidate ||
                                                      fpvHunterLastResult.best.valid ||
                                                      fpvHunterTrack.valid));
        if (!enabled) {
            fpvHunterCandidates.clear();
            fpvHunterCandidateIndex = -1;
            fpvHunterLastFollowCenterHz = std::numeric_limits<double>::quiet_NaN();
            fpvHunterLastFollowBandwidthHz = std::numeric_limits<double>::quiet_NaN();
            fpvHunterControls->setCandidateNavigationEnabled(false);
            fpvHunterControls->setCandidateIndex(-1, 0);
            fpvHunterControls->setFollowEnabled(false);
            fpvHunterControls->setStatusText(uiText(QStringLiteral("fpv_hunter_off"),
                                                    QStringLiteral("FPV Hunter: off")));
        }
    }
    updateFpvHunterHistoryControls();
}

void YourClassName::rememberFpvHunterCandidate(const FpvHunterCandidate &candidate,
                                               bool startNewEvent,
                                               qint64 nowMs) {
    if (!candidate.valid || !std::isfinite(candidate.centerHz)) {
        return;
    }
    if (nowMs < 0) {
        nowMs = 0;
    }

    int eventIndex = fpvHunterActiveEventIndex;
    if (startNewEvent || eventIndex < 0 || eventIndex >= fpvHunterEvents.size()) {
        eventIndex = -1;
        for (int i = 0; i < fpvHunterEvents.size(); ++i) {
            const FpvHunterEvent &event = fpvHunterEvents.at(i);
            if (!event.valid || event.lastSeenMsec < 0 ||
                nowMs - event.lastSeenMsec > FPV_HUNTER_TRACK_HOLD_MS) {
                continue;
            }
            const double matchWindowHz =
                (std::max)(2500000.0, (std::max)(candidate.widthHz, event.widthHz) * 0.70);
            if (std::isfinite(event.centerHz) &&
                std::abs(candidate.centerHz - event.centerHz) <= matchWindowHz) {
                eventIndex = i;
                break;
            }
        }
    }

    FpvHunterEvent event;
    if (eventIndex >= 0 && eventIndex < fpvHunterEvents.size()) {
        event = fpvHunterEvents.at(eventIndex);
        constexpr double alpha = 0.30;
        event.centerHz = event.centerHz * (1.0 - alpha) + candidate.centerHz * alpha;
        event.widthHz = event.widthHz * (1.0 - alpha) + candidate.widthHz * alpha;
        event.peakDb = (std::max)(event.peakDb, candidate.peakDb);
        event.averageDb =
            static_cast<float>(event.averageDb * (1.0 - alpha) + candidate.averageDb * alpha);
        event.excessDb =
            static_cast<float>(event.excessDb * (1.0 - alpha) + candidate.excessDb * alpha);
        event.score = (std::max)(event.score, candidate.score);
        event.hits = (std::min)(event.hits + 1, 999);
        event.lastSeenMsec = nowMs;
        if (!candidate.type.isEmpty()) {
            event.type = candidate.type;
        }
        fpvHunterEvents.removeAt(eventIndex);
    } else {
        event.valid = true;
        event.id = fpvHunterNextEventId++;
        event.centerHz = candidate.centerHz;
        event.widthHz = candidate.widthHz;
        event.peakDb = candidate.peakDb;
        event.averageDb = candidate.averageDb;
        event.excessDb = candidate.excessDb;
        event.score = candidate.score;
        event.hits = 1;
        event.firstSeenMsec = nowMs;
        event.lastSeenMsec = nowMs;
        event.type = candidate.type;
    }

    fpvHunterEvents.prepend(event);
    while (fpvHunterEvents.size() > FPV_HUNTER_MAX_EVENTS) {
        fpvHunterEvents.removeLast();
    }
    fpvHunterActiveEventIndex = 0;
    updateFpvHunterHistoryControls();
}

void YourClassName::updateFpvHunterHistoryControls() {
    const bool hasEvents = !fpvHunterEvents.isEmpty();
    if (fpvHunterHistoryCombo) {
        QSignalBlocker blocker(fpvHunterHistoryCombo);
        fpvHunterHistoryCombo->clear();
        if (!hasEvents) {
            fpvHunterHistoryCombo->addItem(QStringLiteral("No FPV events yet"), -1);
        } else {
            const qint64 nowMs = fpvHunterClock.isValid() ? fpvHunterClock.elapsed() : -1;
            for (int i = 0; i < fpvHunterEvents.size(); ++i) {
                const FpvHunterEvent &event = fpvHunterEvents.at(i);
                const double durationSec =
                    event.firstSeenMsec >= 0 && event.lastSeenMsec >= event.firstSeenMsec
                        ? (event.lastSeenMsec - event.firstSeenMsec) / 1000.0
                        : 0.0;
                const double ageSec =
                    nowMs >= 0 && event.lastSeenMsec >= 0
                        ? (std::max)(static_cast<qint64>(0), nowMs - event.lastSeenMsec) / 1000.0
                        : 0.0;
                const QString item =
                    QStringLiteral("%1 MHz, W %2 MHz, pk %3 dB, %4, %5 s, age %6 s")
                        .arg(event.centerHz / 1000000.0, 0, 'f', 3)
                        .arg(event.widthHz / 1000000.0, 0, 'f', 2)
                        .arg(event.peakDb, 0, 'f', 1)
                        .arg(event.type.isEmpty() ? QStringLiteral("wide video") : event.type)
                        .arg(durationSec, 0, 'f', 1)
                        .arg(ageSec, 0, 'f', 1);
                fpvHunterHistoryCombo->addItem(item, i);
            }
        }
    }
    if (fpvHunterHistoryTuneButton) {
        fpvHunterHistoryTuneButton->setEnabled(hasEvents);
    }
    if (fpvHunterHistoryClearButton) {
        fpvHunterHistoryClearButton->setEnabled(hasEvents);
    }
}

void YourClassName::tuneFpvHunterHistorySelection() {
    if (!fpvHunterHistoryCombo || fpvHunterEvents.isEmpty()) {
        return;
    }
    bool ok = false;
    int index = fpvHunterHistoryCombo->currentData().toInt(&ok);
    if (!ok || index < 0 || index >= fpvHunterEvents.size()) {
        index = fpvHunterHistoryCombo->currentIndex();
    }
    if (index < 0 || index >= fpvHunterEvents.size()) {
        return;
    }

    const FpvHunterEvent &event = fpvHunterEvents.at(index);
    if (!event.valid || !std::isfinite(event.centerHz)) {
        return;
    }

    FpvHunterCandidate candidate;
    candidate.valid = true;
    candidate.centerHz = event.centerHz;
    candidate.widthHz = event.widthHz;
    candidate.peakDb = event.peakDb;
    candidate.averageDb = event.averageDb;
    candidate.excessDb = event.excessDb;
    candidate.score = event.score;
    candidate.type = event.type;

    fpvHunterActiveEventIndex = index;
    fpvHunterLastResult.best = candidate;
    fpvHunterCandidates = {candidate};
    fpvHunterCandidateIndex = 0;
    tuneFpvHunterCandidateValue(candidate, true);
    updateFpvHunterControls();
}

void YourClassName::clearFpvHunterHistory() {
    fpvHunterEvents.clear();
    fpvHunterActiveEventIndex = -1;
    updateFpvHunterHistoryControls();
}

void YourClassName::applyFpvHunterPresetToScan() {
    if (!fpvHunterControls || !agileScanRangesEdit || !agileScanStepSpin) {
        return;
    }

    const QString spec = fpvHunterControls->currentPresetSpec();
    const QString ranges = agileScanPresetRanges(spec);
    const double step = agileScanPresetStepMhz(spec, 5.0);
    if (ranges.isEmpty()) {
        return;
    }

    agileScanRangesMhz = ranges;
    agileScanStepMhz = step;
    agileScanEnabled = true;
    if (agileScanCheckbox) {
        QSignalBlocker blocker(agileScanCheckbox);
        agileScanCheckbox->setChecked(true);
    }
    {
        QSignalBlocker blocker(agileScanRangesEdit);
        agileScanRangesEdit->setText(agileScanRangesMhz);
    }
    {
        QSignalBlocker blocker(agileScanStepSpin);
        agileScanStepSpin->setValue(agileScanStepMhz);
    }
    updateAgileScanControls();
    savePersistentSettings();
}

void YourClassName::tuneFpvHunterCandidate() {
    if (fpvHunterCandidateIndex >= 0 &&
        fpvHunterCandidateIndex < static_cast<int>(fpvHunterCandidates.size())) {
        tuneFpvHunterCandidateIndex(fpvHunterCandidateIndex);
        return;
    }

    if (!fpvHunterLastResult.best.valid || !std::isfinite(fpvHunterLastResult.best.centerHz)) {
        return;
    }

    FpvHunterCandidate candidate = fpvHunterLastResult.best;
    fpvHunterCandidates = {candidate};
    fpvHunterCandidateIndex = 0;
    tuneFpvHunterCandidateIndex(0);
}

void YourClassName::selectFpvHunterCandidate(int direction) {
    if (fpvHunterCandidates.empty()) {
        updateFpvHunterControls();
        return;
    }

    const int count = static_cast<int>(fpvHunterCandidates.size());
    int nextIndex = fpvHunterCandidateIndex;
    if (nextIndex < 0 || nextIndex >= count) {
        nextIndex = direction < 0 ? count - 1 : 0;
    } else {
        nextIndex = (nextIndex + direction) % count;
        if (nextIndex < 0) {
            nextIndex += count;
        }
    }

    fpvHunterCandidateIndex = nextIndex;
    updateFpvHunterControls();
    tuneFpvHunterCandidateIndex(fpvHunterCandidateIndex);
}

void YourClassName::tuneFpvHunterCandidateIndex(int index) {
    if (index < 0 || index >= static_cast<int>(fpvHunterCandidates.size())) {
        return;
    }

    const FpvHunterCandidate &candidate = fpvHunterCandidates[static_cast<std::size_t>(index)];
    tuneFpvHunterCandidateValue(candidate, true);
}

void YourClassName::tuneFpvHunterCandidateValue(const FpvHunterCandidate &candidate, bool saveSettings) {
    if (!candidate.valid || !std::isfinite(candidate.centerHz)) {
        return;
    }

    if (pendingSettings.modulationType != MOD_ATV) {
        if (modulationButtonGroup) {
            if (QAbstractButton *button = modulationButtonGroup->button(MOD_ATV)) {
                modulationButtonGroup->blockSignals(true);
                button->setChecked(true);
                modulationButtonGroup->blockSignals(false);
            }
        }
        onModulationChanged(MOD_ATV);
    }

    videoDecodeEnabled = true;
    if (videoDecodeCheckbox) {
        QSignalBlocker blocker(videoDecodeCheckbox);
        videoDecodeCheckbox->setChecked(true);
    }
    if (videoDemodCombo && videoDemodCombo->currentIndex() != 0) {
        QSignalBlocker blocker(videoDemodCombo);
        videoDemodCombo->setCurrentIndex(0);
    }
    if (videoToggleButton && !videoToggleButton->isChecked()) {
        QSignalBlocker blocker(videoToggleButton);
        videoToggleButton->setChecked(true);
    }
    if (videoDock && !videoDock->isVisible()) {
        videoDock->show();
    }

    const double videoBandwidthHz = recommendedFpvDemodBandwidthHz(candidate.widthHz);
    pendingSettings.bandwidth = videoBandwidthHz;
    if (bandwidthControl) {
        QSignalBlocker blocker(bandwidthControl);
        bandwidthControl->setValueHz(pendingSettings.bandwidth);
    }
    publishSettingsToGlobals();
    updateVideoProcessorMode();
    updateIqFrameProducerSettings();
    settingRange();
    updateTuningFromScale(candidate.centerHz, candidate.centerHz);
    fpvHunterLastFollowCenterHz = candidate.centerHz;
    fpvHunterLastFollowBandwidthHz = videoBandwidthHz;
    if (saveSettings) {
        savePersistentSettings();
    }
}

void YourClassName::updateDigitalVideoHunter(const std::vector<float> &frequencies,
                                             const std::vector<float> &magnitudes) {
    if (!digitalVideoHunterControls) {
        return;
    }

    double previousSelectedCenterHz = std::numeric_limits<double>::quiet_NaN();
    if (digitalVideoHunterCandidateIndex >= 0 &&
        digitalVideoHunterCandidateIndex < static_cast<int>(digitalVideoHunterCandidates.size())) {
        previousSelectedCenterHz =
            digitalVideoHunterCandidates[static_cast<std::size_t>(digitalVideoHunterCandidateIndex)].centerHz;
    }

    digitalVideoHunterSettings = DigitalVideoHunterDetector::normalizedSettings(digitalVideoHunterSettings);
    digitalVideoHunterLastResult = DigitalVideoHunterDetector::analyze(frequencies,
                                                                       magnitudes,
                                                                       digitalVideoHunterSettings);
    digitalVideoHunterCandidates = digitalVideoHunterLastResult.candidateList;
    digitalVideoHunterCandidateIndex = -1;
    if (!digitalVideoHunterCandidates.empty()) {
        int bestIndex = 0;
        if (std::isfinite(previousSelectedCenterHz)) {
            double bestDeltaHz = std::numeric_limits<double>::max();
            for (int i = 0; i < static_cast<int>(digitalVideoHunterCandidates.size()); ++i) {
                const double centerHz = digitalVideoHunterCandidates[static_cast<std::size_t>(i)].centerHz;
                if (!std::isfinite(centerHz)) {
                    continue;
                }
                const double deltaHz = std::abs(centerHz - previousSelectedCenterHz);
                if (deltaHz < bestDeltaHz) {
                    bestDeltaHz = deltaHz;
                    bestIndex = i;
                }
            }
        }
        digitalVideoHunterCandidateIndex = bestIndex;
    }

    QString statusText = digitalVideoHunterLastResult.statusText;
    if (digitalVideoHunterCandidateIndex >= 0 &&
        digitalVideoHunterCandidateIndex < static_cast<int>(digitalVideoHunterCandidates.size())) {
        const DigitalVideoHunterCandidate &candidate =
            digitalVideoHunterCandidates[static_cast<std::size_t>(digitalVideoHunterCandidateIndex)];
        statusText += QStringLiteral("\nSelected %1/%2: %3 at %4 MHz, width %5 MHz, BW %6 MHz, flat %7 dB, occ %8%, peak %9 dB")
                          .arg(digitalVideoHunterCandidateIndex + 1)
                          .arg(static_cast<int>(digitalVideoHunterCandidates.size()))
                          .arg(candidate.type.isEmpty() ? QStringLiteral("wide digital") : candidate.type)
                          .arg(candidate.centerHz / 1000000.0, 0, 'f', 3)
                          .arg(candidate.widthHz / 1000000.0, 0, 'f', 2)
                          .arg(recommendedDigitalVideoBandwidthHz(candidate.widthHz) / 1000000.0, 0, 'f', 1)
                          .arg(candidate.flatnessDb, 0, 'f', 1)
                          .arg(candidate.occupancy * 100.0f, 0, 'f', 0)
                          .arg(candidate.peakDb, 0, 'f', 1);
    }
    digitalVideoHunterControls->setStatusText(statusText);
    updateDigitalVideoHunterControls();
}

void YourClassName::updateDigitalVideoHunterControls() {
    digitalVideoHunterSettings = DigitalVideoHunterDetector::normalizedSettings(digitalVideoHunterSettings);
    const bool enabled = digitalVideoHunterSettings.enabled;
    if (digitalVideoHunterControls) {
        digitalVideoHunterControls->setDetectChecked(enabled);
        digitalVideoHunterControls->setWidthValues(digitalVideoHunterSettings.minWidthMhz,
                                                   digitalVideoHunterSettings.maxWidthMhz,
                                                   digitalVideoHunterSettings.thresholdDb);
        digitalVideoHunterControls->setControlsEnabled(enabled);
        const bool hasCandidate = !digitalVideoHunterCandidates.empty();
        digitalVideoHunterControls->setCandidateNavigationEnabled(enabled && hasCandidate);
        digitalVideoHunterControls->setCandidateIndex(enabled ? digitalVideoHunterCandidateIndex : -1,
                                                      enabled ? static_cast<int>(digitalVideoHunterCandidates.size()) : 0);
        digitalVideoHunterControls->setTuneEnabled(enabled && (hasCandidate || digitalVideoHunterLastResult.best.valid));
        if (!enabled) {
            digitalVideoHunterCandidates.clear();
            digitalVideoHunterCandidateIndex = -1;
            digitalVideoHunterControls->setCandidateNavigationEnabled(false);
            digitalVideoHunterControls->setCandidateIndex(-1, 0);
            digitalVideoHunterControls->setStatusText(uiText(QStringLiteral("digital_video_hunter_off"),
                                                             QStringLiteral("Digital Video Hunter: off")));
        }
    }
}

void YourClassName::applyDigitalVideoHunterPresetToScan() {
    if (!digitalVideoHunterControls || !agileScanRangesEdit || !agileScanStepSpin) {
        return;
    }

    const QString spec = digitalVideoHunterControls->currentPresetSpec();
    const QString ranges = agileScanPresetRanges(spec);
    const double step = agileScanPresetStepMhz(spec, 5.0);
    if (ranges.isEmpty()) {
        return;
    }

    agileScanRangesMhz = ranges;
    agileScanStepMhz = step;
    agileScanEnabled = true;
    if (agileScanCheckbox) {
        QSignalBlocker blocker(agileScanCheckbox);
        agileScanCheckbox->setChecked(true);
    }
    {
        QSignalBlocker blocker(agileScanRangesEdit);
        agileScanRangesEdit->setText(agileScanRangesMhz);
    }
    {
        QSignalBlocker blocker(agileScanStepSpin);
        agileScanStepSpin->setValue(agileScanStepMhz);
    }
    updateAgileScanControls();
    savePersistentSettings();
}

void YourClassName::tuneDigitalVideoHunterCandidate() {
    if (digitalVideoHunterCandidateIndex >= 0 &&
        digitalVideoHunterCandidateIndex < static_cast<int>(digitalVideoHunterCandidates.size())) {
        tuneDigitalVideoHunterCandidateIndex(digitalVideoHunterCandidateIndex);
        return;
    }

    if (!digitalVideoHunterLastResult.best.valid ||
        !std::isfinite(digitalVideoHunterLastResult.best.centerHz)) {
        return;
    }

    DigitalVideoHunterCandidate candidate = digitalVideoHunterLastResult.best;
    digitalVideoHunterCandidates = {candidate};
    digitalVideoHunterCandidateIndex = 0;
    tuneDigitalVideoHunterCandidateIndex(0);
}

void YourClassName::selectDigitalVideoHunterCandidate(int direction) {
    if (digitalVideoHunterCandidates.empty()) {
        updateDigitalVideoHunterControls();
        return;
    }

    const int count = static_cast<int>(digitalVideoHunterCandidates.size());
    int nextIndex = digitalVideoHunterCandidateIndex;
    if (nextIndex < 0 || nextIndex >= count) {
        nextIndex = direction < 0 ? count - 1 : 0;
    } else {
        nextIndex = (nextIndex + direction) % count;
        if (nextIndex < 0) {
            nextIndex += count;
        }
    }

    digitalVideoHunterCandidateIndex = nextIndex;
    updateDigitalVideoHunterControls();
    tuneDigitalVideoHunterCandidateIndex(digitalVideoHunterCandidateIndex);
}

void YourClassName::tuneDigitalVideoHunterCandidateIndex(int index) {
    if (index < 0 || index >= static_cast<int>(digitalVideoHunterCandidates.size())) {
        return;
    }

    const DigitalVideoHunterCandidate &candidate =
        digitalVideoHunterCandidates[static_cast<std::size_t>(index)];
    tuneDigitalVideoHunterCandidateValue(candidate, true);
}

void YourClassName::tuneDigitalVideoHunterCandidateValue(const DigitalVideoHunterCandidate &candidate,
                                                        bool saveSettings) {
    if (!candidate.valid || !std::isfinite(candidate.centerHz)) {
        return;
    }

    pendingSettings.bandwidth = recommendedDigitalVideoBandwidthHz(candidate.widthHz);
    if (bandwidthControl) {
        QSignalBlocker blocker(bandwidthControl);
        bandwidthControl->setValueHz(pendingSettings.bandwidth);
    }

    publishSettingsToGlobals();
    updateIqFrameProducerSettings();
    settingRange();
    updateTuningFromScale(candidate.centerHz, candidate.centerHz);
    if (saveSettings) {
        savePersistentSettings();
    }
}

void YourClassName::resetScanMeasurementPeaks() {
    for (auto &bin : scanMeasurementBins) {
        bin.peakDb = bin.currentDb;
    }
    updateScanMeasurementStatus();
}

void YourClassName::clearScanMeasurement() {
    scanMeasurementBins.clear();
    scanMeasurementSequence = 0;
    updateScanMeasurementStatus();
}

void YourClassName::exportScanMeasurementCsv() {
    if (scanMeasurementBins.isEmpty()) {
        QMessageBox::information(this,
                                 uiText(QStringLiteral("scan_measurement_title"),
                                        QStringLiteral("Scan measurement")),
                                 uiText(QStringLiteral("scan_measurement_no_data_export"),
                                        QStringLiteral("No scan measurement data to export.")));
        return;
    }

    const QString defaultPath =
        QDir(QCoreApplication::applicationDirPath()).filePath(
            QStringLiteral("scan_measurement_%1.csv").arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"))));
    const QString path = QFileDialog::getSaveFileName(this,
                                                      uiText(QStringLiteral("export_scan_measurement_csv"),
                                                             QStringLiteral("Export scan measurement CSV")),
                                                      defaultPath,
                                                      uiText(QStringLiteral("csv_files_filter"),
                                                             QStringLiteral("CSV files (*.csv)")));
    if (path.isEmpty()) {
        return;
    }

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        QMessageBox::warning(this,
                             uiText(QStringLiteral("scan_measurement_title"),
                                    QStringLiteral("Scan measurement")),
                             uiText(QStringLiteral("scan_measurement_csv_write_failed"),
                                    QStringLiteral("Cannot write CSV file.")));
        return;
    }

    QTextStream out(&file);
    out << "frequency_mhz,current_db,peak_db,baseline_db,delta_db,seen_count,baseline_count\n";
    for (const ScanMeasurementBin &bin : scanMeasurementBins) {
        const bool hasBaseline = bin.baselineCount > 0;
        const float delta = hasBaseline ? (bin.peakDb - bin.baselineDb) : 0.0f;
        out << QString::number(bin.frequencyHz / 1000000.0, 'f', 6) << ','
            << QString::number(bin.currentDb, 'f', 2) << ','
            << QString::number(bin.peakDb, 'f', 2) << ','
            << (hasBaseline ? QString::number(bin.baselineDb, 'f', 2) : QString()) << ','
            << (hasBaseline ? QString::number(delta, 'f', 2) : QString()) << ','
            << bin.seenCount << ','
            << bin.baselineCount << '\n';
    }
    updateScanMeasurementStatus();
}

void YourClassName::startSpurCalibration() {
    spurCalibrationBins.clear();
    spurCalibrationFramesDone = 0;
    spurCalibrationTargetFrames = SPUR_CALIBRATION_TARGET_FRAMES;
    spurCalibrationBinHz = 0.0;
    spurCalibrationActive = true;
    updateSpurSuppressionStatus();
    qDebug() << "[Spur] calibration started"
             << "targetFrames" << spurCalibrationTargetFrames
             << "sampleRate" << pendingSettings.sampleRate
             << "fftLength" << pendingSettings.fftLength
             << "inputMode" << pendingSettings.inputMode;
}

void YourClassName::clearSpurMask() {
    spurCalibrationActive = false;
    spurCalibrationBins.clear();
    spurMaskEntries.clear();
    spurSuppressionEnabled = false;
    if (spurSuppressionCheckbox) {
        QSignalBlocker blocker(spurSuppressionCheckbox);
        spurSuppressionCheckbox->setChecked(false);
    }
    updateSpurSuppressionStatus();
    savePersistentSettings();
}

void YourClassName::updateSpurCalibration(const std::vector<float> &frequencies,
                                          const std::vector<float> &magnitudes,
                                          double centerFrequency) {
    if (!spurCalibrationActive ||
        !std::isfinite(centerFrequency) ||
        frequencies.empty() ||
        magnitudes.empty()) {
        return;
    }

    const int dataCount = std::min(static_cast<int>(frequencies.size()),
                                   static_cast<int>(magnitudes.size()));
    if (dataCount <= SPUR_CALIBRATION_OUTER_BINS * 2 + 4 ||
        qFuzzyCompare(frequencies.front(), frequencies.back())) {
        return;
    }

    const double spanHz = std::abs(static_cast<double>(frequencies.back()) -
                                   static_cast<double>(frequencies.front()));
    const double binHz = (std::max)(25.0, spanHz / static_cast<double>((std::max)(1, dataCount)) * 3.0);
    if (spurCalibrationBinHz <= 0.0) {
        spurCalibrationBinHz = binHz;
    }

    std::vector<float> levels(static_cast<std::size_t>(dataCount), -160.0f);
    std::vector<double> prefixSum(static_cast<std::size_t>(dataCount + 1), 0.0);
    std::vector<int> prefixCount(static_cast<std::size_t>(dataCount + 1), 0);
    for (int i = 0; i < dataCount; ++i) {
        float level = magnitudes[static_cast<std::size_t>((i + dataCount / 2) % dataCount)];
        if (!std::isfinite(level)) {
            level = -160.0f;
        }
        levels[static_cast<std::size_t>(i)] = level;
        prefixSum[static_cast<std::size_t>(i + 1)] =
            prefixSum[static_cast<std::size_t>(i)] + static_cast<double>(level);
        prefixCount[static_cast<std::size_t>(i + 1)] =
            prefixCount[static_cast<std::size_t>(i)] + (std::isfinite(level) ? 1 : 0);
    }

    struct FrameCandidate {
        double offsetHz = 0.0;
        float level = -160.0f;
        float prominenceDb = 0.0f;
    };
    QMap<qint64, FrameCandidate> frameCandidates;

    auto rangeAverage = [&](int start, int end, int *countOut) {
        start = (std::clamp)(start, 0, dataCount);
        end = (std::clamp)(end, 0, dataCount);
        if (end <= start) {
            if (countOut) {
                *countOut = 0;
            }
            return -160.0;
        }
        const double sum = prefixSum[static_cast<std::size_t>(end)] -
                           prefixSum[static_cast<std::size_t>(start)];
        const int count = prefixCount[static_cast<std::size_t>(end)] -
                          prefixCount[static_cast<std::size_t>(start)];
        if (countOut) {
            *countOut = count;
        }
        return count > 0 ? sum / static_cast<double>(count) : -160.0;
    };

    for (int i = SPUR_CALIBRATION_OUTER_BINS;
         i < dataCount - SPUR_CALIBRATION_OUTER_BINS;
         ++i) {
        const float level = levels[static_cast<std::size_t>(i)];
        if (!std::isfinite(level) ||
            level < levels[static_cast<std::size_t>(i - 1)] ||
            level < levels[static_cast<std::size_t>(i + 1)]) {
            continue;
        }

        const float sideMax = (std::max)(levels[static_cast<std::size_t>(i - SPUR_CALIBRATION_INNER_BINS)],
                                         levels[static_cast<std::size_t>(i + SPUR_CALIBRATION_INNER_BINS)]);
        if (level - sideMax < SPUR_CALIBRATION_MIN_NARROW_DB) {
            continue;
        }

        int leftCount = 0;
        int rightCount = 0;
        const double leftAverage = rangeAverage(i - SPUR_CALIBRATION_OUTER_BINS,
                                                i - SPUR_CALIBRATION_INNER_BINS,
                                                &leftCount);
        const double rightAverage = rangeAverage(i + SPUR_CALIBRATION_INNER_BINS + 1,
                                                 i + SPUR_CALIBRATION_OUTER_BINS + 1,
                                                 &rightCount);
        if (leftCount + rightCount < 12) {
            continue;
        }
        const double baseline = (leftAverage * leftCount + rightAverage * rightCount) /
                                static_cast<double>(leftCount + rightCount);
        const float prominence = static_cast<float>(level - baseline);
        if (!std::isfinite(prominence) ||
            prominence < SPUR_CALIBRATION_MIN_PROMINENCE_DB) {
            continue;
        }

        const double offsetHz = static_cast<double>(frequencies[static_cast<std::size_t>(i)]) - centerFrequency;
        if (!std::isfinite(offsetHz)) {
            continue;
        }

        const qint64 key = static_cast<qint64>(std::llround(offsetHz / binHz));
        auto candidateIt = frameCandidates.find(key);
        if (candidateIt == frameCandidates.end() ||
            prominence > candidateIt.value().prominenceDb) {
            frameCandidates[key] = {offsetHz, level, prominence};
        }
    }

    for (auto it = frameCandidates.constBegin(); it != frameCandidates.constEnd(); ++it) {
        const FrameCandidate &candidate = it.value();
        const double weight = (std::max)(1.0, static_cast<double>(candidate.prominenceDb));
        SpurCalibrationBin &bin = spurCalibrationBins[it.key()];
        bin.offsetWeightedSum += candidate.offsetHz * weight;
        bin.weightSum += weight;
        bin.maxProminenceDb = (std::max)(bin.maxProminenceDb, candidate.prominenceDb);
        ++bin.hits;
    }

    ++spurCalibrationFramesDone;
    updateSpurSuppressionStatus();
    if (spurCalibrationFramesDone >= spurCalibrationTargetFrames) {
        finishSpurCalibration();
    }
}

void YourClassName::finishSpurCalibration() {
    spurCalibrationActive = false;

    QVector<SpurMaskEntry> candidates;
    const int minHits = (std::max)(4, spurCalibrationTargetFrames / 5);
    const double widthHz = (std::clamp)(spurCalibrationBinHz * 3.5,
                                        SPUR_MIN_MASK_WIDTH_HZ,
                                        SPUR_MAX_MASK_WIDTH_HZ);
    for (auto it = spurCalibrationBins.constBegin(); it != spurCalibrationBins.constEnd(); ++it) {
        const SpurCalibrationBin &bin = it.value();
        if (bin.hits < minHits || bin.weightSum <= 0.0) {
            continue;
        }
        SpurMaskEntry entry;
        entry.offsetHz = bin.offsetWeightedSum / bin.weightSum;
        entry.widthHz = widthHz;
        entry.prominenceDb = bin.maxProminenceDb;
        entry.hits = bin.hits;
        if (std::isfinite(entry.offsetHz) && std::isfinite(entry.widthHz)) {
            candidates.append(entry);
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const SpurMaskEntry &a, const SpurMaskEntry &b) {
        if (a.hits != b.hits) {
            return a.hits > b.hits;
        }
        return a.prominenceDb > b.prominenceDb;
    });

    QVector<SpurMaskEntry> merged;
    for (const SpurMaskEntry &candidate : std::as_const(candidates)) {
        bool mergedIntoExisting = false;
        for (SpurMaskEntry &existing : merged) {
            const double mergeDistance = (std::max)(existing.widthHz, candidate.widthHz);
            if (std::abs(existing.offsetHz - candidate.offsetHz) <= mergeDistance) {
                if (candidate.prominenceDb > existing.prominenceDb || candidate.hits > existing.hits) {
                    existing.offsetHz = candidate.offsetHz;
                    existing.prominenceDb = (std::max)(existing.prominenceDb, candidate.prominenceDb);
                    existing.hits = (std::max)(existing.hits, candidate.hits);
                    existing.widthHz = (std::max)(existing.widthHz, candidate.widthHz);
                }
                mergedIntoExisting = true;
                break;
            }
        }
        if (!mergedIntoExisting) {
            merged.append(candidate);
        }
        if (merged.size() >= SPUR_MAX_MASK_ENTRIES) {
            break;
        }
    }

    std::sort(merged.begin(), merged.end(), [](const SpurMaskEntry &a, const SpurMaskEntry &b) {
        return a.offsetHz < b.offsetHz;
    });

    spurMaskEntries = merged;
    spurCalibrationBins.clear();
    spurSuppressionEnabled = !spurMaskEntries.isEmpty();
    if (spurSuppressionCheckbox) {
        QSignalBlocker blocker(spurSuppressionCheckbox);
        spurSuppressionCheckbox->setChecked(spurSuppressionEnabled);
    }
    updateSpurSuppressionStatus();
    savePersistentSettings();

    QStringList offsets;
    for (const SpurMaskEntry &entry : std::as_const(spurMaskEntries)) {
        offsets << QStringLiteral("%1 kHz").arg(entry.offsetHz / 1000.0, 0, 'f', 1);
    }
    qDebug() << "[Spur] calibration finished"
             << "entries" << spurMaskEntries.size()
             << "offsets" << offsets.join(QStringLiteral(", "));
}

void YourClassName::applySpurSuppression(const std::vector<float> &frequencies,
                                         std::vector<float> &magnitudes,
                                         double centerFrequency) const {
    if (!spurSuppressionEnabled ||
        spurMaskEntries.isEmpty() ||
        !std::isfinite(centerFrequency) ||
        frequencies.empty() ||
        magnitudes.empty()) {
        return;
    }

    const int dataCount = std::min(static_cast<int>(frequencies.size()),
                                   static_cast<int>(magnitudes.size()));
    if (dataCount <= 8) {
        return;
    }

    for (const SpurMaskEntry &entry : spurMaskEntries) {
        if (!std::isfinite(entry.offsetHz) || !std::isfinite(entry.widthHz) || entry.widthHz <= 0.0) {
            continue;
        }
        const double targetFrequency = centerFrequency + entry.offsetHz;
        const double halfWidth = entry.widthHz * 0.5;
        const auto lower = std::lower_bound(frequencies.begin(),
                                            frequencies.begin() + dataCount,
                                            static_cast<float>(targetFrequency - halfWidth));
        const auto upper = std::upper_bound(frequencies.begin(),
                                            frequencies.begin() + dataCount,
                                            static_cast<float>(targetFrequency + halfWidth));
        int start = static_cast<int>(std::distance(frequencies.begin(), lower));
        int end = static_cast<int>(std::distance(frequencies.begin(), upper));
        start = (std::clamp)(start, 0, dataCount);
        end = (std::clamp)(end, 0, dataCount);
        if (end <= start) {
            continue;
        }

        const int guardBins = (std::max)(2, end - start);
        const int leftStart = (std::max)(0, start - guardBins * 3);
        const int leftEnd = (std::max)(leftStart, start - guardBins);
        const int rightStart = (std::min)(dataCount, end + guardBins);
        const int rightEnd = (std::min)(dataCount, end + guardBins * 3);

        double replacementSum = 0.0;
        int replacementCount = 0;
        auto accumulate = [&](int from, int to) {
            for (int i = from; i < to; ++i) {
                const int magnitudeIndex = (i + dataCount / 2) % dataCount;
                const float level = magnitudes[static_cast<std::size_t>(magnitudeIndex)];
                if (std::isfinite(level)) {
                    replacementSum += level;
                    ++replacementCount;
                }
            }
        };
        accumulate(leftStart, leftEnd);
        accumulate(rightStart, rightEnd);
        if (replacementCount <= 0) {
            continue;
        }

        const float replacement = static_cast<float>(replacementSum / replacementCount);
        for (int i = start; i < end; ++i) {
            const int magnitudeIndex = (i + dataCount / 2) % dataCount;
            float &level = magnitudes[static_cast<std::size_t>(magnitudeIndex)];
            if (!std::isfinite(level) || level > replacement) {
                level = replacement;
            }
        }
    }
}

void YourClassName::updateSpurSuppressionStatus() {
    if (!spurSuppressionStatusLabel) {
        return;
    }

    auto setSpurStatus = [this](const QString &text) {
        spurSuppressionStatusLabel->setToolTip(text);
        spurSuppressionStatusLabel->setText(text);
    };

    if (spurCalibrationActive) {
        setSpurStatus(
            uiText(QStringLiteral("spur_cal_status"),
                   QStringLiteral("Spur cal: %1/%2 frames, %3 candidates"))
                .arg(spurCalibrationFramesDone)
                .arg(spurCalibrationTargetFrames)
                .arg(spurCalibrationBins.size()));
        return;
    }

    if (spurMaskEntries.isEmpty()) {
        setSpurStatus(uiText(QStringLiteral("spur_mask_no_profile"),
                             QStringLiteral("Spur mask: no profile")));
        return;
    }

    QStringList offsets;
    for (int i = 0; i < spurMaskEntries.size() && i < 6; ++i) {
        offsets << QStringLiteral("%1k").arg(spurMaskEntries.at(i).offsetHz / 1000.0, 0, 'f', 1);
    }
    const QString suffix = spurMaskEntries.size() > 6 ? QStringLiteral(", ...") : QString();
    setSpurStatus(
        uiText(QStringLiteral("spur_mask_status"),
               QStringLiteral("Spur mask: %1, %2 offsets [%3%4]"))
            .arg(spurSuppressionEnabled
                     ? uiText(QStringLiteral("on"), QStringLiteral("on"))
                     : uiText(QStringLiteral("off"), QStringLiteral("off")))
            .arg(spurMaskEntries.size())
            .arg(offsets.join(QStringLiteral(", ")))
            .arg(suffix));
}

void YourClassName::saveAgileScanPreset() {
    if (!agileScanPresetCombo) {
        return;
    }
    QString name = agileScanPresetCombo->currentText().trimmed();
    if (name.isEmpty()) {
        name = QStringLiteral("Preset %1").arg(agileScanPresets.size() + 1);
    }
    refreshSettingsFromUi();
    agileScanPresets[name] = agileScanPresetSpec(agileScanRangesMhz, agileScanStepMhz);
    agileScanPresetCombo->setEditText(name);
    updateAgileScanControls();
    savePersistentSettings();
}

void YourClassName::deleteAgileScanPreset() {
    if (!agileScanPresetCombo) {
        return;
    }
    const QString name = agileScanPresetCombo->currentText().trimmed();
    if (!name.isEmpty()) {
        agileScanPresets.remove(name);
    }
    updateAgileScanControls();
    savePersistentSettings();
}

void YourClassName::ensureDefaultFrequencyPresets() {
    if (centerFrequencyPresets.isEmpty()) {
        centerFrequencyPresets[QStringLiteral("FM broadcast 100 MHz")] = 100000000.0;
        centerFrequencyPresets[QStringLiteral("Airband 125 MHz")] = 125000000.0;
        centerFrequencyPresets[QStringLiteral("VHF 145 MHz")] = 145000000.0;
        centerFrequencyPresets[QStringLiteral("UHF 433 MHz")] = 433000000.0;
        centerFrequencyPresets[QStringLiteral("GSM/LTE 900 MHz")] = 900000000.0;
        centerFrequencyPresets[QStringLiteral("FPV 1.2 GHz")] = 1200000000.0;
        centerFrequencyPresets[QStringLiteral("FPV 2.4 GHz")] = 2400000000.0;
        centerFrequencyPresets[QStringLiteral("Experimental 7.0 GHz")] = 7000000000.0;
        centerFrequencyPresets[QStringLiteral("Experimental 7.5 GHz")] = 7500000000.0;
    }
    if (listeningFrequencyPresets.isEmpty()) {
        listeningFrequencyPresets[QStringLiteral("HF center 0 Hz")] = 0.0;
        listeningFrequencyPresets[QStringLiteral("HF 500 kHz")] = 500000.0;
        listeningFrequencyPresets[QStringLiteral("HF 1.25 MHz")] = 1250000.0;
        listeningFrequencyPresets[QStringLiteral("80 m 3.65 MHz")] = 3650000.0;
        listeningFrequencyPresets[QStringLiteral("40 m 7.05 MHz")] = 7050000.0;
        listeningFrequencyPresets[QStringLiteral("20 m FT8 14.074 MHz")] = 14074000.0;
        listeningFrequencyPresets[QStringLiteral("VHF 145 MHz")] = 145000000.0;
        listeningFrequencyPresets[QStringLiteral("UHF 433 MHz")] = 433000000.0;
    }
    auto addMissingFrequencyPreset = [this](const QString &name, double valueHz) {
        if (!centerFrequencyPresets.contains(name)) {
            centerFrequencyPresets[name] = valueHz;
        }
        if (!listeningFrequencyPresets.contains(name)) {
            listeningFrequencyPresets[name] = valueHz;
        }
    };
    const QVector<double> fpvVideoPresetMhz = {
        1440.0, 1450.0, 1600.0, 1620.0,
        4990.0, 5010.0, 5360.0, 5460.0,
        5640.0, 5660.0, 5680.0, 5880.0,
        5890.0, 5910.0
    };
    for (double mhz : fpvVideoPresetMhz) {
        addMissingFrequencyPreset(QStringLiteral("FPV video %1 MHz").arg(mhz, 0, 'f', 0),
                                  mhz * 1000000.0);
    }
    if (bandwidthValuePresets.isEmpty()) {
        bandwidthValuePresets[QStringLiteral("CW 500 Hz")] = 500.0;
        bandwidthValuePresets[QStringLiteral("SSB 2.7 kHz")] = 2700.0;
        bandwidthValuePresets[QStringLiteral("FT8 3 kHz")] = 3000.0;
        bandwidthValuePresets[QStringLiteral("AM 6 kHz")] = 6000.0;
        bandwidthValuePresets[QStringLiteral("AM 10 kHz")] = 10000.0;
        bandwidthValuePresets[QStringLiteral("NFM 12.5 kHz")] = 12500.0;
        bandwidthValuePresets[QStringLiteral("DMR 12.5 kHz")] = 12500.0;
        bandwidthValuePresets[QStringLiteral("WFM 200 kHz")] = 200000.0;
        bandwidthValuePresets[QStringLiteral("SSTV 3 kHz")] = 3000.0;
        bandwidthValuePresets[QStringLiteral("NOAA APT 40 kHz")] = 40000.0;
        bandwidthValuePresets[QStringLiteral("WEFAX 3 kHz")] = 3000.0;
        bandwidthValuePresets[QStringLiteral("LRPT 140 kHz")] = 140000.0;
        bandwidthValuePresets[QStringLiteral("ATV 3 MHz")] = 3000000.0;
        bandwidthValuePresets[QStringLiteral("ATV 5 MHz")] = 5000000.0;
        bandwidthValuePresets[QStringLiteral("FPV 8 MHz")] = 8000000.0;
        bandwidthValuePresets[QStringLiteral("FPV 10 MHz")] = 10000000.0;
    }
    auto addMissingBandwidthPreset = [this](const QString &name, double valueHz) {
        if (!bandwidthValuePresets.contains(name)) {
            bandwidthValuePresets[name] = valueHz;
        }
    };
    addMissingBandwidthPreset(QStringLiteral("ATV 3 MHz"), 3000000.0);
    addMissingBandwidthPreset(QStringLiteral("ATV 5 MHz"), 5000000.0);
    addMissingBandwidthPreset(QStringLiteral("FPV 8 MHz"), 8000000.0);
    addMissingBandwidthPreset(QStringLiteral("FPV 10 MHz"), 10000000.0);
}

void YourClassName::ensureDefaultBandMarkers() {
    if (bandMarkersCustomized || !bandMarkers.isEmpty()) {
        return;
    }

    auto addMhz = [this](const char *label, double startMhz, double endMhz, bool amateur) {
        GraphBandMarker marker;
        marker.startHz = startMhz * 1000000.0;
        marker.endHz = endMhz * 1000000.0;
        marker.label = QString::fromLatin1(label);
        marker.amateur = amateur;
        bandMarkers.append(marker);
    };

    addMhz("MW BC", 0.5265, 1.705, false);
    addMhz("SW 49m", 5.9, 6.2, false);
    addMhz("SW 41m", 7.2, 7.45, false);
    addMhz("SW 31m", 9.4, 9.9, false);
    addMhz("SW 25m", 11.6, 12.1, false);
    addMhz("SW 19m", 15.1, 15.8, false);
    addMhz("CB", 26.965, 27.405, false);
    addMhz("FM BC", 87.5, 108.0, false);
    addMhz("Air", 118.0, 137.0, false);
    addMhz("WX Sat", 137.0, 138.0, false);
    addMhz("Marine", 156.0, 162.05, false);
    addMhz("UHF Satcom", 240.0, 270.0, false);
    addMhz("TETRA", 380.0, 430.0, false);
    addMhz("PMR446", 446.0, 446.2, false);
    addMhz("ADS-B", 1089.5, 1090.5, false);
    addMhz("L-band Sat", 1525.0, 1660.5, false);
    addMhz("GNSS L1", 1559.0, 1610.0, false);
    addMhz("ISM 2.4", 2400.0, 2483.5, false);
    addMhz("FPV 5.8", 5650.0, 5925.0, false);

    addMhz("2200m", 0.1357, 0.1378, true);
    addMhz("630m", 0.472, 0.479, true);
    addMhz("160m", 1.81, 2.0, true);
    addMhz("80m", 3.5, 3.8, true);
    addMhz("60m", 5.3515, 5.3665, true);
    addMhz("40m", 7.0, 7.2, true);
    addMhz("30m", 10.1, 10.15, true);
    addMhz("20m", 14.0, 14.35, true);
    addMhz("17m", 18.068, 18.168, true);
    addMhz("15m", 21.0, 21.45, true);
    addMhz("12m", 24.89, 24.99, true);
    addMhz("10m", 28.0, 29.7, true);
    addMhz("6m", 50.0, 52.0, true);
    addMhz("4m", 70.0, 70.5, true);
    addMhz("2m", 144.0, 146.0, true);
    addMhz("70cm", 430.0, 440.0, true);
    addMhz("23cm", 1240.0, 1300.0, true);
    addMhz("13cm", 2300.0, 2450.0, true);
    addMhz("6cm", 5650.0, 5850.0, true);
}

QVector<QPair<QString, double>> YourClassName::presetMapToVector(const QMap<QString, double> &presets) const {
    QVector<QPair<QString, double>> values;
    values.reserve(presets.size());
    for (auto it = presets.constBegin(); it != presets.constEnd(); ++it) {
        if (!it.key().trimmed().isEmpty() && std::isfinite(it.value())) {
            values.append(qMakePair(it.key(), it.value()));
        }
    }
    return values;
}

void YourClassName::updateFrequencyPresetControls() {
    ensureDefaultFrequencyPresets();
    if (frequencyControl) {
        frequencyControl->setValuePresets(presetMapToVector(centerFrequencyPresets));
    }
    if (listeningFrequencyControl) {
        listeningFrequencyControl->setValuePresets(presetMapToVector(listeningFrequencyPresets));
    }
    if (bandwidthControl) {
        bandwidthControl->setValuePresets(presetMapToVector(bandwidthValuePresets));
    }
    updateAgileScanControls();
}

void YourClassName::updateGraphBandMarkers() {
    if (!graphWidget) {
        return;
    }
    graphWidget->setBandMarkers(bandMarkers);
    graphWidget->setBandMarkersEnabled(showGeneralBandMarkers, showAmateurBandMarkers);
    graphWidget->setBandMarkersCompact(compactBandMarkers);
}

void YourClassName::setControlsPanelVisible(bool visible) {
    if (!controlsDock) {
        return;
    }

    if (!visible) {
        controlsDock->hide();
        return;
    }

    if (controlsDock->isFloating()) {
        controlsDock->hide();
        controlsDock->setFloating(false);
        addDockWidget(Qt::LeftDockWidgetArea, controlsDock);
    }
    controlsDock->show();
}

void YourClassName::openPresetManager() {
    ensureDefaultFrequencyPresets();
    ensureDefaultBandMarkers();

    QDialog dialog(this);
    dialog.setWindowTitle(uiText(QStringLiteral("preset_manager"), QStringLiteral("Preset Manager")));
    dialog.resize(720, 520);

    QVBoxLayout *rootLayout = new QVBoxLayout(&dialog);
    QTabWidget *tabs = new QTabWidget(&dialog);

    auto makeNumericTab = [this, &dialog](const QMap<QString, double> &presets,
                                          const QString &valueHeader,
                                          double minimum,
                                          double maximum) -> QTableWidget * {
        QWidget *page = new QWidget(&dialog);
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        QTableWidget *table = new QTableWidget(page);
        table->setColumnCount(2);
        table->setHorizontalHeaderLabels({uiText(QStringLiteral("name"), QStringLiteral("Name")), valueHeader});
        table->horizontalHeader()->setStretchLastSection(true);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setRowCount(presets.size());
        int row = 0;
        for (auto it = presets.constBegin(); it != presets.constEnd(); ++it, ++row) {
            table->setItem(row, 0, new QTableWidgetItem(it.key()));
            table->setItem(row, 1, new QTableWidgetItem(QString::number(it.value(), 'f', 3)));
        }

        QHBoxLayout *buttonLayout = new QHBoxLayout();
        QPushButton *addButton = new QPushButton(uiText(QStringLiteral("add"), QStringLiteral("Add")), page);
        QPushButton *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), page);
        buttonLayout->addWidget(addButton);
        buttonLayout->addWidget(removeButton);
        buttonLayout->addStretch();
        pageLayout->addWidget(table);
        pageLayout->addLayout(buttonLayout);

        QObject::connect(addButton, &QPushButton::clicked, table, [this, table, minimum]() {
            const int row = table->rowCount();
            table->insertRow(row);
            table->setItem(row, 0, new QTableWidgetItem(uiText(QStringLiteral("new_preset"), QStringLiteral("New preset"))));
            table->setItem(row, 1, new QTableWidgetItem(QString::number((std::max)(0.0, minimum), 'f', 3)));
            table->setCurrentCell(row, 0);
            table->editItem(table->item(row, 0));
        });
        QObject::connect(removeButton, &QPushButton::clicked, table, [this, table]() {
            const int row = table->currentRow();
            if (row >= 0) {
                table->removeRow(row);
            }
        });

        table->setProperty("minimumValue", minimum);
        table->setProperty("maximumValue", maximum);
        table->setProperty("pageWidget", QVariant::fromValue(static_cast<void*>(page)));
        return table;
    };

    auto makeAgileTab = [this, &dialog](const QMap<QString, QString> &presets) -> QTableWidget * {
        QWidget *page = new QWidget(&dialog);
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        QTableWidget *table = new QTableWidget(page);
        table->setColumnCount(3);
        table->setHorizontalHeaderLabels({uiText(QStringLiteral("name"), QStringLiteral("Name")),
                                          uiText(QStringLiteral("ranges_mhz_plain"), QStringLiteral("Ranges MHz")),
                                          uiText(QStringLiteral("step_mhz"), QStringLiteral("Step MHz"))});
        table->horizontalHeader()->setStretchLastSection(true);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setRowCount(presets.size());
        int row = 0;
        for (auto it = presets.constBegin(); it != presets.constEnd(); ++it, ++row) {
            table->setItem(row, 0, new QTableWidgetItem(it.key()));
            table->setItem(row, 1, new QTableWidgetItem(agileScanPresetRanges(it.value())));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(agileScanPresetStepMhz(it.value(), 0.0125), 'f', 6)));
        }

        QHBoxLayout *buttonLayout = new QHBoxLayout();
        QPushButton *addButton = new QPushButton(uiText(QStringLiteral("add"), QStringLiteral("Add")), page);
        QPushButton *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), page);
        buttonLayout->addWidget(addButton);
        buttonLayout->addWidget(removeButton);
        buttonLayout->addStretch();
        pageLayout->addWidget(table);
        pageLayout->addLayout(buttonLayout);

        QObject::connect(addButton, &QPushButton::clicked, table, [this, table]() {
            const int row = table->rowCount();
            table->insertRow(row);
            table->setItem(row, 0, new QTableWidgetItem(uiText(QStringLiteral("new_scan_preset"), QStringLiteral("New scan preset"))));
            table->setItem(row, 1, new QTableWidgetItem(QStringLiteral("430-432")));
            table->setItem(row, 2, new QTableWidgetItem(QStringLiteral("0.0125")));
            table->setCurrentCell(row, 0);
            table->editItem(table->item(row, 0));
        });
        QObject::connect(removeButton, &QPushButton::clicked, table, [this, table]() {
            const int row = table->currentRow();
            if (row >= 0) {
                table->removeRow(row);
            }
        });

        table->setProperty("pageWidget", QVariant::fromValue(static_cast<void*>(page)));
        return table;
    };

    auto makeStandardScanTab = [this, &dialog](const QMap<QString, QString> &presets) -> QTableWidget * {
        QWidget *page = new QWidget(&dialog);
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        QTableWidget *table = new QTableWidget(page);
        table->setColumnCount(4);
        table->setHorizontalHeaderLabels({uiText(QStringLiteral("name"), QStringLiteral("Name")),
                                          uiText(QStringLiteral("centers_mhz_plain"), QStringLiteral("Centers MHz")),
                                          uiText(QStringLiteral("dwell_ms"), QStringLiteral("Dwell ms")),
                                          uiText(QStringLiteral("settle_ms"), QStringLiteral("Settle ms"))});
        table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setRowCount(presets.size());
        int row = 0;
        for (auto it = presets.constBegin(); it != presets.constEnd(); ++it, ++row) {
            table->setItem(row, 0, new QTableWidgetItem(it.key()));
            table->setItem(row, 1, new QTableWidgetItem(standardScanPresetCenters(it.value())));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(standardScanPresetDwellMs(it.value(), standardScanDwellMs))));
            table->setItem(row, 3, new QTableWidgetItem(QString::number(standardScanPresetSettleMs(it.value(), standardScanSettleMs))));
        }

        QHBoxLayout *buttonLayout = new QHBoxLayout();
        QPushButton *addButton = new QPushButton(uiText(QStringLiteral("add"), QStringLiteral("Add")), page);
        QPushButton *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), page);
        buttonLayout->addWidget(addButton);
        buttonLayout->addWidget(removeButton);
        buttonLayout->addStretch();
        pageLayout->addWidget(table);
        pageLayout->addLayout(buttonLayout);

        QObject::connect(addButton, &QPushButton::clicked, table, [this, table]() {
            const int row = table->rowCount();
            table->insertRow(row);
            table->setItem(row, 0, new QTableWidgetItem(uiText(QStringLiteral("new_standard_scan_preset"),
                                                               QStringLiteral("New standard scan preset"))));
            table->setItem(row, 1, new QTableWidgetItem(standardScanCentersMhz));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(standardScanDwellMs)));
            table->setItem(row, 3, new QTableWidgetItem(QString::number(standardScanSettleMs)));
            table->setCurrentCell(row, 0);
            table->editItem(table->item(row, 0));
        });
        QObject::connect(removeButton, &QPushButton::clicked, table, [table]() {
            const int row = table->currentRow();
            if (row >= 0) {
                table->removeRow(row);
            }
        });

        table->setProperty("pageWidget", QVariant::fromValue(static_cast<void*>(page)));
        return table;
    };

    auto makeBandMarkerTab = [this, &dialog](const QVector<GraphBandMarker> &markers) -> QTableWidget * {
        QWidget *page = new QWidget(&dialog);
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        QTableWidget *table = new QTableWidget(page);
        table->setColumnCount(4);
        table->setHorizontalHeaderLabels({uiText(QStringLiteral("layer"), QStringLiteral("Layer")),
                                          uiText(QStringLiteral("label"), QStringLiteral("Label")),
                                          uiText(QStringLiteral("start_mhz"), QStringLiteral("Start MHz")),
                                          uiText(QStringLiteral("end_mhz"), QStringLiteral("End MHz"))});
        table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);

        auto setLayerCell = [this, table](int row, bool amateur) {
            QComboBox *layerCombo = new QComboBox(table);
            layerCombo->addItem(uiText(QStringLiteral("common"), QStringLiteral("Common")), QStringLiteral("general"));
            layerCombo->addItem(QStringLiteral("HAM"), QStringLiteral("amateur"));
            layerCombo->setCurrentIndex(amateur ? 1 : 0);
            table->setCellWidget(row, 0, layerCombo);
        };
        auto setBandRow = [table, setLayerCell](int row, const GraphBandMarker &marker) {
            setLayerCell(row, marker.amateur);
            table->setItem(row, 1, new QTableWidgetItem(marker.label));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(marker.startHz / 1000000.0, 'f', 6)));
            table->setItem(row, 3, new QTableWidgetItem(QString::number(marker.endHz / 1000000.0, 'f', 6)));
        };

        table->setRowCount(markers.size());
        for (int row = 0; row < markers.size(); ++row) {
            setBandRow(row, markers.at(row));
        }

        QHBoxLayout *buttonLayout = new QHBoxLayout();
        QPushButton *addCommonButton = new QPushButton(uiText(QStringLiteral("add_common"), QStringLiteral("Add common")), page);
        QPushButton *addHamButton = new QPushButton(uiText(QStringLiteral("add_ham"), QStringLiteral("Add HAM")), page);
        QPushButton *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), page);
        buttonLayout->addWidget(addCommonButton);
        buttonLayout->addWidget(addHamButton);
        buttonLayout->addWidget(removeButton);
        buttonLayout->addStretch();
        pageLayout->addWidget(table);
        pageLayout->addLayout(buttonLayout);

        auto addBandRow = [this, table, setBandRow](bool amateur) {
            const int row = table->rowCount();
            table->insertRow(row);
            GraphBandMarker marker;
            marker.label = amateur ? uiText(QStringLiteral("new_ham_band"), QStringLiteral("New HAM band"))
                                   : uiText(QStringLiteral("new_band"), QStringLiteral("New band"));
            marker.startHz = amateur ? 144000000.0 : 118000000.0;
            marker.endHz = amateur ? 146000000.0 : 137000000.0;
            marker.amateur = amateur;
            setBandRow(row, marker);
            table->setCurrentCell(row, 1);
            table->editItem(table->item(row, 1));
        };
        QObject::connect(addCommonButton, &QPushButton::clicked, table, [addBandRow]() {
            addBandRow(false);
        });
        QObject::connect(addHamButton, &QPushButton::clicked, table, [addBandRow]() {
            addBandRow(true);
        });
        QObject::connect(removeButton, &QPushButton::clicked, table, [table]() {
            const int row = table->currentRow();
            if (row >= 0) {
                table->removeRow(row);
            }
        });

        table->setProperty("pageWidget", QVariant::fromValue(static_cast<void*>(page)));
        return table;
    };

    QTableWidget *centerTable = makeNumericTab(centerFrequencyPresets,
                                              uiText(QStringLiteral("frequency_hz"), QStringLiteral("Frequency Hz")),
                                              0.0,
                                              RF_EXPERIMENTAL_MAX_FREQUENCY);
    QTableWidget *listeningTable = makeNumericTab(listeningFrequencyPresets,
                                                 uiText(QStringLiteral("frequency_hz"), QStringLiteral("Frequency Hz")),
                                                 -RF_EXPERIMENTAL_MAX_FREQUENCY,
                                                 RF_EXPERIMENTAL_MAX_FREQUENCY);
    QTableWidget *bandwidthTable = makeNumericTab(bandwidthValuePresets,
                                                 uiText(QStringLiteral("bandwidth_hz"), QStringLiteral("Bandwidth Hz")),
                                                 1.0,
                                                 20000000.0);
    QTableWidget *agileTable = makeAgileTab(agileScanPresets);
    QTableWidget *standardScanTable = makeStandardScanTab(standardScanPresets);
    QTableWidget *bandMarkerTable = makeBandMarkerTab(bandMarkers);

    tabs->addTab(static_cast<QWidget*>(centerTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("preset_tab_center"), QStringLiteral("Center")));
    tabs->addTab(static_cast<QWidget*>(listeningTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("preset_tab_listen"), QStringLiteral("Listen")));
    tabs->addTab(static_cast<QWidget*>(bandwidthTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("preset_tab_audio_bw"), QStringLiteral("Audio BW")));
    tabs->addTab(static_cast<QWidget*>(agileTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("agile_scan"), QStringLiteral("Agile scan")));
    tabs->addTab(static_cast<QWidget*>(standardScanTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("preset_tab_standard_scan"), QStringLiteral("Standard scan")));
    tabs->addTab(static_cast<QWidget*>(bandMarkerTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("general_band_markers"), QStringLiteral("Band markers")));

    QLabel *hintLabel = new QLabel(uiText(QStringLiteral("presets_hint"),
                                          QStringLiteral("Values are stored in Hz for frequency/audio presets. Scan presets and band-marker ranges are edited in MHz. HAM defaults are Region-1-style hints; edit them for local rules.")),
                                   &dialog);
    hintLabel->setWordWrap(true);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    if (QPushButton *okButton = buttonBox->button(QDialogButtonBox::Ok)) {
        okButton->setText(uiText(QStringLiteral("ok"), QStringLiteral("OK")));
    }
    if (QPushButton *cancelButton = buttonBox->button(QDialogButtonBox::Cancel)) {
        cancelButton->setText(uiText(QStringLiteral("cancel"), QStringLiteral("Cancel")));
    }
    rootLayout->addWidget(tabs);
    rootLayout->addWidget(hintLabel);
    rootLayout->addWidget(buttonBox);

    auto readNumericTable = [](QTableWidget *table, QMap<QString, double> &target, QString *error) {
        QMap<QString, double> next;
        const double minimum = table->property("minimumValue").toDouble();
        const double maximum = table->property("maximumValue").toDouble();
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString name = table->item(row, 0) ? table->item(row, 0)->text().trimmed() : QString();
            const QString valueText = table->item(row, 1) ? table->item(row, 1)->text().trimmed() : QString();
            if (name.isEmpty() && valueText.isEmpty()) {
                continue;
            }
            bool ok = false;
            const double value = valueText.toDouble(&ok);
            if (name.isEmpty() || !ok || !std::isfinite(value) || value < minimum || value > maximum) {
                if (error) {
                    *error = QStringLiteral("Bad numeric preset at row %1").arg(row + 1);
                }
                return false;
            }
            next[name] = value;
        }
        target = next;
        return true;
    };

    auto readAgileTable = [](QTableWidget *table, QMap<QString, QString> &target, QString *error) {
        QMap<QString, QString> next;
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString name = table->item(row, 0) ? table->item(row, 0)->text().trimmed() : QString();
            const QString ranges = table->item(row, 1) ? table->item(row, 1)->text().trimmed() : QString();
            const QString stepText = table->item(row, 2) ? table->item(row, 2)->text().trimmed() : QString();
            if (name.isEmpty() && ranges.isEmpty() && stepText.isEmpty()) {
                continue;
            }
            bool ok = false;
            const double step = stepText.toDouble(&ok);
            if (name.isEmpty() || ranges.isEmpty() || !ok ||
                !std::isfinite(step) ||
                step < AGILE_SCAN_MIN_STEP_MHZ ||
                step > AGILE_SCAN_MAX_STEP_MHZ) {
                if (error) {
                    *error = QStringLiteral("Bad Agile scan preset at row %1").arg(row + 1);
                }
                return false;
            }
            QString parseError;
            parseAgileScanFrequenciesMhz(ranges, step, &parseError);
            if (!parseError.isEmpty()) {
                if (error) {
                    *error = QStringLiteral("%1: %2").arg(name, parseError);
                }
                return false;
            }
            next[name] = agileScanPresetSpec(ranges, step);
        }
        target = next;
        return true;
    };

    auto readStandardScanTable = [this](QTableWidget *table, QMap<QString, QString> &target, QString *error) {
        QMap<QString, QString> next;
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString name = table->item(row, 0) ? table->item(row, 0)->text().trimmed() : QString();
            const QString centers = table->item(row, 1) ? table->item(row, 1)->text().trimmed() : QString();
            const QString dwellText = table->item(row, 2) ? table->item(row, 2)->text().trimmed() : QString();
            const QString settleText = table->item(row, 3) ? table->item(row, 3)->text().trimmed() : QString();
            if (name.isEmpty() && centers.isEmpty() && dwellText.isEmpty() && settleText.isEmpty()) {
                continue;
            }
            bool dwellOk = false;
            bool settleOk = false;
            const int dwellMs = dwellText.toInt(&dwellOk);
            const int settleMs = settleText.toInt(&settleOk);
            if (name.isEmpty() ||
                centers.isEmpty() ||
                !dwellOk ||
                !settleOk ||
                dwellMs < STANDARD_SCAN_MIN_DWELL_MS ||
                dwellMs > STANDARD_SCAN_MAX_DWELL_MS ||
                settleMs < STANDARD_SCAN_MIN_SETTLE_MS ||
                settleMs > STANDARD_SCAN_MAX_SETTLE_MS) {
                if (error) {
                    *error = QStringLiteral("Bad standard scan preset at row %1").arg(row + 1);
                }
                return false;
            }
            QString parseError;
            parseStandardScanCentersMhz(centers,
                                        pendingSettings.sampleRate,
                                        AGILE_SCAN_MIN_POINTS,
                                        &parseError,
                                        nullptr);
            if (!parseError.isEmpty()) {
                if (error) {
                    *error = QStringLiteral("%1: %2").arg(name, parseError);
                }
                return false;
            }
            next[name] = standardScanPresetSpec(centers, dwellMs, settleMs);
        }
        target = next;
        return true;
    };

    auto readBandMarkerTable = [](QTableWidget *table, QVector<GraphBandMarker> &target, QString *error) {
        QVector<GraphBandMarker> next;
        for (int row = 0; row < table->rowCount(); ++row) {
            QComboBox *layerCombo = qobject_cast<QComboBox*>(table->cellWidget(row, 0));
            const QString layer = layerCombo ? layerCombo->currentData().toString() : QStringLiteral("general");
            const QString label = table->item(row, 1) ? table->item(row, 1)->text().trimmed() : QString();
            const QString startText = table->item(row, 2) ? table->item(row, 2)->text().trimmed() : QString();
            const QString endText = table->item(row, 3) ? table->item(row, 3)->text().trimmed() : QString();
            if (label.isEmpty() && startText.isEmpty() && endText.isEmpty()) {
                continue;
            }

            bool startOk = false;
            bool endOk = false;
            const double startMhz = startText.toDouble(&startOk);
            const double endMhz = endText.toDouble(&endOk);
            if (label.isEmpty() ||
                !startOk ||
                !endOk ||
                !std::isfinite(startMhz) ||
                !std::isfinite(endMhz) ||
                startMhz < 0.0 ||
                endMhz <= startMhz ||
                endMhz > 100000.0) {
                if (error) {
                    *error = QStringLiteral("Bad band marker at row %1").arg(row + 1);
                }
                return false;
            }

            GraphBandMarker marker;
            marker.startHz = startMhz * 1000000.0;
            marker.endHz = endMhz * 1000000.0;
            marker.label = label;
            marker.amateur = layer == QStringLiteral("amateur");
            next.append(marker);
        }
        target = next;
        return true;
    };

    connect(buttonBox, &QDialogButtonBox::accepted, &dialog, [&]() {
        QString error;
        QMap<QString, double> nextCenter = centerFrequencyPresets;
        QMap<QString, double> nextListening = listeningFrequencyPresets;
        QMap<QString, double> nextBandwidth = bandwidthValuePresets;
        QMap<QString, QString> nextAgile = agileScanPresets;
        QMap<QString, QString> nextStandardScan = standardScanPresets;
        QVector<GraphBandMarker> nextBandMarkers = bandMarkers;
        if (!readNumericTable(centerTable, nextCenter, &error) ||
            !readNumericTable(listeningTable, nextListening, &error) ||
            !readNumericTable(bandwidthTable, nextBandwidth, &error) ||
            !readAgileTable(agileTable, nextAgile, &error) ||
            !readStandardScanTable(standardScanTable, nextStandardScan, &error) ||
            !readBandMarkerTable(bandMarkerTable, nextBandMarkers, &error)) {
            QMessageBox::warning(&dialog,
                                  uiText(QStringLiteral("preset_manager"), QStringLiteral("Preset Manager")),
                                  error);
            return;
        }
        centerFrequencyPresets = nextCenter;
        listeningFrequencyPresets = nextListening;
        bandwidthValuePresets = nextBandwidth;
        agileScanPresets = nextAgile;
        standardScanPresets = nextStandardScan;
        bandMarkers = nextBandMarkers;
        bandMarkersCustomized = true;
        updateFrequencyPresetControls();
        updateGraphBandMarkers();
        savePersistentSettings();
        dialog.accept();
    });
    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    dialog.exec();
}

void YourClassName::openApplicationSettings() {
    QDialog dialog(this);
    dialog.setWindowTitle(uiText(QStringLiteral("settings"), QStringLiteral("Settings...")));
    dialog.setMinimumWidth(420);

    QVBoxLayout *rootLayout = new QVBoxLayout(&dialog);

    QFormLayout *generalLayout = new QFormLayout();
    QComboBox *languageCombo = new QComboBox(&dialog);
    languageCombo->addItem(QStringLiteral("English"), QStringLiteral("en"));
    languageCombo->addItem(QString::fromUtf8("Українська"), QStringLiteral("uk"));
    languageCombo->setCurrentIndex(languageCombo->findData(uiLanguage));
    if (languageCombo->currentIndex() < 0) {
        languageCombo->setCurrentIndex(0);
    }

    QComboBox *fineTuneModeCombo = new QComboBox(&dialog);
    fineTuneModeCombo->addItem(uiText(QStringLiteral("fine_tune_scale"), QStringLiteral("Horizontal scale (mouse wheel)")),
                               FINE_TUNE_MODE_SCALE);
    fineTuneModeCombo->addItem(uiText(QStringLiteral("fine_tune_dial"), QStringLiteral("Round dial")),
                               FINE_TUNE_MODE_DIAL);
    const int fineTuneIndex = fineTuneModeCombo->findData(fineTuneControlMode);
    fineTuneModeCombo->setCurrentIndex(fineTuneIndex >= 0 ? fineTuneIndex : 0);

    QSpinBox *spectrumUpdateSpin = new QSpinBox(&dialog);
    spectrumUpdateSpin->setRange(SPECTRUM_UPDATE_AUTO_MS, SPECTRUM_UPDATE_MAX_MS);
    spectrumUpdateSpin->setSpecialValueText(uiText(QStringLiteral("auto"), QStringLiteral("Auto")));
    spectrumUpdateSpin->setSuffix(QStringLiteral(" ms"));
    spectrumUpdateSpin->setSingleStep(5);
    spectrumUpdateSpin->setValue(spectrumUpdateIntervalMs);
    spectrumUpdateSpin->setToolTip(uiText(
        QStringLiteral("spectrum_update_interval_tooltip"),
        QStringLiteral("Spectrum and waterfall update interval. Auto keeps the FFT-dependent default.")));

    generalLayout->addRow(uiText(QStringLiteral("language"), QStringLiteral("Lang:")), languageCombo);
    generalLayout->addRow(uiText(QStringLiteral("fine_tune"), QStringLiteral("Fine tune")), fineTuneModeCombo);
    generalLayout->addRow(uiText(QStringLiteral("spectrum_update_interval"), QStringLiteral("Spectrum/waterfall update")), spectrumUpdateSpin);
    rootLayout->addLayout(generalLayout);

    QGroupBox *quickOptionsBox = new QGroupBox(uiText(QStringLiteral("quick_options"), QStringLiteral("Quick options")), &dialog);
    QGridLayout *quickOptionsLayout = new QGridLayout(quickOptionsBox);
    QCheckBox *audioOption = new QCheckBox(uiText(QStringLiteral("audio"), QStringLiteral("Audio")), quickOptionsBox);
    QCheckBox *syncOption = new QCheckBox(uiText(QStringLiteral("sync"), QStringLiteral("Sync")), quickOptionsBox);
    QCheckBox *spectrum2Option = new QCheckBox(uiText(QStringLiteral("spectrum2"), QStringLiteral("Spectr 2")), quickOptionsBox);
    QCheckBox *colorOption = new QCheckBox(uiText(QStringLiteral("colorful"), QStringLiteral("Colorful")), quickOptionsBox);
    QCheckBox *generalBandMarkersOption = new QCheckBox(uiText(QStringLiteral("general_band_markers"), QStringLiteral("Band markers")), quickOptionsBox);
    QCheckBox *amateurBandMarkersOption = new QCheckBox(uiText(QStringLiteral("amateur_band_markers"), QStringLiteral("HAM bands")), quickOptionsBox);
    QCheckBox *compactBandMarkersOption = new QCheckBox(uiText(QStringLiteral("compact_band_markers"), QStringLiteral("Collapsed")), quickOptionsBox);
    QCheckBox *loggingOption = new QCheckBox(uiText(QStringLiteral("logging"), QString::fromUtf8("Логування")), quickOptionsBox);
    loggingOption->setToolTip(uiText(QStringLiteral("logging_tooltip"),
                                     QStringLiteral("Write detailed diagnostic logs and DMR dumps")));
    audioOption->setChecked(audioCheckbox && audioCheckbox->isChecked());
    syncOption->setChecked(syncCheckbox && syncCheckbox->isChecked());
    syncOption->setEnabled(false);
    syncOption->setToolTip(syncCheckbox ? syncCheckbox->toolTip() : QString());
    spectrum2Option->setChecked(graphCheckbox && graphCheckbox->isChecked());
    colorOption->setChecked(colorCheckbox && colorCheckbox->isChecked());
    generalBandMarkersOption->setChecked(showGeneralBandMarkers);
    amateurBandMarkersOption->setChecked(showAmateurBandMarkers);
    compactBandMarkersOption->setChecked(compactBandMarkers);
    loggingOption->setChecked(diagnosticVerboseLogging);
    quickOptionsLayout->addWidget(audioOption, 0, 0);
    quickOptionsLayout->addWidget(syncOption, 0, 1);
    quickOptionsLayout->addWidget(spectrum2Option, 1, 0);
    quickOptionsLayout->addWidget(colorOption, 1, 1);
    quickOptionsLayout->addWidget(generalBandMarkersOption, 2, 0);
    quickOptionsLayout->addWidget(amateurBandMarkersOption, 2, 1);
    quickOptionsLayout->addWidget(compactBandMarkersOption, 2, 2);
    quickOptionsLayout->addWidget(loggingOption, 3, 0);
    rootLayout->addWidget(quickOptionsBox);

    auto applyLanguage = [this, languageCombo]() {
        const QString nextLanguage = languageCombo->currentData().toString();
        uiLanguage = nextLanguage == QStringLiteral("uk") ? QStringLiteral("uk") : QStringLiteral("en");
        applyUiLanguage();
        savePersistentSettings();
    };
    auto applyFineTuneMode = [this, fineTuneModeCombo]() {
        fineTuneControlMode = fineTuneModeCombo->currentData().toInt();
        if (fineTuneControlMode != FINE_TUNE_MODE_DIAL) {
            fineTuneControlMode = FINE_TUNE_MODE_SCALE;
        }
        updateFineTuneControlMode();
        savePersistentSettings();
    };
    auto applySpectrumUpdateInterval = [this, spectrumUpdateSpin]() {
        int value = spectrumUpdateSpin->value();
        if (value > SPECTRUM_UPDATE_AUTO_MS && value < SPECTRUM_UPDATE_MIN_MS) {
            value = SPECTRUM_UPDATE_MIN_MS;
            QSignalBlocker blocker(spectrumUpdateSpin);
            spectrumUpdateSpin->setValue(value);
        }
        spectrumUpdateIntervalMs = value;
        updateSpectrumTimerInterval();
        savePersistentSettings();
    };

    connect(languageCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog, [applyLanguage](int) {
        applyLanguage();
    });
    connect(fineTuneModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog, [applyFineTuneMode](int) {
        applyFineTuneMode();
    });
    connect(spectrumUpdateSpin, QOverload<int>::of(&QSpinBox::valueChanged), &dialog, [applySpectrumUpdateInterval](int) {
        applySpectrumUpdateInterval();
    });
    connect(audioOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        if (audioCheckbox) {
            audioCheckbox->setChecked(checked);
        }
        savePersistentSettings();
    });
    connect(spectrum2Option, &QCheckBox::toggled, &dialog, [this](bool checked) {
        if (graphCheckbox) {
            graphCheckbox->setChecked(checked);
        }
        savePersistentSettings();
    });
    connect(colorOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        if (colorCheckbox) {
            colorCheckbox->setChecked(checked);
        }
        savePersistentSettings();
    });
    connect(generalBandMarkersOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        showGeneralBandMarkers = checked;
        updateGraphBandMarkers();
        savePersistentSettings();
    });
    connect(amateurBandMarkersOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        showAmateurBandMarkers = checked;
        updateGraphBandMarkers();
        savePersistentSettings();
    });
    connect(compactBandMarkersOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        compactBandMarkers = checked;
        updateGraphBandMarkers();
        savePersistentSettings();
    });
    connect(loggingOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        diagnosticVerboseLogging = checked;
        setFobosVerboseLoggingEnabled(checked);
        qDebug() << "[Log] Verbose diagnostic logging"
                 << (checked ? "enabled" : "disabled");
        savePersistentSettings();
    });

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    if (QPushButton *closeButton = buttonBox->button(QDialogButtonBox::Close)) {
        closeButton->setText(uiText(QStringLiteral("close"), QStringLiteral("Close")));
    }
    rootLayout->addWidget(buttonBox);

    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    dialog.exec();
}

bool YourClassName::applyAgileScanSettings(bool forceStop) {
    if (forceStop || !agileScanEnabled) {
        if (activeFobosApiKind == FobosApiKind::Agile && agileDevice && agileScanRunning) {
            const int result = stopFobosAgileScanSafely(agileDevice);
            qDebug() << "[AgileScan] stop" << "result" << result;
            agileScanRunning = false;
            activeAgileScanFrequencies.clear();
            scanVisualAssembler.reset();
            if (graphWidget) {
                graphWidget->setScanSegments({});
            }
            if (waterfallWidget) {
                waterfallWidget->setScanSegments({});
            }
            if (result != FOBOS_ERR_OK) {
                return false;
            }
        }
        if (agileScanStatusLabel) {
            agileScanStatusLabel->setText(QStringLiteral("Agile scan: off"));
        }
        return true;
    }

    if (activeFobosApiKind != FobosApiKind::Agile || !agileDevice) {
        if (agileScanStatusLabel) {
            agileScanStatusLabel->setText(QStringLiteral("Agile receiver required"));
        }
        if (agileScanEnabled) {
            qDebug() << "[AgileScan] disabling saved scan flag because active receiver is not Agile"
                     << "apiKind" << fobosApiKindName(activeFobosApiKind);
        }
        agileScanEnabled = false;
        agileScanRunning = false;
        activeAgileScanFrequencies.clear();
        scanVisualAssembler.reset();
        if (agileScanCheckbox && agileScanCheckbox->isChecked()) {
            QSignalBlocker blocker(agileScanCheckbox);
            agileScanCheckbox->setChecked(false);
        }
        return true;
    }

    if (pendingSettings.inputMode != INPUT_RF) {
        if (agileScanRunning) {
            const int result = stopFobosAgileScanSafely(agileDevice);
            qDebug() << "[AgileScan] stopped outside RF mode" << "result" << result;
            agileScanRunning = false;
            activeAgileScanFrequencies.clear();
        }
        if (agileScanStatusLabel) {
            agileScanStatusLabel->setText(QStringLiteral("Agile scan works in RF mode"));
        }
        return true;
    }

    QString error;
    QVector<double> frequencies = agileScanFrequencyList(&error);
    if (!error.isEmpty() || frequencies.isEmpty()) {
        if (agileScanStatusLabel) {
            agileScanStatusLabel->setText(error.isEmpty() ? QStringLiteral("Bad Agile scan list") : error);
        }
        qDebug() << "[AgileScan] invalid scan list" << error;
        return false;
    }
    bool scanListChanged = activeAgileScanFrequencies.size() != frequencies.size();
    if (!scanListChanged) {
        for (int i = 0; i < frequencies.size(); ++i) {
            if (std::abs(activeAgileScanFrequencies.at(i) - frequencies.at(i)) > 0.5) {
                scanListChanged = true;
                break;
            }
        }
    }
    if (scanListChanged && !scanMeasurementBins.isEmpty()) {
        clearScanMeasurement();
    }

    if (agileScanRunning) {
        const int stopResult = stopFobosAgileScanSafely(agileDevice);
        qDebug() << "[AgileScan] restart stop" << "result" << stopResult;
        agileScanRunning = false;
        activeAgileScanFrequencies.clear();
        if (stopResult != FOBOS_ERR_OK) {
            if (agileScanStatusLabel) {
                agileScanStatusLabel->setText(QStringLiteral("Scan stop failed: %1").arg(stopResult));
            }
            return false;
        }
    }

    pendingSettings.centerFrequency = frequencies.first();
    pendingSettings.actualFrequency = frequencies.first();
    if (!scanListeningLockEnabled &&
        (pendingSettings.listeningFrequency < pendingSettings.centerFrequency - pendingSettings.sampleRate / 2.0 ||
         pendingSettings.listeningFrequency > pendingSettings.centerFrequency + pendingSettings.sampleRate / 2.0)) {
        pendingSettings.listeningFrequency = pendingSettings.centerFrequency;
    }

    const int result = startFobosAgileScanSafely(agileDevice,
                                                frequencies.data(),
                                                static_cast<unsigned int>(frequencies.size()));
    const int scanning = result == FOBOS_ERR_OK ? isFobosAgileScanningSafely(agileDevice) : result;
    const int index = result == FOBOS_ERR_OK ? getFobosAgileScanIndexSafely(agileDevice) : -1;
    qDebug() << "[AgileScan] start"
             << "result" << result
             << "points" << frequencies.size()
             << "firstHz" << frequencies.first()
             << "lastHz" << frequencies.last()
             << "isScanning" << scanning
             << "index" << index;
    agileScanRunning = result == FOBOS_ERR_OK;
    activeAgileScanFrequencies = agileScanRunning ? frequencies : QVector<double>();
    if (agileScanStatusLabel) {
        agileScanStatusLabel->setText(result == FOBOS_ERR_OK
                                          ? QStringLiteral("Scan active: %1 points").arg(frequencies.size())
                                          : QStringLiteral("Scan start failed: %1").arg(result));
    }
    return result == FOBOS_ERR_OK;
}

void YourClassName::resetStandardScanState(bool clearSegments) {
    standardScanRunning = false;
    activeStandardScanFrequencies.clear();
    standardScanIndex = 0;
    standardScanDwellTimer.invalidate();
    if (standardScanAdvanceTimer) {
        standardScanAdvanceTimer->stop();
    }

    if (!clearSegments) {
        return;
    }

    scanVisualAssembler.reset();
    if (graphWidget) {
        graphWidget->setScanSegments({});
    }
    if (waterfallWidget) {
        waterfallWidget->setScanSegments({});
    }
    if (scaleWidget) {
        scaleWidget->setScanSegments({});
    }
}

bool YourClassName::applyStandardScanRetune(double targetFrequencyHz, const char *reason) {
    if (!hasActiveFobosDevice() ||
        pendingSettings.inputMode != INPUT_RF ||
        (activeFobosApiKind == FobosApiKind::Agile && agileScanRunning) ||
        !std::isfinite(targetFrequencyHz) ||
        targetFrequencyHz <= 0.0) {
        return false;
    }

    const double requestedFrequency = (std::clamp)(targetFrequencyHz,
                                                   RF_MIN_CENTER_FREQUENCY,
                                                   RF_EXPERIMENTAL_MAX_FREQUENCY);
    double tunedFrequency = requestedFrequency;
    QElapsedTimer retuneTimer;
    retuneTimer.start();
    const int result = setActiveFrequencySafely(requestedFrequency, &tunedFrequency);
    const qint64 retuneCallMs = retuneTimer.elapsed();
    if (result != FOBOS_ERR_OK) {
        qDebug() << "[StandardScan] retune failed"
                 << "reason" << (reason ? reason : "")
                 << "requested" << requestedFrequency
                 << "callMs" << retuneCallMs
                 << "error" << result;
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(
                uiText(QStringLiteral("standard_scan_retune_failed"),
                       QStringLiteral("Standard scan retune failed: %1"))
                    .arg(result));
        }
        return false;
    }

    pendingSettings.centerFrequency = requestedFrequency;
    pendingSettings.actualFrequency = tunedFrequency;
    actualFrequency = tunedFrequency;
    if (hardwareSettingsApplied) {
        appliedHardwareSettings.centerFrequency = requestedFrequency;
        appliedHardwareSettings.actualFrequency = tunedFrequency;
    }
    publishSettingsToGlobals();
    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }
    if (!scanListeningLockEnabled) {
        settingRange();
    } else {
        updateFineTuneLabel();
    }

    IqBuffer::clear();
    fftResult = std::make_unique<FFTResult>();
    networkSpectrumFrameMetadataValid = false;
    networkSpectrumFrameMinFrequency = 0.0;
    networkSpectrumFrameMaxFrequency = 0.0;
    networkSpectrumFrameFftLength = 0;
    liveRetuneSettleDurationMs = (std::clamp)(standardScanSettleMs,
                                              STANDARD_SCAN_MIN_SETTLE_MS,
                                              STANDARD_SCAN_MAX_SETTLE_MS);
    liveRetuneSettleTimer.start();
    standardScanDwellTimer.restart();
    spectrumTuningDebugFramesRemaining = fobosVerboseLoggingEnabled() ? 4 : spectrumTuningDebugFramesRemaining;

    qDebug() << "[StandardScan] retune"
             << "reason" << (reason ? reason : "")
             << "index" << standardScanIndex
             << "requested" << requestedFrequency
             << "actual" << tunedFrequency
             << "callMs" << retuneCallMs
             << "settleMs" << liveRetuneSettleDurationMs;
    return true;
}

bool YourClassName::applyStandardScanSettings(bool forceStop) {
    if (forceStop || !standardScanEnabled) {
        resetStandardScanState(true);
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_off"),
                                                    QStringLiteral("Standard scan: off")));
        }
        return true;
    }

    if (!hasActiveFobosDevice()) {
        resetStandardScanState(true);
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_firmware_required"),
                                                    QStringLiteral("Fobos receiver required")));
        }
        return true;
    }

    if (activeFobosApiKind == FobosApiKind::Agile && agileScanRunning) {
        resetStandardScanState(true);
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_disable_agile"),
                                                    QStringLiteral("Disable Agile firmware scan first")));
        }
        return true;
    }

    if (pendingSettings.inputMode != INPUT_RF) {
        resetStandardScanState(true);
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_rf_only"),
                                                    QStringLiteral("Standard scan works in RF mode")));
        }
        return true;
    }

    normalizeStandardScanCentersUi(true);
    QString error;
    const QVector<double> frequencies = standardScanFrequencyList(&error);
    if (!error.isEmpty() || frequencies.size() < AGILE_SCAN_MIN_POINTS) {
        resetStandardScanState(true);
        if (standardScanStatusLabel) {
            standardScanStatusLabel->setText(error.isEmpty()
                                                 ? uiText(QStringLiteral("standard_scan_bad_list"),
                                                          QStringLiteral("Bad standard scan list"))
                                                 : error);
        }
        qDebug() << "[StandardScan] invalid list" << error;
        return false;
    }

    bool scanListChanged = activeStandardScanFrequencies.size() != frequencies.size();
    if (!scanListChanged) {
        for (int i = 0; i < frequencies.size(); ++i) {
            if (std::abs(activeStandardScanFrequencies.at(i) - frequencies.at(i)) > 0.5) {
                scanListChanged = true;
                break;
            }
        }
    }
    if (scanListChanged && !scanMeasurementBins.isEmpty()) {
        clearScanMeasurement();
    }

    activeStandardScanFrequencies = frequencies;
    if (scanListChanged) {
        standardScanIndex = 0;
        scanVisualAssembler.reset();
    } else {
        standardScanIndex = (std::clamp)(standardScanIndex, 0, activeStandardScanFrequencies.size() - 1);
    }
    standardScanRunning = true;

    const double firstCenter = activeStandardScanFrequencies.at(standardScanIndex);
    pendingSettings.centerFrequency = firstCenter;
    pendingSettings.actualFrequency = firstCenter;
    if (!scanListeningLockEnabled &&
        (pendingSettings.listeningFrequency < firstCenter - pendingSettings.sampleRate * 0.5 ||
         pendingSettings.listeningFrequency > firstCenter + pendingSettings.sampleRate * 0.5)) {
        pendingSettings.listeningFrequency = firstCenter;
    }

    if (!applyStandardScanRetune(firstCenter, scanListChanged ? "standard scan start" : "standard scan refresh")) {
        resetStandardScanState(true);
        return false;
    }
    if (standardScanAdvanceTimer && !standardScanAdvanceTimer->isActive()) {
        standardScanAdvanceTimer->start();
    }

    if (standardScanStatusLabel) {
        standardScanStatusLabel->setText(uiText(QStringLiteral("standard_scan_active"),
                                                QStringLiteral("Standard scan active: %1 centers"))
                                         .arg(activeStandardScanFrequencies.size()));
    }
    return true;
}

void YourClassName::advanceStandardScanIfNeeded() {
    if (!standardScanRunning ||
        !standardScanEnabled ||
        !hasActiveFobosDevice() ||
        pendingSettings.inputMode != INPUT_RF ||
        activeStandardScanFrequencies.size() < AGILE_SCAN_MIN_POINTS ||
        runState != RadioRunState::Running ||
        (activeFobosApiKind == FobosApiKind::Agile && agileScanRunning)) {
        return;
    }

    if (liveRetuneSettleTimer.isValid()) {
        const qint64 elapsedMs = liveRetuneSettleTimer.elapsed();
        const qint64 settleMs = liveRetuneSettleDurationMs > 0
                                    ? liveRetuneSettleDurationMs
                                    : STANDARD_SCAN_SETTLE_MS;
        IqBuffer::clear();
        if (elapsedMs < settleMs) {
            return;
        }
        liveRetuneSettleTimer.invalidate();
        standardScanDwellTimer.restart();
        return;
    }

    if (!standardScanDwellTimer.isValid()) {
        standardScanDwellTimer.start();
        return;
    }
    const qint64 dwellMs = (std::clamp)(standardScanDwellMs,
                                        STANDARD_SCAN_MIN_DWELL_MS,
                                        STANDARD_SCAN_MAX_DWELL_MS);
    if (standardScanDwellTimer.elapsed() < dwellMs) {
        return;
    }

    standardScanIndex = (standardScanIndex + 1) % activeStandardScanFrequencies.size();
    const double nextCenter = activeStandardScanFrequencies.at(standardScanIndex);
    applyStandardScanRetune(nextCenter, "standard scan advance");
}

bool YourClassName::stopAgileScanForNormalRf(const char *reason) {
    if (activeFobosApiKind != FobosApiKind::Agile || !agileDevice) {
        agileScanRunning = false;
        activeAgileScanFrequencies.clear();
        scanVisualAssembler.reset();
        return true;
    }

    const int scanning = isFobosAgileScanningSafely(agileDevice);
    if (scanning < 0 && scanning != FOBOS_ERR_NOT_OPEN) {
        qDebug() << "[AgileScan] normal RF guard could not query scan state"
                 << "reason" << (reason ? reason : "")
                 << "result" << scanning
                 << "flag" << agileScanRunning;
        return false;
    }

    if (scanning <= 0 && !agileScanRunning) {
        return true;
    }

    qDebug() << "[AgileScan] normal RF guard stopping active scan before tuning"
             << "reason" << (reason ? reason : "")
             << "isScanning" << scanning
             << "flag" << agileScanRunning;
    const int stopResult = stopFobosAgileScanSafely(agileDevice);
    const int afterStop = stopResult == FOBOS_ERR_OK
                              ? isFobosAgileScanningSafely(agileDevice)
                              : stopResult;
    qDebug() << "[AgileScan] normal RF guard stop complete"
             << "reason" << (reason ? reason : "")
             << "result" << stopResult
             << "isScanningAfter" << afterStop;

    agileScanRunning = false;
    activeAgileScanFrequencies.clear();
    scanVisualAssembler.reset();
    if (graphWidget) {
        graphWidget->setScanSegments({});
    }
    if (waterfallWidget) {
        waterfallWidget->setScanSegments({});
    }
    if (stopResult == FOBOS_ERR_OK) {
        clearLiveSpectrumSnapshot();
        return true;
    }
    return false;
}

bool YourClassName::isIdle() const {
    return runState == RadioRunState::Idle;
}

bool YourClassName::isRunningOrTransitioning() const {
    return runState != RadioRunState::Idle;
}

void YourClassName::refreshSettingsFromUi() {
    if (comboBox) {
        bool ok = false;
        const int selectedIndex = comboBox->currentData().toInt(&ok);
        pendingSettings.deviceIndex = ok ? selectedIndex : std::max(0, comboBox->currentIndex());
    }
    if (clkBox) {
        pendingSettings.clockSource = clkBox->currentData().toInt();
    }
    if (modeBox) {
        pendingSettings.inputMode = modeBox->currentData().toInt();
    }
    if (sampleBox) {
        bool ok = false;
        const double sampleRate = sampleBox->currentData().toDouble(&ok);
        if (ok && sampleRate > 0.0) {
            pendingSettings.sampleRate = sampleRate;
        }
    }
    if (frequencyControl) {
        frequencyControl->commitPendingValue();
        double frequency = frequencyControl->valueHz();
        if (pendingSettings.inputMode == INPUT_RF && frequency < 50000000.0) {
            frequency = 50000000.0;
        }
        pendingSettings.centerFrequency = pendingSettings.inputMode == INPUT_RF ? frequency : 0.0;
    }
    if (listeningFrequencyControl) {
        listeningFrequencyControl->commitPendingValue();
        pendingSettings.listeningFrequency = listeningFrequencyControl->valueHz();
    }
    if (bandwidthControl) {
        bandwidthControl->commitPendingValue();
        const double bandwidth = bandwidthControl->valueHz();
        if (bandwidth > 0.0) {
            pendingSettings.bandwidth = bandwidth;
        }
    }
    if (fftComboBox) {
        const int selectedFftLength = fftComboBox->currentText().toInt();
        if (selectedFftLength > 0) {
            pendingSettings.fftLength = selectedFftLength;
        }
    }
    if (lnaGainSlider) {
        pendingSettings.lnaGain = lnaGainSlider->value();
    }
    if (vgaGainSlider) {
        pendingSettings.vgaGain = vgaGainSlider->value();
    }
    if (audioDeviceComboBox) {
        pendingSettings.audioDeviceId = audioDeviceComboBox->currentData().toInt();
    }
    if (audioLowPassSlider) {
        pendingSettings.audioLowPassHz = audioLowPassSliderValueToHz(audioLowPassSlider->value());
    }
    if (audioHighPassSlider) {
        pendingSettings.audioHighPassHz = audioHighPassSliderValueToHz(audioHighPassSlider->value());
    }
    if (hfNoiseCancelDepthSlider) {
        pendingSettings.hfNoiseCancelDepth =
            hfNoiseCancelSliderValueToDepth(hfNoiseCancelDepthSlider->value());
    }
    if (hfNoiseCancelRefGainSlider) {
        pendingSettings.hfNoiseCancelRefGainDb =
            hfNoiseCancelSliderValueToRefGainDb(hfNoiseCancelRefGainSlider->value());
    }
    if (hfNoiseCancelRefDelaySlider) {
        pendingSettings.hfNoiseCancelRefDelayNs =
            hfNoiseCancelSliderValueToRefDelayNs(hfNoiseCancelRefDelaySlider->value());
    }
    if (hfNoiseCancelRefTiltSlider) {
        pendingSettings.hfNoiseCancelRefTiltDb =
            hfNoiseCancelSliderValueToRefTiltDb(hfNoiseCancelRefTiltSlider->value());
    }
    if (hfNoiseCancelFreezeCheckbox) {
        pendingSettings.hfNoiseCancelFreeze = hfNoiseCancelFreezeCheckbox->isChecked();
    }
    if (audioCheckbox) {
        pendingSettings.audioEnabled = audioCheckbox->isChecked();
    }
    if (syncCheckbox) {
        if (syncCheckbox->isChecked()) {
            syncCheckbox->blockSignals(true);
            syncCheckbox->setChecked(false);
            syncCheckbox->blockSignals(false);
        }
        pendingSettings.syncEnabled = false;
    }
    if (dmrBasebandRateCombo) {
        pendingSettings.dmrBasebandSampleRate =
            normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt());
    } else {
        pendingSettings.dmrBasebandSampleRate =
            normalizedDmrBasebandSampleRate(pendingSettings.dmrBasebandSampleRate);
    }
    if (dmrAmbeLayoutCombo) {
        pendingSettings.dmrAmbeLayout =
            normalizedDmrAmbeLayout(dmrAmbeLayoutCombo->currentData().toInt());
    } else {
        pendingSettings.dmrAmbeLayout =
            normalizedDmrAmbeLayout(pendingSettings.dmrAmbeLayout);
    }
    if (dmrManualTimingCheckbox) {
        pendingSettings.dmrManualTimingEnabled = dmrManualTimingCheckbox->isChecked();
    }
    if (dmrTimingOffsetSpin) {
        pendingSettings.dmrManualTimingOffset = dmrTimingOffsetSpin->value();
    }
    if (dmrSlicerRatioSpin) {
        pendingSettings.dmrSlicerRatio = dmrSlicerRatioSpin->value();
    }
    if (dmrAdaptiveSlicerCheckbox) {
        pendingSettings.dmrAdaptiveSlicer = dmrAdaptiveSlicerCheckbox->isChecked();
    }
    if (agileScanCheckbox) {
        agileScanEnabled = agileScanCheckbox->isChecked();
    }
    if (agileScanRangesEdit) {
        agileScanRangesMhz = agileScanRangesEdit->text().trimmed();
    }
    if (agileScanStepSpin) {
        agileScanStepMhz = (std::clamp)(agileScanStepSpin->value(),
                                        AGILE_SCAN_MIN_STEP_MHZ,
                                        AGILE_SCAN_MAX_STEP_MHZ);
    }
    if (standardScanCheckbox) {
        standardScanEnabled = standardScanCheckbox->isChecked();
    }
    if (scanListeningLockCheckbox) {
        scanListeningLockEnabled = scanListeningLockCheckbox->isChecked();
    }
    if (standardScanCentersEdit) {
        standardScanCentersMhz = standardScanCentersEdit->text().trimmed();
    }
    if (standardScanDwellSpin) {
        standardScanDwellMs = (std::clamp)(standardScanDwellSpin->value(),
                                           STANDARD_SCAN_MIN_DWELL_MS,
                                           STANDARD_SCAN_MAX_DWELL_MS);
    }
    if (standardScanSettleSpin) {
        standardScanSettleMs = (std::clamp)(standardScanSettleSpin->value(),
                                            STANDARD_SCAN_MIN_SETTLE_MS,
                                            STANDARD_SCAN_MAX_SETTLE_MS);
    }
    if (standardScanRangeStartEdit) {
        standardScanRangeStartMhz = standardScanRangeStartEdit->text().trimmed();
    }
    if (standardScanRangeEndEdit) {
        standardScanRangeEndMhz = standardScanRangeEndEdit->text().trimmed();
    }
    if (agileScanEnabled && standardScanEnabled) {
        agileScanEnabled = false;
        if (agileScanCheckbox) {
            QSignalBlocker blocker(agileScanCheckbox);
            agileScanCheckbox->setChecked(false);
        }
    }
    normalizeStandardScanCentersUi(false);
    if (scanMeasurementCheckbox) {
        scanMeasurementEnabled = scanMeasurementCheckbox->isChecked();
    }
    if (scanMeasurementBinSpin) {
        scanMeasurementBinMhz = (std::clamp)(scanMeasurementBinSpin->value(),
                                             SCAN_MEASUREMENT_MIN_BIN_MHZ,
                                             SCAN_MEASUREMENT_MAX_BIN_MHZ);
    }
    if (spurSuppressionCheckbox) {
        spurSuppressionEnabled = spurSuppressionCheckbox->isChecked();
    }
    pendingSettings.gpoValue = currentGpoValue();
    normalizeTuning(pendingSettings);
}

void YourClassName::publishSettingsToGlobals() {
    globalMode = pendingSettings.inputMode;
    globalFrequency = pendingSettings.centerFrequency;
    actualFrequency = pendingSettings.actualFrequency;
    listeningFrequency = pendingSettings.listeningFrequency;
    globalSampleRate = pendingSettings.sampleRate;
    globalBandwidth = pendingSettings.bandwidth;
    globalModulationType = pendingSettings.modulationType;
    fftLength = pendingSettings.fftLength;
    syncWariable = pendingSettings.syncEnabled;
    deviceID = pendingSettings.audioDeviceId;
    if (audioProcessor) {
        audioProcessor->configure(audioProcessorSettings());
    }
    const bool iqFrameProducerActive =
        isChannelIqRecordingActive() ||
        (networkMode == NetworkMode::Server && isClientIqProcessingMode());
    if (runState == RadioRunState::Running && iqFrameProducerActive) {
        updateIqFrameProducerSettings();
    }
}

bool YourClassName::isNetworkClientMode() const {
    return networkMode == NetworkMode::Client;
}

bool YourClassName::isClientIqProcessingMode() const {
    return isChannelIqProcessingMode() || isFullIqProcessingMode();
}

bool YourClassName::isChannelIqProcessingMode() const {
    return networkProcessingMode == NetworkProcessingMode::ChannelIqClientSide;
}

bool YourClassName::isFullIqProcessingMode() const {
    return networkProcessingMode == NetworkProcessingMode::FullIqClientSide;
}

bool YourClassName::isClientIqProcessingMode(NetworkProcessingMode mode) const {
    return mode == NetworkProcessingMode::ChannelIqClientSide ||
           mode == NetworkProcessingMode::FullIqClientSide;
}

RecordingManager::Mode YourClassName::selectedRecordingMode() const {
    if (!recordingModeCombo) {
        return RecordingManager::Mode::AudioWav;
    }
    bool ok = false;
    const int value = recordingModeCombo->currentData().toInt(&ok);
    return ok ? static_cast<RecordingManager::Mode>(value) : RecordingManager::Mode::AudioWav;
}

QJsonObject YourClassName::recordingLabMetadata() const {
    QJsonObject lab;
    if (pendingSettings.modulationType != MOD_DMR) {
        return lab;
    }

    bool hasMetadata = false;
    lab["schema"] = QStringLiteral("dmr-profile");
    lab["schemaVersion"] = 2;
    lab["mode"] = QStringLiteral("DMR");
    lab["locked"] = dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked();
    lab["basebandSampleRate"] =
        dmrBasebandRateCombo
            ? normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt())
            : normalizedDmrBasebandSampleRate(pendingSettings.dmrBasebandSampleRate);
    lab["manualTiming"] = dmrManualTimingCheckbox && dmrManualTimingCheckbox->isChecked();
    lab["timingOffset"] = dmrTimingOffsetSpin ? dmrTimingOffsetSpin->value() : 0;
    lab["slicerRatio"] = dmrSlicerRatioSpin ? dmrSlicerRatioSpin->value() : pendingSettings.dmrSlicerRatio;
    lab["adaptiveSlicer"] = !dmrAdaptiveSlicerCheckbox || dmrAdaptiveSlicerCheckbox->isChecked();
    lab["ambeLayout"] =
        QString::fromLatin1(dmrAmbeLayoutName(dmrAmbeLayoutCombo
                                                  ? dmrAmbeLayoutCombo->currentData().toInt()
                                                  : pendingSettings.dmrAmbeLayout));
    lab["description"] = QStringLiteral("DMR metadata learned from the signal or locked by the user.");

    if (dmrLabColorCodeCombo) {
        const int colorCode = dmrLabColorCodeCombo->currentData().toInt();
        if (colorCode >= 0) {
            lab["colorCode"] = colorCode;
            hasMetadata = true;
        }
    }
    if (dmrLabSlotCombo) {
        const int slot = dmrLabSlotCombo->currentData().toInt();
        if (slot == 1 || slot == 2) {
            lab["timeslot"] = slot;
            hasMetadata = true;
        }
    }
    if (dmrLabCallTypeCombo) {
        const QString callType = dmrLabCallTypeCombo->currentData().toString();
        if (!callType.isEmpty() && callType != QStringLiteral("unknown")) {
            lab["callType"] = callType;
            hasMetadata = true;
        }
    }

    auto addText = [&lab, &hasMetadata](const QString &key, const QLineEdit *edit) {
        if (!edit) {
            return;
        }
        const QString text = edit->text().trimmed();
        if (!text.isEmpty()) {
            lab[key] = text;
            hasMetadata = true;
        }
    };
    addText(QStringLiteral("sourceId"), dmrLabSourceIdEdit);
    addText(QStringLiteral("targetId"), dmrLabTargetIdEdit);
    addText(QStringLiteral("radio"), dmrLabRadioEdit);
    addText(QStringLiteral("notes"), dmrLabNotesEdit);
    if (!hasMetadata && !(dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked())) {
        return QJsonObject();
    }
    return lab;
}

bool YourClassName::isChannelIqRecordingActive() const {
    return recordingManager &&
           recordingManager->isRecording() &&
           recordingManager->mode() == RecordingManager::Mode::ChannelIqWav;
}

void YourClassName::updateIqFrameProducerSettings() {
    if (!processor) {
        return;
    }

    const bool serverIqStreaming = networkMode == NetworkMode::Server && isClientIqProcessingMode();
    const bool serverFullIqStreaming = networkMode == NetworkMode::Server && isFullIqProcessingMode();
    const bool serverChannelIqStreaming = networkMode == NetworkMode::Server && isChannelIqProcessingMode();
    const bool channelIqRecording = isChannelIqRecordingActive();
    processor->configureNetworkIqStreaming(pendingSettings,
                                           serverIqStreaming || channelIqRecording,
                                           serverChannelIqStreaming || channelIqRecording);
}

void YourClassName::updateRecordingStatus(const QString &status) {
    if (recordingStatusLabel) {
        recordingStatusLabel->setProperty("statusRawText", status);
        recordingStatusLabel->setText(localizedStatusText(status));
    }
}

QString YourClassName::selectedPlaybackFilePath() const {
    if (!playbackFileCombo) {
        return QString();
    }
    return playbackFileCombo->currentData().toString();
}

void YourClassName::appendNetworkState(QJsonObject &command) const {
    command["processingMode"] = static_cast<int>(networkProcessingMode);
    command["serverDisableLocalVisualAudio"] = serverDisableLocalVisualAudio;
    command["fullResolutionSpectrumFrames"] = networkFullResolutionSpectrumFrames;
}

void YourClassName::applyNetworkStateFromCommand(const QJsonObject &command) {
    if (command.contains("processingMode")) {
        networkProcessingMode = static_cast<NetworkProcessingMode>(command.value("processingMode").toInt(
            static_cast<int>(networkProcessingMode)));
    }
    if (command.contains("serverDisableLocalVisualAudio")) {
        serverDisableLocalVisualAudio = command.value("serverDisableLocalVisualAudio").toBool(serverDisableLocalVisualAudio);
    }
    if (command.contains("fullResolutionSpectrumFrames")) {
        networkFullResolutionSpectrumFrames =
            command.value("fullResolutionSpectrumFrames").toBool(networkFullResolutionSpectrumFrames);
    }
    onNetworkStatusChanged(networkController ? networkController->statusText() : QString());
}

void YourClassName::handleDataProcessorFailure(int errorCode, bool stoppedByRequest) {
    const bool unexpectedCleanEnd = errorCode == FOBOS_ERR_OK;
    qDebug() << "[FobosLifecycle] DataProcessor reader failure"
             << "error" << errorCode
             << "unexpectedCleanEnd" << unexpectedCleanEnd
             << "stoppedByRequest" << stoppedByRequest
             << "state" << runStateName(runState)
             << "deviceOpened" << deviceOpened
             << "processorRunning" << (processor && processor->isRunning())
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind)
             << "sampleRate" << pendingSettings.sampleRate
             << "inputMode" << pendingSettings.inputMode;

    if (stoppedByRequest || isNetworkClientMode()) {
        return;
    }

    restartAfterStartupWatchdog = false;
    automaticStreamRestart = false;
    pendingAudioStartAfterStreamReady = false;
    if (streamWatchdogTimer) {
        streamWatchdogTimer->stop();
    }
    clearSpectrumAfterStop = true;
    const bool startupFailure =
        streamStartElapsedTimer.isValid() &&
        streamStartElapsedTimer.elapsed() < 2500 &&
        streamStartupRetryCount < 1;
    if (startupFailure) {
        ++streamStartupRetryCount;
        restartAfterStartupWatchdog = true;
        qDebug() << "[FobosLifecycle] reader startup failure will retry once"
                 << "retryCount" << streamStartupRetryCount
                 << "error" << errorCode;
    }

    if (runState == RadioRunState::Stopping) {
        return;
    }

    if (runState == RadioRunState::Idle && !deviceOpened && !hasActiveFobosDevice()) {
        return;
    }

    qDebug() << "[FobosLifecycle] recovering from reader failure; closing Fobos session";
    if (recordingManager && recordingManager->isRecording()) {
        qDebug() << "[Recording] stopping because receiver stream ended unexpectedly";
        stopRecording(false);
        updateRecordingStatus(unexpectedCleanEnd
                                  ? QStringLiteral("Recording stopped: receiver stream ended")
                                  : QStringLiteral("Recording stopped: receiver error %1").arg(errorCode));
    }
    if (digitalStatusLabel) {
        onDigitalDecoderStatusChanged(unexpectedCleanEnd
                                          ? QStringLiteral("Receiver stream ended unexpectedly; check USB connection")
                                          : QStringLiteral("Receiver stream failed: error %1").arg(errorCode));
    }
    if (updateTimer) {
        updateTimer->stop();
    }
    if (audioProcessor) {
        audioProcessor->stopDemodulation();
    }

    runState = RadioRunState::Stopping;
    updateUiForRunState();

    if (processor && processor->isRunning()) {
        stopCancelRetryCount = 0;
        processor->requestStop();
        stopElapsedTimer.restart();
        if (stopPollTimer) {
            stopPollTimer->start();
        }
        return;
    }

    finishFobosStop(false);
}

void YourClassName::connectDataProcessorSignals() {
    if (!processor) {
        return;
    }

    DataProcessor *connectedProcessor = processor;
    connect(processor,
            &DataProcessor::readerFailed,
            this,
            [this, connectedProcessor](int errorCode, bool stoppedByRequest) {
                if (connectedProcessor != processor) {
                    qDebug() << "[FobosLifecycle] ignoring stale DataProcessor reader failure"
                             << "error" << errorCode;
                    return;
                }
                handleDataProcessorFailure(errorCode, stoppedByRequest);
            },
            Qt::QueuedConnection);

    connect(processor,
            &DataProcessor::iqFrameReady,
            this,
            [this](const QByteArray &iqData, double sampleRate, int sampleCount) {
                if (recordingManager &&
                    recordingManager->isRecording() &&
                    recordingManager->mode() == RecordingManager::Mode::ChannelIqWav) {
                    recordingManager->appendIqFrame(iqData, sampleRate, sampleCount);
                }
                processVideoIqFrame(iqData, sampleRate, sampleCount);
                sendNetworkIqFrame(iqData, sampleRate, sampleCount);
            },
            Qt::QueuedConnection);
}

void YourClassName::applyServerLocalOutputPolicy() {
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();

    if (audioProcessor) {
        audioProcessor->setLocalPlaybackEnabled(!suppressServerLocalOutput);
    }
    updateVideoProcessorMode();
    updateIqFrameProducerSettings();

    if (!updateTimer || runState != RadioRunState::Running || networkMode != NetworkMode::Server) {
        return;
    }

    const bool spectrumNeededForNetwork =
        networkController &&
        networkController->isControlReady() &&
        !isFullIqProcessingMode();
    const bool localVisualNeeded = !suppressServerLocalOutput;

    if (spectrumNeededForNetwork || localVisualNeeded) {
        if (!updateTimer->isActive()) {
            updateTimer->start();
        }
    } else if (updateTimer->isActive()) {
        updateTimer->stop();
    }
}

bool YourClassName::applyCenterFrequencyToHardwareIfNeeded(const RadioSettings &previousSettings,
                                                           const char *reason) {
    if (pendingSettings.inputMode != INPUT_RF ||
        isIdle() ||
        !hasActiveFobosDevice() ||
        std::abs(previousSettings.centerFrequency - pendingSettings.centerFrequency) <= 0.5) {
        return true;
    }

    const bool liveAgileRfRetune =
        activeFobosApiKind == FobosApiKind::Agile &&
        !agileScanEnabled &&
        processor &&
        processor->isRunning();
    if (liveAgileRfRetune) {
        const uint64_t generation = ++liveCenterRetuneGeneration;
        qDebug() << "[LiveTune]" << reason
                 << "center changed on Agile RF stream; applying live retune without reader restart"
                 << "previous" << previousSettings.centerFrequency
                 << "requested" << pendingSettings.centerFrequency
                 << "generation" << generation;
        return applyLiveAgileCenterRetune(generation, QString::fromUtf8(reason ? reason : "live"));
    }

    double tunedFrequency = pendingSettings.centerFrequency;
    const int result = setActiveFrequencySafely(pendingSettings.centerFrequency, &tunedFrequency);
    if (result == FOBOS_ERR_OK) {
        pendingSettings.actualFrequency = tunedFrequency;
        if (hardwareSettingsApplied) {
            appliedHardwareSettings.centerFrequency = pendingSettings.centerFrequency;
            appliedHardwareSettings.actualFrequency = tunedFrequency;
        }
        qDebug() << "[LiveTune]" << reason
                 << "center applied"
                 << "requested" << pendingSettings.centerFrequency
                 << "actual" << tunedFrequency;
        networkSpectrumFrameMetadataValid = false;
        networkSpectrumFrameMinFrequency = 0.0;
        networkSpectrumFrameMaxFrequency = 0.0;
        networkSpectrumFrameFftLength = 0;
        clearLiveSpectrumSnapshot(false);
        liveRetuneSettleDurationMs = LIVE_RETUNE_SETTLE_MS;
        liveRetuneSettleTimer.start();
        spectrumTuningDebugFramesRemaining = 32;
        qDebug() << "[LiveTune]" << reason
                 << "cleared live IQ after center retune; preserving visual history"
                 << "settleMs" << liveRetuneSettleDurationMs;
        return true;
    }

    qDebug() << "[LiveTune]" << reason
             << "center apply failed"
             << "requested" << pendingSettings.centerFrequency
             << "error" << result;
    return false;
}

bool YourClassName::applyLiveAgileCenterRetune(uint64_t generation, const QString &reason) {
    if (generation != liveCenterRetuneGeneration) {
        qDebug() << "[LiveTune]" << reason
                 << "skipping stale Agile live retune"
                 << "generation" << generation
                 << "current" << liveCenterRetuneGeneration;
        return true;
    }

    if (runState != RadioRunState::Running ||
        pendingSettings.inputMode != INPUT_RF ||
        !hasActiveFobosDevice() ||
        activeFobosApiKind != FobosApiKind::Agile ||
        agileScanEnabled ||
        !processor ||
        !processor->isRunning()) {
        qDebug() << "[LiveTune]" << reason
                 << "skipping Agile live retune because stream is not in normal RF running state"
                 << "generation" << generation
                 << "state" << static_cast<int>(runState)
                 << "inputMode" << pendingSettings.inputMode
                 << "apiKind" << static_cast<int>(activeFobosApiKind)
                 << "agileScan" << agileScanEnabled;
        return true;
    }

    if (liveRetuneSettleTimer.isValid()) {
        const qint64 elapsedMs = liveRetuneSettleTimer.elapsed();
        const qint64 settleMs = liveRetuneSettleDurationMs > 0 ? liveRetuneSettleDurationMs : LIVE_RETUNE_SETTLE_MS;
        if (elapsedMs < settleMs) {
            const int delayMs = static_cast<int>((std::clamp)(settleMs - elapsedMs + 40, qint64(40), qint64(240)));
            qDebug() << "[LiveTune]" << reason
                     << "deferring Agile live retune until previous settle completes"
                     << "generation" << generation
                     << "delayMs" << delayMs
                     << "elapsedMs" << elapsedMs
                     << "settleMs" << settleMs;
            QTimer::singleShot(delayMs, this, [this, generation, reason]() {
                applyLiveAgileCenterRetune(generation, reason);
            });
            return true;
        }
        liveRetuneSettleTimer.invalidate();
    }

    if (!stopAgileScanForNormalRf("live center retune")) {
        qDebug() << "[LiveTune]" << reason
                 << "Agile live center retune aborted because scan mode could not be stopped"
                 << "generation" << generation;
        return false;
    }

    const double requestedFrequency = pendingSettings.centerFrequency;
    double tunedFrequency = requestedFrequency;
    clearLiveSpectrumSnapshot(false);
    const int result = setActiveFrequencySafely(requestedFrequency, &tunedFrequency);
    if (result != FOBOS_ERR_OK) {
        qDebug() << "[LiveTune]" << reason
                 << "Agile live center retune failed"
                 << "generation" << generation
                 << "requested" << requestedFrequency
                 << "error" << result;
        return false;
    }

    pendingSettings.actualFrequency = tunedFrequency;
    actualFrequency = tunedFrequency;
    if (hardwareSettingsApplied) {
        appliedHardwareSettings.centerFrequency = requestedFrequency;
        appliedHardwareSettings.actualFrequency = tunedFrequency;
    }
    publishSettingsToGlobals();
    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }
    settingRange();

    networkSpectrumFrameMetadataValid = false;
    networkSpectrumFrameMinFrequency = 0.0;
    networkSpectrumFrameMaxFrequency = 0.0;
    networkSpectrumFrameFftLength = 0;
    clearLiveSpectrumSnapshot(false);
    liveRetuneSettleDurationMs = agileRfLiveSettleMs(pendingSettings.sampleRate, false);
    liveRetuneSettleTimer.start();
    spectrumTuningDebugFramesRemaining = 32;

    qDebug() << "[LiveTune]" << reason
             << "Agile live center retune applied"
             << "generation" << generation
             << "requested" << requestedFrequency
             << "actual" << tunedFrequency
             << "settleMs" << liveRetuneSettleDurationMs;
    return true;
}

void YourClassName::clearLiveSpectrumSnapshot(bool clearVisualHistory) {
    IqBuffer::clear();
    fftResult = std::make_unique<FFTResult>();
    scanVisualAssembler.reset();
    networkSpectrumFrameMetadataValid = false;
    networkSpectrumFrameMinFrequency = 0.0;
    networkSpectrumFrameMaxFrequency = 0.0;
    networkSpectrumFrameFftLength = 0;

    if (!clearVisualHistory) {
        return;
    }

    if (graphWidget) {
        graphWidget->clearData();
        graphWidget->update();
    }
    if (waterfallWidget) {
        waterfallWidget->clearData();
        waterfallWidget->update();
    }
}

void YourClassName::schedulePostStartRetune(const QString &reason) {
    if (pendingSettings.inputMode != INPUT_RF ||
        !hasActiveFobosDevice() ||
        standardScanEnabled ||
        (activeFobosApiKind == FobosApiKind::Agile && agileScanEnabled)) {
        return;
    }

    const double requestedFrequency = pendingSettings.centerFrequency;
    if (!std::isfinite(requestedFrequency) || requestedFrequency <= 0.0) {
        return;
    }

    if (activeFobosApiKind == FobosApiKind::Agile) {
        const uint64_t generation = ++liveCenterRetuneGeneration;
        constexpr std::array<int, 1> retuneDelaysMs = {120};
        for (int attempt = 0; attempt < static_cast<int>(retuneDelaysMs.size()); ++attempt) {
            QTimer::singleShot(retuneDelaysMs[attempt],
                               this,
                               [this, requestedFrequency, reason, generation, attempt]() {
                if (runState != RadioRunState::Running ||
                    !deviceOpened ||
                    !hasActiveFobosDevice() ||
                    activeFobosApiKind != FobosApiKind::Agile ||
                    pendingSettings.inputMode != INPUT_RF ||
                    agileScanEnabled ||
                    std::abs(pendingSettings.centerFrequency - requestedFrequency) > 0.5) {
                    return;
                }

                qDebug() << "[FobosLifecycle] post-start Agile same-frequency re-arm"
                         << "attempt" << (attempt + 1)
                         << "reason" << reason
                         << "requested" << requestedFrequency
                         << "generation" << generation;
                applyLiveAgileCenterRetune(generation, reason + QStringLiteral(" post-start re-arm"));
            });
        }
        return;
    }

    const FobosApiKind expectedApiKind = activeFobosApiKind;
    constexpr std::array<int, 1> retuneDelaysMs = {700};
    for (int attempt = 0; attempt < static_cast<int>(retuneDelaysMs.size()); ++attempt) {
        QTimer::singleShot(retuneDelaysMs[attempt],
                           this,
                           [this, requestedFrequency, expectedApiKind, reason, attempt]() {
            if (runState != RadioRunState::Running ||
                !deviceOpened ||
                !hasActiveFobosDevice() ||
                activeFobosApiKind != expectedApiKind ||
                pendingSettings.inputMode != INPUT_RF ||
                (activeFobosApiKind == FobosApiKind::Agile && agileScanEnabled) ||
                std::abs(pendingSettings.centerFrequency - requestedFrequency) > 0.5) {
                return;
            }

            double tunedFrequency = requestedFrequency;
            const int result = setActiveFrequencySafely(requestedFrequency, &tunedFrequency);
            qDebug() << "[FobosLifecycle] post-start retune"
                     << "attempt" << (attempt + 1)
                     << "reason" << reason
                     << "requested" << requestedFrequency
                     << "result" << result
                     << "actual" << tunedFrequency;

            if (result != FOBOS_ERR_OK) {
                return;
            }

            pendingSettings.actualFrequency = tunedFrequency;
            actualFrequency = tunedFrequency;
            if (hardwareSettingsApplied) {
                appliedHardwareSettings.centerFrequency = requestedFrequency;
                appliedHardwareSettings.actualFrequency = tunedFrequency;
            }
            publishSettingsToGlobals();
            if (frequencyControl) {
                QSignalBlocker blocker(frequencyControl);
                frequencyControl->setValueHz(pendingSettings.centerFrequency);
            }
            settingRange();
            networkSpectrumFrameMetadataValid = false;
            networkSpectrumFrameMinFrequency = 0.0;
            networkSpectrumFrameMaxFrequency = 0.0;
            networkSpectrumFrameFftLength = 0;
            clearLiveSpectrumSnapshot(false);
            liveRetuneSettleDurationMs = agileRfLiveSettleMs(pendingSettings.sampleRate, false);
            liveRetuneSettleTimer.start();
            qDebug() << "[FobosLifecycle] post-start retune cleared live IQ; preserving visual history"
                     << "attempt" << (attempt + 1)
                     << "settleMs" << liveRetuneSettleDurationMs;
        });
    }
}

bool YourClassName::stabilizeAgileFrequencyBeforeStreaming(const char *reason) {
    if (activeFobosApiKind != FobosApiKind::Agile ||
        pendingSettings.inputMode != INPUT_RF ||
        agileScanEnabled ||
        !hasActiveFobosDevice()) {
        return true;
    }

    const double requestedFrequency = pendingSettings.centerFrequency;
    if (!std::isfinite(requestedFrequency) || requestedFrequency <= 0.0) {
        return true;
    }

    if (!stopAgileScanForNormalRf("pre-stream frequency settle")) {
        qDebug() << "[FobosLifecycle] Agile pre-stream frequency settle failed to leave scan mode"
                 << "reason" << (reason ? reason : "")
                 << "requested" << requestedFrequency;
        return false;
    }

    double tunedFrequency = requestedFrequency;
    qDebug() << "[FobosLifecycle] Agile pre-stream frequency settle begin"
             << "reason" << (reason ? reason : "")
             << "requested" << requestedFrequency;
    int result = setActiveFrequencySafely(requestedFrequency, &tunedFrequency);
    qDebug() << "[FobosLifecycle] Agile pre-stream frequency settle first"
             << "result" << result
             << "actual" << tunedFrequency;
    if (result != FOBOS_ERR_OK) {
        return false;
    }

    QThread::msleep(40);

    double nudgeFrequency = requestedFrequency + AGILE_PRE_STREAM_FREQUENCY_NUDGE_HZ;
    if (nudgeFrequency > RF_EXPERIMENTAL_MAX_FREQUENCY) {
        nudgeFrequency = requestedFrequency - AGILE_PRE_STREAM_FREQUENCY_NUDGE_HZ;
    }
    if (nudgeFrequency >= RF_MIN_CENTER_FREQUENCY &&
        nudgeFrequency <= RF_EXPERIMENTAL_MAX_FREQUENCY) {
        double nudgeActualFrequency = nudgeFrequency;
        result = setActiveFrequencySafely(nudgeFrequency, &nudgeActualFrequency);
        qDebug() << "[FobosLifecycle] Agile pre-stream frequency settle nudge"
                 << "requested" << nudgeFrequency
                 << "result" << result
                 << "actual" << nudgeActualFrequency;
        if (result != FOBOS_ERR_OK) {
            return false;
        }
        QThread::msleep(40);
    }

    double verifiedFrequency = requestedFrequency;
    result = setActiveFrequencySafely(requestedFrequency, &verifiedFrequency);
    qDebug() << "[FobosLifecycle] Agile pre-stream frequency settle verify"
             << "result" << result
             << "actual" << verifiedFrequency;
    if (result != FOBOS_ERR_OK) {
        return false;
    }

    pendingSettings.actualFrequency = verifiedFrequency;
    actualFrequency = verifiedFrequency;
    appliedHardwareSettings.centerFrequency = requestedFrequency;
    appliedHardwareSettings.actualFrequency = verifiedFrequency;
    publishSettingsToGlobals();
    return true;
}

void YourClassName::resetNetworkIqReceptionState(bool clearGraph, bool clearWaterfall, bool restartAudioPrebuffer) {
    IqBuffer::clear();
    networkIqStreamMetadataValid = false;
    networkIqStreamWasChannelized = false;
    networkIqStreamSampleRate = 0.0;
    networkIqStreamCenterFrequency = 0.0;
    networkIqStreamListeningFrequency = 0.0;
    networkIqStreamInputMode = 0;
    networkSpectrumFrameMetadataValid = false;
    networkSpectrumFrameMinFrequency = 0.0;
    networkSpectrumFrameMaxFrequency = 0.0;
    networkSpectrumFrameFftLength = 0;

    if (audioProcessor) {
        audioProcessor->stopDemodulation();
    }
    pendingNetworkAudioStartAfterIqPrebuffer = restartAudioPrebuffer && pendingSettings.audioEnabled;

    if (clearGraph && graphWidget) {
        graphWidget->clearData();
    }
    if (clearWaterfall && waterfallWidget) {
        waterfallWidget->clearData();
    }

    fftResult = std::make_unique<FFTResult>();
    spectrumDebugFramesRemaining = fobosVerboseLoggingEnabled() ? 12 : 0;
    updateSpectrumTimerInterval();
}

void YourClassName::startNetworkClientProcessing() {
    if (!isNetworkClientMode() || !isClientIqProcessingMode()) {
        return;
    }

    if (networkClientIqProcessingActive &&
        activeNetworkClientProcessingMode == networkProcessingMode &&
        runState == RadioRunState::Running) {
        if (audioProcessor) {
            audioProcessor->setLocalPlaybackEnabled(true);
        }
        if (updateTimer && isFullIqProcessingMode() && !updateTimer->isActive()) {
            updateTimer->start();
        }
        return;
    }

    if (remoteAudioPlayer) {
        remoteAudioPlayer->stop();
    }
    if (updateTimer) {
        updateTimer->stop();
    }
    if (audioProcessor) {
        audioProcessor->setLocalPlaybackEnabled(true);
    }

    const bool switchingBetweenClientIqModes =
        networkClientIqProcessingActive &&
        activeNetworkClientProcessingMode != networkProcessingMode;
    resetNetworkIqReceptionState(false,
                                 switchingBetweenClientIqModes,
                                 pendingSettings.audioEnabled && !isFullIqProcessingMode());
    IqBuffer::setSampleRateEstimate(pendingSettings.sampleRate);
    publishSettingsToGlobals();

    if (updateTimer && isFullIqProcessingMode()) {
        updateTimer->start();
    }
    networkClientIqProcessingActive = true;
    activeNetworkClientProcessingMode = networkProcessingMode;

    qDebug() << "[NetworkIQ] client-side IQ processing started"
             << "sampleRate" << pendingSettings.sampleRate
             << "audio" << pendingSettings.audioEnabled
             << "audioPrebuffer" << pendingNetworkAudioStartAfterIqPrebuffer;
}

void YourClassName::stopNetworkClientProcessing() {
    if (updateTimer) {
        updateTimer->stop();
    }
    if (audioProcessor) {
        audioProcessor->stopDemodulation();
    }
    if (remoteAudioPlayer) {
        remoteAudioPlayer->stop();
    }
    resetNetworkIqReceptionState(false, false, false);
    networkClientIqProcessingActive = false;
    activeNetworkClientProcessingMode = NetworkProcessingMode::ServerSide;
    qDebug() << "[NetworkIQ] client-side IQ processing stopped";
}

QJsonObject YourClassName::settingsToJson() const {
    QJsonObject settings;
    settings["deviceIndex"] = pendingSettings.deviceIndex;
    settings["clockSource"] = pendingSettings.clockSource;
    settings["inputMode"] = pendingSettings.inputMode;
    settings["centerFrequency"] = pendingSettings.centerFrequency;
    settings["actualFrequency"] = pendingSettings.actualFrequency;
    settings["listeningFrequency"] = pendingSettings.listeningFrequency;
    settings["sampleRate"] = pendingSettings.sampleRate;
    settings["bandwidth"] = pendingSettings.bandwidth;
    settings["modulationType"] = pendingSettings.modulationType;
    settings["fftLength"] = pendingSettings.fftLength;
    settings["lnaGain"] = pendingSettings.lnaGain;
    settings["vgaGain"] = pendingSettings.vgaGain;
    settings["audioLowPassHz"] = pendingSettings.audioLowPassHz;
    settings["audioHighPassHz"] = pendingSettings.audioHighPassHz;
    settings["hfNoiseCancelDepth"] = pendingSettings.hfNoiseCancelDepth;
    settings["hfNoiseCancelRefGainDb"] = pendingSettings.hfNoiseCancelRefGainDb;
    settings["hfNoiseCancelRefDelayNs"] = pendingSettings.hfNoiseCancelRefDelayNs;
    settings["hfNoiseCancelRefTiltDb"] = pendingSettings.hfNoiseCancelRefTiltDb;
    settings["hfNoiseCancelFreeze"] = pendingSettings.hfNoiseCancelFreeze;
    settings["audioEnabled"] = pendingSettings.audioEnabled;
    settings["syncEnabled"] = false;
    settings["gpoValue"] = static_cast<int>(pendingSettings.gpoValue);
    settings["dmrBasebandSampleRate"] = normalizedDmrBasebandSampleRate(pendingSettings.dmrBasebandSampleRate);
    settings["dmrAmbeLayout"] = normalizedDmrAmbeLayout(pendingSettings.dmrAmbeLayout);
    settings["dmrManualTimingEnabled"] = pendingSettings.dmrManualTimingEnabled;
    settings["dmrManualTimingOffset"] = pendingSettings.dmrManualTimingOffset;
    settings["dmrSlicerRatio"] = pendingSettings.dmrSlicerRatio;
    settings["dmrAdaptiveSlicer"] = pendingSettings.dmrAdaptiveSlicer;
    settings["scalePercent"] = currentScale;
    settings["agileScanEnabled"] = agileScanEnabled;
    settings["agileScanRangesMhz"] = agileScanRangesMhz;
    settings["agileScanStepMhz"] = agileScanStepMhz;
    settings["scanListeningLockEnabled"] = scanListeningLockEnabled;
    settings["standardScanEnabled"] = standardScanEnabled;
    settings["standardScanCentersMhz"] = standardScanCentersMhz;
    settings["standardScanDwellMs"] = standardScanDwellMs;
    settings["standardScanSettleMs"] = standardScanSettleMs;
    settings["standardScanRangeStartMhz"] = standardScanRangeStartMhz;
    settings["standardScanRangeEndMhz"] = standardScanRangeEndMhz;
    settings["spectrumUpdateIntervalMs"] = spectrumUpdateIntervalMs;
    settings["spurSuppressionEnabled"] = spurSuppressionEnabled;
    QJsonArray spurMask;
    for (const SpurMaskEntry &entry : spurMaskEntries) {
        if (!std::isfinite(entry.offsetHz) ||
            !std::isfinite(entry.widthHz) ||
            entry.widthHz <= 0.0) {
            continue;
        }
        QJsonObject object;
        object["offsetHz"] = entry.offsetHz;
        object["widthHz"] = entry.widthHz;
        object["prominenceDb"] = entry.prominenceDb;
        object["hits"] = entry.hits;
        spurMask.append(object);
    }
    settings["spurMask"] = spurMask;
    return settings;
}

bool YourClassName::sendRemoteControlCommand(const QString &action, const QJsonObject &extra) {
    if (!networkController || networkMode != NetworkMode::Client) {
        return false;
    }

    if (action != QStringLiteral("settings")) {
        cancelPendingRemoteSettingsCommand();
    }

    const bool controlActionAllowed =
        action == QStringLiteral("requestPriority") ||
        action == QStringLiteral("priorityResponse");
    if (!networkController->clientHasControl() && !controlActionAllowed) {
        qDebug() << "[Network] remote command blocked because this client is observer" << action;
        return false;
    }

    if (action == QStringLiteral("settings") || action == QStringLiteral("start")) {
        networkClientSettingsGuardTimer.restart();
    }

    refreshSettingsFromUi();

    QJsonObject command;
    command["type"] = "control";
    command["action"] = action;
    appendNetworkState(command);
    command["settings"] = settingsToJson();
    for (auto it = extra.constBegin(); it != extra.constEnd(); ++it) {
        command[it.key()] = it.value();
    }

    const bool sent = networkController->sendControlCommand(command);
    if (!sent) {
        qDebug() << "[Network] remote command could not be sent" << action;
    } else {
        qDebug() << "[Network] remote command sent" << action;
    }
    return sent;
}

void YourClassName::scheduleRemoteSettingsCommand(int delayMs) {
    if (!isNetworkClientMode() ||
        !networkController ||
        !networkController->clientHasControl()) {
        return;
    }

    networkClientSettingsGuardTimer.restart();

    if (!networkSettingsDebounceTimer) {
        sendRemoteControlCommand("settings");
        return;
    }

    const int clampedDelayMs = (std::clamp)(delayMs, 0, 1000);
    if (clampedDelayMs == 0) {
        networkSettingsDebounceTimer->stop();
        sendRemoteControlCommand("settings");
        return;
    }

    if (!networkSettingsDebounceTimer->isActive()) {
        networkSettingsDebounceTimer->start(clampedDelayMs);
    }
}

void YourClassName::cancelPendingRemoteSettingsCommand() {
    if (networkSettingsDebounceTimer) {
        networkSettingsDebounceTimer->stop();
    }
}

void YourClassName::applySettingsFromJson(const QJsonObject &settingsJson) {
    auto readInt = [&settingsJson](const char *key, int currentValue) {
        return settingsJson.contains(key) ? settingsJson.value(key).toInt(currentValue) : currentValue;
    };
    auto readDouble = [&settingsJson](const char *key, double currentValue) {
        return settingsJson.contains(key) ? settingsJson.value(key).toDouble(currentValue) : currentValue;
    };
    auto readBool = [&settingsJson](const char *key, bool currentValue) {
        return settingsJson.contains(key) ? settingsJson.value(key).toBool(currentValue) : currentValue;
    };

    pendingSettings.deviceIndex = readInt("deviceIndex", pendingSettings.deviceIndex);
    pendingSettings.clockSource = readInt("clockSource", pendingSettings.clockSource);
    pendingSettings.inputMode = (std::clamp)(readInt("inputMode", pendingSettings.inputMode),
                                             static_cast<int>(INPUT_RF),
                                             static_cast<int>(INPUT_HF_NOISE_CANCEL));
    pendingSettings.centerFrequency = readDouble("centerFrequency", pendingSettings.centerFrequency);
    pendingSettings.actualFrequency = readDouble("actualFrequency", pendingSettings.actualFrequency);
    pendingSettings.listeningFrequency = readDouble("listeningFrequency", pendingSettings.listeningFrequency);
    pendingSettings.sampleRate = readDouble("sampleRate", pendingSettings.sampleRate);
    pendingSettings.bandwidth = readDouble("bandwidth", pendingSettings.bandwidth);
    pendingSettings.modulationType = readInt("modulationType", pendingSettings.modulationType);
    pendingSettings.fftLength = readInt("fftLength", pendingSettings.fftLength);
    pendingSettings.lnaGain = readInt("lnaGain", pendingSettings.lnaGain);
    pendingSettings.vgaGain = readInt("vgaGain", pendingSettings.vgaGain);
    pendingSettings.audioLowPassHz = clampAudioLowPassHz(readDouble("audioLowPassHz", pendingSettings.audioLowPassHz));
    pendingSettings.audioHighPassHz = clampAudioHighPassHz(readDouble("audioHighPassHz", pendingSettings.audioHighPassHz));
    pendingSettings.hfNoiseCancelDepth = clampHfNoiseCancelDepth(readDouble("hfNoiseCancelDepth", pendingSettings.hfNoiseCancelDepth));
    pendingSettings.hfNoiseCancelRefGainDb =
        clampHfNoiseCancelRefGainDb(readDouble("hfNoiseCancelRefGainDb", pendingSettings.hfNoiseCancelRefGainDb));
    pendingSettings.hfNoiseCancelRefDelayNs =
        clampHfNoiseCancelRefDelayNs(readDouble("hfNoiseCancelRefDelayNs", pendingSettings.hfNoiseCancelRefDelayNs));
    pendingSettings.hfNoiseCancelRefTiltDb =
        clampHfNoiseCancelRefTiltDb(readDouble("hfNoiseCancelRefTiltDb", pendingSettings.hfNoiseCancelRefTiltDb));
    pendingSettings.hfNoiseCancelFreeze = readBool("hfNoiseCancelFreeze", pendingSettings.hfNoiseCancelFreeze);
    pendingSettings.audioEnabled = readBool("audioEnabled", pendingSettings.audioEnabled);
    pendingSettings.syncEnabled = false;
    pendingSettings.gpoValue = static_cast<std::uint8_t>(readInt("gpoValue", pendingSettings.gpoValue));
    pendingSettings.dmrBasebandSampleRate =
        normalizedDmrBasebandSampleRate(readInt("dmrBasebandSampleRate",
                                                pendingSettings.dmrBasebandSampleRate));
    pendingSettings.dmrAmbeLayout =
        normalizedDmrAmbeLayout(readInt("dmrAmbeLayout", pendingSettings.dmrAmbeLayout));
    pendingSettings.dmrManualTimingEnabled =
        readBool("dmrManualTimingEnabled", pendingSettings.dmrManualTimingEnabled);
    pendingSettings.dmrManualTimingOffset =
        (std::clamp)(readInt("dmrManualTimingOffset", pendingSettings.dmrManualTimingOffset),
                     -80,
                     80);
    pendingSettings.dmrSlicerRatio =
        (std::clamp)(readDouble("dmrSlicerRatio", pendingSettings.dmrSlicerRatio),
                     0.45,
                     0.80);
    pendingSettings.dmrAdaptiveSlicer =
        readBool("dmrAdaptiveSlicer", pendingSettings.dmrAdaptiveSlicer);
    currentScale = readDouble("scalePercent", currentScale);
    agileScanEnabled = readBool("agileScanEnabled", agileScanEnabled);
    agileScanRangesMhz = settingsJson.value("agileScanRangesMhz").toString(agileScanRangesMhz).trimmed();
    agileScanStepMhz = (std::clamp)(readDouble("agileScanStepMhz", agileScanStepMhz),
                                    AGILE_SCAN_MIN_STEP_MHZ,
                                    AGILE_SCAN_MAX_STEP_MHZ);
    scanListeningLockEnabled = readBool("scanListeningLockEnabled", scanListeningLockEnabled);
    standardScanEnabled = readBool("standardScanEnabled", standardScanEnabled);
    standardScanCentersMhz =
        settingsJson.value("standardScanCentersMhz").toString(standardScanCentersMhz).trimmed();
    standardScanDwellMs = (std::clamp)(readInt("standardScanDwellMs", standardScanDwellMs),
                                       STANDARD_SCAN_MIN_DWELL_MS,
                                       STANDARD_SCAN_MAX_DWELL_MS);
    standardScanSettleMs = (std::clamp)(readInt("standardScanSettleMs", standardScanSettleMs),
                                        STANDARD_SCAN_MIN_SETTLE_MS,
                                        STANDARD_SCAN_MAX_SETTLE_MS);
    standardScanRangeStartMhz =
        settingsJson.value("standardScanRangeStartMhz").toString(standardScanRangeStartMhz).trimmed();
    standardScanRangeEndMhz =
        settingsJson.value("standardScanRangeEndMhz").toString(standardScanRangeEndMhz).trimmed();
    spectrumUpdateIntervalMs = (std::clamp)(readInt("spectrumUpdateIntervalMs", spectrumUpdateIntervalMs),
                                            SPECTRUM_UPDATE_AUTO_MS,
                                            SPECTRUM_UPDATE_MAX_MS);
    if (spectrumUpdateIntervalMs > 0 && spectrumUpdateIntervalMs < SPECTRUM_UPDATE_MIN_MS) {
        spectrumUpdateIntervalMs = SPECTRUM_UPDATE_MIN_MS;
    }
    {
        bool adjusted = false;
        QString standardScanError;
        const QVector<double> normalized =
            parseStandardScanCentersMhz(standardScanCentersMhz,
                                        pendingSettings.sampleRate,
                                        0,
                                        &standardScanError,
                                        &adjusted);
        if (adjusted && standardScanError.isEmpty() && !normalized.isEmpty()) {
            standardScanCentersMhz = formatMhzList(normalized);
        }
    }
    spurSuppressionEnabled = readBool("spurSuppressionEnabled", spurSuppressionEnabled);
    if (settingsJson.contains(QStringLiteral("spurMask"))) {
        QVector<SpurMaskEntry> nextMask;
        const QJsonArray array = settingsJson.value(QStringLiteral("spurMask")).toArray();
        for (const QJsonValue &value : array) {
            const QJsonObject object = value.toObject();
            SpurMaskEntry entry;
            entry.offsetHz = object.value(QStringLiteral("offsetHz")).toDouble(std::numeric_limits<double>::quiet_NaN());
            entry.widthHz = object.value(QStringLiteral("widthHz")).toDouble(SPUR_MIN_MASK_WIDTH_HZ);
            entry.prominenceDb = static_cast<float>(object.value(QStringLiteral("prominenceDb")).toDouble(0.0));
            entry.hits = object.value(QStringLiteral("hits")).toInt(0);
            if (std::isfinite(entry.offsetHz) &&
                std::isfinite(entry.widthHz) &&
                entry.widthHz > 0.0) {
                nextMask.append(entry);
            }
        }
        spurMaskEntries = nextMask;
    }
    if (spurSuppressionCheckbox) {
        QSignalBlocker blocker(spurSuppressionCheckbox);
        spurSuppressionCheckbox->setChecked(spurSuppressionEnabled);
    }
    updateSpurSuppressionStatus();
    normalizeTuning(pendingSettings);
}

void YourClassName::loadUiTranslations() {
    auto baseLanguage = [](std::initializer_list<std::pair<const char *, const char *>> entries) {
        QJsonObject object;
        for (const auto &entry : entries) {
            object[QString::fromLatin1(entry.first)] = QString::fromUtf8(entry.second);
        }
        return object;
    };

    uiTranslations = QJsonObject();
    uiTranslations[QStringLiteral("en")] = baseLanguage({
        {"controls", "Controls"},
        {"digital_audio", "Digital Audio"},
        {"video", "Video"},
        {"settings_short", "Cfg"},
        {"settings", "Settings..."},
        {"decode", "Decode"},
        {"dmr_lock", "Lock DMR"},
        {"clear", "Clear"},
        {"invert", "Invert"},
        {"test", "Test"},
        {"refresh_usb", "Refresh USB"},
        {"refresh_usb_devices", "Refresh USB Devices"},
        {"show_fobos_details", "Show Fobos Details"},
        {"language", "Lang:"},
        {"clock", "Clock:"},
        {"mode", "Mode:"},
        {"sample", "Sample:"},
        {"fft", "FFT:"},
        {"central_frequency", "Central Frequency:"},
        {"listening_frequency", "Listening Frequency:"},
        {"presets", "Presets..."},
        {"audio_bandwidth", "Audio Bandwidth:"},
        {"network", "Network"},
        {"recording_idle", "Recording: idle"},
        {"playback_idle", "Playback: idle"},
        {"audio_wav", "Audio WAV"},
        {"channel_iq_wav", "Channel IQ WAV"},
        {"no_wav_recordings_found", "No WAV recordings found"},
        {"recording_blocked_channel_iq_full_iq", "Recording blocked: Channel IQ cannot run during Full IQ streaming"},
        {"recording_stopped_channel_iq_full_iq", "Recording stopped: Channel IQ cannot run during Full IQ streaming"},
        {"recording_failed", "Recording failed"},
        {"recording_audio", "Recording audio"},
        {"recording_channel_iq_waiting", "Recording channel IQ: waiting for IQ frames"},
        {"recording_channel_iq", "Recording channel IQ"},
        {"iq_recording_failed", "IQ recording failed"},
        {"recording_saved", "Recording saved"},
        {"recording_stopped_no_data", "Recording stopped: no data"},
        {"playback_blocked_stop_receiver", "Playback blocked: stop receiver first"},
        {"playback_blocked_stop_recording", "Playback blocked: stop recording first"},
        {"playback_no_file_selected", "Playback: no file selected"},
        {"playback_failed", "Playback failed"},
        {"playback_audio_wav_mono", "Playback failed: audio WAV must be mono 48 kHz"},
        {"playback_stopped", "Playback: stopped"},
        {"playback", "Playback"},
        {"record", "Record"},
        {"hold_f9", "Hold F9"},
        {"stop_rec", "Stop Rec"},
        {"refresh_playback", "Refresh Playback"},
        {"play", "Play"},
        {"stop_play", "Stop Play"},
        {"start", "Start"},
        {"stop", "Stop"},
        {"audio", "Audio"},
        {"sync", "Sync"},
        {"spectrum2", "Spectr 2"},
        {"colorful", "Colorful"},
        {"general_band_markers", "Band markers"},
        {"amateur_band_markers", "HAM bands"},
        {"compact_band_markers", "Collapsed"},
        {"logging", "Logging"},
        {"logging_tooltip", "Write detailed diagnostic logs and DMR dumps"},
        {"internal", "Internal"},
        {"external", "External"},
        {"hf_cancel_lab", "HF1 - HF2 cancel lab"},
        {"volume", "Volume"},
        {"lna_gain", "LNA Gain"},
        {"vga_gain", "VGA Gain"},
        {"scale", "Scale"},
        {"contrast", "Contrast"},
        {"sensitivity", "Sensitivity"},
        {"min", "Min"},
        {"max", "Max"},
        {"fine_tune", "Fine tune"},
        {"fine_tune_scale", "Horizontal scale (mouse wheel)"},
        {"fine_tune_dial", "Round dial"},
        {"quick_options", "Quick options"},
        {"audio_lpf", "Audio LPF"},
        {"audio_hpf", "Audio HPF"},
        {"auto", "Auto"},
        {"off", "Off"},
        {"hf_cancel", "HF cancel"},
        {"ref_gain", "Ref gain"},
        {"ref_delay", "Ref delay"},
        {"ref_tilt", "Ref tilt"},
        {"freeze", "Freeze"},
        {"agile_scan", "Agile scan"},
        {"enable_scan", "Enable scan"},
        {"save", "Save"},
        {"delete_short", "Del"},
        {"ranges_mhz", "Ranges MHz:"},
        {"step", "Step:"}
    });
    uiTranslations[QStringLiteral("uk")] = baseLanguage({
        {"controls", "Керування"},
        {"digital_audio", "Цифрове аудіо"},
        {"video", "Відео"},
        {"settings_short", "Cfg"},
        {"decode", "Декод"},
        {"dmr_lock", "Lock DMR"},
        {"clear", "Очистити"},
        {"invert", "Інверсія"},
        {"test", "Тест"},
        {"refresh_usb", "Оновити USB"},
        {"refresh_usb_devices", "Оновити USB-пристрої"},
        {"show_fobos_details", "Деталі Fobos"},
        {"language", "Мова:"},
        {"clock", "Такт:"},
        {"mode", "Режим:"},
        {"sample", "Семпл:"},
        {"fft", "FFT:"},
        {"central_frequency", "Центральна частота:"},
        {"listening_frequency", "Частота прослух.:"},
        {"audio_bandwidth", "Смуга аудіо:"},
        {"network", "Мережа"},
        {"recording_idle", "Recording: idle"},
        {"playback_idle", "Playback: idle"},
        {"audio_wav", "Audio WAV"},
        {"channel_iq_wav", "Channel IQ WAV"},
        {"no_wav_recordings_found", "No WAV recordings found"},
        {"recording_blocked_channel_iq_full_iq", "Recording blocked: Channel IQ cannot run during Full IQ streaming"},
        {"recording_stopped_channel_iq_full_iq", "Recording stopped: Channel IQ cannot run during Full IQ streaming"},
        {"recording_failed", "Recording failed"},
        {"recording_audio", "Recording audio"},
        {"recording_channel_iq_waiting", "Recording channel IQ: waiting for IQ frames"},
        {"recording_channel_iq", "Recording channel IQ"},
        {"iq_recording_failed", "IQ recording failed"},
        {"recording_saved", "Recording saved"},
        {"recording_stopped_no_data", "Recording stopped: no data"},
        {"playback_blocked_stop_receiver", "Playback blocked: stop receiver first"},
        {"playback_blocked_stop_recording", "Playback blocked: stop recording first"},
        {"playback_no_file_selected", "Playback: no file selected"},
        {"playback_failed", "Playback failed"},
        {"playback_audio_wav_mono", "Playback failed: audio WAV must be mono 48 kHz"},
        {"playback_stopped", "Playback: stopped"},
        {"playback", "Playback"},
        {"record", "Запис"},
        {"hold_f9", "F9 утрим."},
        {"stop_rec", "Стоп запис"},
        {"refresh_playback", "Оновити записи"},
        {"play", "Відтворити"},
        {"stop_play", "Стоп плей"},
        {"start", "Старт"},
        {"stop", "Стоп"},
        {"audio", "Аудіо"},
        {"sync", "Синхр."},
        {"spectrum2", "Спектр 2"},
        {"colorful", "Колір"},
        {"internal", "Внутр."},
        {"external", "Зовн."},
        {"hf_cancel_lab", "HF1 - HF2 лаб."},
        {"volume", "Гучність"},
        {"lna_gain", "LNA"},
        {"vga_gain", "VGA"},
        {"scale", "Масштаб"},
        {"contrast", "Контраст"},
        {"sensitivity", "Чутливість"},
        {"min", "Мін"},
        {"max", "Макс"},
        {"audio_lpf", "Аудіо ФНЧ"},
        {"audio_hpf", "Аудіо ФВЧ"},
        {"auto", "Авто"},
        {"off", "Викл."},
        {"hf_cancel", "HF компенсація"},
        {"ref_gain", "Опора"},
        {"ref_delay", "Затримка"},
        {"ref_tilt", "Нахил"},
        {"freeze", "Фікс."}
    });

    const QString appDir = QCoreApplication::applicationDirPath();
    const QStringList candidates = {
        QDir(appDir).absoluteFilePath(QStringLiteral("translations.json")),
        QDir::current().absoluteFilePath(QStringLiteral("translations.json")),
        QDir(appDir).absoluteFilePath(QStringLiteral("../../translations.json"))
    };

    for (const QString &path : candidates) {
        QFile file(path);
        if (!file.open(QIODevice::ReadOnly)) {
            continue;
        }
        QJsonParseError parseError;
        const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
        if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
            qDebug() << "[Translations] failed to parse" << path << parseError.errorString();
            continue;
        }
        const QJsonObject externalRoot = document.object();
        for (const QString &language : {QStringLiteral("en"), QStringLiteral("uk")}) {
            QJsonObject merged = uiTranslations.value(language).toObject();
            const QJsonObject externalLanguage = externalRoot.value(language).toObject();
            for (auto it = externalLanguage.constBegin(); it != externalLanguage.constEnd(); ++it) {
                merged[it.key()] = it.value();
            }
            uiTranslations[language] = merged;
        }
        qDebug() << "[Translations] loaded" << path;
        break;
    }
}

QString YourClassName::uiText(const QString &key, const QString &fallback) const {
    const QJsonObject language = uiTranslations.value(uiLanguage).toObject();
    const QString translated = language.value(key).toString();
    if (!translated.isEmpty()) {
        return translated;
    }
    const QString english = uiTranslations.value(QStringLiteral("en")).toObject().value(key).toString();
    return english.isEmpty() ? fallback : english;
}

QString YourClassName::localizedStatusText(const QString &status) const {
    const auto prefixStatus = [this, &status](const QString &prefix,
                                             const QString &key,
                                             const QString &fallback,
                                             const QString &separator) -> QString {
        if (!status.startsWith(prefix)) {
            return QString();
        }
        const QString base = uiText(key, fallback);
        const QString suffix = status.mid(prefix.size());
        return suffix.isEmpty() ? base : QStringLiteral("%1%2%3").arg(base, separator, suffix);
    };

    if (status == QStringLiteral("Recording: idle")) {
        return uiText(QStringLiteral("recording_idle"), status);
    }
    if (status == QStringLiteral("Playback: idle")) {
        return uiText(QStringLiteral("playback_idle"), status);
    }
    if (status == QStringLiteral("Recording blocked: Channel IQ cannot run during Full IQ streaming")) {
        return uiText(QStringLiteral("recording_blocked_channel_iq_full_iq"), status);
    }
    if (status == QStringLiteral("Recording stopped: Channel IQ cannot run during Full IQ streaming")) {
        return uiText(QStringLiteral("recording_stopped_channel_iq_full_iq"), status);
    }
    if (status == QStringLiteral("Recording stopped: no data")) {
        return uiText(QStringLiteral("recording_stopped_no_data"), status);
    }
    if (status == QStringLiteral("Playback blocked: stop receiver first")) {
        return uiText(QStringLiteral("playback_blocked_stop_receiver"), status);
    }
    if (status == QStringLiteral("Playback blocked: stop recording first")) {
        return uiText(QStringLiteral("playback_blocked_stop_recording"), status);
    }
    if (status == QStringLiteral("Playback: no file selected")) {
        return uiText(QStringLiteral("playback_no_file_selected"), status);
    }
    if (status == QStringLiteral("Playback failed: audio WAV must be mono 48 kHz")) {
        return uiText(QStringLiteral("playback_audio_wav_mono"), status);
    }
    if (status == QStringLiteral("Playback: stopped")) {
        return uiText(QStringLiteral("playback_stopped"), status);
    }

    const QStringList prefixTranslations = {
        prefixStatus(QStringLiteral("Recording failed: "), QStringLiteral("recording_failed"), QStringLiteral("Recording failed"), QStringLiteral(": ")),
        prefixStatus(QStringLiteral("Recording audio: "), QStringLiteral("recording_audio"), QStringLiteral("Recording audio"), QStringLiteral(": ")),
        prefixStatus(QStringLiteral("Recording channel IQ: waiting for IQ frames"),
                     QStringLiteral("recording_channel_iq_waiting"),
                     QStringLiteral("Recording channel IQ: waiting for IQ frames"),
                     QString()),
        prefixStatus(QStringLiteral("Recording channel IQ: "), QStringLiteral("recording_channel_iq"), QStringLiteral("Recording channel IQ"), QStringLiteral(": ")),
        prefixStatus(QStringLiteral("IQ recording failed: "), QStringLiteral("iq_recording_failed"), QStringLiteral("IQ recording failed"), QStringLiteral(": ")),
        prefixStatus(QStringLiteral("Recording saved: "), QStringLiteral("recording_saved"), QStringLiteral("Recording saved"), QStringLiteral(": ")),
        prefixStatus(QStringLiteral("Playback failed: "), QStringLiteral("playback_failed"), QStringLiteral("Playback failed"), QStringLiteral(": ")),
        prefixStatus(QStringLiteral("Playback: "), QStringLiteral("playback"), QStringLiteral("Playback"), QStringLiteral(": "))
    };

    for (const QString &translated : prefixTranslations) {
        if (!translated.isEmpty()) {
            return translated;
        }
    }
    return status;
}

void YourClassName::markTranslatable(QWidget *widget, const QString &key, const QString &fallback) {
    if (!widget) {
        return;
    }
    widget->setProperty("i18nKey", key);
    widget->setProperty("i18nFallback", fallback);
    const QString text = uiText(key, fallback);
    if (auto *label = qobject_cast<QLabel*>(widget)) {
        label->setText(text);
    } else if (auto *button = qobject_cast<QAbstractButton*>(widget)) {
        button->setText(text);
    } else if (auto *group = qobject_cast<QGroupBox*>(widget)) {
        group->setTitle(text);
    } else if (auto *dock = qobject_cast<QDockWidget*>(widget)) {
        dock->setWindowTitle(text);
    }
}

void YourClassName::setComboItemText(QComboBox *combo,
                                     const QVariant &data,
                                     const QString &key,
                                     const QString &fallback) {
    if (!combo) {
        return;
    }
    const int index = combo->findData(data);
    if (index >= 0) {
        combo->setItemText(index, uiText(key, fallback));
    }
}

void YourClassName::applyUiLanguage() {
    const auto widgets = findChildren<QWidget*>();
    for (QWidget *widget : widgets) {
        const QString key = widget->property("i18nKey").toString();
        if (key.isEmpty()) {
            continue;
        }
        markTranslatable(widget, key, widget->property("i18nFallback").toString());
    }

    if (languageComboBox) {
        QSignalBlocker blocker(languageComboBox);
        const int index = languageComboBox->findData(uiLanguage);
        if (index >= 0) {
            languageComboBox->setCurrentIndex(index);
        }
    }

    setComboItemText(clkBox, 0, QStringLiteral("internal"), QStringLiteral("Internal"));
    setComboItemText(clkBox, 1, QStringLiteral("external"), QStringLiteral("External"));
    setComboItemText(modeBox, static_cast<int>(INPUT_HF_NOISE_CANCEL),
                     QStringLiteral("hf_cancel_lab"),
                     QStringLiteral("HF1 - HF2 cancel lab"));
    setComboItemText(recordingModeCombo,
                     static_cast<int>(RecordingManager::Mode::AudioWav),
                     QStringLiteral("audio_wav"),
                     QStringLiteral("Audio WAV"));
    setComboItemText(recordingModeCombo,
                     static_cast<int>(RecordingManager::Mode::ChannelIqWav),
                     QStringLiteral("channel_iq_wav"),
                     QStringLiteral("Channel IQ WAV"));
    if (playbackFileCombo &&
        playbackFileCombo->count() == 1 &&
        playbackFileCombo->itemData(0).toString().isEmpty()) {
        playbackFileCombo->setItemText(0,
                                       uiText(QStringLiteral("no_wav_recordings_found"),
                                              QStringLiteral("No WAV recordings found")));
    }

    if (volumeLabel) {
        volumeLabel->setText(QStringLiteral("%1: %2%").arg(uiText(QStringLiteral("volume"), QStringLiteral("Volume"))).arg(volumePercent));
    }
    if (lnaGainLabel) {
        lnaGainLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("lna_gain"), QStringLiteral("LNA Gain"))).arg(pendingSettings.lnaGain));
    }
    if (vgaGainLabel) {
        vgaGainLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("vga_gain"), QStringLiteral("VGA Gain"))).arg(pendingSettings.vgaGain));
    }
    if (scaleLabel) {
        scaleLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("scale"), QStringLiteral("Scale")),
                                                        formatScalePercent(currentScale)));
    }
    if (contrastLabel) {
        contrastLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("contrast"), QStringLiteral("Contrast"))).arg(contrast));
    }
    if (sensitivityLabel) {
        sensitivityLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("sensitivity"), QStringLiteral("Sensitivity"))).arg(sensitivity));
    }
    if (levelMinLabel) {
        levelMinLabel->setText(levelLabelText(uiText(QStringLiteral("min"), QStringLiteral("Min")), displayLevelMin));
    }
    if (levelMaxLabel) {
        levelMaxLabel->setText(levelLabelText(uiText(QStringLiteral("max"), QStringLiteral("Max")), displayLevelMax));
    }
    updateFineTuneLabel();
    if (scanMeasurementBaselineButton) {
        scanMeasurementBaselineButton->setText(scanMeasurementBaselineRecording
                                                   ? uiText(QStringLiteral("stop_bg"), QStringLiteral("Stop BG"))
                                                   : uiText(QStringLiteral("bg_rec"), QStringLiteral("BG Rec")));
    }
    if (recordingStatusLabel) {
        const QString rawStatus = recordingStatusLabel->property("statusRawText").toString();
        recordingStatusLabel->setText(localizedStatusText(rawStatus.isEmpty()
                                                              ? QStringLiteral("Recording: idle")
                                                              : rawStatus));
    }
    if (playbackStatusLabel) {
        const QString rawStatus = playbackStatusLabel->property("statusRawText").toString();
        playbackStatusLabel->setText(localizedStatusText(rawStatus.isEmpty()
                                                             ? QStringLiteral("Playback: idle")
                                                             : rawStatus));
    }

    updateAudioFilterLabels();
    updateHfNoiseCancelControls();
    applySpectrumHunterTranslations();
    updateAgileScanControls();
    updateScanMeasurementStatus();
    updateSpurSuppressionStatus();
    updateVideoProcessorMode();
    updateNetworkButtonText();
}

void YourClassName::applySpectrumHunterTranslations() {
    auto applyHunter = [this](SpectrumHunterControls *controls,
                              const QString &titleKey,
                              const QString &titleFallback,
                              const QString &tooltipKey,
                              const QString &tooltipFallback) {
        if (!controls) {
            return;
        }
        controls->setUiText(
            uiText(titleKey, titleFallback),
            uiText(tooltipKey, tooltipFallback),
            uiText(QStringLiteral("detect"), QStringLiteral("Detect")),
            uiText(QStringLiteral("use_scan"), QStringLiteral("Use scan")),
            uiText(QStringLiteral("use_scan_tooltip"),
                   QStringLiteral("Copy this preset into Agile scan ranges and step")),
            uiText(QStringLiteral("tune"), QStringLiteral("Tune")),
            uiText(QStringLiteral("tune_candidate_tooltip"),
                   QStringLiteral("Tune to the best detected candidate center")),
            uiText(QStringLiteral("follow"), QStringLiteral("Follow")),
            uiText(QStringLiteral("follow_candidate_tooltip"),
                   QStringLiteral("Keep tuning to the selected detected candidate as the scan updates")),
            uiText(QStringLiteral("previous_candidate_tooltip"),
                   QStringLiteral("Tune to previous detected candidate")),
            uiText(QStringLiteral("next_candidate_tooltip"),
                   QStringLiteral("Tune to next detected candidate")),
            uiText(QStringLiteral("min_width_short"), QStringLiteral("Min W:")),
            uiText(QStringLiteral("max_width_short"), QStringLiteral("Max W:")),
            uiText(QStringLiteral("threshold_short"), QStringLiteral("Thr:")));
    };

    applyHunter(dmrHunterControls,
                QStringLiteral("dmr_hunter"),
                QStringLiteral("DMR Hunter"),
                QStringLiteral("dmr_hunter_tooltip"),
                QStringLiteral("Detect DMR-like TDMA bursts in the current spectrum"));
    applyHunter(fpvHunterControls,
                QStringLiteral("fpv_hunter"),
                QStringLiteral("FPV Hunter"),
                QStringLiteral("fpv_hunter_tooltip"),
                QStringLiteral("Detect wide FPV-like video carriers in the current spectrum"));
    applyHunter(digitalVideoHunterControls,
                QStringLiteral("digital_video_hunter"),
                QStringLiteral("Digital Video Hunter"),
                QStringLiteral("digital_video_hunter_tooltip"),
                QStringLiteral("Detect wide digital video / OFDM-like carriers in the current spectrum"));
}

void YourClassName::updateUiFromPendingSettings() {
    if (comboBox) {
        comboBox->blockSignals(true);
        if (pendingSettings.deviceIndex >= 0 && pendingSettings.deviceIndex < comboBox->count()) {
            comboBox->setCurrentIndex(pendingSettings.deviceIndex);
        }
        comboBox->blockSignals(false);
    }
    if (clkBox) {
        clkBox->blockSignals(true);
        const int index = clkBox->findData(pendingSettings.clockSource);
        if (index >= 0) {
            clkBox->setCurrentIndex(index);
        }
        clkBox->blockSignals(false);
    }
    if (modeBox) {
        modeBox->blockSignals(true);
        const int index = modeBox->findData(pendingSettings.inputMode);
        if (index >= 0) {
            modeBox->setCurrentIndex(index);
        }
        modeBox->blockSignals(false);
    }
    if (sampleBox) {
        sampleBox->blockSignals(true);
        int bestIndex = -1;
        double bestDelta = std::numeric_limits<double>::max();
        for (int i = 0; i < sampleBox->count(); ++i) {
            bool ok = false;
            const double value = sampleBox->itemData(i).toDouble(&ok);
            if (!ok) {
                continue;
            }
            const double delta = std::abs(value - pendingSettings.sampleRate);
            if (delta < bestDelta) {
                bestDelta = delta;
                bestIndex = i;
            }
        }
        if (bestIndex >= 0 && bestDelta <= 0.5) {
            sampleBox->setCurrentIndex(bestIndex);
        } else {
            sampleBox->addItem(formatSampleRate(pendingSettings.sampleRate), pendingSettings.sampleRate);
            sampleBox->setCurrentIndex(sampleBox->count() - 1);
        }
        sampleBox->blockSignals(false);
    }
    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
        if (frequencyControlUiStateRestorePending) {
            frequencyControl->setSelectedStepName(centerFrequencyStepName);
            frequencyControl->setSelectedValuePresetName(centerFrequencyPresetName);
            frequencyControl->setSelectedUnitIndex(centerFrequencyUnitIndex);
        }
    }
    if (listeningFrequencyControl) {
        QSignalBlocker blocker(listeningFrequencyControl);
        if (pendingSettings.inputMode == INPUT_RF) {
            listeningFrequencyControl->setRangeHz(RF_MIN_LISTENING_FREQUENCY, RF_EXPERIMENTAL_MAX_FREQUENCY);
        } else {
            listeningFrequencyControl->setRangeHz(directMinFrequencyForMode(pendingSettings.inputMode,
                                                                            pendingSettings.sampleRate),
                                                  directMaxFrequency(pendingSettings.sampleRate));
        }
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
        if (frequencyControlUiStateRestorePending) {
            listeningFrequencyControl->setSelectedStepName(listeningFrequencyStepName);
            listeningFrequencyControl->setSelectedValuePresetName(listeningFrequencyPresetName);
            listeningFrequencyControl->setSelectedUnitIndex(listeningFrequencyUnitIndex);
        }
    }
    if (bandwidthControl) {
        QSignalBlocker blocker(bandwidthControl);
        bandwidthControl->setValueHz(pendingSettings.bandwidth);
        if (frequencyControlUiStateRestorePending) {
            bandwidthControl->setSelectedStepName(bandwidthStepName);
            bandwidthControl->setSelectedValuePresetName(bandwidthPresetName);
            bandwidthControl->setSelectedUnitIndex(bandwidthUnitIndex);
        }
    }
    frequencyControlUiStateRestorePending = false;
    if (fftComboBox) {
        fftComboBox->blockSignals(true);
        fftComboBox->setCurrentText(QString::number(pendingSettings.fftLength));
        fftComboBox->blockSignals(false);
    }
    if (modulationButtonGroup) {
        QAbstractButton *button = modulationButtonGroup->button(pendingSettings.modulationType);
        if (button) {
            modulationButtonGroup->blockSignals(true);
            button->setChecked(true);
            modulationButtonGroup->blockSignals(false);
        }
    }
    if (lnaGainSlider) {
        lnaGainSlider->blockSignals(true);
        lnaGainSlider->setValue(pendingSettings.lnaGain);
        lnaGainSlider->blockSignals(false);
    }
    if (lnaGainLabel) {
        lnaGainLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("lna_gain"), QStringLiteral("LNA Gain"))).arg(pendingSettings.lnaGain));
    }
    if (vgaGainSlider) {
        vgaGainSlider->blockSignals(true);
        vgaGainSlider->setValue(pendingSettings.vgaGain);
        vgaGainSlider->blockSignals(false);
    }
    if (vgaGainLabel) {
        vgaGainLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("vga_gain"), QStringLiteral("VGA Gain"))).arg(pendingSettings.vgaGain));
    }
    if (audioCheckbox) {
        audioCheckbox->blockSignals(true);
        audioCheckbox->setChecked(pendingSettings.audioEnabled);
        audioCheckbox->blockSignals(false);
    }
    if (audioDeviceComboBox) {
        audioDeviceComboBox->blockSignals(true);
        const int index = audioDeviceComboBox->findData(pendingSettings.audioDeviceId);
        if (index >= 0) {
            audioDeviceComboBox->setCurrentIndex(index);
        }
        audioDeviceComboBox->blockSignals(false);
    }
    for (int i = 0; i < 8; ++i) {
        if (checkBoxes[i]) {
            checkBoxes[i]->blockSignals(true);
            checkBoxes[i]->setChecked((pendingSettings.gpoValue & (1 << i)) != 0);
            checkBoxes[i]->blockSignals(false);
        }
    }
    if (scaleWidget) {
        scaleWidget->setTuning(pendingSettings.listeningFrequency,
                               pendingSettings.centerFrequency,
                               pendingSettings.bandwidth,
                               pendingSettings.modulationType);
    }
    if (scaleSlider) {
        scaleSlider->blockSignals(true);
        scaleSlider->setValue(scalePercentToSliderValue(currentScale));
        scaleSlider->blockSignals(false);
    }
    if (scaleLabel) {
        scaleLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("scale"), QStringLiteral("Scale")),
                                                        formatScalePercent(currentScale)));
    }
    if (contrastSlider) {
        contrastSlider->blockSignals(true);
        contrastSlider->setValue(static_cast<int>(std::lround(contrast)));
        contrastSlider->blockSignals(false);
    }
    if (contrastLabel) {
        contrastLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("contrast"), QStringLiteral("Contrast"))).arg(contrast));
    }
    if (sensitivitySlider) {
        sensitivitySlider->blockSignals(true);
        sensitivitySlider->setValue(static_cast<int>(std::lround(sensitivity)));
        sensitivitySlider->blockSignals(false);
    }
    if (sensitivityLabel) {
        sensitivityLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("sensitivity"), QStringLiteral("Sensitivity"))).arg(sensitivity));
    }
    if (levelMinSlider) {
        levelMinSlider->blockSignals(true);
        levelMinSlider->setValue(levelToSliderValue(displayLevelMin));
        levelMinSlider->blockSignals(false);
    }
    if (levelMaxSlider) {
        levelMaxSlider->blockSignals(true);
        levelMaxSlider->setValue(levelToSliderValue(displayLevelMax));
        levelMaxSlider->blockSignals(false);
    }
    if (levelMinLabel) {
        levelMinLabel->setText(levelLabelText(uiText(QStringLiteral("min"), QStringLiteral("Min")), displayLevelMin));
    }
    if (levelMaxLabel) {
        levelMaxLabel->setText(levelLabelText(uiText(QStringLiteral("max"), QStringLiteral("Max")), displayLevelMax));
    }
    if (graphCheckbox) {
        graphCheckbox->blockSignals(true);
        graphCheckbox->setChecked(secondGraph);
        graphCheckbox->blockSignals(false);
    }
    if (colorCheckbox) {
        colorCheckbox->blockSignals(true);
        colorCheckbox->setChecked(colorf);
        colorCheckbox->blockSignals(false);
    }
    if (volumeSlider) {
        volumeSlider->blockSignals(true);
        volumeSlider->setValue(volumePercent);
        volumeSlider->blockSignals(false);
    }
    if (volumeLabel) {
        volumeLabel->setText(QStringLiteral("%1: %2%").arg(uiText(QStringLiteral("volume"), QStringLiteral("Volume"))).arg(volumePercent));
    }
    if (audioLowPassSlider) {
        audioLowPassSlider->blockSignals(true);
        audioLowPassSlider->setValue(audioLowPassHzToSliderValue(pendingSettings.audioLowPassHz));
        audioLowPassSlider->blockSignals(false);
    }
    if (audioHighPassSlider) {
        audioHighPassSlider->blockSignals(true);
        audioHighPassSlider->setValue(audioHighPassHzToSliderValue(pendingSettings.audioHighPassHz));
        audioHighPassSlider->blockSignals(false);
    }
    if (hfNoiseCancelDepthSlider) {
        hfNoiseCancelDepthSlider->blockSignals(true);
        hfNoiseCancelDepthSlider->setValue(hfNoiseCancelDepthToSliderValue(pendingSettings.hfNoiseCancelDepth));
        hfNoiseCancelDepthSlider->blockSignals(false);
    }
    if (hfNoiseCancelRefGainSlider) {
        hfNoiseCancelRefGainSlider->blockSignals(true);
        hfNoiseCancelRefGainSlider->setValue(hfNoiseCancelRefGainToSliderValue(pendingSettings.hfNoiseCancelRefGainDb));
        hfNoiseCancelRefGainSlider->blockSignals(false);
    }
    if (hfNoiseCancelRefDelaySlider) {
        hfNoiseCancelRefDelaySlider->blockSignals(true);
        hfNoiseCancelRefDelaySlider->setValue(hfNoiseCancelRefDelayToSliderValue(pendingSettings.hfNoiseCancelRefDelayNs));
        hfNoiseCancelRefDelaySlider->blockSignals(false);
    }
    if (hfNoiseCancelRefTiltSlider) {
        hfNoiseCancelRefTiltSlider->blockSignals(true);
        hfNoiseCancelRefTiltSlider->setValue(hfNoiseCancelRefTiltToSliderValue(pendingSettings.hfNoiseCancelRefTiltDb));
        hfNoiseCancelRefTiltSlider->blockSignals(false);
    }
    if (hfNoiseCancelFreezeCheckbox) {
        hfNoiseCancelFreezeCheckbox->blockSignals(true);
        hfNoiseCancelFreezeCheckbox->setChecked(pendingSettings.hfNoiseCancelFreeze);
        hfNoiseCancelFreezeCheckbox->blockSignals(false);
    }
    if (agileScanCheckbox) {
        QSignalBlocker blocker(agileScanCheckbox);
        agileScanCheckbox->setChecked(agileScanEnabled);
    }
    if (agileScanRangesEdit) {
        QSignalBlocker blocker(agileScanRangesEdit);
        agileScanRangesEdit->setText(agileScanRangesMhz);
    }
    if (agileScanStepSpin) {
        QSignalBlocker blocker(agileScanStepSpin);
        agileScanStepSpin->setValue(agileScanStepMhz);
    }
    if (standardScanCheckbox) {
        QSignalBlocker blocker(standardScanCheckbox);
        standardScanCheckbox->setChecked(standardScanEnabled);
    }
    if (scanListeningLockCheckbox) {
        QSignalBlocker blocker(scanListeningLockCheckbox);
        scanListeningLockCheckbox->setChecked(scanListeningLockEnabled);
    }
    if (standardScanCentersEdit) {
        QSignalBlocker blocker(standardScanCentersEdit);
        standardScanCentersEdit->setText(standardScanCentersMhz);
    }
    if (standardScanDwellSpin) {
        QSignalBlocker blocker(standardScanDwellSpin);
        standardScanDwellSpin->setValue(standardScanDwellMs);
    }
    if (standardScanSettleSpin) {
        QSignalBlocker blocker(standardScanSettleSpin);
        standardScanSettleSpin->setValue(standardScanSettleMs);
    }
    if (standardScanRangeStartEdit) {
        QSignalBlocker blocker(standardScanRangeStartEdit);
        standardScanRangeStartEdit->setText(standardScanRangeStartMhz);
    }
    if (standardScanRangeEndEdit) {
        QSignalBlocker blocker(standardScanRangeEndEdit);
        standardScanRangeEndEdit->setText(standardScanRangeEndMhz);
    }
    if (scanMeasurementCheckbox) {
        QSignalBlocker blocker(scanMeasurementCheckbox);
        scanMeasurementCheckbox->setChecked(scanMeasurementEnabled);
    }
    if (scanMeasurementBinSpin) {
        QSignalBlocker blocker(scanMeasurementBinSpin);
        scanMeasurementBinSpin->setValue(scanMeasurementBinMhz);
    }
    if (dmrHunterControls) {
        dmrHunterControls->setDetectChecked(dmrHunterSettings.enabled);
        dmrHunterControls->setWidthValues(dmrHunterSettings.minWidthKhz,
                                          dmrHunterSettings.maxWidthKhz,
                                          dmrHunterSettings.thresholdDb);
    }
    if (fpvHunterControls) {
        fpvHunterControls->setDetectChecked(fpvHunterSettings.enabled);
        fpvHunterControls->setWidthValues(fpvHunterSettings.minWidthMhz,
                                          fpvHunterSettings.maxWidthMhz,
                                          fpvHunterSettings.thresholdDb);
    }
    if (digitalVideoHunterControls) {
        digitalVideoHunterControls->setDetectChecked(digitalVideoHunterSettings.enabled);
        digitalVideoHunterControls->setWidthValues(digitalVideoHunterSettings.minWidthMhz,
                                                   digitalVideoHunterSettings.maxWidthMhz,
                                                   digitalVideoHunterSettings.thresholdDb);
    }
    if (scanMeasurementBaselineButton) {
        QSignalBlocker blocker(scanMeasurementBaselineButton);
        scanMeasurementBaselineButton->setChecked(scanMeasurementBaselineRecording);
        scanMeasurementBaselineButton->setText(scanMeasurementBaselineRecording
                                                   ? uiText(QStringLiteral("stop_bg"), QStringLiteral("Stop BG"))
                                                   : uiText(QStringLiteral("bg_rec"), QStringLiteral("BG Rec")));
    }
    updateAgileScanControls();
    updateScanMeasurementStatus();
    updateDmrHunterControls();
    updateFpvHunterControls();
    updateDigitalVideoHunterControls();
    updateAudioFilterLabels();
    updateHfNoiseCancelControls();
    if (digitalDecodeCheckbox) {
        digitalDecodeCheckbox->blockSignals(true);
        digitalDecodeCheckbox->setChecked(digitalDecodeEnabled);
        digitalDecodeCheckbox->blockSignals(false);
    }
    if (dmrBasebandRateCombo) {
        QSignalBlocker blocker(dmrBasebandRateCombo);
        const int index = dmrBasebandRateCombo->findData(
            normalizedDmrBasebandSampleRate(pendingSettings.dmrBasebandSampleRate));
        if (index >= 0) {
            dmrBasebandRateCombo->setCurrentIndex(index);
        }
    }
    if (dmrAmbeLayoutCombo) {
        QSignalBlocker blocker(dmrAmbeLayoutCombo);
        const int index = dmrAmbeLayoutCombo->findData(
            normalizedDmrAmbeLayout(pendingSettings.dmrAmbeLayout));
        if (index >= 0) {
            dmrAmbeLayoutCombo->setCurrentIndex(index);
        }
    }
    if (dmrManualTimingCheckbox) {
        QSignalBlocker blocker(dmrManualTimingCheckbox);
        dmrManualTimingCheckbox->setChecked(pendingSettings.dmrManualTimingEnabled);
    }
    if (dmrTimingOffsetSpin) {
        QSignalBlocker blocker(dmrTimingOffsetSpin);
        dmrTimingOffsetSpin->setValue(pendingSettings.dmrManualTimingOffset);
        dmrTimingOffsetSpin->setEnabled(!dmrManualTimingCheckbox ||
                                        dmrManualTimingCheckbox->isChecked());
    }
    if (dmrSlicerRatioSpin) {
        QSignalBlocker blocker(dmrSlicerRatioSpin);
        dmrSlicerRatioSpin->setValue(pendingSettings.dmrSlicerRatio);
    }
    if (dmrAdaptiveSlicerCheckbox) {
        QSignalBlocker blocker(dmrAdaptiveSlicerCheckbox);
        dmrAdaptiveSlicerCheckbox->setChecked(pendingSettings.dmrAdaptiveSlicer);
    }
    if (videoDecodeCheckbox) {
        videoDecodeCheckbox->blockSignals(true);
        videoDecodeCheckbox->setChecked(videoDecodeEnabled);
        videoDecodeCheckbox->blockSignals(false);
    }
    const float volume = volumePercent / 100.0f;
    if (audioProcessor) {
        audioProcessor->setVolume(volume);
    }
    if (remoteAudioPlayer) {
        remoteAudioPlayer->setVolume(volume);
    }
    if (graphWidget) {
        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
    if (waterfallWidget) {
        waterfallWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
    applyUiLanguage();
    updateSpectrumTimerInterval();
    settingRange();
    updateDigitalDecoderMode();
    updateVideoProcessorMode();
}

void YourClassName::loadPersistentSettings() {
    const QString settingsPath = persistentSettingsFilePath();
    const bool settingsFileExists = QFile::exists(settingsPath);
    QSettings settings(settingsPath, QSettings::IniFormat);
    if (settings.status() != QSettings::NoError) {
        qDebug() << "[Settings] unable to read settings file" << settingsPath
                 << "status" << settings.status();
        return;
    }

    pendingSettings.deviceIndex = (std::max)(0, settings.value("receiver/deviceIndex", pendingSettings.deviceIndex).toInt());
    pendingSettings.clockSource = settings.value("receiver/clockSource", pendingSettings.clockSource).toInt();
    pendingSettings.inputMode = (std::clamp)(settings.value("receiver/inputMode", pendingSettings.inputMode).toInt(),
                                             static_cast<int>(INPUT_RF),
                                             static_cast<int>(INPUT_HF_NOISE_CANCEL));
    pendingSettings.centerFrequency = settings.value("receiver/centerFrequency", pendingSettings.centerFrequency).toDouble();
    pendingSettings.actualFrequency = settings.value("receiver/actualFrequency", pendingSettings.actualFrequency).toDouble();
    pendingSettings.listeningFrequency = settings.value("receiver/listeningFrequency", pendingSettings.listeningFrequency).toDouble();
    pendingSettings.sampleRate = (std::max)(1000.0, settings.value("receiver/sampleRate", pendingSettings.sampleRate).toDouble());
    pendingSettings.bandwidth = (std::max)(1.0, settings.value("receiver/bandwidth", pendingSettings.bandwidth).toDouble());
    pendingSettings.modulationType = settings.value("receiver/modulationType", pendingSettings.modulationType).toInt();
    pendingSettings.fftLength = (std::max)(1024, settings.value("receiver/fftLength", pendingSettings.fftLength).toInt());
    pendingSettings.lnaGain = (std::clamp)(settings.value("receiver/lnaGain", pendingSettings.lnaGain).toInt(), 0, 1);
    pendingSettings.vgaGain = (std::clamp)(settings.value("receiver/vgaGain", pendingSettings.vgaGain).toInt(), 0, 31);
    pendingSettings.audioDeviceId = (std::max)(0, settings.value("receiver/audioDeviceId", pendingSettings.audioDeviceId).toInt());
    pendingSettings.audioLowPassHz = clampAudioLowPassHz(settings.value("audio/lowPassHz", pendingSettings.audioLowPassHz).toDouble());
    pendingSettings.audioHighPassHz = clampAudioHighPassHz(settings.value("audio/highPassHz", pendingSettings.audioHighPassHz).toDouble());
    pendingSettings.hfNoiseCancelDepth = clampHfNoiseCancelDepth(settings.value("hfNoiseCancel/depth", pendingSettings.hfNoiseCancelDepth).toDouble());
    pendingSettings.hfNoiseCancelRefGainDb =
        clampHfNoiseCancelRefGainDb(settings.value("hfNoiseCancel/refGainDb", pendingSettings.hfNoiseCancelRefGainDb).toDouble());
    pendingSettings.hfNoiseCancelRefDelayNs =
        clampHfNoiseCancelRefDelayNs(settings.value("hfNoiseCancel/refDelayNs", pendingSettings.hfNoiseCancelRefDelayNs).toDouble());
    pendingSettings.hfNoiseCancelRefTiltDb =
        clampHfNoiseCancelRefTiltDb(settings.value("hfNoiseCancel/refTiltDb", pendingSettings.hfNoiseCancelRefTiltDb).toDouble());
    pendingSettings.hfNoiseCancelFreeze = settings.value("hfNoiseCancel/freeze", pendingSettings.hfNoiseCancelFreeze).toBool();
    pendingSettings.audioEnabled = settings.value("receiver/audioEnabled", pendingSettings.audioEnabled).toBool();
    pendingSettings.syncEnabled = false;
    pendingSettings.gpoValue = static_cast<std::uint8_t>((std::clamp)(settings.value("receiver/gpoValue", static_cast<int>(pendingSettings.gpoValue)).toInt(), 0, 255));
    agileScanEnabled = settings.value("agileScan/enabled", agileScanEnabled).toBool();
    agileScanRangesMhz = settings.value("agileScan/rangesMhz", agileScanRangesMhz).toString().trimmed();
    agileScanStepMhz = (std::clamp)(settings.value("agileScan/stepMhz", agileScanStepMhz).toDouble(),
                                    AGILE_SCAN_MIN_STEP_MHZ,
                                    AGILE_SCAN_MAX_STEP_MHZ);
    standardScanEnabled = settings.value("standardScan/enabled", standardScanEnabled).toBool();
    scanListeningLockEnabled = settings.value("standardScan/listenLock", scanListeningLockEnabled).toBool();
    standardScanCentersMhz = settings.value("standardScan/centersMhz", standardScanCentersMhz).toString().trimmed();
    standardScanDwellMs = (std::clamp)(settings.value("standardScan/dwellMs", standardScanDwellMs).toInt(),
                                       STANDARD_SCAN_MIN_DWELL_MS,
                                       STANDARD_SCAN_MAX_DWELL_MS);
    standardScanSettleMs = (std::clamp)(settings.value("standardScan/settleMs", standardScanSettleMs).toInt(),
                                        STANDARD_SCAN_MIN_SETTLE_MS,
                                        STANDARD_SCAN_MAX_SETTLE_MS);
    standardScanRangeStartMhz = settings.value("standardScan/rangeStartMhz", standardScanRangeStartMhz).toString().trimmed();
    standardScanRangeEndMhz = settings.value("standardScan/rangeEndMhz", standardScanRangeEndMhz).toString().trimmed();
    {
        bool adjusted = false;
        QString standardScanError;
        const QVector<double> normalized =
            parseStandardScanCentersMhz(standardScanCentersMhz,
                                        pendingSettings.sampleRate,
                                        0,
                                        &standardScanError,
                                        &adjusted);
        if (adjusted && standardScanError.isEmpty() && !normalized.isEmpty()) {
            standardScanCentersMhz = formatMhzList(normalized);
        }
    }
    scanMeasurementEnabled = settings.value("agileScan/measurementEnabled", scanMeasurementEnabled).toBool();
    scanMeasurementBinMhz = (std::clamp)(settings.value("agileScan/measurementBinMhz", scanMeasurementBinMhz).toDouble(),
                                         SCAN_MEASUREMENT_MIN_BIN_MHZ,
                                         SCAN_MEASUREMENT_MAX_BIN_MHZ);
    dmrHunterSettings.enabled = settings.value("dmrHunter/enabled", dmrHunterSettings.enabled).toBool();
    dmrHunterSettings.minWidthKhz =
        settings.value("dmrHunter/minWidthKhz", dmrHunterSettings.minWidthKhz).toDouble();
    dmrHunterSettings.maxWidthKhz =
        settings.value("dmrHunter/maxWidthKhz", dmrHunterSettings.maxWidthKhz).toDouble();
    dmrHunterSettings.thresholdDb =
        settings.value("dmrHunter/thresholdDb", dmrHunterSettings.thresholdDb).toDouble();
    dmrHunterSettings = DmrHunterDetector::normalizedSettings(dmrHunterSettings);
    fpvHunterSettings.enabled = settings.value("fpvHunter/enabled", fpvHunterSettings.enabled).toBool();
    fpvHunterSettings.minWidthMhz =
        settings.value("fpvHunter/minWidthMhz", fpvHunterSettings.minWidthMhz).toDouble();
    fpvHunterSettings.maxWidthMhz =
        settings.value("fpvHunter/maxWidthMhz", fpvHunterSettings.maxWidthMhz).toDouble();
    fpvHunterSettings.thresholdDb =
        settings.value("fpvHunter/thresholdDb", fpvHunterSettings.thresholdDb).toDouble();
    fpvHunterSettings = FpvHunterDetector::normalizedSettings(fpvHunterSettings);
    fpvHunterFollowEnabled = settings.value("fpvHunter/followEnabled", fpvHunterFollowEnabled).toBool();
    digitalVideoHunterSettings.enabled =
        settings.value("digitalVideoHunter/enabled", digitalVideoHunterSettings.enabled).toBool();
    digitalVideoHunterSettings.minWidthMhz =
        settings.value("digitalVideoHunter/minWidthMhz", digitalVideoHunterSettings.minWidthMhz).toDouble();
    digitalVideoHunterSettings.maxWidthMhz =
        settings.value("digitalVideoHunter/maxWidthMhz", digitalVideoHunterSettings.maxWidthMhz).toDouble();
    digitalVideoHunterSettings.thresholdDb =
        settings.value("digitalVideoHunter/thresholdDb", digitalVideoHunterSettings.thresholdDb).toDouble();
    digitalVideoHunterSettings = DigitalVideoHunterDetector::normalizedSettings(digitalVideoHunterSettings);
    agileScanPresets.clear();
    const int scanPresetCount = settings.beginReadArray("agileScan/presets");
    for (int i = 0; i < scanPresetCount; ++i) {
        settings.setArrayIndex(i);
        const QString name = settings.value("name").toString().trimmed();
        const QString ranges = settings.value("rangesMhz").toString().trimmed();
        const double step = (std::clamp)(settings.value("stepMhz", agileScanStepMhz).toDouble(),
                                         AGILE_SCAN_MIN_STEP_MHZ,
                                         AGILE_SCAN_MAX_STEP_MHZ);
        if (!name.isEmpty() && !ranges.isEmpty()) {
            agileScanPresets[name] = agileScanPresetSpec(ranges, step);
        }
    }
    settings.endArray();
    if (agileScanPresets.isEmpty()) {
        agileScanPresets[QStringLiteral("Narrow DMR example")] =
            agileScanPresetSpec(QStringLiteral("430-432"), 0.0125);
        agileScanPresets[QStringLiteral("VHF DMR 160-174 coarse")] =
            agileScanPresetSpec(QStringLiteral("160-174"), 0.1);
        agileScanPresets[QStringLiteral("UHF DMR 400-470 coarse")] =
            agileScanPresetSpec(QStringLiteral("400-470"), 0.5);
        agileScanPresets[QStringLiteral("FPV 1.2/2.4 sparse")] =
            agileScanPresetSpec(QStringLiteral("1080-1360\\2300-2500"), 5.0);
    }
    if (!agileScanPresets.contains(QStringLiteral("REB broad check 300/600/5800"))) {
        agileScanPresets[QStringLiteral("REB broad check 300/600/5800")] =
            agileScanPresetSpec(QStringLiteral("300-400\\600-1200\\5650-5950"), 5.0);
    }
    if (!agileScanPresets.contains(QStringLiteral("REB 300-400 1MHz"))) {
        agileScanPresets[QStringLiteral("REB 300-400 1MHz")] =
            agileScanPresetSpec(QStringLiteral("300-400"), 1.0);
    }
    if (!agileScanPresets.contains(QStringLiteral("REB 600-1200 5MHz"))) {
        agileScanPresets[QStringLiteral("REB 600-1200 5MHz")] =
            agileScanPresetSpec(QStringLiteral("600-1200"), 5.0);
    }
    if (!agileScanPresets.contains(QStringLiteral("REB 5.8GHz 5MHz"))) {
        agileScanPresets[QStringLiteral("REB 5.8GHz 5MHz")] =
            agileScanPresetSpec(QStringLiteral("5650-5950"), 5.0);
    }
    if (!agileScanPresets.contains(QStringLiteral("Digital video sparse"))) {
        agileScanPresets[QStringLiteral("Digital video sparse")] =
            agileScanPresetSpec(QStringLiteral("1080-1360\\2300-2500\\3200-3500\\4900-5925"), 5.0);
    }

    standardScanPresets.clear();
    const int standardScanPresetCount = settings.beginReadArray("standardScan/presets");
    for (int i = 0; i < standardScanPresetCount; ++i) {
        settings.setArrayIndex(i);
        const QString name = settings.value("name").toString().trimmed();
        const QString centers = settings.value("centersMhz").toString().trimmed();
        const int dwellMs = (std::clamp)(settings.value("dwellMs", standardScanDwellMs).toInt(),
                                         STANDARD_SCAN_MIN_DWELL_MS,
                                         STANDARD_SCAN_MAX_DWELL_MS);
        const int settleMs = (std::clamp)(settings.value("settleMs", standardScanSettleMs).toInt(),
                                          STANDARD_SCAN_MIN_SETTLE_MS,
                                          STANDARD_SCAN_MAX_SETTLE_MS);
        QString parseError;
        parseStandardScanCentersMhz(centers, pendingSettings.sampleRate, AGILE_SCAN_MIN_POINTS, &parseError, nullptr);
        if (!name.isEmpty() && parseError.isEmpty()) {
            standardScanPresets[name] = standardScanPresetSpec(centers, dwellMs, settleMs);
        }
    }
    settings.endArray();
    if (standardScanPresets.isEmpty()) {
        standardScanPresets[QStringLiteral("RF 100-300 by 50MHz")] =
            standardScanPresetSpec(QStringLiteral("100, 150, 200, 250, 300"), 120, 40);
        standardScanPresets[QStringLiteral("UHF broad 400-700 by 50MHz")] =
            standardScanPresetSpec(QStringLiteral("400, 450, 500, 550, 600, 650, 700"), 120, 40);
    }

    auto readFrequencyPresetArray = [&settings](const char *path, QMap<QString, double> &target) {
        target.clear();
        const int count = settings.beginReadArray(QString::fromLatin1(path));
        for (int i = 0; i < count; ++i) {
            settings.setArrayIndex(i);
            const QString name = settings.value("name").toString().trimmed();
            bool ok = false;
            const double value = settings.value("valueHz").toDouble(&ok);
            if (!name.isEmpty() && ok && std::isfinite(value)) {
                target[name] = value;
            }
        }
        settings.endArray();
    };
    readFrequencyPresetArray("frequencyPresets/center", centerFrequencyPresets);
    readFrequencyPresetArray("frequencyPresets/listening", listeningFrequencyPresets);
    readFrequencyPresetArray("frequencyPresets/bandwidth", bandwidthValuePresets);
    ensureDefaultFrequencyPresets();
    centerFrequencyUnitIndex = (std::clamp)(settings.value("frequencyControls/centerUnitIndex", centerFrequencyUnitIndex).toInt(), 0, 3);
    listeningFrequencyUnitIndex = (std::clamp)(settings.value("frequencyControls/listeningUnitIndex", listeningFrequencyUnitIndex).toInt(), 0, 3);
    bandwidthUnitIndex = (std::clamp)(settings.value("frequencyControls/bandwidthUnitIndex", bandwidthUnitIndex).toInt(), 0, 3);
    centerFrequencyStepName = settings.value("frequencyControls/centerStepName", centerFrequencyStepName).toString();
    listeningFrequencyStepName = settings.value("frequencyControls/listeningStepName", listeningFrequencyStepName).toString();
    bandwidthStepName = settings.value("frequencyControls/bandwidthStepName", bandwidthStepName).toString();
    centerFrequencyPresetName = settings.value("frequencyControls/centerPresetName", centerFrequencyPresetName).toString();
    listeningFrequencyPresetName = settings.value("frequencyControls/listeningPresetName", listeningFrequencyPresetName).toString();
    bandwidthPresetName = settings.value("frequencyControls/bandwidthPresetName", bandwidthPresetName).toString();
    frequencyControlUiStateRestorePending = true;

    bandMarkers.clear();
    bandMarkersCustomized = settings.contains(QStringLiteral("bandMarkers/size"));
    const int bandMarkerCount = settings.beginReadArray(QStringLiteral("bandMarkers"));
    for (int i = 0; i < bandMarkerCount; ++i) {
        settings.setArrayIndex(i);
        GraphBandMarker marker;
        marker.startHz = settings.value(QStringLiteral("startHz")).toDouble();
        marker.endHz = settings.value(QStringLiteral("endHz")).toDouble();
        marker.label = settings.value(QStringLiteral("label")).toString().trimmed();
        marker.amateur = settings.value(QStringLiteral("amateur")).toBool();
        if (!marker.label.isEmpty() &&
            std::isfinite(marker.startHz) &&
            std::isfinite(marker.endHz) &&
            marker.endHz > marker.startHz) {
            bandMarkers.append(marker);
        }
    }
    settings.endArray();
    if (bandMarkers.isEmpty()) {
        bandMarkersCustomized = false;
    }
    if (!bandMarkersCustomized) {
        ensureDefaultBandMarkers();
    }

    currentScale = (std::clamp)(settings.value("display/scalePercent", currentScale).toDouble(),
                                MIN_SCALE_PERCENT,
                                MAX_SCALE_PERCENT);
    contrast = static_cast<float>((std::clamp)(settings.value("display/contrast", static_cast<double>(contrast)).toDouble(), 1.0, 15.0));
    sensitivity = static_cast<float>((std::clamp)(settings.value("display/sensitivity", static_cast<double>(sensitivity)).toDouble(), 1.0, 30.0));
    displayLevelMin = static_cast<float>((std::clamp)(settings.value("display/levelMin", static_cast<double>(displayLevelMin)).toDouble(), -160.0, 20.0));
    displayLevelMax = static_cast<float>((std::clamp)(settings.value("display/levelMax", static_cast<double>(displayLevelMax)).toDouble(), -160.0, 20.0));
    if (displayLevelMin >= displayLevelMax) {
        displayLevelMin = (std::max)(-160.0f, displayLevelMax - MIN_LEVEL_GAP);
    }
    secondGraph = settings.value("display/secondGraph", secondGraph).toBool();
    colorf = settings.value("display/color", colorf).toBool();
    showGeneralBandMarkers = settings.value("display/generalBandMarkers", showGeneralBandMarkers).toBool();
    showAmateurBandMarkers = settings.value("display/amateurBandMarkers", showAmateurBandMarkers).toBool();
    compactBandMarkers = settings.value("display/compactBandMarkers", compactBandMarkers).toBool();
    spurSuppressionEnabled = settings.value("display/spurSuppressionEnabled", spurSuppressionEnabled).toBool();
    diagnosticVerboseLogging =
        settings.value("diagnostics/verboseLogging",
                       fobosVerboseLoggingDefaultEnabled()).toBool();
    setFobosVerboseLoggingEnabled(diagnosticVerboseLogging);
    qDebug() << "[Log] Verbose diagnostic logging"
             << (diagnosticVerboseLogging ? "enabled" : "disabled")
             << "source" << (settings.contains("diagnostics/verboseLogging") ? "settings" : "default");
    spurMaskEntries.clear();
    const int spurMaskCount = settings.beginReadArray(QStringLiteral("display/spurMask"));
    for (int i = 0; i < spurMaskCount; ++i) {
        settings.setArrayIndex(i);
        SpurMaskEntry entry;
        entry.offsetHz = settings.value(QStringLiteral("offsetHz")).toDouble();
        entry.widthHz = settings.value(QStringLiteral("widthHz"), SPUR_MIN_MASK_WIDTH_HZ).toDouble();
        entry.prominenceDb = static_cast<float>(settings.value(QStringLiteral("prominenceDb"), 0.0).toDouble());
        entry.hits = settings.value(QStringLiteral("hits"), 0).toInt();
        if (std::isfinite(entry.offsetHz) &&
            std::isfinite(entry.widthHz) &&
            entry.widthHz > 0.0) {
            spurMaskEntries.append(entry);
        }
    }
    settings.endArray();
    if (spurSuppressionCheckbox) {
        QSignalBlocker blocker(spurSuppressionCheckbox);
        spurSuppressionCheckbox->setChecked(spurSuppressionEnabled);
    }
    updateSpurSuppressionStatus();
    volumePercent = (std::clamp)(settings.value("audio/volumePercent", volumePercent).toInt(), 0, 200);
    uiLanguage = settings.value("ui/language", uiLanguage).toString() == QStringLiteral("uk")
                     ? QStringLiteral("uk")
                     : QStringLiteral("en");
    fineTuneControlMode = (std::clamp)(settings.value("ui/fineTuneControlMode", fineTuneControlMode).toInt(),
                                       FINE_TUNE_MODE_SCALE,
                                       FINE_TUNE_MODE_DIAL);
    spectrumUpdateIntervalMs =
        (std::clamp)(settings.value("ui/spectrumUpdateIntervalMs", spectrumUpdateIntervalMs).toInt(),
                     SPECTRUM_UPDATE_AUTO_MS,
                     SPECTRUM_UPDATE_MAX_MS);
    if (spectrumUpdateIntervalMs > 0 && spectrumUpdateIntervalMs < SPECTRUM_UPDATE_MIN_MS) {
        spectrumUpdateIntervalMs = SPECTRUM_UPDATE_MIN_MS;
    }
    fineTuneScaleHoldMode = settings.value("ui/fineTuneScaleHoldMode", fineTuneScaleHoldMode).toBool();

    networkMode = NetworkMode::Disabled;
    const int processingModeValue = (std::clamp)(settings.value("network/processingMode", static_cast<int>(networkProcessingMode)).toInt(),
                                                 static_cast<int>(NetworkProcessingMode::ServerSide),
                                                 static_cast<int>(NetworkProcessingMode::FullIqClientSide));
    networkProcessingMode = static_cast<NetworkProcessingMode>(processingModeValue);
    networkServerAddress = settings.value("network/serverAddress", networkServerAddress).toString();
    networkBindAddress = settings.value("network/bindAddress", networkBindAddress).toString();
    networkControlPort = static_cast<quint16>((std::clamp)(settings.value("network/controlPort", static_cast<int>(networkControlPort)).toInt(), 1, 65535));
    serverDisableLocalVisualAudio = settings.value("network/serverDisableLocalVisualAudio", serverDisableLocalVisualAudio).toBool();
    networkFullResolutionSpectrumFrames =
        settings.value("network/fullResolutionSpectrumFrames", networkFullResolutionSpectrumFrames).toBool();
    audioRelayTransmitEnabled = settings.value("audioRelay/transmitEnabled", audioRelayTransmitEnabled).toBool();
    audioRelayHost = settings.value("audioRelay/host", audioRelayHost).toString();
    audioRelayPort = static_cast<quint16>((std::clamp)(settings.value("audioRelay/port", static_cast<int>(audioRelayPort)).toInt(), 1, 65535));
    audioRelayReceiveEnabled = settings.value("audioRelay/receiveEnabled", audioRelayReceiveEnabled).toBool();
    audioRelayListenPort = static_cast<quint16>((std::clamp)(settings.value("audioRelay/listenPort", static_cast<int>(audioRelayListenPort)).toInt(), 1, 65535));
    audioHttpStreamEnabled = settings.value("audioHttpStream/enabled", audioHttpStreamEnabled).toBool();
    audioHttpStreamPort = static_cast<quint16>((std::clamp)(settings.value("audioHttpStream/port", static_cast<int>(audioHttpStreamPort)).toInt(), 1, 65535));
    digitalDecodeEnabled = settings.value("digital/decodeEnabled", digitalDecodeEnabled).toBool();
    auto setComboToData = [](QComboBox *combo, const QVariant &data) {
        if (!combo) {
            return;
        }
        const int index = combo->findData(data);
        if (index >= 0) {
            combo->setCurrentIndex(index);
        }
    };
    if (dmrLabCaptureCheckbox) {
        dmrLabCaptureCheckbox->setChecked(
            settings.value("digital/dmrLockEnabled",
                           settings.value("digital/dmrLabCaptureEnabled", false)).toBool());
    }
    setComboToData(dmrLabColorCodeCombo, settings.value("digital/dmrLabColorCode", -1));
    setComboToData(dmrLabSlotCombo, settings.value("digital/dmrLabTimeslot", 0));
    setComboToData(dmrLabCallTypeCombo, settings.value("digital/dmrLabCallType", QStringLiteral("unknown")));
    pendingSettings.dmrBasebandSampleRate =
        normalizedDmrBasebandSampleRate(settings.value("digital/dmrBasebandSampleRate",
                                                       pendingSettings.dmrBasebandSampleRate).toInt());
    setComboToData(dmrBasebandRateCombo, pendingSettings.dmrBasebandSampleRate);
    pendingSettings.dmrAmbeLayout =
        normalizedDmrAmbeLayout(settings.value("digital/dmrAmbeLayout",
                                               pendingSettings.dmrAmbeLayout).toInt());
    setComboToData(dmrAmbeLayoutCombo, pendingSettings.dmrAmbeLayout);
    pendingSettings.dmrManualTimingEnabled =
        settings.value("digital/dmrManualTimingEnabled",
                       pendingSettings.dmrManualTimingEnabled).toBool();
    pendingSettings.dmrManualTimingOffset =
        (std::clamp)(settings.value("digital/dmrManualTimingOffset",
                                    pendingSettings.dmrManualTimingOffset).toInt(),
                     -80,
                     80);
    pendingSettings.dmrSlicerRatio =
        (std::clamp)(settings.value("digital/dmrSlicerRatio",
                                    pendingSettings.dmrSlicerRatio).toDouble(),
                     0.45,
                     0.80);
    pendingSettings.dmrAdaptiveSlicer =
        settings.value("digital/dmrAdaptiveSlicer",
                       pendingSettings.dmrAdaptiveSlicer).toBool();
    if (dmrManualTimingCheckbox) {
        dmrManualTimingCheckbox->setChecked(pendingSettings.dmrManualTimingEnabled);
    }
    if (dmrTimingOffsetSpin) {
        dmrTimingOffsetSpin->setValue(pendingSettings.dmrManualTimingOffset);
    }
    if (dmrSlicerRatioSpin) {
        dmrSlicerRatioSpin->setValue(pendingSettings.dmrSlicerRatio);
    }
    if (dmrAdaptiveSlicerCheckbox) {
        dmrAdaptiveSlicerCheckbox->setChecked(pendingSettings.dmrAdaptiveSlicer);
    }
    if (dmrLabSourceIdEdit) {
        dmrLabSourceIdEdit->setText(settings.value("digital/dmrLabSourceId").toString());
    }
    if (dmrLabTargetIdEdit) {
        dmrLabTargetIdEdit->setText(settings.value("digital/dmrLabTargetId").toString());
    }
    if (dmrLabRadioEdit) {
        dmrLabRadioEdit->setText(settings.value("digital/dmrLabRadio").toString());
    }
    if (dmrLabNotesEdit) {
        dmrLabNotesEdit->setText(settings.value("digital/dmrLabNotes").toString());
    }
    videoDecodeEnabled = settings.value("video/decodeEnabled", videoDecodeEnabled).toBool();
    if (videoDemodCombo) {
        const int demodMode = settings.value("video/demodMode", VideoProcessor::FmVideo).toInt();
        const int demodIndex = videoDemodCombo->findData(demodMode);
        if (demodIndex >= 0) {
            videoDemodCombo->setCurrentIndex(demodIndex);
        }
    }
    if (videoStandardCombo) {
        const int standardIndex = (std::clamp)(settings.value("video/standardIndex", 0).toInt(),
                                               0,
                                               (std::max)(0, videoStandardCombo->count() - 1));
        videoStandardCombo->setCurrentIndex(standardIndex);
    }
    if (videoInvertCheckbox) {
        videoInvertCheckbox->setChecked(settings.value("video/invert", false).toBool());
    }
    if (videoHSyncCheckbox) {
        videoHSyncCheckbox->setChecked(settings.value("video/hSync", true).toBool());
    }
    if (videoVSyncCheckbox) {
        videoVSyncCheckbox->setChecked(settings.value("video/vSync", true).toBool());
    }

    normalizeTuning(pendingSettings);
    const bool dmrCenterRealigned = realignDmrCenterToListening(pendingSettings);
    updateFrequencyPresetControls();
    if (dmrCenterRealigned) {
        qDebug() << "[Settings] DMR center realigned to listening frequency while loading"
                 << "center" << pendingSettings.centerFrequency
                 << "listening" << pendingSettings.listeningFrequency;
    }
    qDebug() << (settingsFileExists ? "[Settings] loaded" : "[Settings] using defaults; settings file will be created on clean exit")
             << settingsPath
             << "sampleRate" << pendingSettings.sampleRate
             << "center" << pendingSettings.centerFrequency
             << "listening" << pendingSettings.listeningFrequency;
}

void YourClassName::savePersistentSettings() {
    if (!persistentSettingsReady) {
        return;
    }

    QSettings settings(persistentSettingsFilePath(), QSettings::IniFormat);
    RadioSettings settingsToSave = pendingSettings;
    const bool savedDmrCenterRealigned = realignDmrCenterToListening(settingsToSave);

    settings.setValue("receiver/deviceIndex", settingsToSave.deviceIndex);
    settings.setValue("receiver/clockSource", settingsToSave.clockSource);
    settings.setValue("receiver/inputMode", settingsToSave.inputMode);
    settings.setValue("receiver/centerFrequency", settingsToSave.centerFrequency);
    settings.setValue("receiver/actualFrequency", settingsToSave.actualFrequency);
    settings.setValue("receiver/listeningFrequency", settingsToSave.listeningFrequency);
    settings.setValue("receiver/sampleRate", settingsToSave.sampleRate);
    settings.setValue("receiver/bandwidth", settingsToSave.bandwidth);
    settings.setValue("receiver/modulationType", settingsToSave.modulationType);
    settings.setValue("receiver/fftLength", settingsToSave.fftLength);
    settings.setValue("receiver/lnaGain", settingsToSave.lnaGain);
    settings.setValue("receiver/vgaGain", settingsToSave.vgaGain);
    settings.setValue("receiver/audioDeviceId", settingsToSave.audioDeviceId);
    settings.setValue("receiver/audioEnabled", settingsToSave.audioEnabled);
    settings.setValue("receiver/gpoValue", static_cast<int>(settingsToSave.gpoValue));
    if (savedDmrCenterRealigned) {
        qDebug() << "[Settings] DMR center realigned to listening frequency while saving"
                 << "center" << settingsToSave.centerFrequency
                 << "listening" << settingsToSave.listeningFrequency;
    }
    settings.setValue("agileScan/enabled", agileScanEnabled);
    settings.setValue("agileScan/rangesMhz", agileScanRangesMhz);
    settings.setValue("agileScan/stepMhz", agileScanStepMhz);
    settings.setValue("standardScan/enabled", standardScanEnabled);
    settings.setValue("standardScan/listenLock", scanListeningLockEnabled);
    settings.setValue("standardScan/centersMhz", standardScanCentersMhz);
    settings.setValue("standardScan/dwellMs", standardScanDwellMs);
    settings.setValue("standardScan/settleMs", standardScanSettleMs);
    settings.setValue("standardScan/rangeStartMhz", standardScanRangeStartMhz);
    settings.setValue("standardScan/rangeEndMhz", standardScanRangeEndMhz);
    settings.setValue("agileScan/measurementEnabled", scanMeasurementEnabled);
    settings.setValue("agileScan/measurementBinMhz", scanMeasurementBinMhz);
    settings.setValue("dmrHunter/enabled", dmrHunterSettings.enabled);
    settings.setValue("dmrHunter/minWidthKhz", dmrHunterSettings.minWidthKhz);
    settings.setValue("dmrHunter/maxWidthKhz", dmrHunterSettings.maxWidthKhz);
    settings.setValue("dmrHunter/thresholdDb", dmrHunterSettings.thresholdDb);
    settings.setValue("fpvHunter/enabled", fpvHunterSettings.enabled);
    settings.setValue("fpvHunter/minWidthMhz", fpvHunterSettings.minWidthMhz);
    settings.setValue("fpvHunter/maxWidthMhz", fpvHunterSettings.maxWidthMhz);
    settings.setValue("fpvHunter/thresholdDb", fpvHunterSettings.thresholdDb);
    settings.setValue("fpvHunter/followEnabled", fpvHunterFollowEnabled);
    settings.setValue("digitalVideoHunter/enabled", digitalVideoHunterSettings.enabled);
    settings.setValue("digitalVideoHunter/minWidthMhz", digitalVideoHunterSettings.minWidthMhz);
    settings.setValue("digitalVideoHunter/maxWidthMhz", digitalVideoHunterSettings.maxWidthMhz);
    settings.setValue("digitalVideoHunter/thresholdDb", digitalVideoHunterSettings.thresholdDb);
    settings.beginWriteArray("agileScan/presets");
    int scanPresetIndex = 0;
    for (auto it = agileScanPresets.constBegin(); it != agileScanPresets.constEnd(); ++it) {
        settings.setArrayIndex(scanPresetIndex++);
        settings.setValue("name", it.key());
        settings.setValue("rangesMhz", agileScanPresetRanges(it.value()));
        settings.setValue("stepMhz", agileScanPresetStepMhz(it.value(), agileScanStepMhz));
    }
    settings.endArray();
    settings.beginWriteArray("standardScan/presets");
    int standardScanPresetIndex = 0;
    for (auto it = standardScanPresets.constBegin(); it != standardScanPresets.constEnd(); ++it) {
        settings.setArrayIndex(standardScanPresetIndex++);
        settings.setValue("name", it.key());
        settings.setValue("centersMhz", standardScanPresetCenters(it.value()));
        settings.setValue("dwellMs", standardScanPresetDwellMs(it.value(), standardScanDwellMs));
        settings.setValue("settleMs", standardScanPresetSettleMs(it.value(), standardScanSettleMs));
    }
    settings.endArray();

    auto writeFrequencyPresetArray = [&settings](const char *path, const QMap<QString, double> &presets) {
        settings.beginWriteArray(QString::fromLatin1(path));
        int index = 0;
        for (auto it = presets.constBegin(); it != presets.constEnd(); ++it) {
            if (it.key().trimmed().isEmpty() || !std::isfinite(it.value())) {
                continue;
            }
            settings.setArrayIndex(index++);
            settings.setValue("name", it.key());
            settings.setValue("valueHz", it.value());
        }
        settings.endArray();
    };
    writeFrequencyPresetArray("frequencyPresets/center", centerFrequencyPresets);
    writeFrequencyPresetArray("frequencyPresets/listening", listeningFrequencyPresets);
    writeFrequencyPresetArray("frequencyPresets/bandwidth", bandwidthValuePresets);
    if (frequencyControl) {
        settings.setValue("frequencyControls/centerUnitIndex", frequencyControl->selectedUnitIndex());
        settings.setValue("frequencyControls/centerStepName", frequencyControl->selectedStepName());
        settings.setValue("frequencyControls/centerPresetName", frequencyControl->selectedValuePresetName());
    }
    if (listeningFrequencyControl) {
        settings.setValue("frequencyControls/listeningUnitIndex", listeningFrequencyControl->selectedUnitIndex());
        settings.setValue("frequencyControls/listeningStepName", listeningFrequencyControl->selectedStepName());
        settings.setValue("frequencyControls/listeningPresetName", listeningFrequencyControl->selectedValuePresetName());
    }
    if (bandwidthControl) {
        settings.setValue("frequencyControls/bandwidthUnitIndex", bandwidthControl->selectedUnitIndex());
        settings.setValue("frequencyControls/bandwidthStepName", bandwidthControl->selectedStepName());
        settings.setValue("frequencyControls/bandwidthPresetName", bandwidthControl->selectedValuePresetName());
    }

    settings.beginWriteArray(QStringLiteral("bandMarkers"));
    int bandMarkerIndex = 0;
    for (const GraphBandMarker &marker : std::as_const(bandMarkers)) {
        if (marker.label.trimmed().isEmpty() ||
            !std::isfinite(marker.startHz) ||
            !std::isfinite(marker.endHz) ||
            marker.endHz <= marker.startHz) {
            continue;
        }
        settings.setArrayIndex(bandMarkerIndex++);
        settings.setValue(QStringLiteral("label"), marker.label.trimmed());
        settings.setValue(QStringLiteral("startHz"), marker.startHz);
        settings.setValue(QStringLiteral("endHz"), marker.endHz);
        settings.setValue(QStringLiteral("amateur"), marker.amateur);
    }
    settings.endArray();

    settings.setValue("display/scalePercent", currentScale);
    settings.setValue("display/contrast", contrast);
    settings.setValue("display/sensitivity", sensitivity);
    settings.setValue("display/levelMin", displayLevelMin);
    settings.setValue("display/levelMax", displayLevelMax);
    settings.setValue("display/secondGraph", secondGraph);
    settings.setValue("display/color", colorf);
    settings.setValue("display/generalBandMarkers", showGeneralBandMarkers);
    settings.setValue("display/amateurBandMarkers", showAmateurBandMarkers);
    settings.setValue("display/compactBandMarkers", compactBandMarkers);
    settings.setValue("display/spurSuppressionEnabled", spurSuppressionEnabled);
    settings.setValue("diagnostics/verboseLogging", diagnosticVerboseLogging);
    settings.beginWriteArray(QStringLiteral("display/spurMask"));
    int spurMaskIndex = 0;
    for (const SpurMaskEntry &entry : std::as_const(spurMaskEntries)) {
        if (!std::isfinite(entry.offsetHz) ||
            !std::isfinite(entry.widthHz) ||
            entry.widthHz <= 0.0) {
            continue;
        }
        settings.setArrayIndex(spurMaskIndex++);
        settings.setValue(QStringLiteral("offsetHz"), entry.offsetHz);
        settings.setValue(QStringLiteral("widthHz"), entry.widthHz);
        settings.setValue(QStringLiteral("prominenceDb"), entry.prominenceDb);
        settings.setValue(QStringLiteral("hits"), entry.hits);
    }
    settings.endArray();
    settings.setValue("audio/volumePercent", volumePercent);
    settings.setValue("ui/language", uiLanguage);
    settings.setValue("ui/fineTuneControlMode", fineTuneControlMode);
    settings.setValue("ui/spectrumUpdateIntervalMs", spectrumUpdateIntervalMs);
    settings.setValue("ui/fineTuneScaleHoldMode", fineTuneScaleHoldMode);
    settings.setValue("audio/lowPassHz", pendingSettings.audioLowPassHz);
    settings.setValue("audio/highPassHz", pendingSettings.audioHighPassHz);
    settings.setValue("hfNoiseCancel/depth", pendingSettings.hfNoiseCancelDepth);
    settings.setValue("hfNoiseCancel/refGainDb", pendingSettings.hfNoiseCancelRefGainDb);
    settings.setValue("hfNoiseCancel/refDelayNs", pendingSettings.hfNoiseCancelRefDelayNs);
    settings.setValue("hfNoiseCancel/refTiltDb", pendingSettings.hfNoiseCancelRefTiltDb);
    settings.setValue("hfNoiseCancel/freeze", pendingSettings.hfNoiseCancelFreeze);

    settings.setValue("network/serverAddress", networkServerAddress);
    settings.setValue("network/bindAddress", networkBindAddress);
    settings.setValue("network/controlPort", static_cast<int>(networkControlPort));
    settings.setValue("network/processingMode", static_cast<int>(networkProcessingMode));
    settings.setValue("network/serverDisableLocalVisualAudio", serverDisableLocalVisualAudio);
    settings.setValue("network/fullResolutionSpectrumFrames", networkFullResolutionSpectrumFrames);
    settings.setValue("audioRelay/transmitEnabled", audioRelayTransmitEnabled);
    settings.setValue("audioRelay/host", audioRelayHost);
    settings.setValue("audioRelay/port", static_cast<int>(audioRelayPort));
    settings.setValue("audioRelay/receiveEnabled", audioRelayReceiveEnabled);
    settings.setValue("audioRelay/listenPort", static_cast<int>(audioRelayListenPort));
    settings.setValue("audioHttpStream/enabled", audioHttpStreamEnabled);
    settings.setValue("audioHttpStream/port", static_cast<int>(audioHttpStreamPort));
    settings.setValue("digital/decodeEnabled", digitalDecodeEnabled);
    settings.setValue("digital/dmrLockEnabled", dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked());
    settings.setValue("digital/dmrLabColorCode", dmrLabColorCodeCombo ? dmrLabColorCodeCombo->currentData().toInt() : -1);
    settings.setValue("digital/dmrLabTimeslot", dmrLabSlotCombo ? dmrLabSlotCombo->currentData().toInt() : 0);
    settings.setValue("digital/dmrLabCallType", dmrLabCallTypeCombo ? dmrLabCallTypeCombo->currentData().toString() : QStringLiteral("unknown"));
    settings.setValue("digital/dmrBasebandSampleRate",
                      dmrBasebandRateCombo
                          ? normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt())
                          : normalizedDmrBasebandSampleRate(pendingSettings.dmrBasebandSampleRate));
    settings.setValue("digital/dmrAmbeLayout",
                      dmrAmbeLayoutCombo
                          ? normalizedDmrAmbeLayout(dmrAmbeLayoutCombo->currentData().toInt())
                          : normalizedDmrAmbeLayout(pendingSettings.dmrAmbeLayout));
    settings.setValue("digital/dmrManualTimingEnabled",
                      dmrManualTimingCheckbox && dmrManualTimingCheckbox->isChecked());
    settings.setValue("digital/dmrManualTimingOffset",
                      dmrTimingOffsetSpin ? dmrTimingOffsetSpin->value() : pendingSettings.dmrManualTimingOffset);
    settings.setValue("digital/dmrSlicerRatio",
                      dmrSlicerRatioSpin ? dmrSlicerRatioSpin->value() : pendingSettings.dmrSlicerRatio);
    settings.setValue("digital/dmrAdaptiveSlicer",
                      !dmrAdaptiveSlicerCheckbox || dmrAdaptiveSlicerCheckbox->isChecked());
    settings.setValue("digital/dmrLabSourceId", dmrLabSourceIdEdit ? dmrLabSourceIdEdit->text().trimmed() : QString());
    settings.setValue("digital/dmrLabTargetId", dmrLabTargetIdEdit ? dmrLabTargetIdEdit->text().trimmed() : QString());
    settings.setValue("digital/dmrLabRadio", dmrLabRadioEdit ? dmrLabRadioEdit->text().trimmed() : QString());
    settings.setValue("digital/dmrLabNotes", dmrLabNotesEdit ? dmrLabNotesEdit->text().trimmed() : QString());
    settings.setValue("video/decodeEnabled", videoDecodeEnabled);
    settings.setValue("video/demodMode", videoDemodCombo ? videoDemodCombo->currentData().toInt() : VideoProcessor::FmVideo);
    settings.setValue("video/standardIndex", videoStandardCombo ? videoStandardCombo->currentIndex() : 0);
    settings.setValue("video/invert", videoInvertCheckbox && videoInvertCheckbox->isChecked());
    settings.setValue("video/hSync", !videoHSyncCheckbox || videoHSyncCheckbox->isChecked());
    settings.setValue("video/vSync", !videoVSyncCheckbox || videoVSyncCheckbox->isChecked());
    settings.sync();

    if (settings.status() == QSettings::NoError) {
        qDebug() << "[Settings] saved" << persistentSettingsFilePath();
    } else {
        qDebug() << "[Settings] save failed" << persistentSettingsFilePath()
                 << "status" << settings.status();
    }
}

void YourClassName::applyLiveRemoteSettings(const RadioSettings &previousSettings) {
    if (!hasActiveFobosDevice() || isIdle()) {
        return;
    }

    applyCenterFrequencyToHardwareIfNeeded(previousSettings, "remote settings");

    if (hardwareSettingsApplied && previousSettings.lnaGain != pendingSettings.lnaGain) {
        const int result = setActiveLnaGainSafely(static_cast<unsigned int>(pendingSettings.lnaGain));
        qDebug() << "[Network] remote LNA apply result" << result;
        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.lnaGain = pendingSettings.lnaGain;
        }
    }
    if (hardwareSettingsApplied && previousSettings.vgaGain != pendingSettings.vgaGain) {
        const int result = setActiveVgaGainSafely(static_cast<unsigned int>(pendingSettings.vgaGain));
        qDebug() << "[Network] remote VGA apply result" << result;
        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.vgaGain = pendingSettings.vgaGain;
        }
    }
    if (hardwareSettingsApplied && previousSettings.gpoValue != pendingSettings.gpoValue) {
        const int result = setActiveGpoSafely(pendingSettings.gpoValue);
        qDebug() << "[Network] remote GPO apply result" << result;
        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.gpoValue = pendingSettings.gpoValue;
        }
    }

    if (std::abs(previousSettings.sampleRate - pendingSettings.sampleRate) > 0.5 ||
        previousSettings.inputMode != pendingSettings.inputMode ||
        previousSettings.clockSource != pendingSettings.clockSource) {
        qDebug() << "[Network] remote settings include restart-only changes; they will be applied on next server start";
    }
}

void YourClassName::onNetworkControlCommandReceived(const QJsonObject &command) {
    if (networkMode == NetworkMode::Client && command.value("type").toString() == "iq") {
        receiveNetworkIqFrame(command);
        return;
    }

    if (networkMode == NetworkMode::Client && !isChannelIqProcessingMode() && command.value("type").toString() == "audio") {
        playNetworkAudioFrame(command);
        return;
    }

    if (networkMode == NetworkMode::Client && !isFullIqProcessingMode() && command.value("type").toString() == "spectrum") {
        displayNetworkSpectrumFrame(command);
        return;
    }

    if (networkMode == NetworkMode::Client && command.value("type").toString() == "control") {
        const QString action = command.value("action").toString();
        if (action == QStringLiteral("role")) {
            const bool canControl = command.value("canControl").toBool(true);
            const QString peerLabel = command.value("peerLabel").toString();
            qDebug() << "[Network] client role update"
                     << "canControl" << canControl
                     << "peer" << peerLabel
                     << "controllerPeerId" << command.value("controllerPeerId").toString();
            updateUiForRunState();
            updateNetworkButtonText();
            return;
        }

        if (action == QStringLiteral("priorityRequest")) {
            if (!networkController || !networkController->clientHasControl()) {
                return;
            }

            const QString requesterId = command.value("requesterId").toString();
            const QString requesterLabel = command.value("requesterLabel").toString(QStringLiteral("unknown client"));
            QMessageBox box(this);
            box.setWindowTitle("Control Request");
            box.setText(QString("Client %1 requests control of the receiver.").arg(requesterLabel));
            box.setInformativeText("If you allow it, this client will become observer.");
            box.setIcon(QMessageBox::Question);
            QCheckBox *blockRequesterCheck = new QCheckBox("Block further requests from this client", &box);
            box.setCheckBox(blockRequesterCheck);
            QPushButton *allowButton = box.addButton("Allow", QMessageBox::AcceptRole);
            box.addButton("Deny", QMessageBox::RejectRole);
            box.exec();

            QJsonObject response;
            response["requesterId"] = requesterId;
            response["accepted"] = box.clickedButton() == allowButton;
            response["blocked"] = blockRequesterCheck && blockRequesterCheck->isChecked();
            sendRemoteControlCommand("priorityResponse", response);
            return;
        }

        if (action == QStringLiteral("priorityDenied") ||
            action == QStringLiteral("controlRejected")) {
            const QString reason = command.value("reason").toString("Request denied");
            qDebug() << "[Network]" << reason;
            QMessageBox::information(this, "Network Control", reason);
            return;
        }

        return;
    }

    if (networkMode != NetworkMode::Server || command.value("type").toString() != "control") {
        return;
    }

    const QString action = command.value("action").toString();
    const QJsonObject settingsJson = command.value("settings").toObject();
    const QString peerId = command.value("_networkPeerId").toString();
    const QString peerLabel = command.value("_networkPeerLabel").toString(QStringLiteral("unknown client"));
    const bool peerIsController = command.value("_networkPeerIsController").toBool(true);

    if (action == QStringLiteral("requestPriority")) {
        if (!networkController || peerId.isEmpty()) {
            return;
        }

        if (peerIsController) {
            QJsonObject role;
            role["type"] = "control";
            role["action"] = "role";
            role["canControl"] = true;
            networkController->sendControlCommandToPeer(peerId, role);
            return;
        }

        if (networkController->isPriorityRequestBlocked(peerId)) {
            QJsonObject denied;
            denied["type"] = "control";
            denied["action"] = "priorityDenied";
            denied["reason"] = "Control request is blocked by current controller";
            networkController->sendControlCommandToPeer(peerId, denied);
            qDebug() << "[Network] blocked control request from" << peerLabel;
            return;
        }

        if (networkController->controllerPeerId().isEmpty()) {
            networkController->setControllerPeer(peerId);
            return;
        }

        QJsonObject request;
        request["type"] = "control";
        request["action"] = "priorityRequest";
        request["requesterId"] = peerId;
        request["requesterLabel"] = peerLabel;
        if (!networkController->sendControlCommandToController(request)) {
            qDebug() << "[Network] controller unavailable; granting control to requester" << peerLabel;
            networkController->setControllerPeer(peerId);
        }
        return;
    }

    if (action == QStringLiteral("priorityResponse")) {
        if (!networkController || !peerIsController) {
            return;
        }

        const QString requesterId = command.value("requesterId").toString();
        const bool accepted = command.value("accepted").toBool(false);
        const bool blocked = command.value("blocked").toBool(false);
        if (blocked && !accepted) {
            networkController->blockPriorityRequestsFromPeer(requesterId);
        }

        if (accepted && networkController->setControllerPeer(requesterId)) {
            qDebug() << "[Network] control transferred to" << requesterId;
        } else {
            QJsonObject denied;
            denied["type"] = "control";
            denied["action"] = "priorityDenied";
            denied["reason"] = blocked
                                    ? "Control request denied and further requests were blocked"
                                    : "Control request denied";
            networkController->sendControlCommandToPeer(requesterId, denied);
        }
        return;
    }

    if (!peerIsController) {
        QJsonObject rejected;
        rejected["type"] = "control";
        rejected["action"] = "controlRejected";
        rejected["reason"] = "This client is observer. Request control in Network Settings first.";
        if (networkController && !peerId.isEmpty()) {
            networkController->sendControlCommandToPeer(peerId, rejected);
        }
        qDebug() << "[Network] observer command rejected"
                 << "action" << action
                 << "peer" << peerLabel;
        return;
    }

    const NetworkProcessingMode previousProcessingMode = networkProcessingMode;
    const bool previousServerDisableLocalVisualAudio = serverDisableLocalVisualAudio;
    applyNetworkStateFromCommand(command);
    const bool serverLocalOutputChanged =
        previousServerDisableLocalVisualAudio != serverDisableLocalVisualAudio;
    qDebug() << "[Network] received remote control command" << action;

    if (action == "settings" || action == "start") {
        const RadioSettings previousSettings = pendingSettings;
        const bool previousAgileScanEnabled = agileScanEnabled;
        const QString previousAgileScanRangesMhz = agileScanRangesMhz;
        const double previousAgileScanStepMhz = agileScanStepMhz;
        const bool previousStandardScanEnabled = standardScanEnabled;
        const QString previousStandardScanCentersMhz = standardScanCentersMhz;
        const int previousStandardScanDwellMs = standardScanDwellMs;
        applySettingsFromJson(settingsJson);
        publishSettingsToGlobals();
        updateUiFromPendingSettings();
        applyLiveRemoteSettings(previousSettings);
        const bool audioChanged =
            previousSettings.audioEnabled != pendingSettings.audioEnabled;
        const bool fftChanged =
            previousSettings.fftLength != pendingSettings.fftLength;
        const bool streamModeChanged =
            previousProcessingMode != networkProcessingMode;
        const bool fullIqServerAudioPathChanged =
            audioChanged && isFullIqProcessingMode();
        const bool agileScanChanged =
            previousAgileScanEnabled != agileScanEnabled ||
            previousAgileScanRangesMhz != agileScanRangesMhz ||
            std::abs(previousAgileScanStepMhz - agileScanStepMhz) > 0.0000001;
        const bool standardScanChanged =
            previousStandardScanEnabled != standardScanEnabled ||
            previousStandardScanCentersMhz != standardScanCentersMhz ||
            previousStandardScanDwellMs != standardScanDwellMs;

        if (fftChanged) {
            applyFftLengthChange(pendingSettings.fftLength, false);
        }

        if (audioChanged && !isIdle()) {
            if (pendingSettings.audioEnabled) {
                if (audioProcessor) {
                    audioProcessor->startDemodulation();
                }
            } else {
                if (audioProcessor) {
                    audioProcessor->stopDemodulation();
                }
                pendingAudioStartAfterStreamReady = false;
            }
        }
        const bool restartRequired =
            std::abs(previousSettings.sampleRate - pendingSettings.sampleRate) > 0.5 ||
            previousSettings.inputMode != pendingSettings.inputMode ||
            previousSettings.clockSource != pendingSettings.clockSource ||
            agileScanChanged ||
            standardScanChanged ||
            streamModeChanged ||
            fullIqServerAudioPathChanged;

        if (restartRequired && !isIdle()) {
            restartStreamForHardwareChange();
        } else if (serverLocalOutputChanged && !isIdle()) {
            applyServerLocalOutputPolicy();
        }

        if (processor && processor->isRunning()) {
            updateIqFrameProducerSettings();
        }
    }

    if (action == "start") {
        if (isIdle()) {
            startFobosProcessing();
        } else {
            qDebug() << "[Network] remote start ignored because server is not idle";
        }
    } else if (action == "stop") {
        stopFobosProcessing();
    }
}

void YourClassName::sendNetworkSpectrumFrame(const std::vector<float> &frequencies,
                                             const std::vector<float> &magnitudes,
                                             const std::vector<float> &referenceMagnitudes,
                                             double frameCenterFrequency,
                                             double frameMinFrequency,
                                             double frameMaxFrequency,
                                             const QVector<ScanVisualSegment> &scanSegments) {
    if (networkMode != NetworkMode::Server ||
        isFullIqProcessingMode() ||
        !networkController ||
        !networkController->isControlReady() ||
        frequencies.empty() ||
        magnitudes.empty()) {
        return;
    }

    const int dataCount = std::min(static_cast<int>(frequencies.size()), static_cast<int>(magnitudes.size()));
    if (dataCount <= 0) {
        return;
    }
    if ((isChannelIqProcessingMode() || networkFullResolutionSpectrumFrames) &&
        networkController->pendingBytes() > NETWORK_SPECTRUM_MAX_PENDING_BYTES) {
        return;
    }

    const int minIntervalMs = networkFullResolutionSpectrumFrames
                                  ? NETWORK_FULL_RESOLUTION_SPECTRUM_INTERVAL_MS
                                  : (isChannelIqProcessingMode()
                                         ? NETWORK_CHANNEL_SPECTRUM_INTERVAL_MS
                                         : NETWORK_SPECTRUM_INTERVAL_MS);
    if (networkSpectrumFrameTimer.isValid() &&
        networkSpectrumFrameTimer.elapsed() < minIntervalMs) {
        return;
    }
    networkSpectrumFrameTimer.restart();

    if (!std::isfinite(frameCenterFrequency)) {
        double scanCenterFrequency = currentAgileScanCenterFrequencyHz();
        if (!std::isfinite(scanCenterFrequency)) {
            scanCenterFrequency = currentStandardScanCenterFrequencyHz();
        }
        frameCenterFrequency = std::isfinite(scanCenterFrequency)
                                   ? scanCenterFrequency
                                   : pendingSettings.centerFrequency;
    }
    if (!std::isfinite(frameMinFrequency) || !std::isfinite(frameMaxFrequency)) {
        frameMinFrequency = minFrequency;
        frameMaxFrequency = maxFrequency;
    }
    if (!std::isfinite(frameMinFrequency) ||
        !std::isfinite(frameMaxFrequency) ||
        frameMaxFrequency <= frameMinFrequency) {
        frameMinFrequency = frequencies.front();
        frameMaxFrequency = frequencies.back();
    }
    double frameListeningFrequency = pendingSettings.listeningFrequency;
    const bool scanListeningValid =
        !scanSegments.isEmpty() &&
        actualFrequencyInsideScanSegments(frameListeningFrequency, scanSegments);
    if (!std::isfinite(frameListeningFrequency) ||
        (!scanListeningValid &&
         (scanSegments.isEmpty() ||
          frameListeningFrequency < frameMinFrequency ||
          frameListeningFrequency > frameMaxFrequency))) {
        frameListeningFrequency =
            !scanSegments.isEmpty()
                ? fallbackActualFrequencyForScanSegments(scanSegments, frameCenterFrequency)
                : frameCenterFrequency;
    }

    auto lower = std::lower_bound(frequencies.begin(), frequencies.begin() + dataCount, static_cast<float>(frameMinFrequency));
    auto upper = std::upper_bound(frequencies.begin(), frequencies.begin() + dataCount, static_cast<float>(frameMaxFrequency));
    int sourceStart = static_cast<int>(std::distance(frequencies.begin(), lower));
    int sourceEnd = static_cast<int>(std::distance(frequencies.begin(), upper));
    sourceStart = (std::max)(0, sourceStart - 1);
    sourceEnd = (std::min)(dataCount, sourceEnd + 1);
    if (sourceEnd <= sourceStart) {
        sourceStart = 0;
        sourceEnd = dataCount;
        frameMinFrequency = frequencies.front();
        frameMaxFrequency = frequencies.back();
    }

    const int sourceCount = sourceEnd - sourceStart;
    const int maxBins = isChannelIqProcessingMode()
                            ? NETWORK_CHANNEL_SPECTRUM_MAX_BINS
                            : NETWORK_SPECTRUM_MAX_BINS;
    const int targetCount = networkFullResolutionSpectrumFrames
                                ? sourceCount
                                : (std::min)(maxBins, sourceCount);
    if (targetCount <= 0) {
        return;
    }
    const double step = static_cast<double>(sourceCount) / static_cast<double>(targetCount);
    QJsonArray frequencyArray;
    QJsonArray magnitudeArray;
    QJsonArray referenceMagnitudeArray;
    std::vector<float> resampledMagnitudes(static_cast<std::size_t>(targetCount),
                                           -160.0f);
    std::vector<float> resampledReferenceMagnitudes(static_cast<std::size_t>(targetCount),
                                                    -160.0f);
    const bool haveReferenceMagnitudes =
        static_cast<int>(referenceMagnitudes.size()) >= dataCount &&
        pendingSettings.inputMode == INPUT_HF_NOISE_CANCEL;
    const bool useBinarySpectrumFrame = networkFullResolutionSpectrumFrames;

    for (int i = 0; i < targetCount; ++i) {
        const int index = (std::min)(sourceEnd - 1,
                                     sourceStart + static_cast<int>(std::floor(i * step)));
        const int magnitudeIndex = (index + dataCount / 2) % dataCount;
        if (!std::isfinite(frequencies[index]) || !std::isfinite(magnitudes[magnitudeIndex])) {
            continue;
        }
        if (!useBinarySpectrumFrame) {
            frequencyArray.append(frequencies[index]);
        }
        resampledMagnitudes[static_cast<std::size_t>((i + targetCount / 2) % targetCount)] =
            magnitudes[magnitudeIndex];
        if (haveReferenceMagnitudes && std::isfinite(referenceMagnitudes[magnitudeIndex])) {
            resampledReferenceMagnitudes[static_cast<std::size_t>((i + targetCount / 2) % targetCount)] =
                referenceMagnitudes[magnitudeIndex];
        }
    }

    if (!useBinarySpectrumFrame) {
        for (const float value : resampledMagnitudes) {
            magnitudeArray.append(std::isfinite(value) ? value : -160.0f);
        }
        if (haveReferenceMagnitudes) {
            for (const float value : resampledReferenceMagnitudes) {
                referenceMagnitudeArray.append(std::isfinite(value) ? value : -160.0f);
            }
        }
    }

    QJsonObject frame;
    frame["type"] = "spectrum";
    frame["sequence"] = QString::number(++networkSpectrumFrameSequence);
    frame["centerFrequency"] = frameCenterFrequency;
    frame["listeningFrequency"] = frameListeningFrequency;
    frame["sampleRate"] = pendingSettings.sampleRate;
    frame["bandwidth"] = pendingSettings.bandwidth;
    frame["modulationType"] = pendingSettings.modulationType;
    frame["inputMode"] = pendingSettings.inputMode;
    frame["fftLength"] = targetCount;
    frame["sourceFftLength"] = dataCount;
    frame["fullResolution"] = networkFullResolutionSpectrumFrames;
    frame["minFrequency"] = frameMinFrequency;
    frame["maxFrequency"] = frameMaxFrequency;
    if (!scanSegments.isEmpty()) {
        QJsonArray segmentArray;
        for (const ScanVisualSegment &segment : scanSegments) {
            if (!std::isfinite(segment.startHz) ||
                !std::isfinite(segment.endHz) ||
                segment.endHz <= segment.startHz) {
                continue;
            }
            QJsonObject item;
            item["startHz"] = segment.startHz;
            item["endHz"] = segment.endHz;
            item["centerHz"] = segment.centerHz;
            item["actualStartHz"] = segment.actualStartHz;
            item["actualEndHz"] = segment.actualEndHz;
            item["actualCenterHz"] = segment.actualCenterHz;
            item["label"] = segment.label;
            segmentArray.append(item);
        }
        if (!segmentArray.isEmpty()) {
            frame["scanSegments"] = segmentArray;
        }
    }
    if (useBinarySpectrumFrame) {
        frame["type"] = "spectrumbin";
        frame["binFormat"] = "f32le";
        frame["hasReferenceMagnitudes"] = haveReferenceMagnitudes;
        frame["frequencyLayout"] = "linear-min-max";

        QByteArray payload;
        payload.reserve(static_cast<int>(targetCount * static_cast<int>(sizeof(float)) *
                                         (haveReferenceMagnitudes ? 2 : 1)));
        payload.append(reinterpret_cast<const char*>(resampledMagnitudes.data()),
                       targetCount * static_cast<int>(sizeof(float)));
        if (haveReferenceMagnitudes) {
            payload.append(reinterpret_cast<const char*>(resampledReferenceMagnitudes.data()),
                           targetCount * static_cast<int>(sizeof(float)));
        }
        networkController->sendBinaryCommand(frame, payload);
        return;
    }
    frame["frequencies"] = frequencyArray;
    frame["magnitudes"] = magnitudeArray;
    if (haveReferenceMagnitudes) {
        frame["referenceMagnitudes"] = referenceMagnitudeArray;
    }

    networkController->sendControlCommand(frame);
}

void YourClassName::displayNetworkSpectrumFrame(const QJsonObject &frame) {
    if (networkMode == NetworkMode::Client &&
        runState == RadioRunState::Idle &&
        networkController &&
        !networkController->clientHasControl()) {
        runState = RadioRunState::Running;
        updateUiForRunState();
    }

    const QJsonArray frequencyArray = frame.value("frequencies").toArray();
    const QJsonArray magnitudeArray = frame.value("magnitudes").toArray();
    const QJsonArray referenceMagnitudeArray = frame.value("referenceMagnitudes").toArray();
    const int dataCount = (std::min)(frequencyArray.size(), magnitudeArray.size());
    if (dataCount <= 0) {
        return;
    }

    std::vector<float> frequencies;
    std::vector<float> magnitudes;
    std::vector<float> referenceMagnitudes;
    frequencies.reserve(dataCount);
    magnitudes.reserve(dataCount);
    if (referenceMagnitudeArray.size() >= dataCount) {
        referenceMagnitudes.reserve(dataCount);
    }

    for (int i = 0; i < dataCount; ++i) {
        const double frequency = frequencyArray.at(i).toDouble(std::numeric_limits<double>::quiet_NaN());
        const double magnitude = magnitudeArray.at(i).toDouble(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite(frequency) || !std::isfinite(magnitude)) {
            continue;
        }
        frequencies.push_back(static_cast<float>(frequency));
        magnitudes.push_back(static_cast<float>(magnitude));
        if (referenceMagnitudeArray.size() >= dataCount) {
            const double referenceMagnitude =
                referenceMagnitudeArray.at(i).toDouble(std::numeric_limits<double>::quiet_NaN());
            referenceMagnitudes.push_back(std::isfinite(referenceMagnitude)
                                              ? static_cast<float>(referenceMagnitude)
                                              : -160.0f);
        }
    }

    if (frequencies.empty() || magnitudes.empty()) {
        return;
    }

    const double frameMinFrequency = frame.value("minFrequency").toDouble(minFrequency);
    const double frameMaxFrequency = frame.value("maxFrequency").toDouble(maxFrequency);
    const double frameCenterFrequency = frame.value("centerFrequency").toDouble(pendingSettings.centerFrequency);
    const double frameListeningFrequency = frame.value("listeningFrequency").toDouble(pendingSettings.listeningFrequency);
    const double frameSampleRate = frame.value("sampleRate").toDouble(pendingSettings.sampleRate);
    const double frameBandwidth = frame.value("bandwidth").toDouble(pendingSettings.bandwidth);
    const int frameModulationType = frame.value("modulationType").toInt(pendingSettings.modulationType);
    const int frameInputMode = (std::clamp)(frame.value("inputMode").toInt(pendingSettings.inputMode),
                                            static_cast<int>(INPUT_RF),
                                            static_cast<int>(INPUT_HF_NOISE_CANCEL));
    const QVector<ScanVisualSegment> frameScanSegments = scanSegmentsFromFrame(frame);

    const bool protectLocalControlSettings =
        networkController &&
        networkController->clientHasControl() &&
        networkClientSettingsGuardTimer.isValid() &&
        networkClientSettingsGuardTimer.elapsed() < NETWORK_CLIENT_SETTINGS_GUARD_MS;

    double displayCenterFrequency = frameCenterFrequency;
    double displayListeningFrequency = frameListeningFrequency;
    double displayBandwidth = frameBandwidth;
    int displayModulationType = frameModulationType;

    if (protectLocalControlSettings) {
        displayCenterFrequency = pendingSettings.centerFrequency;
        displayListeningFrequency = pendingSettings.listeningFrequency;
        displayBandwidth = pendingSettings.bandwidth;
        displayModulationType = pendingSettings.modulationType;
    } else {
        bool settingsChanged = false;
        auto updateDouble = [&settingsChanged](double &target, double value) {
            if (std::isfinite(value) && std::abs(target - value) > 0.5) {
                target = value;
                settingsChanged = true;
            }
        };
        auto updateInt = [&settingsChanged](int &target, int value) {
            if (target != value) {
                target = value;
                settingsChanged = true;
            }
        };

        updateDouble(pendingSettings.centerFrequency, frameCenterFrequency);
        updateDouble(pendingSettings.listeningFrequency, frameListeningFrequency);
        updateDouble(pendingSettings.sampleRate, frameSampleRate);
        updateDouble(pendingSettings.bandwidth, frameBandwidth);
        updateInt(pendingSettings.modulationType, frameModulationType);
        updateInt(pendingSettings.inputMode, frameInputMode);

        if (settingsChanged) {
            normalizeTuning(pendingSettings);
            displayCenterFrequency = pendingSettings.centerFrequency;
            displayListeningFrequency = pendingSettings.listeningFrequency;
            displayBandwidth = pendingSettings.bandwidth;
            displayModulationType = pendingSettings.modulationType;
            updateUiFromPendingSettings();
        }
        publishSettingsToGlobals();
    }

    if (scaleWidget) {
        scaleWidget->setScanSegments(frameScanSegments);
        scaleWidget->setTuning(displayListeningFrequency,
                               displayCenterFrequency,
                               displayBandwidth,
                               displayModulationType);
        scaleWidget->setRange(frameMinFrequency, frameMaxFrequency);
    }
    const std::vector<float> hunterFrequencies =
        !frameScanSegments.isEmpty()
            ? actualFrequenciesFromScanSegments(frequencies, frameScanSegments)
            : std::vector<float>();
    const std::vector<float> &detectorFrequencies =
        hunterFrequencies.size() == frequencies.size() ? hunterFrequencies : frequencies;
    updateDmrHunter(detectorFrequencies, magnitudes);
    updateFpvHunter(detectorFrequencies,
                    magnitudes);
    updateDigitalVideoHunter(detectorFrequencies, magnitudes);

    const int frameFftLength = static_cast<int>(frequencies.size());
    const bool spectrumShapeChanged =
        !networkSpectrumFrameMetadataValid ||
        std::abs(networkSpectrumFrameMinFrequency - frameMinFrequency) > 0.5 ||
        std::abs(networkSpectrumFrameMaxFrequency - frameMaxFrequency) > 0.5 ||
        networkSpectrumFrameFftLength != frameFftLength;
    if (spectrumShapeChanged) {
        qDebug() << "[NetworkSpectrum] frame range changed; preserving waterfall history"
                 << "min" << frameMinFrequency
                 << "max" << frameMaxFrequency
                 << "bins" << frameFftLength;
        networkSpectrumFrameMetadataValid = true;
        networkSpectrumFrameMinFrequency = frameMinFrequency;
        networkSpectrumFrameMaxFrequency = frameMaxFrequency;
        networkSpectrumFrameFftLength = frameFftLength;
    }

    const double frequencyStep =
        frameFftLength > 1
            ? (frameMaxFrequency - frameMinFrequency) / static_cast<double>(frameFftLength - 1)
            : 0.0;
    const bool haveReferenceMagnitudes =
        static_cast<int>(referenceMagnitudes.size()) == frameFftLength;

    if (graphWidget) {
        const int graphTargetCount =
            (std::min)(frameFftLength,
                       (std::max)(512, graphWidget->width() > 0 ? graphWidget->width() * 2 : 2048));
        std::vector<float> graphFrequencies(static_cast<std::size_t>(graphTargetCount), 0.0f);
        std::vector<float> graphMagnitudes(static_cast<std::size_t>(graphTargetCount), -160.0f);
        std::vector<float> graphReferenceMagnitudes;
        if (haveReferenceMagnitudes) {
            graphReferenceMagnitudes.assign(static_cast<std::size_t>(graphTargetCount), -160.0f);
        }

        const double pointsPerGraphBin =
            static_cast<double>(frameFftLength) / static_cast<double>((std::max)(1, graphTargetCount));
        for (int i = 0; i < graphTargetCount; ++i) {
            const int sourceBegin =
                (std::clamp)(static_cast<int>(std::floor(i * pointsPerGraphBin)), 0, frameFftLength - 1);
            const int sourceEnd =
                (std::clamp)(static_cast<int>(std::ceil((i + 1) * pointsPerGraphBin)), sourceBegin + 1, frameFftLength);
            float peakMagnitude = -160.0f;
            float peakReferenceMagnitude = -160.0f;
            for (int sourceIndex = sourceBegin; sourceIndex < sourceEnd; ++sourceIndex) {
                const int shiftedIndex = (sourceIndex + frameFftLength / 2) % frameFftLength;
                const float value = magnitudes[static_cast<std::size_t>(shiftedIndex)];
                if (std::isfinite(value)) {
                    peakMagnitude = (std::max)(peakMagnitude, value);
                }
                if (haveReferenceMagnitudes) {
                    const float referenceValue = referenceMagnitudes[static_cast<std::size_t>(shiftedIndex)];
                    if (std::isfinite(referenceValue)) {
                        peakReferenceMagnitude = (std::max)(peakReferenceMagnitude, referenceValue);
                    }
                }
            }
            const double centerSourceIndex = (sourceBegin + sourceEnd - 1) * 0.5;
            graphFrequencies[static_cast<std::size_t>(i)] =
                static_cast<float>(frameMinFrequency + frequencyStep * centerSourceIndex);
            graphMagnitudes[static_cast<std::size_t>((i + graphTargetCount / 2) % graphTargetCount)] =
                peakMagnitude;
            if (haveReferenceMagnitudes) {
                graphReferenceMagnitudes[static_cast<std::size_t>((i + graphTargetCount / 2) % graphTargetCount)] =
                    peakReferenceMagnitude;
            }
        }

        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
        graphWidget->setScanSegments(frameScanSegments);
        graphWidget->setData(graphFrequencies,
                             graphMagnitudes,
                             frameMinFrequency,
                             frameMaxFrequency,
                             graphTargetCount,
                             colorf);
        graphWidget->setOverlayData(graphReferenceMagnitudes,
                                    static_cast<int>(graphReferenceMagnitudes.size()) == graphTargetCount);
    }
    if (waterfallWidget) {
        waterfallWidget->setData(frequencies, magnitudes, frameMinFrequency, frameMaxFrequency, frameFftLength,
                                 secondGraph, contrast, sensitivity, displayLevelMin, displayLevelMax);
        waterfallWidget->setScanSegments(frameScanSegments);
    }
}

void YourClassName::displayNetworkSpectrumFrameBinary(const QJsonObject &frame, const QByteArray &payload) {
    if (networkMode == NetworkMode::Client &&
        runState == RadioRunState::Idle &&
        networkController &&
        !networkController->clientHasControl()) {
        runState = RadioRunState::Running;
        updateUiForRunState();
    }

    const int frameFftLength = frame.value("fftLength").toInt();
    const double frameMinFrequency = frame.value("minFrequency").toDouble(minFrequency);
    const double frameMaxFrequency = frame.value("maxFrequency").toDouble(maxFrequency);
    if (frameFftLength <= 0 ||
        payload.size() < frameFftLength * static_cast<int>(sizeof(float)) ||
        !std::isfinite(frameMinFrequency) ||
        !std::isfinite(frameMaxFrequency) ||
        frameMaxFrequency <= frameMinFrequency) {
        qDebug() << "[NetworkSpectrum] invalid binary spectrum frame"
                 << "bins" << frameFftLength
                 << "payload" << payload.size()
                 << "min" << frameMinFrequency
                 << "max" << frameMaxFrequency;
        return;
    }

    std::vector<float> frequencies(static_cast<std::size_t>(frameFftLength), 0.0f);
    std::vector<float> magnitudes(static_cast<std::size_t>(frameFftLength), -160.0f);
    std::memcpy(magnitudes.data(), payload.constData(), frameFftLength * sizeof(float));

    const bool haveReferenceMagnitudes =
        frame.value("hasReferenceMagnitudes").toBool(false) &&
        payload.size() >= frameFftLength * static_cast<int>(sizeof(float)) * 2;
    std::vector<float> referenceMagnitudes;
    if (haveReferenceMagnitudes) {
        referenceMagnitudes.resize(static_cast<std::size_t>(frameFftLength), -160.0f);
        std::memcpy(referenceMagnitudes.data(),
                    payload.constData() + frameFftLength * static_cast<int>(sizeof(float)),
                    frameFftLength * sizeof(float));
    }

    const double frequencyStep =
        frameFftLength > 1
            ? (frameMaxFrequency - frameMinFrequency) / static_cast<double>(frameFftLength - 1)
            : 0.0;
    for (int i = 0; i < frameFftLength; ++i) {
        frequencies[static_cast<std::size_t>(i)] =
            static_cast<float>(frameMinFrequency + frequencyStep * static_cast<double>(i));
        if (!std::isfinite(magnitudes[static_cast<std::size_t>(i)])) {
            magnitudes[static_cast<std::size_t>(i)] = -160.0f;
        }
        if (haveReferenceMagnitudes &&
            !std::isfinite(referenceMagnitudes[static_cast<std::size_t>(i)])) {
            referenceMagnitudes[static_cast<std::size_t>(i)] = -160.0f;
        }
    }

    const double frameCenterFrequency = frame.value("centerFrequency").toDouble(pendingSettings.centerFrequency);
    const double frameListeningFrequency = frame.value("listeningFrequency").toDouble(pendingSettings.listeningFrequency);
    const double frameSampleRate = frame.value("sampleRate").toDouble(pendingSettings.sampleRate);
    const double frameBandwidth = frame.value("bandwidth").toDouble(pendingSettings.bandwidth);
    const int frameModulationType = frame.value("modulationType").toInt(pendingSettings.modulationType);
    const int frameInputMode = (std::clamp)(frame.value("inputMode").toInt(pendingSettings.inputMode),
                                            static_cast<int>(INPUT_RF),
                                            static_cast<int>(INPUT_HF_NOISE_CANCEL));
    const QVector<ScanVisualSegment> frameScanSegments = scanSegmentsFromFrame(frame);

    const bool protectLocalControlSettings =
        networkController &&
        networkController->clientHasControl() &&
        networkClientSettingsGuardTimer.isValid() &&
        networkClientSettingsGuardTimer.elapsed() < NETWORK_CLIENT_SETTINGS_GUARD_MS;

    double displayCenterFrequency = frameCenterFrequency;
    double displayListeningFrequency = frameListeningFrequency;
    double displayBandwidth = frameBandwidth;
    int displayModulationType = frameModulationType;

    if (protectLocalControlSettings) {
        displayCenterFrequency = pendingSettings.centerFrequency;
        displayListeningFrequency = pendingSettings.listeningFrequency;
        displayBandwidth = pendingSettings.bandwidth;
        displayModulationType = pendingSettings.modulationType;
    } else {
        bool settingsChanged = false;
        auto updateDouble = [&settingsChanged](double &target, double value) {
            if (std::isfinite(value) && std::abs(target - value) > 0.5) {
                target = value;
                settingsChanged = true;
            }
        };
        auto updateInt = [&settingsChanged](int &target, int value) {
            if (target != value) {
                target = value;
                settingsChanged = true;
            }
        };

        updateDouble(pendingSettings.centerFrequency, frameCenterFrequency);
        updateDouble(pendingSettings.listeningFrequency, frameListeningFrequency);
        updateDouble(pendingSettings.sampleRate, frameSampleRate);
        updateDouble(pendingSettings.bandwidth, frameBandwidth);
        updateInt(pendingSettings.modulationType, frameModulationType);
        updateInt(pendingSettings.inputMode, frameInputMode);

        if (settingsChanged) {
            normalizeTuning(pendingSettings);
            displayCenterFrequency = pendingSettings.centerFrequency;
            displayListeningFrequency = pendingSettings.listeningFrequency;
            displayBandwidth = pendingSettings.bandwidth;
            displayModulationType = pendingSettings.modulationType;
            updateUiFromPendingSettings();
        }
        publishSettingsToGlobals();
    }

    if (scaleWidget) {
        scaleWidget->setScanSegments(frameScanSegments);
        scaleWidget->setTuning(displayListeningFrequency,
                               displayCenterFrequency,
                               displayBandwidth,
                               displayModulationType);
        scaleWidget->setRange(frameMinFrequency, frameMaxFrequency);
    }
    const std::vector<float> hunterFrequencies =
        !frameScanSegments.isEmpty()
            ? actualFrequenciesFromScanSegments(frequencies, frameScanSegments)
            : std::vector<float>();
    const std::vector<float> &detectorFrequencies =
        hunterFrequencies.size() == frequencies.size() ? hunterFrequencies : frequencies;
    updateDmrHunter(detectorFrequencies, magnitudes);
    updateFpvHunter(detectorFrequencies,
                    magnitudes);
    updateDigitalVideoHunter(detectorFrequencies, magnitudes);

    const bool spectrumShapeChanged =
        !networkSpectrumFrameMetadataValid ||
        std::abs(networkSpectrumFrameMinFrequency - frameMinFrequency) > 0.5 ||
        std::abs(networkSpectrumFrameMaxFrequency - frameMaxFrequency) > 0.5 ||
        networkSpectrumFrameFftLength != frameFftLength;
    if (spectrumShapeChanged) {
        qDebug() << "[NetworkSpectrum] binary frame range changed; preserving waterfall history"
                 << "min" << frameMinFrequency
                 << "max" << frameMaxFrequency
                 << "bins" << frameFftLength;
        networkSpectrumFrameMetadataValid = true;
        networkSpectrumFrameMinFrequency = frameMinFrequency;
        networkSpectrumFrameMaxFrequency = frameMaxFrequency;
        networkSpectrumFrameFftLength = frameFftLength;
    }

    if (graphWidget) {
        const int graphTargetCount =
            (std::min)(frameFftLength,
                       (std::max)(512, graphWidget->width() > 0 ? graphWidget->width() * 2 : 2048));
        std::vector<float> graphFrequencies(static_cast<std::size_t>(graphTargetCount), 0.0f);
        std::vector<float> graphMagnitudes(static_cast<std::size_t>(graphTargetCount), -160.0f);
        std::vector<float> graphReferenceMagnitudes;
        if (haveReferenceMagnitudes) {
            graphReferenceMagnitudes.assign(static_cast<std::size_t>(graphTargetCount), -160.0f);
        }

        const double pointsPerGraphBin =
            static_cast<double>(frameFftLength) / static_cast<double>((std::max)(1, graphTargetCount));
        for (int i = 0; i < graphTargetCount; ++i) {
            const int sourceBegin =
                (std::clamp)(static_cast<int>(std::floor(i * pointsPerGraphBin)), 0, frameFftLength - 1);
            const int sourceEnd =
                (std::clamp)(static_cast<int>(std::ceil((i + 1) * pointsPerGraphBin)), sourceBegin + 1, frameFftLength);
            float peakMagnitude = -160.0f;
            float peakReferenceMagnitude = -160.0f;
            for (int sourceIndex = sourceBegin; sourceIndex < sourceEnd; ++sourceIndex) {
                const int shiftedIndex = (sourceIndex + frameFftLength / 2) % frameFftLength;
                const float value = magnitudes[static_cast<std::size_t>(shiftedIndex)];
                if (std::isfinite(value)) {
                    peakMagnitude = (std::max)(peakMagnitude, value);
                }
                if (haveReferenceMagnitudes) {
                    const float referenceValue = referenceMagnitudes[static_cast<std::size_t>(shiftedIndex)];
                    if (std::isfinite(referenceValue)) {
                        peakReferenceMagnitude = (std::max)(peakReferenceMagnitude, referenceValue);
                    }
                }
            }
            const double centerSourceIndex = (sourceBegin + sourceEnd - 1) * 0.5;
            graphFrequencies[static_cast<std::size_t>(i)] =
                static_cast<float>(frameMinFrequency + frequencyStep * centerSourceIndex);
            graphMagnitudes[static_cast<std::size_t>((i + graphTargetCount / 2) % graphTargetCount)] =
                peakMagnitude;
            if (haveReferenceMagnitudes) {
                graphReferenceMagnitudes[static_cast<std::size_t>((i + graphTargetCount / 2) % graphTargetCount)] =
                    peakReferenceMagnitude;
            }
        }

        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
        graphWidget->setScanSegments(frameScanSegments);
        graphWidget->setData(graphFrequencies,
                             graphMagnitudes,
                             frameMinFrequency,
                             frameMaxFrequency,
                             graphTargetCount,
                             colorf);
        graphWidget->setOverlayData(graphReferenceMagnitudes,
                                    static_cast<int>(graphReferenceMagnitudes.size()) == graphTargetCount);
    }
    if (waterfallWidget) {
        waterfallWidget->setData(frequencies, magnitudes, frameMinFrequency, frameMaxFrequency, frameFftLength,
                                 secondGraph, contrast, sensitivity, displayLevelMin, displayLevelMax);
        waterfallWidget->setScanSegments(frameScanSegments);
    }
}

void YourClassName::sendNetworkAudioFrame(const QByteArray &pcmData) {
    if (networkMode != NetworkMode::Server ||
        isChannelIqProcessingMode() ||
        !networkController ||
        !networkController->isControlReady() ||
        pcmData.isEmpty()) {
        return;
    }

    QJsonObject frame;
    frame["type"] = "audio";
    frame["sampleRate"] = 48000;
    frame["channels"] = 1;
    frame["sampleFormat"] = "pcm_s16le";
    frame["pcm"] = QString::fromLatin1(pcmData.toBase64());
    networkController->sendControlCommand(frame);
}

void YourClassName::playNetworkAudioFrame(const QJsonObject &frame) {
    if (networkMode != NetworkMode::Client || !remoteAudioPlayer) {
        return;
    }
    if (runState != RadioRunState::Running) {
        return;
    }

    if (frame.value("sampleRate").toInt(48000) != 48000 ||
        frame.value("channels").toInt(1) != 1 ||
        frame.value("sampleFormat").toString() != QStringLiteral("pcm_s16le")) {
        qDebug() << "[NetworkAudio] unsupported remote audio frame format";
        return;
    }

    const QByteArray pcmData = QByteArray::fromBase64(frame.value("pcm").toString().toLatin1());
    processDigitalAudioFrame(pcmData);
    processSstvAudioFrame(pcmData);
    processAptAudioFrame(pcmData);
    processWefaxAudioFrame(pcmData);
    sendAudioRelayFrame(pcmData);
    sendAudioHttpFrame(pcmData);
    remoteAudioPlayer->playPcmFrame(pcmData);
}

void YourClassName::updateAudioRelaySocket() {
    if (!audioRelaySocket) {
        return;
    }

    if (!audioRelayReceiveEnabled) {
        if (audioRelaySocket->state() != QAbstractSocket::UnconnectedState) {
            audioRelaySocket->close();
        }
        return;
    }

    if (audioRelaySocket->state() != QAbstractSocket::UnconnectedState &&
        audioRelaySocket->localPort() == audioRelayListenPort) {
        return;
    }

    audioRelaySocket->close();
    const bool bound = audioRelaySocket->bind(QHostAddress::AnyIPv4,
                                              audioRelayListenPort,
                                              QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    if (bound) {
        qDebug() << "[AudioRelay] receive enabled on UDP port" << audioRelayListenPort;
    } else {
        qWarning() << "[AudioRelay] could not bind UDP port"
                   << audioRelayListenPort
                   << audioRelaySocket->errorString();
    }
}

void YourClassName::sendAudioRelayFrame(const QByteArray &pcmData) {
    if (!audioRelayTransmitEnabled ||
        !audioRelaySocket ||
        audioRelayHost.trimmed().isEmpty() ||
        audioRelayPort == 0 ||
        pcmData.isEmpty()) {
        return;
    }

    QHostAddress targetAddress;
    if (!targetAddress.setAddress(audioRelayHost.trimmed())) {
        static QString lastInvalidHost;
        if (lastInvalidHost != audioRelayHost) {
            lastInvalidHost = audioRelayHost;
            qWarning() << "[AudioRelay] target must be a numeric IP address:" << audioRelayHost;
        }
        return;
    }

    QByteArray datagram;
    datagram.reserve(AUDIO_RELAY_HEADER_BYTES + pcmData.size());
    datagram.append("FBA1", 4);
    appendLe32(datagram, ++audioRelaySequence);
    appendLe32(datagram, static_cast<quint32>(pcmData.size()));
    datagram.append(pcmData);

    audioRelaySocket->writeDatagram(datagram, targetAddress, audioRelayPort);
}

void YourClassName::receiveAudioRelayDatagrams() {
    if (!audioRelayReceiveEnabled || !audioRelaySocket || !remoteAudioPlayer) {
        return;
    }

    while (audioRelaySocket->hasPendingDatagrams()) {
        const qint64 pendingSize = audioRelaySocket->pendingDatagramSize();
        if (pendingSize < AUDIO_RELAY_HEADER_BYTES || pendingSize > 65507) {
            QByteArray discard;
            discard.resize(static_cast<int>((std::min)(pendingSize, qint64(65507))));
            audioRelaySocket->readDatagram(discard.data(), discard.size());
            continue;
        }

        QByteArray datagram;
        datagram.resize(static_cast<int>(pendingSize));
        audioRelaySocket->readDatagram(datagram.data(), datagram.size());

        if (!datagram.startsWith("FBA1")) {
            continue;
        }

        const quint32 payloadBytes = readLe32(datagram.constData() + 8);
        if (payloadBytes == 0 ||
            payloadBytes > static_cast<quint32>(datagram.size() - AUDIO_RELAY_HEADER_BYTES)) {
            continue;
        }

        const QByteArray pcmData = datagram.mid(AUDIO_RELAY_HEADER_BYTES, static_cast<int>(payloadBytes));
        remoteAudioPlayer->playPcmFrame(pcmData);
    }
}

void YourClassName::updateAudioHttpStreamServer() {
    if (!audioHttpServer) {
        return;
    }

    if (!audioHttpStreamEnabled) {
        if (audioHttpServer->isListening()) {
            audioHttpServer->close();
            qDebug() << "[AudioHTTP] stream server stopped";
        }
        for (QTcpSocket *client : std::as_const(audioHttpClients)) {
            if (client) {
                client->disconnectFromHost();
                client->deleteLater();
            }
        }
        audioHttpClients.clear();
        return;
    }

    if (audioHttpServer->isListening() && audioHttpServer->serverPort() == audioHttpStreamPort) {
        return;
    }

    for (QTcpSocket *client : std::as_const(audioHttpClients)) {
        if (client) {
            client->disconnectFromHost();
            client->deleteLater();
        }
    }
    audioHttpClients.clear();
    audioHttpServer->close();

    const bool listening = audioHttpServer->listen(QHostAddress::AnyIPv4, audioHttpStreamPort);
    if (listening) {
        qDebug() << "[AudioHTTP] VLC-compatible audio stream listening on"
                 << QString("http://0.0.0.0:%1/audio.wav").arg(audioHttpStreamPort);
    } else {
        qWarning() << "[AudioHTTP] could not listen on port"
                   << audioHttpStreamPort
                   << audioHttpServer->errorString();
    }
}

void YourClassName::acceptAudioHttpClient() {
    if (!audioHttpServer) {
        return;
    }

    while (QTcpSocket *client = audioHttpServer->nextPendingConnection()) {
        audioHttpClients.append(client);
        connect(client, &QTcpSocket::disconnected, this, [this, client]() {
            removeAudioHttpClient(client);
        });

        QByteArray response;
        response.append("HTTP/1.1 200 OK\r\n");
        response.append("Content-Type: audio/wav\r\n");
        response.append("Cache-Control: no-cache, no-store, must-revalidate\r\n");
        response.append("Pragma: no-cache\r\n");
        response.append("Connection: close\r\n");
        response.append("\r\n");
        response.append(streamingWavHeader());
        client->write(response);
        qDebug() << "[AudioHTTP] client connected"
                 << client->peerAddress().toString()
                 << "clients" << audioHttpClients.size();
    }
}

void YourClassName::removeAudioHttpClient(QTcpSocket *client) {
    if (!client) {
        return;
    }

    audioHttpClients.removeAll(client);
    client->deleteLater();
    qDebug() << "[AudioHTTP] client disconnected"
             << "clients" << audioHttpClients.size();
}

void YourClassName::sendAudioHttpFrame(const QByteArray &pcmData) {
    if (!audioHttpStreamEnabled || audioHttpClients.isEmpty() || pcmData.isEmpty()) {
        return;
    }

    for (int i = audioHttpClients.size() - 1; i >= 0; --i) {
        QTcpSocket *client = audioHttpClients.at(i);
        if (!client || client->state() != QAbstractSocket::ConnectedState) {
            if (client) {
                removeAudioHttpClient(client);
            } else {
                audioHttpClients.removeAt(i);
            }
            continue;
        }

        if (client->bytesToWrite() > AUDIO_HTTP_MAX_PENDING_BYTES) {
            continue;
        }
        client->write(pcmData);
    }
}

void YourClassName::processDigitalAudioFrame(const QByteArray &pcmData, int sampleRate) {
    if (!digitalDecoder ||
        !digitalDecoderThread ||
        !digitalDecodeEnabled ||
        !digitalDecodeCheckbox ||
        !digitalDecodeCheckbox->isChecked()) {
        return;
    }
    if (sampleRate <= 0) {
        sampleRate = 48000;
    }

    RadioSettings settings = pendingSettings;
    settings.dmrLabEnabled = dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked();
    settings.dmrLabColorCode = dmrLabColorCodeCombo
                                   ? dmrLabColorCodeCombo->currentData().toInt()
                                   : -1;
    settings.dmrLabTimeslot = dmrLabSlotCombo
                                  ? dmrLabSlotCombo->currentData().toInt()
                                  : 0;
    settings.dmrBasebandSampleRate =
        dmrBasebandRateCombo
            ? normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt())
            : normalizedDmrBasebandSampleRate(settings.dmrBasebandSampleRate);
    settings.dmrAmbeLayout =
        dmrAmbeLayoutCombo
            ? normalizedDmrAmbeLayout(dmrAmbeLayoutCombo->currentData().toInt())
            : normalizedDmrAmbeLayout(settings.dmrAmbeLayout);
    settings.dmrManualTimingEnabled =
        dmrManualTimingCheckbox && dmrManualTimingCheckbox->isChecked();
    settings.dmrManualTimingOffset =
        dmrTimingOffsetSpin ? dmrTimingOffsetSpin->value() : settings.dmrManualTimingOffset;
    settings.dmrSlicerRatio =
        dmrSlicerRatioSpin ? dmrSlicerRatioSpin->value() : settings.dmrSlicerRatio;
    settings.dmrAdaptiveSlicer =
        !dmrAdaptiveSlicerCheckbox || dmrAdaptiveSlicerCheckbox->isChecked();
    const auto parseDmrLabId = [](const QLineEdit *edit) {
        if (!edit) {
            return 0;
        }
        bool ok = false;
        const int value = edit->text().trimmed().toInt(&ok);
        return ok && value > 0 ? value : 0;
    };
    settings.dmrLabSourceId = parseDmrLabId(dmrLabSourceIdEdit);
    settings.dmrLabTargetId = parseDmrLabId(dmrLabTargetIdEdit);
    const bool isDmr = settings.modulationType == MOD_DMR;
    const uint64_t decoderGeneration =
        digitalDecoderGeneration.load(std::memory_order_relaxed);
    QByteArray decoderPcmData = pcmData;
    if (isDmr) {
        constexpr int dmrPcmBytesPerSample = static_cast<int>(sizeof(qint16));
        constexpr int dmrPcmChunkMs = 60;
        const int dmrPcmChunkBytes =
            (std::max)(1, sampleRate * dmrPcmBytesPerSample * dmrPcmChunkMs / 1000);
        const int dmrPcmMaxBufferedBytes = dmrPcmChunkBytes * 3;

        if (pendingDmrDecoderSampleRate != sampleRate) {
            qDebug() << "[Digital] DMR input sample-rate changed"
                     << "oldRate" << pendingDmrDecoderSampleRate
                     << "newRate" << sampleRate
                     << "chunkBytes" << dmrPcmChunkBytes;
            if (!pendingDmrDecoderPcm.isEmpty()) {
                qDebug() << "[Digital] clearing DMR input buffer after sample-rate change"
                         << "oldRate" << pendingDmrDecoderSampleRate
                         << "newRate" << sampleRate
                         << "droppedBytes" << pendingDmrDecoderPcm.size();
            }
            pendingDmrDecoderPcm.clear();
            pendingDmrDecoderSampleRate = sampleRate;
        }

        pendingDmrDecoderPcm.append(pcmData);
        if (pendingDmrDecoderPcm.size() > dmrPcmMaxBufferedBytes) {
            const int bytesToDrop = pendingDmrDecoderPcm.size() - dmrPcmChunkBytes;
            pendingDmrDecoderPcm.remove(0, bytesToDrop);
            qWarning() << "[Digital] trimming DMR PCM input buffer"
                       << "droppedBytes" << bytesToDrop
                       << "keptBytes" << pendingDmrDecoderPcm.size();
        }
        if (pendingDmrDecoderPcm.size() < dmrPcmChunkBytes) {
            return;
        }
        decoderPcmData = pendingDmrDecoderPcm.left(dmrPcmChunkBytes);
        pendingDmrDecoderPcm.remove(0, dmrPcmChunkBytes);
    } else if (!pendingDmrDecoderPcm.isEmpty()) {
        pendingDmrDecoderPcm.clear();
        pendingDmrDecoderSampleRate = 48000;
    }

    const int maxQueuedFrames = isDmr ? 6 : 32;
    const int queuedBefore = pendingDigitalDecoderFrames.fetch_add(1, std::memory_order_relaxed);
    if (queuedBefore >= maxQueuedFrames) {
        pendingDigitalDecoderFrames.fetch_sub(1, std::memory_order_relaxed);
        const int dropped = droppedDigitalDecoderFramesSinceLog.fetch_add(1, std::memory_order_relaxed) + 1;
        if (dropped == 1 || dropped % 50 == 0) {
            qWarning() << "[Digital] dropping stale PCM frame"
                       << "mode" << settings.modulationType
                       << "queued" << queuedBefore
                       << "limit" << maxQueuedFrames
                       << "dropped" << dropped
                       << "bytes" << decoderPcmData.size();
        }
        return;
    }

    QMetaObject::invokeMethod(digitalDecoder,
                              [this,
                               decoder = digitalDecoder,
                               pcmData = decoderPcmData,
                               settings,
                               sampleRate,
                               decoderGeneration]() {
                                  const auto releaseQueuedFrame = qScopeGuard([this]() {
                                      pendingDigitalDecoderFrames.fetch_sub(1, std::memory_order_relaxed);
                                  });
                                  if (decoderGeneration !=
                                      digitalDecoderGeneration.load(std::memory_order_relaxed)) {
                                      return;
                                  }
                                  decoder->processPcmFrame(pcmData, settings, sampleRate);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::processSstvAudioFrame(const QByteArray &pcmData) {
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    if (!videoProcessor ||
        !videoProcessorThread ||
        !videoDecodeEnabled ||
        !videoDock ||
        !videoDock->isVisible() ||
        !videoDecodeCheckbox ||
        !videoDecodeCheckbox->isChecked() ||
        pendingSettings.modulationType != MOD_SSTV ||
        pcmData.isEmpty() ||
        suppressServerLocalOutput ||
        (videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked())) {
        return;
    }

    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, pcmData]() {
                                  processor->processSstvPcmFrame(pcmData, 48000);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::processAptAudioFrame(const QByteArray &pcmData) {
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    if (!videoProcessor ||
        !videoProcessorThread ||
        !videoDecodeEnabled ||
        !videoDock ||
        !videoDock->isVisible() ||
        !videoDecodeCheckbox ||
        !videoDecodeCheckbox->isChecked() ||
        pendingSettings.modulationType != MOD_APT ||
        pcmData.isEmpty() ||
        suppressServerLocalOutput ||
        (videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked())) {
        return;
    }

    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, pcmData]() {
                                  processor->processAptPcmFrame(pcmData, 48000);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::processWefaxAudioFrame(const QByteArray &pcmData) {
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    if (!videoProcessor ||
        !videoProcessorThread ||
        !videoDecodeEnabled ||
        !videoDock ||
        !videoDock->isVisible() ||
        !videoDecodeCheckbox ||
        !videoDecodeCheckbox->isChecked() ||
        pendingSettings.modulationType != MOD_WEFAX ||
        pcmData.isEmpty() ||
        suppressServerLocalOutput ||
        (videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked())) {
        return;
    }

    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, pcmData]() {
                                  processor->processWefaxPcmFrame(pcmData, 48000);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::updateDigitalDecoderMode() {
    if (!digitalDecoder || !digitalDecoderThread) {
        return;
    }
    const bool enabled = digitalDecodeEnabled;
    RadioSettings settings = pendingSettings;
    settings.dmrBasebandSampleRate =
        dmrBasebandRateCombo
            ? normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt())
            : normalizedDmrBasebandSampleRate(settings.dmrBasebandSampleRate);
    settings.dmrAmbeLayout =
        dmrAmbeLayoutCombo
            ? normalizedDmrAmbeLayout(dmrAmbeLayoutCombo->currentData().toInt())
            : normalizedDmrAmbeLayout(settings.dmrAmbeLayout);
    settings.dmrManualTimingEnabled =
        dmrManualTimingCheckbox && dmrManualTimingCheckbox->isChecked();
    settings.dmrManualTimingOffset =
        dmrTimingOffsetSpin ? dmrTimingOffsetSpin->value() : settings.dmrManualTimingOffset;
    settings.dmrSlicerRatio =
        dmrSlicerRatioSpin ? dmrSlicerRatioSpin->value() : settings.dmrSlicerRatio;
    settings.dmrAdaptiveSlicer =
        !dmrAdaptiveSlicerCheckbox || dmrAdaptiveSlicerCheckbox->isChecked();
    const int decoderSampleRate =
        settings.modulationType == MOD_DMR ? settings.dmrBasebandSampleRate : 48000;
    QMetaObject::invokeMethod(digitalDecoder,
                              [decoder = digitalDecoder, enabled, settings, decoderSampleRate]() {
                                  decoder->setEnabled(enabled);
                                  decoder->configure(settings, decoderSampleRate);
                              },
                              Qt::QueuedConnection);
}

bool YourClassName::isVideoDecodeActive() const {
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    return videoDecodeEnabled &&
           videoDock &&
           videoDock->isVisible() &&
           videoDecodeCheckbox &&
           videoDecodeCheckbox->isChecked() &&
           (pendingSettings.modulationType == MOD_ATV ||
            pendingSettings.modulationType == MOD_LRPT) &&
           (!videoTestPatternCheckbox || !videoTestPatternCheckbox->isChecked()) &&
           !suppressServerLocalOutput;
}

void YourClassName::processVideoIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount) {
    if (!isVideoDecodeActive() ||
        !videoProcessor ||
        !videoProcessorThread ||
        iqData.isEmpty() ||
        sampleRate <= 0.0 ||
        sampleCount <= 0) {
        return;
    }
    if (videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked()) {
        return;
    }
    if (videoIqFramePending.exchange(true)) {
        return;
    }
    if (pendingSettings.modulationType == MOD_LRPT) {
        const int bytesPerIq = iqData.size() >= sampleCount * 4 ? 4 : 2;
        std::vector<float> floatSamples;
        floatSamples.reserve(static_cast<std::size_t>(sampleCount) * 2);
        const auto *src = reinterpret_cast<const uchar *>(iqData.constData());
        for (int i = 0; i < sampleCount; ++i) {
            float iSample = 0.0f;
            float qSample = 0.0f;
            if (bytesPerIq == 4) {
                const int offset = i * 4;
                if (offset + 3 >= iqData.size()) {
                    break;
                }
                const qint16 rawI = static_cast<qint16>(src[offset] | (src[offset + 1] << 8));
                const qint16 rawQ = static_cast<qint16>(src[offset + 2] | (src[offset + 3] << 8));
                iSample = rawI / 32768.0f;
                qSample = rawQ / 32768.0f;
            } else {
                const int offset = i * 2;
                if (offset + 1 >= iqData.size()) {
                    break;
                }
                iSample = (static_cast<int>(src[offset]) - 128) / 128.0f;
                qSample = (static_cast<int>(src[offset + 1]) - 128) / 128.0f;
            }
            floatSamples.push_back(iSample);
            floatSamples.push_back(qSample);
        }
        if (floatSamples.size() < 8) {
            videoIqFramePending.store(false);
            return;
        }
        RadioSettings settings = spectrumProcessingSettings();
        settings.sampleRate = sampleRate;
        QMetaObject::invokeMethod(videoProcessor,
                                  [this, processor = videoProcessor, samples = std::move(floatSamples), settings]() {
                                      processor->processFloatIqSnapshot(samples, settings);
                                      QMetaObject::invokeMethod(this,
                                                                [this]() {
                                                                    videoIqFramePending.store(false);
                                                                },
                                                                Qt::QueuedConnection);
                                  },
                                  Qt::QueuedConnection);
        return;
    }

    if (iqData.size() == sampleCount * 2 && sampleRate > 10000000.0) {
        if (videoStatusLabel) {
            videoStatusLabel->setText(QStringLiteral("Video: switch to ATV/channel IQ for wide signals"));
        }
        videoIqFramePending.store(false);
        return;
    }

    QMetaObject::invokeMethod(videoProcessor,
                              [this, processor = videoProcessor, iqData, sampleRate, sampleCount]() {
                                  processor->processIqFrame(iqData, sampleRate, sampleCount);
                                  QMetaObject::invokeMethod(this,
                                                            [this]() {
                                                                videoIqFramePending.store(false);
                                                            },
                                                            Qt::QueuedConnection);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::processVideoSnapshotFrame() {
    const bool channelIqStreamMode =
        networkMode != NetworkMode::Disabled &&
        isChannelIqProcessingMode();
    if (!isVideoDecodeActive() ||
        channelIqStreamMode ||
        !videoProcessor ||
        !videoProcessorThread) {
        return;
    }
    if (videoIqFramePending.exchange(true)) {
        return;
    }

    std::vector<float> snapshot;
    std::uint64_t sequence = 0;
    if (!IqBuffer::snapshot(snapshot, &sequence) || snapshot.size() < 4) {
        videoIqFramePending.store(false);
        return;
    }

    if (snapshot.size() > VIDEO_SNAPSHOT_MAX_FLOATS) {
        std::vector<float> tail(snapshot.end() - static_cast<std::ptrdiff_t>(VIDEO_SNAPSHOT_MAX_FLOATS),
                                snapshot.end());
        snapshot.swap(tail);
    }

    RadioSettings settings = spectrumProcessingSettings();
    QMetaObject::invokeMethod(videoProcessor,
                              [this, processor = videoProcessor, samples = std::move(snapshot), settings]() {
                                  processor->processFloatIqSnapshot(samples, settings);
                                  QMetaObject::invokeMethod(this,
                                                            [this]() {
                                                                videoIqFramePending.store(false);
                                                            },
                                                            Qt::QueuedConnection);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::updateVideoProcessorMode() {
    if (!videoProcessor || !videoProcessorThread) {
        return;
    }

    const bool iqVideoEnabled = isVideoDecodeActive();
    const bool analogVideoEnabled = iqVideoEnabled && pendingSettings.modulationType == MOD_ATV;
    videoIqFramePending.store(false);
    const bool testPatternEnabled = videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked();
    const bool analogVideoTest = testPatternEnabled && pendingSettings.modulationType == MOD_ATV;
    const bool sstvTest = testPatternEnabled && pendingSettings.modulationType == MOD_SSTV;
    const bool aptTest = testPatternEnabled && pendingSettings.modulationType == MOD_APT;
    const bool wefaxTest = testPatternEnabled && pendingSettings.modulationType == MOD_WEFAX;
    const bool lrptTest = testPatternEnabled && pendingSettings.modulationType == MOD_LRPT;
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    const bool sstvEnabled =
        videoDock &&
        videoDock->isVisible() &&
        pendingSettings.modulationType == MOD_SSTV &&
        (sstvTest ||
         (videoDecodeEnabled &&
          videoDecodeCheckbox &&
          videoDecodeCheckbox->isChecked())) &&
        !suppressServerLocalOutput;
    const bool aptEnabled =
        videoDock &&
        videoDock->isVisible() &&
        pendingSettings.modulationType == MOD_APT &&
        (aptTest ||
         (videoDecodeEnabled &&
          videoDecodeCheckbox &&
          videoDecodeCheckbox->isChecked())) &&
        !suppressServerLocalOutput;
    const bool wefaxEnabled =
        videoDock &&
        videoDock->isVisible() &&
        pendingSettings.modulationType == MOD_WEFAX &&
        (wefaxTest ||
         (videoDecodeEnabled &&
          videoDecodeCheckbox &&
          videoDecodeCheckbox->isChecked())) &&
        !suppressServerLocalOutput;
    const bool lrptEnabled =
        videoDock &&
        videoDock->isVisible() &&
        pendingSettings.modulationType == MOD_LRPT &&
        (lrptTest ||
         (videoDecodeEnabled &&
          videoDecodeCheckbox &&
          videoDecodeCheckbox->isChecked())) &&
        !suppressServerLocalOutput;
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, analogVideoTest]() {
                                  processor->setTestPatternEnabled(analogVideoTest);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, sstvEnabled]() {
                                  processor->configureSstv(sstvEnabled);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, sstvTest]() {
                                  processor->setSstvTestPatternEnabled(sstvTest);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, aptEnabled]() {
                                  processor->configureApt(aptEnabled);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, aptTest]() {
                                  processor->setAptTestPatternEnabled(aptTest);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, wefaxEnabled]() {
                                  processor->configureWefax(wefaxEnabled);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, wefaxTest]() {
                                  processor->setWefaxTestPatternEnabled(wefaxTest);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, lrptEnabled]() {
                                  processor->configureLrpt(lrptEnabled);
                              },
                              Qt::QueuedConnection);

    const bool channelIqStreamMode =
        networkMode != NetworkMode::Disabled &&
        isChannelIqProcessingMode();
    const bool snapshotVideoEnabled = iqVideoEnabled && !channelIqStreamMode;
    if (videoSnapshotTimer) {
        if (snapshotVideoEnabled && !videoSnapshotTimer->isActive()) {
            videoSnapshotTimer->start();
        } else if (!snapshotVideoEnabled && videoSnapshotTimer->isActive()) {
            videoSnapshotTimer->stop();
        }
    }
    if (sstvTest) {
        if (videoStatusLabel) {
            videoStatusLabel->setText(QStringLiteral("SSTV Robot36 test stream"));
        }
    }
    if (aptTest) {
        if (videoStatusLabel) {
            videoStatusLabel->setText(QStringLiteral("NOAA APT test stream"));
        }
    }
    if (wefaxTest) {
        if (videoStatusLabel) {
            videoStatusLabel->setText(QStringLiteral("WEFAX test stream"));
        }
    }
    if (lrptTest) {
        if (videoStatusLabel) {
            videoStatusLabel->setText(QStringLiteral("Meteor LRPT beta: QPSK monitor test"));
        }
    }
    if (!iqVideoEnabled && !sstvTest && !aptTest && !wefaxTest && !lrptEnabled && videoWidget) {
        videoWidget->clearFrame();
    }
    if (pendingSettings.modulationType == MOD_SSTV && !sstvTest && videoStatusLabel) {
        videoStatusLabel->setText(QStringLiteral("SSTV: image decoder setup ready"));
    }
    if (pendingSettings.modulationType == MOD_APT && !aptTest && videoStatusLabel) {
        videoStatusLabel->setText(QStringLiteral("NOAA APT: image decoder setup ready"));
    }
    if (pendingSettings.modulationType == MOD_WEFAX && !wefaxTest && videoStatusLabel) {
        videoStatusLabel->setText(QStringLiteral("WEFAX: image decoder setup ready"));
    }
    if (pendingSettings.modulationType == MOD_LRPT && !lrptTest && videoStatusLabel) {
        videoStatusLabel->setText(QStringLiteral("Meteor LRPT beta: QPSK IQ monitor ready"));
    }
    const int demodMode = videoDemodCombo ? videoDemodCombo->currentData().toInt()
                                          : VideoProcessor::FmVideo;
    const double lineRate = videoStandardCombo ? videoStandardCombo->currentData().toDouble()
                                               : 15625.0;
    const bool invertVideo = videoInvertCheckbox && videoInvertCheckbox->isChecked();
    const bool hSyncEnabled = !videoHSyncCheckbox || videoHSyncCheckbox->isChecked();
    const bool vSyncEnabled = !videoVSyncCheckbox || videoVSyncCheckbox->isChecked();
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor,
                               analogVideoEnabled,
                               demodMode,
                               lineRate,
                               invertVideo,
                               hSyncEnabled,
                               vSyncEnabled]() {
                                  processor->configure(analogVideoEnabled,
                                                       demodMode,
                                                       lineRate,
                                                       384,
                                                       288,
                                                       invertVideo,
                                                       hSyncEnabled,
                                                       vSyncEnabled);
                              },
                              Qt::QueuedConnection);
}

RadioSettings YourClassName::audioProcessorSettings() const {
    RadioSettings settings = pendingSettings;
    if (dmrBasebandRateCombo) {
        settings.dmrBasebandSampleRate =
            normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt());
    } else {
        settings.dmrBasebandSampleRate =
            normalizedDmrBasebandSampleRate(settings.dmrBasebandSampleRate);
    }
    if (dmrAmbeLayoutCombo) {
        settings.dmrAmbeLayout =
            normalizedDmrAmbeLayout(dmrAmbeLayoutCombo->currentData().toInt());
    } else {
        settings.dmrAmbeLayout =
            normalizedDmrAmbeLayout(settings.dmrAmbeLayout);
    }
    if (offlineIqPlaybackActive && offlineIqPlaybackSampleRate > 0.0) {
        settings.sampleRate = offlineIqPlaybackSampleRate;
        settings.centerFrequency = pendingSettings.listeningFrequency;
        settings.actualFrequency = pendingSettings.listeningFrequency;
        settings.inputMode = INPUT_RF;
    }
    return settings;
}

RadioSettings YourClassName::spectrumProcessingSettings() const {
    RadioSettings settings = pendingSettings;
    if (dmrBasebandRateCombo) {
        settings.dmrBasebandSampleRate =
            normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt());
    } else {
        settings.dmrBasebandSampleRate =
            normalizedDmrBasebandSampleRate(settings.dmrBasebandSampleRate);
    }
    if (dmrAmbeLayoutCombo) {
        settings.dmrAmbeLayout =
            normalizedDmrAmbeLayout(dmrAmbeLayoutCombo->currentData().toInt());
    } else {
        settings.dmrAmbeLayout =
            normalizedDmrAmbeLayout(settings.dmrAmbeLayout);
    }
    if (offlineIqPlaybackActive && offlineIqPlaybackSampleRate > 0.0) {
        settings.sampleRate = offlineIqPlaybackSampleRate;
        settings.centerFrequency = pendingSettings.listeningFrequency;
        settings.actualFrequency = pendingSettings.listeningFrequency;
        settings.inputMode = INPUT_RF;
    }
    return settings;
}

void YourClassName::startRecording(bool momentary) {
    if (!recordingManager) {
        return;
    }
    if (recordingManager->isRecording()) {
        if (momentary) {
            momentaryRecordingActive = false;
        }
        return;
    }

    const RecordingManager::Mode mode = selectedRecordingMode();
    if (mode == RecordingManager::Mode::ChannelIqWav &&
        networkMode != NetworkMode::Disabled &&
        isFullIqProcessingMode() &&
        runState == RadioRunState::Running) {
        updateRecordingStatus(QStringLiteral("Recording blocked: Channel IQ cannot run during Full IQ streaming"));
        if (recordButton) {
            QSignalBlocker blocker(recordButton);
            recordButton->setChecked(false);
            recordButton->setText(uiText(QStringLiteral("record"), QStringLiteral("Record")));
        }
        momentaryRecordingActive = false;
        return;
    }

    QString errorMessage;
    recordingManager->setDisplayScalePercent(currentScale);
    recordingManager->setLabMetadata(recordingLabMetadata());
    if (!recordingManager->start(mode, pendingSettings, &errorMessage)) {
        updateRecordingStatus(QStringLiteral("Recording failed: %1").arg(errorMessage));
        if (recordButton) {
            QSignalBlocker blocker(recordButton);
            recordButton->setChecked(false);
            recordButton->setText(uiText(QStringLiteral("record"), QStringLiteral("Record")));
        }
        momentaryRecordingActive = false;
        return;
    }

    momentaryRecordingActive = momentary;
    if (recordButton) {
        QSignalBlocker blocker(recordButton);
        recordButton->setChecked(true);
        recordButton->setText(momentary
                                  ? uiText(QStringLiteral("hold_f9"), QStringLiteral("Hold F9"))
                                  : uiText(QStringLiteral("stop_rec"), QStringLiteral("Stop Rec")));
    }
    if (recordingModeCombo) {
        recordingModeCombo->setEnabled(false);
    }
    if (mode == RecordingManager::Mode::ChannelIqWav) {
        if (runState == RadioRunState::Running && !isNetworkClientMode()) {
            qDebug() << "[Recording] restarting receiver to apply Channel IQ recording policy";
            if (!restartStreamForHardwareChange()) {
                qDebug() << "[Recording] Channel IQ recording restart failed; stopping recording";
                stopRecording(false);
                updateRecordingStatus(QStringLiteral("Recording failed: receiver restart failed"));
                return;
            }
        } else {
            updateIqFrameProducerSettings();
        }
    }
}

void YourClassName::stopRecording(bool momentaryRelease) {
    if (!recordingManager) {
        return;
    }
    if (momentaryRelease && !momentaryRecordingActive) {
        return;
    }

    const bool wasChannelIqRecording = isChannelIqRecordingActive();
    momentaryRecordingActive = false;
    recordingManager->stop();
    if (recordButton) {
        QSignalBlocker blocker(recordButton);
        recordButton->setChecked(false);
        recordButton->setText(uiText(QStringLiteral("record"), QStringLiteral("Record")));
    }
    if (recordingModeCombo) {
        recordingModeCombo->setEnabled(true);
    }
    if (wasChannelIqRecording) {
        if (runState == RadioRunState::Running && !isNetworkClientMode()) {
            qDebug() << "[Recording] restarting receiver to restore live audio after Channel IQ recording";
            restartStreamForHardwareChange();
        } else {
            updateIqFrameProducerSettings();
        }
    }
}

void YourClassName::refreshPlaybackFiles() {
    if (!playbackFileCombo) {
        return;
    }

    const QString previousPath = selectedPlaybackFilePath();
    playbackFileCombo->blockSignals(true);
    playbackFileCombo->clear();

    QDir recordingsDir(QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("recordings")));
    const QFileInfoList files = recordingsDir.entryInfoList(QStringList() << QStringLiteral("*.wav"),
                                                            QDir::Files,
                                                            QDir::Time);
    for (const QFileInfo &fileInfo : files) {
        PlaybackManager::WavInfo info;
        if (!PlaybackManager::readWavInfo(fileInfo.absoluteFilePath(), info)) {
            continue;
        }

        const double seconds = info.dataSize / static_cast<double>(
            info.sampleRate * info.channels * (info.bitsPerSample / 8));
        const QString type = info.mode == PlaybackManager::Mode::AudioWav
                                 ? QStringLiteral("Audio")
                                 : QStringLiteral("Ch IQ");
        const QString label = QStringLiteral("%1  %2  %3 Hz  %4 s")
                                  .arg(fileInfo.fileName(),
                                       type,
                                       QString::number(info.sampleRate),
                                       QString::number(seconds, 'f', 1));
        playbackFileCombo->addItem(label, fileInfo.absoluteFilePath());
    }

    if (playbackFileCombo->count() == 0) {
        playbackFileCombo->addItem(uiText(QStringLiteral("no_wav_recordings_found"),
                                          QStringLiteral("No WAV recordings found")),
                                   QString());
        playbackFileCombo->setEnabled(false);
        if (playbackButton) {
            playbackButton->setEnabled(false);
        }
    } else {
        playbackFileCombo->setEnabled(true);
        if (playbackButton) {
            playbackButton->setEnabled(true);
        }
        const int previousIndex = playbackFileCombo->findData(previousPath);
        if (previousIndex >= 0) {
            playbackFileCombo->setCurrentIndex(previousIndex);
        }
    }

    playbackFileCombo->blockSignals(false);
}

void YourClassName::startPlayback() {
    if (!playbackManager) {
        return;
    }
    if (playbackManager->isPlaying()) {
        return;
    }
    if (runState != RadioRunState::Idle || deviceOpened || (processor && processor->isRunning())) {
        onPlaybackStatusChanged(QStringLiteral("Playback blocked: stop receiver first"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }
    if (recordingManager && recordingManager->isRecording()) {
        onPlaybackStatusChanged(QStringLiteral("Playback blocked: stop recording first"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }

    const QString path = selectedPlaybackFilePath();
    if (path.isEmpty()) {
        onPlaybackStatusChanged(QStringLiteral("Playback: no file selected"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }

    PlaybackManager::WavInfo info;
    QString errorMessage;
    if (!PlaybackManager::readWavInfo(path, info, &errorMessage)) {
        onPlaybackStatusChanged(QStringLiteral("Playback failed: %1").arg(errorMessage));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }
    if (info.mode == PlaybackManager::Mode::AudioWav && (info.channels != 1 || info.sampleRate != 48000)) {
        onPlaybackStatusChanged(QStringLiteral("Playback failed: audio WAV must be mono 48 kHz"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }

    if (!playbackManager->start(path, &errorMessage)) {
        onPlaybackStatusChanged(QStringLiteral("Playback failed: %1").arg(errorMessage));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
    }
}

void YourClassName::stopPlayback() {
    if (playbackManager && playbackManager->isPlaying()) {
        playbackManager->stop();
        return;
    }
    if (playbackButton && playbackButton->isChecked()) {
        QSignalBlocker blocker(playbackButton);
        playbackButton->setChecked(false);
        playbackButton->setText(uiText(QStringLiteral("play"), QStringLiteral("Play")));
    }
}

void YourClassName::onPlaybackStarted(const QString &filePath, PlaybackManager::WavInfo info) {
    Q_UNUSED(filePath);
    if (playbackButton) {
        QSignalBlocker blocker(playbackButton);
        playbackButton->setChecked(true);
        playbackButton->setText(uiText(QStringLiteral("stop_play"), QStringLiteral("Stop Play")));
    }
    if (playbackFileCombo) {
        playbackFileCombo->setEnabled(false);
    }
    if (playbackRefreshButton) {
        playbackRefreshButton->setEnabled(false);
    }
    if (startButton) {
        startButton->setEnabled(false);
    }

    if (info.mode == PlaybackManager::Mode::ChannelIqWav) {
        if (!playbackSettingsSaved) {
            settingsBeforePlayback = pendingSettings;
            playbackSettingsSaved = true;
        }
        offlineIqPlaybackActive = true;
        offlineIqPlaybackHasMetadata = info.hasRadioSettings;
        offlineIqPlaybackSampleRate = info.sampleRate;
        IqBuffer::clear();
        IqBuffer::setSampleRateEstimate(info.sampleRate);
        const bool audioEnabledBeforePlayback = pendingSettings.audioEnabled;
        const int audioDeviceBeforePlayback = pendingSettings.audioDeviceId;
        if (info.hasRadioSettings) {
            pendingSettings.deviceIndex = info.radioSettings.deviceIndex;
            pendingSettings.clockSource = info.radioSettings.clockSource;
            pendingSettings.inputMode = info.radioSettings.inputMode;
            pendingSettings.centerFrequency = info.radioSettings.centerFrequency;
            pendingSettings.actualFrequency = info.radioSettings.actualFrequency;
            pendingSettings.listeningFrequency = info.radioSettings.listeningFrequency;
            pendingSettings.sampleRate = info.radioSettings.sampleRate;
            pendingSettings.bandwidth = info.radioSettings.bandwidth;
            pendingSettings.modulationType = info.radioSettings.modulationType;
            pendingSettings.fftLength = info.radioSettings.fftLength;
            if (info.radioSettings.lnaGain >= 0) {
                pendingSettings.lnaGain = info.radioSettings.lnaGain;
            }
            if (info.radioSettings.vgaGain >= 0) {
                pendingSettings.vgaGain = info.radioSettings.vgaGain;
            }
            const double playbackCenter = info.radioSettings.listeningFrequency > 0.0
                                              ? info.radioSettings.listeningFrequency
                                              : info.radioSettings.centerFrequency;
            if (playbackCenter > 0.0) {
                pendingSettings.listeningFrequency = playbackCenter;
            }
        } else {
            pendingSettings.sampleRate = info.sampleRate;
            pendingSettings.centerFrequency = pendingSettings.listeningFrequency;
            pendingSettings.actualFrequency = pendingSettings.listeningFrequency;
            pendingSettings.inputMode = INPUT_RF;
        }
        pendingSettings.audioEnabled = audioEnabledBeforePlayback;
        pendingSettings.audioDeviceId = audioDeviceBeforePlayback;
        if (info.hasScalePercent) {
            currentScale = std::clamp(info.scalePercent,
                                      static_cast<double>(minScale) / 10.0,
                                      static_cast<double>(maxScale) / 10.0);
        } else if (info.hasRadioSettings &&
                   info.radioSettings.sampleRate > 0.0 &&
                   info.sampleRate > 0) {
            const double channelScalePercent =
                (static_cast<double>(info.sampleRate) / info.radioSettings.sampleRate) * 100.0;
            currentScale = std::clamp(channelScalePercent,
                                      static_cast<double>(minScale) / 10.0,
                                      static_cast<double>(maxScale) / 10.0);
        }
        publishSettingsToGlobals();
        updateUiFromPendingSettings();
        fftResult = std::make_unique<FFTResult>();
        updateSpectrumTimerInterval();
        if (audioProcessor) {
            audioProcessor->configure(audioProcessorSettings());
        }
        pendingPlaybackAudioStartAfterIqPrebuffer = pendingSettings.audioEnabled;
        if (updateTimer) {
            updateTimer->start();
        }
        settingRange();
    } else {
        offlineIqPlaybackActive = false;
        offlineIqPlaybackHasMetadata = false;
        offlineIqPlaybackSampleRate = 0.0;
    }
}

void YourClassName::onPlaybackStopped() {
    if (offlineIqPlaybackActive) {
        offlineIqPlaybackActive = false;
        offlineIqPlaybackHasMetadata = false;
        offlineIqPlaybackSampleRate = 0.0;
        pendingPlaybackAudioStartAfterIqPrebuffer = false;
        if (audioProcessor) {
            audioProcessor->stopDemodulation();
        }
        if (updateTimer && runState == RadioRunState::Idle) {
            updateTimer->stop();
        }
    }
    if (playbackSettingsSaved) {
        pendingSettings = settingsBeforePlayback;
        playbackSettingsSaved = false;
        publishSettingsToGlobals();
        updateUiFromPendingSettings();
    }
    if (remoteAudioPlayer && runState == RadioRunState::Idle) {
        remoteAudioPlayer->stop();
    }
    if (playbackButton) {
        QSignalBlocker blocker(playbackButton);
        playbackButton->setChecked(false);
        playbackButton->setText(uiText(QStringLiteral("play"), QStringLiteral("Play")));
    }
    if (playbackFileCombo) {
        playbackFileCombo->setEnabled(playbackFileCombo->count() > 0 && !selectedPlaybackFilePath().isEmpty());
    }
    if (playbackRefreshButton) {
        playbackRefreshButton->setEnabled(true);
    }
    updateUiForRunState();
}

void YourClassName::onPlaybackStatusChanged(const QString &status) {
    if (playbackStatusLabel) {
        playbackStatusLabel->setProperty("statusRawText", status);
        playbackStatusLabel->setText(localizedStatusText(status));
    }
}

void YourClassName::handlePlaybackAudioFrame(const QByteArray &pcmData) {
    processDigitalAudioFrame(pcmData);
    processSstvAudioFrame(pcmData);
    processAptAudioFrame(pcmData);
    processWefaxAudioFrame(pcmData);
    sendAudioRelayFrame(pcmData);
    sendAudioHttpFrame(pcmData);
    if (remoteAudioPlayer) {
        remoteAudioPlayer->playPcmFrame(pcmData);
    }
}

void YourClassName::handlePlaybackIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount) {
    if (iqData.size() < static_cast<int>(2 * sizeof(qint16))) {
        return;
    }
    processVideoIqFrame(iqData,
                        sampleRate,
                        sampleCount > 0 ? sampleCount : iqData.size() / (2 * static_cast<int>(sizeof(qint16))));
    const int bytesPerIq = 2 * static_cast<int>(sizeof(qint16));
    std::vector<float> floatSamples(static_cast<std::size_t>(iqData.size() / sizeof(qint16)));
    const auto *src = reinterpret_cast<const uchar *>(iqData.constData());
    for (int i = 0, out = 0; i + bytesPerIq - 1 < iqData.size(); i += bytesPerIq) {
        const qint16 iSample = static_cast<qint16>(src[i] | (src[i + 1] << 8));
        const qint16 qSample = static_cast<qint16>(src[i + 2] | (src[i + 3] << 8));
        floatSamples[static_cast<std::size_t>(out++)] = iSample / 32768.0f;
        floatSamples[static_cast<std::size_t>(out++)] = qSample / 32768.0f;
    }
    IqBuffer::setSampleRateEstimate(sampleRate);
    IqBuffer::publish(floatSamples.data(), floatSamples.size(), pendingSettings.audioEnabled);
    if (pendingPlaybackAudioStartAfterIqPrebuffer && pendingSettings.audioEnabled && audioProcessor && sampleRate > 0.0) {
        const double queuedIqSamples = static_cast<double>(IqBuffer::queuedFloatCount()) / 2.0;
        const double queuedSeconds = queuedIqSamples / sampleRate;
        if (queuedSeconds >= NETWORK_AUDIO_PREBUFFER_SECONDS) {
            pendingPlaybackAudioStartAfterIqPrebuffer = false;
            qDebug() << "[PlaybackIQ] starting demodulator after IQ prebuffer"
                     << "queuedSeconds" << queuedSeconds
                     << "queuedBlocks" << IqBuffer::queuedBlocks()
                     << "sampleRate" << sampleRate;
            audioProcessor->startDemodulation();
        }
    }
}

void YourClassName::sendNetworkIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount) {
    if (networkMode != NetworkMode::Server ||
        !isClientIqProcessingMode() ||
        !networkController ||
        !networkController->isControlReady() ||
        iqData.isEmpty()) {
        return;
    }

    const bool fullIqMode = isFullIqProcessingMode();
    const qint64 pendingLimit = fullIqMode
                                    ? NETWORK_IQ_MAX_PENDING_BYTES
                                    : NETWORK_CHANNEL_IQ_LOW_LATENCY_PENDING_BYTES;
    const qint64 pendingBytes = networkController->pendingBytes();
    if (pendingBytes > pendingLimit) {
        ++networkIqFramesDropped;
        if (networkIqFramesDropped == 1 ||
            (networkIqFramesDropped % NETWORK_IQ_DROP_LOG_INTERVAL) == 0) {
            qDebug() << "[NetworkIQ] dropping IQ frame because TCP queue is above low-latency limit"
                     << "dropped" << networkIqFramesDropped
                     << "pendingBytes" << pendingBytes
                     << "pendingLimit" << pendingLimit
                     << "frameBytes" << iqData.size();
        }
        return;
    }

    QJsonObject frame;
    frame["type"] = "iqbin";
    frame["sequence"] = QString::number(++networkIqFrameSequence);
    frame["sampleRate"] = sampleRate;
    frame["sourceSampleRate"] = pendingSettings.sampleRate;
    frame["sampleCount"] = sampleCount;
    frame["sampleFormat"] = isChannelIqProcessingMode() ? "channel_iq_s16le" : "iq_s8_interleaved";
    frame["channelized"] = isChannelIqProcessingMode();
    frame["centerFrequency"] = isChannelIqProcessingMode()
                                   ? pendingSettings.listeningFrequency
                                   : pendingSettings.centerFrequency;
    frame["actualFrequency"] = isChannelIqProcessingMode()
                                   ? pendingSettings.listeningFrequency
                                   : pendingSettings.actualFrequency;
    frame["listeningFrequency"] = pendingSettings.listeningFrequency;
    frame["bandwidth"] = pendingSettings.bandwidth;
    frame["modulationType"] = pendingSettings.modulationType;
    frame["inputMode"] = isChannelIqProcessingMode() ? 0 : pendingSettings.inputMode;
    frame["payloadEncoding"] = "raw";

    networkController->sendBinaryCommand(frame, iqData);
}

void YourClassName::receiveNetworkIqFrame(const QJsonObject &frame) {
    QByteArray iqBytes = QByteArray::fromBase64(frame.value("iq").toString().toLatin1());
    handleNetworkIqPayload(frame, std::move(iqBytes));
}

void YourClassName::receiveNetworkIqFrameBinary(const QJsonObject &frame, const QByteArray &iqData) {
    handleNetworkIqPayload(frame, iqData);
}

void YourClassName::handleNetworkIqPayload(const QJsonObject &frame, QByteArray iqBytes) {
    if (networkMode != NetworkMode::Client || !isClientIqProcessingMode()) {
        return;
    }
    if (runState == RadioRunState::Idle &&
        networkController &&
        !networkController->clientHasControl()) {
        runState = RadioRunState::Running;
        updateUiForRunState();
        startNetworkClientProcessing();
    }
    if (runState == RadioRunState::Idle) {
        return;
    }

    const bool channelizedFrame = frame.value("channelized").toBool(false);
    const QString sampleFormat = frame.value("sampleFormat").toString();
    if ((!channelizedFrame && sampleFormat != QStringLiteral("iq_s8_interleaved")) ||
        (channelizedFrame && sampleFormat != QStringLiteral("channel_iq_s16le"))) {
        qDebug() << "[NetworkIQ] unsupported IQ frame format";
        return;
    }
    if (channelizedFrame != isChannelIqProcessingMode()) {
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[NetworkIQ] dropping IQ frame from previous processing mode"
                     << "frameChannelized" << channelizedFrame
                     << "currentProcessingMode" << static_cast<int>(networkProcessingMode);
        }
        return;
    }

    const int bytesPerFloat = channelizedFrame ? static_cast<int>(sizeof(qint16)) : 1;
    const int bytesPerIqSample = 2 * bytesPerFloat;
    if (iqBytes.size() < bytesPerIqSample) {
        return;
    }
    const int remainder = iqBytes.size() % bytesPerIqSample;
    if (remainder != 0) {
        iqBytes.chop(remainder);
    }

    const double frameSampleRate = frame.value("sampleRate").toDouble(pendingSettings.sampleRate);
    if (frameSampleRate <= 0.0) {
        return;
    }
    const int frameSampleCount = frame.value("sampleCount").toInt(iqBytes.size() / bytesPerIqSample);
    processVideoIqFrame(iqBytes, frameSampleRate, frameSampleCount);
    if (channelizedFrame &&
        recordingManager &&
        recordingManager->isRecording() &&
        recordingManager->mode() == RecordingManager::Mode::ChannelIqWav) {
        recordingManager->appendIqFrame(iqBytes,
                                        frameSampleRate,
                                        frameSampleCount);
    }

    RadioSettings iqSettings = pendingSettings;
    iqSettings.sampleRate = frameSampleRate;
    iqSettings.centerFrequency = frame.value("centerFrequency").toDouble(iqSettings.centerFrequency);
    iqSettings.actualFrequency = frame.value("actualFrequency").toDouble(iqSettings.actualFrequency);
    iqSettings.listeningFrequency = frame.value("listeningFrequency").toDouble(iqSettings.listeningFrequency);
    iqSettings.bandwidth = frame.value("bandwidth").toDouble(iqSettings.bandwidth);
    iqSettings.modulationType = frame.value("modulationType").toInt(iqSettings.modulationType);
    iqSettings.inputMode = frame.value("inputMode").toInt(iqSettings.inputMode);

    if (channelizedFrame) {
        iqSettings.inputMode = INPUT_RF;
        iqSettings.centerFrequency = iqSettings.listeningFrequency;
        iqSettings.actualFrequency = iqSettings.listeningFrequency;
        if (audioProcessor) {
            audioProcessor->configure(iqSettings);
        }
    } else {
        const bool protectLocalControlSettings =
            networkController &&
            networkController->clientHasControl() &&
            networkClientSettingsGuardTimer.isValid() &&
            networkClientSettingsGuardTimer.elapsed() < NETWORK_CLIENT_SETTINGS_GUARD_MS;
        bool processingSettingsChanged = false;
        auto updateDouble = [&processingSettingsChanged](double &target, double value) {
            if (std::abs(target - value) > 0.5) {
                target = value;
                processingSettingsChanged = true;
            }
        };
        auto updateInt = [&processingSettingsChanged](int &target, int value) {
            if (target != value) {
                target = value;
                processingSettingsChanged = true;
            }
        };

        if (!protectLocalControlSettings) {
            updateDouble(pendingSettings.sampleRate, iqSettings.sampleRate);
            updateDouble(pendingSettings.centerFrequency, iqSettings.centerFrequency);
            updateDouble(pendingSettings.actualFrequency, iqSettings.actualFrequency);
            updateDouble(pendingSettings.listeningFrequency, iqSettings.listeningFrequency);
            updateDouble(pendingSettings.bandwidth, iqSettings.bandwidth);
            updateInt(pendingSettings.modulationType, iqSettings.modulationType);
            updateInt(pendingSettings.inputMode, iqSettings.inputMode);
        } else if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[NetworkIQ] preserving local controller tuning while settings command settles"
                     << "frameCenter" << iqSettings.centerFrequency
                     << "frameListening" << iqSettings.listeningFrequency
                     << "localCenter" << pendingSettings.centerFrequency
                     << "localListening" << pendingSettings.listeningFrequency
                     << "guardMs" << networkClientSettingsGuardTimer.elapsed();
        }

        if (processingSettingsChanged && !protectLocalControlSettings) {
            publishSettingsToGlobals();
            settingRange();
        }
    }

    const bool streamShapeChanged =
        !networkIqStreamMetadataValid ||
        networkIqStreamWasChannelized != channelizedFrame ||
        std::abs(networkIqStreamSampleRate - frameSampleRate) > 0.5 ||
        networkIqStreamInputMode != iqSettings.inputMode ||
        (!channelizedFrame &&
         std::abs(networkIqStreamCenterFrequency - iqSettings.centerFrequency) > 0.5) ||
        (channelizedFrame &&
         std::abs(networkIqStreamListeningFrequency - iqSettings.listeningFrequency) > 0.5);

    if (streamShapeChanged) {
        qDebug() << "[NetworkIQ] receiver stream shape changed; resetting client IQ buffers"
                 << "channelized" << channelizedFrame
                 << "sampleRate" << frameSampleRate
                 << "center" << iqSettings.centerFrequency
                 << "listening" << iqSettings.listeningFrequency
                 << "inputMode" << iqSettings.inputMode;
        resetNetworkIqReceptionState(false, false, pendingSettings.audioEnabled && !isFullIqProcessingMode());
        networkIqStreamMetadataValid = true;
        networkIqStreamWasChannelized = channelizedFrame;
        networkIqStreamSampleRate = frameSampleRate;
        networkIqStreamCenterFrequency = iqSettings.centerFrequency;
        networkIqStreamListeningFrequency = iqSettings.listeningFrequency;
        networkIqStreamInputMode = iqSettings.inputMode;
    }

    std::vector<float> floatSamples(static_cast<std::size_t>(iqBytes.size() / bytesPerFloat));
    if (channelizedFrame) {
        const auto *src = reinterpret_cast<const uchar *>(iqBytes.constData());
        for (int i = 0, out = 0; i + 1 < iqBytes.size(); i += 2, ++out) {
            const qint16 value = static_cast<qint16>(src[i] | (src[i + 1] << 8));
            floatSamples[static_cast<std::size_t>(out)] = static_cast<float>(value) / 32767.0f;
        }
    } else {
        const auto *src = reinterpret_cast<const signed char *>(iqBytes.constData());
        for (int i = 0; i < iqBytes.size(); ++i) {
            floatSamples[static_cast<std::size_t>(i)] = static_cast<float>(src[i]) / 127.0f;
        }
    }

    const bool queueClientAudioIq = pendingSettings.audioEnabled && !isFullIqProcessingMode();
    IqBuffer::setSampleRateEstimate(frameSampleRate);
    IqBuffer::publish(floatSamples.data(), floatSamples.size(), queueClientAudioIq);
    if (isFullIqProcessingMode() && updateTimer && !updateTimer->isActive()) {
        qDebug() << "[NetworkIQ] restarting client spectrum timer after IQ frame";
        updateTimer->start();
    }

    if (pendingNetworkAudioStartAfterIqPrebuffer && pendingSettings.audioEnabled && audioProcessor) {
        const double queuedIqSamples = static_cast<double>(IqBuffer::queuedFloatCount()) / 2.0;
        const double queuedSeconds = queuedIqSamples / frameSampleRate;
        if (queuedSeconds >= NETWORK_AUDIO_PREBUFFER_SECONDS) {
            pendingNetworkAudioStartAfterIqPrebuffer = false;
            qDebug() << "[NetworkIQ] starting client demodulator after IQ prebuffer"
                     << "queuedSeconds" << queuedSeconds
                     << "queuedBlocks" << IqBuffer::queuedBlocks()
                     << "sampleRate" << frameSampleRate;
            audioProcessor->startDemodulation();
        }
    }
}

void YourClassName::updateUiForRunState() {
    const bool idle = isIdle();
    const bool clientCanControl =
        !isNetworkClientMode() ||
        !networkController ||
        networkController->clientHasControl();

    if (startButton) startButton->setEnabled(idle && clientCanControl);
    if (stopButton) {
        const bool canStop =
            runState == RadioRunState::Starting ||
            runState == RadioRunState::Running;
        stopButton->setEnabled(canStop && (clientCanControl || isNetworkClientMode()));
    }
    if (comboBox) comboBox->setEnabled(idle);
    if (refreshButton) refreshButton->setEnabled(idle);
    if (fobosButton) fobosButton->setEnabled(idle);
    if (modeBox) modeBox->setEnabled(true);
    if (sampleBox) sampleBox->setEnabled(true);
    if (clkBox) clkBox->setEnabled(idle);
    if (fftComboBox) fftComboBox->setEnabled(idle || runState == RadioRunState::Running);
    if (audioDeviceComboBox) audioDeviceComboBox->setEnabled(idle);
    const bool liveAudioControlsEnabled =
        idle || runState == RadioRunState::Running;

    if (audioCheckbox) audioCheckbox->setEnabled(liveAudioControlsEnabled);
    if (syncCheckbox) syncCheckbox->setEnabled(false);
    const bool liveDemodControlsEnabled =
        idle || runState == RadioRunState::Running;

    if (bandwidthControl) bandwidthControl->setEnabled(liveDemodControlsEnabled);
    const bool gainControlsEnabled =
        idle || runState == RadioRunState::Running;

    if (lnaGainSlider) lnaGainSlider->setEnabled(gainControlsEnabled);
    if (vgaGainSlider) vgaGainSlider->setEnabled(gainControlsEnabled);
    const bool gpioEnabled =
        idle || runState == RadioRunState::Running;
    for (int i = 0; i < 8; ++i) {
        if (checkBoxes[i]) {
            checkBoxes[i]->setEnabled(gpioEnabled);
        }
    }
}

void YourClassName::onAudioEnabledChanged(bool checked) {
    pendingSettings.audioEnabled = checked;
    publishSettingsToGlobals();

    qDebug() << "[Audio] checkbox changed" << checked
             << "networkMode" << static_cast<int>(networkMode)
             << "processingMode" << static_cast<int>(networkProcessingMode)
             << "runState" << runStateName(runState);

    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();

        if (isClientIqProcessingMode()) {
            if (checked) {
                if (isFullIqProcessingMode()) {
                    pendingNetworkAudioStartAfterIqPrebuffer = false;
                    if (audioProcessor) {
                        audioProcessor->stopDemodulation();
                    }
                } else {
                    IqBuffer::clear();
                    pendingNetworkAudioStartAfterIqPrebuffer = true;
                }
            } else {
                pendingNetworkAudioStartAfterIqPrebuffer = false;
                if (audioProcessor) {
                    audioProcessor->stopDemodulation();
                }
                if (remoteAudioPlayer) {
                    remoteAudioPlayer->stop();
                }
            }
        } else {
            if (!checked && remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
        }

        return;
    }

    if (offlineIqPlaybackActive) {
        if (checked) {
            if (audioProcessor) {
                audioProcessor->configure(audioProcessorSettings());
            }
            pendingPlaybackAudioStartAfterIqPrebuffer = true;
        } else if (audioProcessor) {
            pendingPlaybackAudioStartAfterIqPrebuffer = false;
            audioProcessor->stopDemodulation();
        }
        return;
    }

    if (isIdle()) {
        return;
    }

    if (checked) {
        if (processor && processor->isRunning() && audioProcessor) {
            audioProcessor->startDemodulation();
        } else {
            pendingAudioStartAfterStreamReady = true;
        }
    } else {
        pendingAudioStartAfterStreamReady = false;
        if (audioProcessor) {
            audioProcessor->stopDemodulation();
        }
    }
}

void YourClassName::updateSpectrumTimerInterval() {
    if (!updateTimer) {
        return;
    }

    if (spectrumUpdateIntervalMs > SPECTRUM_UPDATE_AUTO_MS) {
        updateTimer->setInterval((std::clamp)(spectrumUpdateIntervalMs,
                                              SPECTRUM_UPDATE_MIN_MS,
                                              SPECTRUM_UPDATE_MAX_MS));
        return;
    }

    int intervalMs = 33;
    if (pendingSettings.fftLength >= 262144) {
        intervalMs = 80;
    } else if (pendingSettings.fftLength >= 131072) {
        intervalMs = 50;
    } else if (pendingSettings.fftLength >= 65536) {
        intervalMs = 40;
    }
    updateTimer->setInterval(intervalMs);
}

void YourClassName::revertHardwareControlsToSettings() {
    if (modeBox) {
        modeBox->blockSignals(true);
        const int index = modeBox->findData(pendingSettings.inputMode);
        if (index >= 0) {
            modeBox->setCurrentIndex(index);
        }
        modeBox->blockSignals(false);
    }
    if (clkBox) {
        clkBox->blockSignals(true);
        const int index = clkBox->findData(pendingSettings.clockSource);
        if (index >= 0) {
            clkBox->setCurrentIndex(index);
        }
        clkBox->blockSignals(false);
    }
    if (sampleBox) {
        sampleBox->blockSignals(true);
        const int index = sampleBox->findData(pendingSettings.sampleRate);
        if (index >= 0) {
            sampleBox->setCurrentIndex(index);
        }
        sampleBox->blockSignals(false);
    }
    if (fftComboBox) {
        fftComboBox->blockSignals(true);
        fftComboBox->setCurrentText(QString::number(pendingSettings.fftLength));
        fftComboBox->blockSignals(false);
    }
    if (audioDeviceComboBox) {
        audioDeviceComboBox->blockSignals(true);
        const int index = audioDeviceComboBox->findData(pendingSettings.audioDeviceId);
        if (index >= 0) {
            audioDeviceComboBox->setCurrentIndex(index);
        }
        audioDeviceComboBox->blockSignals(false);
    }
}

bool YourClassName::openFobosSession() {
    int selectedDevice = pendingSettings.deviceIndex;
    if (selectedDevice < 0) {
        selectedDevice = 0;
    }
    if (availableFobosDevices.isEmpty()) {
        refreshFobosDeviceList();
    }
    FobosDeviceInfo selectedInfo = selectedFobosDeviceInfo();
    selectedDevice = selectedInfo.nativeIndex;

    qDebug() << "[FobosLifecycle] openFobosSession enter"
             << "logicalDevice" << pendingSettings.deviceIndex
             << "nativeDevice" << selectedDevice
             << "apiKind" << fobosApiKindName(selectedInfo.apiKind)
             << "device" << activeFobosDevice()
             << "openedDeviceIndex" << openedDeviceIndex
             << "openedNativeDeviceIndex" << openedNativeDeviceIndex
             << "openedApiKind" << fobosApiKindName(openedDeviceApiKind)
             << "appliedSampleRate" << appliedSampleRate
             << "pendingSampleRate" << pendingSettings.sampleRate
             << "sampleRateReopenRequired" << sampleRateReopenRequired
             << "fobosCloseKnownUnsafe" << fobosCloseKnownUnsafe;

    if (hasActiveFobosDevice() &&
        openedDeviceIndex == pendingSettings.deviceIndex &&
        openedNativeDeviceIndex == selectedInfo.nativeIndex &&
        openedDeviceApiKind == selectedInfo.apiKind) {
        if (selectedInfo.apiKind == FobosApiKind::Agile) {
            qDebug() << "[FobosLifecycle] reusing idle Agile session; settings will be applied in place"
                     << activeFobosDevice();
            return true;
        } else {
            qDebug() << "[FobosLifecycle] reusing idle Fobos session; settings will be applied in place"
                     << activeFobosDevice();
            return true;
        }
    }

    if (hasActiveFobosDevice()) {
        qDebug() << "[FobosLifecycle] closing mismatched existing Fobos session before open";
        if (!closeFobosSession(false)) {
            qDebug() << "[FobosLifecycle] existing Fobos session could not be closed; open aborted";
            return false;
        }
        qDebug() << "[FobosLifecycle] mismatched Fobos session closed before fresh open; waiting before reopen";
        QThread::msleep(350);
    }

    activeFobosApiKind = selectedInfo.apiKind;
    int ret = FOBOS_ERR_OK;
    if (selectedInfo.apiKind == FobosApiKind::Agile) {
        qDebug() << "[FobosLifecycle] fobos_sdr_open begin" << "selectedDevice" << selectedDevice;
        ret = openFobosAgileDeviceSafely(&agileDevice, static_cast<uint32_t>(selectedDevice));
        qDebug() << "[FobosLifecycle] fobos_sdr_open end" << "result" << ret << "device" << agileDevice;
        if (ret != FOBOS_ERR_OK || !agileDevice) {
            qDebug() << "[FobosLifecycle] fobos_sdr_open failed; waiting before one recovery retry"
                     << "result" << ret
                     << "selectedDevice" << selectedDevice;
            agileDevice = nullptr;
            QThread::msleep(700);
            ret = openFobosAgileDeviceSafely(&agileDevice, static_cast<uint32_t>(selectedDevice));
            qDebug() << "[FobosLifecycle] fobos_sdr_open retry end"
                     << "result" << ret
                     << "device" << agileDevice;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_open begin" << "selectedDevice" << selectedDevice;
        ret = openFobosDeviceSafely(&device, static_cast<uint32_t>(selectedDevice));
        qDebug() << "[FobosLifecycle] fobos_rx_open end" << "result" << ret << "device" << device;
    }

    if (ret != FOBOS_ERR_OK || !hasActiveFobosDevice()) {
        qDebug() << "Failed to open Fobos device, error code:" << ret;
        device = nullptr;
        agileDevice = nullptr;
        agileScanRunning = false;
        activeAgileScanFrequencies.clear();
        resetStandardScanState(true);
        activeFobosApiKind = FobosApiKind::Standard;
        openedDeviceIndex = -1;
        openedNativeDeviceIndex = -1;
        return false;
    }

    openedDeviceIndex = pendingSettings.deviceIndex;
    openedNativeDeviceIndex = selectedInfo.nativeIndex;
    openedDeviceApiKind = selectedInfo.apiKind;
    appliedHardwareSettings = RadioSettings{};
    hardwareSettingsApplied = false;
    sampleRateReopenRequired = false;
    fobosCloseKnownUnsafe = false;
    return true;
}

bool YourClassName::closeFobosSession(bool clearIq) {
    qDebug() << "[FobosLifecycle] closeFobosSession enter"
             << "clearIq" << clearIq
             << "device" << activeFobosDevice()
             << "openedDeviceIndex" << openedDeviceIndex
             << "openedNativeDeviceIndex" << openedNativeDeviceIndex
             << "apiKind" << fobosApiKindName(openedDeviceApiKind);
    bool closeOk = true;
    if (clearIq) {
        qDebug() << "[FobosLifecycle] clearing IQ buffer before close";
        IqBuffer::clear();
    }

    if (device) {
        qDebug() << "[FobosLifecycle] fobos_rx_close begin" << device;
        const int closeResult = closeFobosDeviceSafely(device);
        qDebug() << "[FobosLifecycle] fobos_rx_close end" << "result" << closeResult;
        closeOk = closeResult == FOBOS_ERR_OK;
        if (!closeOk) {
            qDebug() << "Fobos close returned error code:" << closeResult;
            qDebug() << "[FobosLifecycle] close failed; skipping synchronous USB reset on stop to keep UI responsive"
                     << device;
            qDebug() << "[FobosLifecycle] abandoning Fobos session pointer after close failure; next start will try a fresh open"
                     << device;
        }
        device = nullptr;
    }
    if (agileDevice) {
        const int scanning = isFobosAgileScanningSafely(agileDevice);
        if (agileScanRunning || scanning > 0) {
            qDebug() << "[FobosLifecycle] stopping Agile scan before close";
            const int scanStopResult = stopFobosAgileScanSafely(agileDevice);
            qDebug() << "[FobosLifecycle] fobos_sdr_stop_scan end"
                     << "result" << scanStopResult
                     << "isScanningBefore" << scanning;
            agileScanRunning = false;
            activeAgileScanFrequencies.clear();
            scanVisualAssembler.reset();
        }
        qDebug() << "[FobosLifecycle] fobos_sdr_close begin" << agileDevice;
        const int closeResult = closeFobosAgileDeviceSafely(agileDevice);
        qDebug() << "[FobosLifecycle] fobos_sdr_close end" << "result" << closeResult;
        closeOk = closeOk && closeResult == FOBOS_ERR_OK;
        if (closeResult != FOBOS_ERR_OK) {
            qDebug() << "Fobos agile close returned error code:" << closeResult;
            qDebug() << "[FobosLifecycle] close failed; skipping synchronous Agile USB reset on stop to keep UI responsive"
                     << agileDevice;
            qDebug() << "[FobosLifecycle] abandoning Agile session pointer after close failure; next start will try a fresh open"
                     << agileDevice;
        }
        agileDevice = nullptr;
    }
    openedDeviceIndex = -1;
    openedNativeDeviceIndex = -1;
    appliedSampleRate = 0.0;
    appliedHardwareSettings = RadioSettings{};
    hardwareSettingsApplied = false;
    sampleRateReopenRequired = false;
    fobosCloseKnownUnsafe = false;
    resetStandardScanState(true);
    activeFobosApiKind = FobosApiKind::Standard;
    openedDeviceApiKind = FobosApiKind::Standard;
    qDebug() << "[FobosLifecycle] closeFobosSession exit";
    return closeOk;
}

bool YourClassName::applyFobosSettings() {
    if (!hasActiveFobosDevice()) {
        qDebug() << "Cannot apply Fobos settings without an active device.";
        return false;
    }

    normalizeTuning(pendingSettings);
    publishSettingsToGlobals();
    const bool firstApply = !hardwareSettingsApplied;
    auto changedDouble = [](double a, double b) {
        return std::abs(a - b) > 0.5;
    };

    qDebug() << "[FobosLifecycle] applyFobosSettings enter"
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind)
             << "firstApply" << firstApply
             << "clock" << pendingSettings.clockSource
             << "inputMode" << pendingSettings.inputMode
             << "sampleRate" << pendingSettings.sampleRate
             << "centerFrequency" << pendingSettings.centerFrequency
             << "lna" << pendingSettings.lnaGain
             << "vga" << pendingSettings.vgaGain
             << "gpo" << pendingSettings.gpoValue;

    int result = FOBOS_ERR_OK;
    if (firstApply || appliedHardwareSettings.clockSource != pendingSettings.clockSource) {
        qDebug() << "[FobosLifecycle] set clock source begin";
        result = setActiveClockSourceSafely(pendingSettings.clockSource);
        qDebug() << "[FobosLifecycle] set clock source end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set clock source, error code:" << result;
            return false;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_clk_source skipped unchanged";
    }

    const int libfobosMode = (pendingSettings.inputMode == INPUT_RF) ? 0 : 1;
    if (firstApply || appliedHardwareSettings.inputMode != pendingSettings.inputMode) {
        qDebug() << "[FobosLifecycle] set direct sampling begin" << "libfobosMode" << libfobosMode;
        result = setActiveDirectSamplingSafely(static_cast<unsigned int>(libfobosMode));
        qDebug() << "[FobosLifecycle] set direct sampling end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set direct sampling mode, error code:" << result;
            return false;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_direct_sampling skipped unchanged";
    }

    if (pendingSettings.sampleRate > 0.0 &&
        (firstApply || changedDouble(appliedHardwareSettings.sampleRate, pendingSettings.sampleRate))) {
        qDebug() << "[FobosLifecycle] set sample rate begin" << "requested" << pendingSettings.sampleRate;
        result = setActiveSampleRateSafely(pendingSettings.sampleRate, &globalSampleRate);
        qDebug() << "[FobosLifecycle] set sample rate end"
                 << "result" << result
                 << "actual" << globalSampleRate;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set sample rate, error code:" << result;
            globalSampleRate = pendingSettings.sampleRate;
            sampleRateReopenRequired = true;
            hardwareSettingsApplied = false;
            return false;
        }
        pendingSettings.sampleRate = globalSampleRate;
        appliedSampleRate = globalSampleRate;
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_samplerate skipped unchanged";
        if (appliedSampleRate > 0.0) {
            globalSampleRate = appliedSampleRate;
            pendingSettings.sampleRate = appliedSampleRate;
        }
    }

    if (activeFobosApiKind == FobosApiKind::Agile &&
        pendingSettings.inputMode == INPUT_RF &&
        !agileScanEnabled &&
        !stopAgileScanForNormalRf("apply settings")) {
        qDebug() << "[FobosLifecycle] failed to leave Agile scan mode before normal RF tuning";
        return false;
    }

    if (pendingSettings.inputMode == INPUT_RF) {
        if (firstApply ||
            appliedHardwareSettings.inputMode != pendingSettings.inputMode ||
            changedDouble(appliedHardwareSettings.centerFrequency, pendingSettings.centerFrequency)) {
            qDebug() << "[FobosLifecycle] set frequency begin" << "requested" << pendingSettings.centerFrequency;
            result = setActiveFrequencySafely(pendingSettings.centerFrequency, &actualFrequency);
            qDebug() << "[FobosLifecycle] set frequency end"
                     << "result" << result
                     << "actual" << actualFrequency;
            if (result != FOBOS_ERR_OK) {
                qDebug() << "Failed to set frequency, error code:" << result;
            }
            pendingSettings.actualFrequency = actualFrequency;
        } else {
            qDebug() << "[FobosLifecycle] fobos_rx_set_frequency skipped unchanged";
            actualFrequency = pendingSettings.actualFrequency;
        }
    } else {
        pendingSettings.centerFrequency = 0.0;
        actualFrequency = 0.0;
        pendingSettings.actualFrequency = 0.0;
    }

    if (activeFobosApiKind == FobosApiKind::Agile &&
        pendingSettings.inputMode == INPUT_RF &&
        !agileScanEnabled) {
        const double autoBandwidthRatio = agileRfAutoBandwidthRatio(pendingSettings.sampleRate);
        qDebug() << "[FobosLifecycle] set Agile auto bandwidth begin" << autoBandwidthRatio;
        const int bandwidthResult = setFobosAgileAutoBandwidthSafely(agileDevice, autoBandwidthRatio);
        qDebug() << "[FobosLifecycle] set Agile auto bandwidth end" << "result" << bandwidthResult;
        if (bandwidthResult != FOBOS_ERR_OK) {
            qDebug() << "Failed to set Agile auto bandwidth, error code:" << bandwidthResult;
        }
    }

    if (firstApply || appliedHardwareSettings.lnaGain != pendingSettings.lnaGain) {
        qDebug() << "[FobosLifecycle] set LNA gain begin" << pendingSettings.lnaGain;
        result = setActiveLnaGainSafely(static_cast<unsigned int>(pendingSettings.lnaGain));
        qDebug() << "[FobosLifecycle] set LNA gain end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set LNA gain, error code:" << result;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_lna_gain skipped unchanged";
    }

    if (firstApply || appliedHardwareSettings.vgaGain != pendingSettings.vgaGain) {
        qDebug() << "[FobosLifecycle] set VGA gain begin" << pendingSettings.vgaGain;
        result = setActiveVgaGainSafely(static_cast<unsigned int>(pendingSettings.vgaGain));
        qDebug() << "[FobosLifecycle] set VGA gain end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set VGA gain, error code:" << result;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_vga_gain skipped unchanged";
    }

    if (firstApply || appliedHardwareSettings.gpoValue != pendingSettings.gpoValue) {
        qDebug() << "[FobosLifecycle] set user GPO begin" << pendingSettings.gpoValue;
        const int gpoResult = setActiveGpoSafely(pendingSettings.gpoValue);
        qDebug() << "[FobosLifecycle] set user GPO end" << "result" << gpoResult;
        if (gpoResult != FOBOS_ERR_OK) {
            qDebug() << "Failed to set user GPO, error code:" << gpoResult;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_user_gpo skipped unchanged";
    }

    if (!applyAgileScanSettings(false)) {
        qDebug() << "[FobosLifecycle] Agile scan settings failed";
        return false;
    }
    if (!applyStandardScanSettings(false)) {
        qDebug() << "[FobosLifecycle] Standard scan settings failed";
        return false;
    }

    appliedHardwareSettings = pendingSettings;
    appliedHardwareSettings.sampleRate = globalSampleRate;
    appliedHardwareSettings.actualFrequency = pendingSettings.actualFrequency;
    hardwareSettingsApplied = true;
    publishSettingsToGlobals();
    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }
    if (listeningFrequencyControl) {
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
    }
    settingRange();
    qDebug() << "[FobosLifecycle] applyFobosSettings exit";
    return true;
}

void YourClassName::updateFrequency() {
   const double frequency = scaleWidget ? scaleWidget->currentListeningFrequency() : listeningFrequency;
   if (listeningFrequencyControl) {
       listeningFrequencyControl->setValueHz(frequency);
   }
   onListeningFrequencyEntered();
}

void YourClassName::updateCentralFrequency() {
   if (pendingSettings.inputMode != INPUT_RF) {
       normalizeTuning(pendingSettings);
       if (scaleWidget) {
           scaleWidget->setTuning(pendingSettings.listeningFrequency,
                                  pendingSettings.centerFrequency,
                                  pendingSettings.bandwidth,
                                  pendingSettings.modulationType);
       }
       return;
   }
   const double frequency = scaleWidget ? scaleWidget->currentCenterFrequency() : globalFrequency;
   if (frequencyControl) {
       frequencyControl->setValueHz(frequency);
   }
   onFrequencyEntered();
}

void YourClassName::updateTuningFromScale(double tunedListeningFrequency, double tunedCenterFrequency) {
    const RadioSettings previousSettings = pendingSettings;
    pendingSettings.listeningFrequency = tunedListeningFrequency;
    pendingSettings.centerFrequency = tunedCenterFrequency;
    normalizeTuning(pendingSettings);

    applyCenterFrequencyToHardwareIfNeeded(previousSettings, "scale");

    publishSettingsToGlobals();
    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }
    if (listeningFrequencyControl) {
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
    }
    settingRange();
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

void YourClassName::showTuneContextMenu(double frequency, const QPoint &globalPos) {
    if (!std::isfinite(frequency)) {
        return;
    }

    const QString frequencyText = QString::number(frequency / 1e6, 'f', 6) + " MHz";
    QMenu menu(this);
    QAction *tuneCenterAction = menu.addAction("Tune signal center here (" + frequencyText + ")");
    QAction *usbEdgeAction = menu.addAction("Set USB lower edge here");
    QAction *lsbEdgeAction = menu.addAction("Set LSB upper edge here");
    menu.addSeparator();
    QAction *centerReceiverAction = menu.addAction("Center receiver here");

    QAction *selected = menu.exec(globalPos);
    if (!selected) {
        return;
    }

    if (selected == tuneCenterAction) {
        tuneSignalCenterAt(frequency);
    } else if (selected == usbEdgeAction) {
        tuneSidebandEdgeAt(frequency, MOD_USB);
    } else if (selected == lsbEdgeAction) {
        tuneSidebandEdgeAt(frequency, MOD_LSB);
    } else if (selected == centerReceiverAction) {
        centerReceiverAt(frequency);
    }
}

void YourClassName::tuneSignalCenterAt(double frequency) {
    double listeningTarget = frequency;
    if (isUpperSidebandMode(pendingSettings.modulationType)) {
        listeningTarget = frequency - pendingSettings.bandwidth * 0.5;
    } else if (isLowerSidebandMode(pendingSettings.modulationType)) {
        listeningTarget = frequency + pendingSettings.bandwidth * 0.5;
    }
    const double visibleSpanHz = std::isfinite(maxFrequency) && std::isfinite(minFrequency) && maxFrequency > minFrequency
                                     ? maxFrequency - minFrequency
                                     : pendingSettings.sampleRate * (currentScale / 100.0);
    const double roundedTarget = roundAutoTuneFrequencyHz(listeningTarget, visibleSpanHz);
    qDebug() << "[Tune] auto center"
             << "detected" << frequency
             << "target" << listeningTarget
             << "rounded" << roundedTarget
             << "step" << autoTuneRoundingStepHz(listeningTarget, visibleSpanHz)
             << "visibleSpan" << visibleSpanHz;
    updateTuningFromScale(roundedTarget, pendingSettings.centerFrequency);
}

void YourClassName::tuneSidebandEdgeAt(double frequency, int modulationType) {
    if (pendingSettings.modulationType != modulationType) {
        if (modulationButtonGroup) {
            if (QAbstractButton *button = modulationButtonGroup->button(modulationType)) {
                modulationButtonGroup->blockSignals(true);
                button->setChecked(true);
                modulationButtonGroup->blockSignals(false);
            }
        }
        onModulationChanged(modulationType);
    }
    updateTuningFromScale(frequency, pendingSettings.centerFrequency);
}

void YourClassName::centerReceiverAt(double frequency) {
    updateTuningFromScale(pendingSettings.listeningFrequency, frequency);
}

void YourClassName::onModulationChanged(int id) {
    pendingSettings.modulationType = id;
    pendingSettings.bandwidth = defaultBandwidthForModulation(id);
    if (bandwidthControl) {
        QSignalBlocker blocker(bandwidthControl);
        bandwidthControl->setValueHz(pendingSettings.bandwidth);
    }
    publishSettingsToGlobals();
    if (scaleWidget) {
        scaleWidget->setModulationType(id);
    }
    updateDigitalDecoderMode();
    updateVideoProcessorMode();
    updateIqFrameProducerSettings();
    settingRange();
    qDebug() << "Modulation type changed to:" << id
             << "bandwidth preset" << pendingSettings.bandwidth;
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

void YourClassName::onDigitalTextDecoded(const QString &text) {
    if (!digitalTextEdit || text.isEmpty()) {
        return;
    }

    qDebug() << "[DigitalText] append" << text.left(120);
    QTextCursor cursor = digitalTextEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    cursor.insertText(text);
    digitalTextEdit->setTextCursor(cursor);
    digitalTextEdit->ensureCursorVisible();
}

void YourClassName::onDigitalDecoderStatusChanged(const QString &status) {
    if (digitalStatusLabel) {
        digitalStatusLabel->setToolTip(status);
        const int textWidth = (std::max)(160, digitalStatusLabel->width() - 8);
        digitalStatusLabel->setText(digitalStatusLabel->fontMetrics().elidedText(status,
                                                                                 Qt::ElideRight,
                                                                                 textWidth));
    }
}

void YourClassName::onDmrMetadataDetected(int colorCode,
                                          int timeslot,
                                          quint32 targetId,
                                          quint32 sourceId,
                                          int flco) {
    if (dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked()) {
        return;
    }

    const auto setComboToData = [](QComboBox *combo, const QVariant &data) {
        if (!combo) {
            return;
        }
        const int index = combo->findData(data);
        if (index >= 0 && combo->currentIndex() != index) {
            QSignalBlocker blocker(combo);
            combo->setCurrentIndex(index);
        }
    };
    const auto setLineEditNumber = [](QLineEdit *edit, quint32 value) {
        if (!edit || value == 0) {
            return;
        }
        const QString text = QString::number(value);
        if (edit->text().trimmed() != text) {
            QSignalBlocker blocker(edit);
            edit->setText(text);
        }
    };

    if (colorCode >= 0 && colorCode <= 15) {
        setComboToData(dmrLabColorCodeCombo, colorCode);
    }
    if (timeslot == 1 || timeslot == 2) {
        setComboToData(dmrLabSlotCombo, timeslot);
    }
    if (targetId == 0x00ffffffU) {
        setComboToData(dmrLabCallTypeCombo, QStringLiteral("all_call"));
    } else if (flco == 3) {
        setComboToData(dmrLabCallTypeCombo, QStringLiteral("private"));
    } else if (flco == 0) {
        setComboToData(dmrLabCallTypeCombo, QStringLiteral("group"));
    }
    setLineEditNumber(dmrLabTargetIdEdit, targetId);
    setLineEditNumber(dmrLabSourceIdEdit, sourceId);
}

void YourClassName::onVideoStatusChanged(const QString &status) {
    if (videoStatusLabel) {
        auto videoText = [this](const QString &raw) {
            if (raw == QStringLiteral("Video decoder disabled")) {
                return uiText(QStringLiteral("video_decoder_disabled"), raw);
            }
            if (raw == QStringLiteral("Video decoder ready")) {
                return uiText(QStringLiteral("video_decoder_ready"), raw);
            }
            if (raw == QStringLiteral("Video test pattern")) {
                return uiText(QStringLiteral("video_test_pattern"), raw);
            }
            return raw;
        };
        if (pendingSettings.modulationType == MOD_SSTV) {
            if (status.startsWith(QStringLiteral("SSTV"))) {
                videoStatusLabel->setText(videoText(status));
                return;
            }
            const bool sstvTest = videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked();
            videoStatusLabel->setText(sstvTest
                                      ? QStringLiteral("SSTV: internal image test pattern")
                                      : QStringLiteral("SSTV: image decoder setup ready"));
            return;
        }
        if (pendingSettings.modulationType == MOD_APT) {
            if (status.startsWith(QStringLiteral("NOAA APT"))) {
                videoStatusLabel->setText(status);
                return;
            }
            const bool aptTest = videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked();
            videoStatusLabel->setText(aptTest
                                      ? QStringLiteral("NOAA APT test stream")
                                      : QStringLiteral("NOAA APT: image decoder setup ready"));
            return;
        }
        if (pendingSettings.modulationType == MOD_WEFAX) {
            if (status.startsWith(QStringLiteral("WEFAX"))) {
                videoStatusLabel->setText(status);
                return;
            }
            const bool wefaxTest = videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked();
            videoStatusLabel->setText(wefaxTest
                                      ? QStringLiteral("WEFAX test stream")
                                      : QStringLiteral("WEFAX: image decoder setup ready"));
            return;
        }
        if (pendingSettings.modulationType == MOD_LRPT) {
            if (status.startsWith(QStringLiteral("Meteor LRPT"))) {
                videoStatusLabel->setText(status);
                return;
            }
            videoStatusLabel->setText(QStringLiteral("Meteor LRPT beta: QPSK IQ monitor ready"));
            return;
        }
        videoStatusLabel->setText(videoText(status));
    }
}

void YourClassName::onScaleChanged(int value) {
    currentScale = sliderValueToScalePercent(value);

    scaleLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("scale"), QStringLiteral("Scale")),
                                                    formatScalePercent(currentScale)));
    settingRange();
    savePersistentSettings();
}

void YourClassName::onSensitivityChanged(int value) {
    sensitivity = value;
    sensitivityLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("sensitivity"), QStringLiteral("Sensitivity"))).arg(value));
    settingRange();
    savePersistentSettings();
}

void YourClassName::onContrastChanged(int value) {
    contrast = value;
    contrastLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("contrast"), QStringLiteral("Contrast"))).arg(value));
    settingRange();
    savePersistentSettings();
}

void YourClassName::onLevelMinChanged(int value) {
    displayLevelMin = sliderValueToLevel(value);
    if (displayLevelMin >= displayLevelMax) {
        displayLevelMin = (std::max)(sliderValueToLevel(MIN_LEVEL_SLIDER_VALUE), displayLevelMax - MIN_LEVEL_GAP);
        if (levelMinSlider) {
            levelMinSlider->blockSignals(true);
            levelMinSlider->setValue(levelToSliderValue(displayLevelMin));
            levelMinSlider->blockSignals(false);
        }
    }
    if (levelMinLabel) {
        levelMinLabel->setText(levelLabelText(uiText(QStringLiteral("min"), QStringLiteral("Min")), displayLevelMin));
    }
    if (levelMaxLabel) {
        levelMaxLabel->setText(levelLabelText(uiText(QStringLiteral("max"), QStringLiteral("Max")), displayLevelMax));
    }
    if (graphWidget) {
        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
    if (waterfallWidget) {
        waterfallWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
    savePersistentSettings();
}

void YourClassName::onLevelMaxChanged(int value) {
    displayLevelMax = sliderValueToLevel(value);
    if (displayLevelMax <= displayLevelMin) {
        displayLevelMax = (std::min)(sliderValueToLevel(MAX_LEVEL_SLIDER_VALUE), displayLevelMin + MIN_LEVEL_GAP);
        if (levelMaxSlider) {
            levelMaxSlider->blockSignals(true);
            levelMaxSlider->setValue(levelToSliderValue(displayLevelMax));
            levelMaxSlider->blockSignals(false);
        }
    }
    if (levelMinLabel) {
        levelMinLabel->setText(levelLabelText(uiText(QStringLiteral("min"), QStringLiteral("Min")), displayLevelMin));
    }
    if (levelMaxLabel) {
        levelMaxLabel->setText(levelLabelText(uiText(QStringLiteral("max"), QStringLiteral("Max")), displayLevelMax));
    }
    if (graphWidget) {
        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
    if (waterfallWidget) {
        waterfallWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
    savePersistentSettings();
}

void YourClassName::onWaterfallScaleChanged(int delta) {
    int value = scaleSlider->value();

    if (delta > 0) {
        value += scaleSlider->singleStep();
    } else {
        value -= scaleSlider->singleStep();
    }

    value = std::clamp(value, scaleSlider->minimum(), scaleSlider->maximum());

    if (value == scaleSlider->value()) {
        return;
    }

    scaleSlider->setValue(value);

    if (isNetworkClientMode() && !isFullIqProcessingMode()) {
        scheduleRemoteSettingsCommand();
    }
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
    Q_UNUSED(checked);
    pendingSettings.syncEnabled = false;
    if (syncCheckbox && syncCheckbox->isChecked()) {
        syncCheckbox->blockSignals(true);
        syncCheckbox->setChecked(false);
        syncCheckbox->blockSignals(false);
    }
    publishSettingsToGlobals();
    qDebug() << "Sync reader disabled; async reader is forced.";
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
                focusedLineEdit->setText(QString::number(currentValue, 'f', 0));

                focusedLineEdit->emit textEdited(focusedLineEdit->text()); 
            }
        }
    }
    QMainWindow::wheelEvent(event); 
}

void YourClassName::populateAudioDevices() {
#ifdef _WIN32
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
#elif defined(FOBOSAPP_HAS_QT_AUDIO)
    const QList<QAudioDeviceInfo> devices = QAudioDeviceInfo::availableDevices(QAudio::AudioOutput);
    qDebug() << "Number of Qt audio output devices found:" << devices.size();
    if (devices.isEmpty()) {
        audioDeviceComboBox->addItem(QStringLiteral("Default audio output"), QVariant(0));
        return;
    }
    for (int i = 0; i < devices.size(); ++i) {
        const QString deviceName = devices.at(i).deviceName();
        qDebug() << "Audio device" << i << ":" << deviceName;
        audioDeviceComboBox->addItem(deviceName, QVariant(i));
    }
#else
    qDebug() << "Native audio device enumeration is not implemented on this platform yet.";
    audioDeviceComboBox->addItem(QStringLiteral("Default audio output"), QVariant(0));
#endif
}

void YourClassName::onAudioDeviceChanged(int index) {
    if (index < 0) return;
    if (!isIdle()) {
        qDebug() << "Audio device change is locked while radio is running.";
        revertHardwareControlsToSettings();
        return;
    }

    QVariant data = audioDeviceComboBox->currentData();
    if (!data.isValid()) {
        qDebug() << "Error: Invalid audio device selected!";
        return;
    }

    // Получаем строку с именем устройства
    QString deviceName = data.toString();
    qDebug() << "Selected audio device:" << deviceName;
    pendingSettings.audioDeviceId = data.toInt();
    publishSettingsToGlobals();

    deviceID = pendingSettings.audioDeviceId;  // Передаём ID вместо имени
    qDebug() << "Selected audio device ID:" << deviceID;

    if (audioProcessor) {
        audioProcessor->setAudioDevice(deviceID);
    } else {
        qDebug() << "Error: audioProcessor is null!";
    }
}


void YourClassName::onBandwidthChanged() {
    if (!bandwidthControl) {
        return;
    }

    const double bandwidth = bandwidthControl->valueHz();
    if (bandwidth <= 0.0) {
        return;
    }

    pendingSettings.bandwidth = bandwidth;
    publishSettingsToGlobals();
    settingRange();

    if (scaleWidget) {
        scaleWidget->setTuning(pendingSettings.listeningFrequency,
                               pendingSettings.centerFrequency,
                               pendingSettings.bandwidth,
                               pendingSettings.modulationType);
    }

    savePersistentSettings();
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

void YourClassName::openNetworkSettingsDialog() {
    QDialog dialog(this);
    dialog.setWindowTitle(uiText(QStringLiteral("network_settings"), QStringLiteral("Network Settings")));
    dialog.setMinimumWidth(420);

    QVBoxLayout *rootLayout = new QVBoxLayout(&dialog);
    QFormLayout *formLayout = new QFormLayout();

    QComboBox *modeCombo = new QComboBox(&dialog);
    modeCombo->addItem(uiText(QStringLiteral("disabled"), QStringLiteral("Disabled")), static_cast<int>(NetworkMode::Disabled));
    modeCombo->addItem(uiText(QStringLiteral("server"), QStringLiteral("Server")), static_cast<int>(NetworkMode::Server));
    modeCombo->addItem(uiText(QStringLiteral("client"), QStringLiteral("Client")), static_cast<int>(NetworkMode::Client));
    modeCombo->setCurrentIndex(modeCombo->findData(static_cast<int>(networkMode)));

    QComboBox *processingCombo = new QComboBox(&dialog);
    processingCombo->addItem(uiText(QStringLiteral("network_processing_server"),
                                    QStringLiteral("Server processing (spectrum/audio stream)")),
                             static_cast<int>(NetworkProcessingMode::ServerSide));
    processingCombo->addItem(uiText(QStringLiteral("network_processing_channel_iq"),
                                    QStringLiteral("Channel IQ + client demod")),
                             static_cast<int>(NetworkProcessingMode::ChannelIqClientSide));
    processingCombo->addItem(uiText(QStringLiteral("network_processing_full_iq"),
                                    QStringLiteral("Full IQ client processing (LAN only)")),
                             static_cast<int>(NetworkProcessingMode::FullIqClientSide));
    processingCombo->setCurrentIndex(processingCombo->findData(static_cast<int>(networkProcessingMode)));

    QLineEdit *serverAddressEdit = new QLineEdit(networkServerAddress, &dialog);
    serverAddressEdit->setPlaceholderText(uiText(QStringLiteral("server_ip_placeholder"),
                                                 QStringLiteral("Server IP address")));

    QLineEdit *bindAddressEdit = new QLineEdit(networkBindAddress, &dialog);
    bindAddressEdit->setPlaceholderText("0.0.0.0");

    QSpinBox *portSpin = new QSpinBox(&dialog);
    portSpin->setRange(1, 65535);
    portSpin->setValue(networkControlPort);

    QCheckBox *serverDisableLocalUiCheck = new QCheckBox(uiText(QStringLiteral("network_disable_local_ui"),
                                                                QStringLiteral("Disable local visual/audio on server when streaming is implemented")),
                                                         &dialog);
    serverDisableLocalUiCheck->setChecked(serverDisableLocalVisualAudio);

    QCheckBox *fullResolutionSpectrumCheck = new QCheckBox(uiText(QStringLiteral("network_full_resolution_frames"),
                                                                  QStringLiteral("Send full-resolution spectrum/waterfall frames (heavy LAN only)")),
                                                           &dialog);
    fullResolutionSpectrumCheck->setChecked(networkFullResolutionSpectrumFrames);

    QCheckBox *audioRelayTransmitCheck = new QCheckBox(uiText(QStringLiteral("audio_relay_tx"),
                                                             QStringLiteral("Send ready audio by UDP")),
                                                       &dialog);
    audioRelayTransmitCheck->setChecked(audioRelayTransmitEnabled);

    QLineEdit *audioRelayHostEdit = new QLineEdit(audioRelayHost, &dialog);
    audioRelayHostEdit->setPlaceholderText(uiText(QStringLiteral("target_ip_placeholder"),
                                                  QStringLiteral("Target IP address")));

    QSpinBox *audioRelayPortSpin = new QSpinBox(&dialog);
    audioRelayPortSpin->setRange(1, 65535);
    audioRelayPortSpin->setValue(audioRelayPort);

    QCheckBox *audioRelayReceiveCheck = new QCheckBox(uiText(QStringLiteral("audio_relay_rx"),
                                                            QStringLiteral("Receive ready audio by UDP")),
                                                      &dialog);
    audioRelayReceiveCheck->setChecked(audioRelayReceiveEnabled);

    QSpinBox *audioRelayListenPortSpin = new QSpinBox(&dialog);
    audioRelayListenPortSpin->setRange(1, 65535);
    audioRelayListenPortSpin->setValue(audioRelayListenPort);

    QCheckBox *audioHttpStreamCheck = new QCheckBox(uiText(QStringLiteral("audio_http_stream"),
                                                          QStringLiteral("Serve VLC-compatible HTTP/WAV audio")),
                                                    &dialog);
    audioHttpStreamCheck->setChecked(audioHttpStreamEnabled);

    QSpinBox *audioHttpStreamPortSpin = new QSpinBox(&dialog);
    audioHttpStreamPortSpin->setRange(1, 65535);
    audioHttpStreamPortSpin->setValue(audioHttpStreamPort);

    QLabel *statusLabel = new QLabel(networkController
                                         ? networkController->statusText()
                                         : uiText(QStringLiteral("network_controller_unavailable"),
                                                  QStringLiteral("Network controller unavailable")),
                                     &dialog);
    statusLabel->setWordWrap(true);

    formLayout->addRow(uiText(QStringLiteral("mode"), QStringLiteral("Mode:")), modeCombo);
    formLayout->addRow(uiText(QStringLiteral("processing"), QStringLiteral("Processing:")), processingCombo);
    formLayout->addRow(uiText(QStringLiteral("server_ip"), QStringLiteral("Server IP:")), serverAddressEdit);
    formLayout->addRow(uiText(QStringLiteral("bind_address"), QStringLiteral("Bind address:")), bindAddressEdit);
    formLayout->addRow(uiText(QStringLiteral("control_port"), QStringLiteral("Control port:")), portSpin);
    formLayout->addRow("", serverDisableLocalUiCheck);
    formLayout->addRow(uiText(QStringLiteral("visual_frames"), QStringLiteral("Visual frames:")), fullResolutionSpectrumCheck);
    formLayout->addRow(uiText(QStringLiteral("audio_relay_tx_label"), QStringLiteral("Audio relay TX:")), audioRelayTransmitCheck);
    formLayout->addRow(uiText(QStringLiteral("relay_target_ip"), QStringLiteral("Relay target IP:")), audioRelayHostEdit);
    formLayout->addRow(uiText(QStringLiteral("relay_target_port"), QStringLiteral("Relay target port:")), audioRelayPortSpin);
    formLayout->addRow(uiText(QStringLiteral("audio_relay_rx_label"), QStringLiteral("Audio relay RX:")), audioRelayReceiveCheck);
    formLayout->addRow(uiText(QStringLiteral("relay_listen_port"), QStringLiteral("Relay listen port:")), audioRelayListenPortSpin);
    formLayout->addRow(uiText(QStringLiteral("vlc_http_audio"), QStringLiteral("VLC HTTP audio:")), audioHttpStreamCheck);
    formLayout->addRow(uiText(QStringLiteral("http_audio_port"), QStringLiteral("HTTP audio port:")), audioHttpStreamPortSpin);

    QPushButton *testButton = new QPushButton(uiText(QStringLiteral("apply_test_channel"), QStringLiteral("Apply / Test Channel")), &dialog);
    QPushButton *requestControlButton = new QPushButton(uiText(QStringLiteral("request_control"), QStringLiteral("Request Control")), &dialog);
    QPushButton *stopButton = new QPushButton(uiText(QStringLiteral("stop_network"), QStringLiteral("Stop Network")), &dialog);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    if (QPushButton *closeButton = buttonBox->button(QDialogButtonBox::Close)) {
        closeButton->setText(uiText(QStringLiteral("close"), QStringLiteral("Close")));
    }

    QHBoxLayout *actionLayout = new QHBoxLayout();
    actionLayout->addWidget(testButton);
    actionLayout->addWidget(requestControlButton);
    actionLayout->addWidget(stopButton);

    rootLayout->addLayout(formLayout);
    rootLayout->addWidget(statusLabel);
    rootLayout->addLayout(actionLayout);
    rootLayout->addWidget(buttonBox);

    auto updateFieldState = [=]() {
        const auto selectedMode = static_cast<NetworkMode>(modeCombo->currentData().toInt());
        serverAddressEdit->setEnabled(selectedMode == NetworkMode::Client);
        bindAddressEdit->setEnabled(selectedMode == NetworkMode::Server);
        serverDisableLocalUiCheck->setEnabled(selectedMode != NetworkMode::Disabled);
        fullResolutionSpectrumCheck->setEnabled(selectedMode != NetworkMode::Disabled);
        processingCombo->setEnabled(selectedMode != NetworkMode::Disabled);
        portSpin->setEnabled(selectedMode != NetworkMode::Disabled);
        requestControlButton->setEnabled(selectedMode == NetworkMode::Client);
        audioRelayHostEdit->setEnabled(audioRelayTransmitCheck->isChecked());
        audioRelayPortSpin->setEnabled(audioRelayTransmitCheck->isChecked());
        audioRelayListenPortSpin->setEnabled(audioRelayReceiveCheck->isChecked());
        audioHttpStreamPortSpin->setEnabled(audioHttpStreamCheck->isChecked());
        testButton->setText(selectedMode == NetworkMode::Disabled
                                ? uiText(QStringLiteral("apply"), QStringLiteral("Apply"))
                                : uiText(QStringLiteral("apply_test_channel"), QStringLiteral("Apply / Test Channel")));
    };
    connect(modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog, updateFieldState);
    connect(audioRelayTransmitCheck, &QCheckBox::toggled, &dialog, updateFieldState);
    connect(audioRelayReceiveCheck, &QCheckBox::toggled, &dialog, updateFieldState);
    connect(audioHttpStreamCheck, &QCheckBox::toggled, &dialog, updateFieldState);
    updateFieldState();

    QMetaObject::Connection statusConnection;
    if (networkController) {
        statusConnection = connect(networkController,
                                   &NetworkController::statusChanged,
                                   &dialog,
                                   [statusLabel](const QString &status) {
                                       statusLabel->setText(status);
                                   });
    }

    connect(testButton, &QPushButton::clicked, &dialog, [=]() {
        const NetworkMode previousNetworkMode = networkMode;
        const NetworkProcessingMode previousProcessingMode = networkProcessingMode;
        const bool previousServerDisableLocalVisualAudio = serverDisableLocalVisualAudio;

        networkMode = static_cast<NetworkMode>(modeCombo->currentData().toInt());
        networkServerAddress = serverAddressEdit->text().trimmed().isEmpty()
                                   ? QString("127.0.0.1")
                                   : serverAddressEdit->text().trimmed();
        networkBindAddress = bindAddressEdit->text().trimmed().isEmpty()
                                 ? QString("0.0.0.0")
                                 : bindAddressEdit->text().trimmed();
        networkControlPort = static_cast<quint16>(portSpin->value());
        serverDisableLocalVisualAudio = serverDisableLocalUiCheck->isChecked();
        networkFullResolutionSpectrumFrames = fullResolutionSpectrumCheck->isChecked();
        networkProcessingMode = static_cast<NetworkProcessingMode>(processingCombo->currentData().toInt());
        audioRelayTransmitEnabled = audioRelayTransmitCheck->isChecked();
        audioRelayHost = audioRelayHostEdit->text().trimmed().isEmpty()
                             ? QString("127.0.0.1")
                             : audioRelayHostEdit->text().trimmed();
        audioRelayPort = static_cast<quint16>(audioRelayPortSpin->value());
        audioRelayReceiveEnabled = audioRelayReceiveCheck->isChecked();
        audioRelayListenPort = static_cast<quint16>(audioRelayListenPortSpin->value());
        audioHttpStreamEnabled = audioHttpStreamCheck->isChecked();
        audioHttpStreamPort = static_cast<quint16>(audioHttpStreamPortSpin->value());
        updateAudioRelaySocket();
        updateAudioHttpStreamServer();
        if (isChannelIqRecordingActive() &&
            networkMode != NetworkMode::Disabled &&
            isFullIqProcessingMode()) {
            stopRecording(false);
            updateRecordingStatus(QStringLiteral("Recording stopped: Channel IQ cannot run during Full IQ streaming"));
        }

        const bool networkModeChanged = previousNetworkMode != networkMode;
        const bool processingModeChanged = previousProcessingMode != networkProcessingMode;
        const bool serverLocalOutputChanged =
            previousServerDisableLocalVisualAudio != serverDisableLocalVisualAudio;

        if (!networkController) {
            statusLabel->setText("Network controller unavailable");
            return;
        }

        if (networkMode == NetworkMode::Disabled) {
            if (previousNetworkMode == NetworkMode::Client) {
                stopNetworkClientProcessing();
            } else if (previousNetworkMode == NetworkMode::Server && runState == RadioRunState::Running) {
                if (audioProcessor) {
                    audioProcessor->setLocalPlaybackEnabled(true);
                }
                if (updateTimer) {
                    updateTimer->start();
                }
            }
            if (remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
            networkController->stop();
            if (runState == RadioRunState::Running) {
                updateIqFrameProducerSettings();
            }
            savePersistentSettings();
            return;
        }

        if (previousNetworkMode == NetworkMode::Client && networkMode != NetworkMode::Client) {
            stopNetworkClientProcessing();
        }

        if (remoteAudioPlayer && networkMode != NetworkMode::Client) {
            remoteAudioPlayer->stop();
        }
        if (networkMode == NetworkMode::Server) {
            networkController->startServer(networkBindAddress, networkControlPort);
            const bool restartRequired =
                runState == RadioRunState::Running &&
                (networkModeChanged || processingModeChanged);
            if (restartRequired) {
                restartStreamForHardwareChange();
            } else if (runState == RadioRunState::Running && serverLocalOutputChanged) {
                applyServerLocalOutputPolicy();
            }
            savePersistentSettings();
            return;
        }

        if (runState == RadioRunState::Running &&
            (networkModeChanged || processingModeChanged)) {
            if (isClientIqProcessingMode()) {
                startNetworkClientProcessing();
            } else {
                stopNetworkClientProcessing();
            }
        }

        networkController->testClientConnection(networkServerAddress, networkControlPort);
        savePersistentSettings();
    });

    connect(stopButton, &QPushButton::clicked, &dialog, [=]() {
        const NetworkMode previousNetworkMode = networkMode;
        networkMode = NetworkMode::Disabled;
        if (modeCombo) {
            modeCombo->setCurrentIndex(modeCombo->findData(static_cast<int>(NetworkMode::Disabled)));
        }
        if (networkController) {
            networkController->stop();
        }
        if (remoteAudioPlayer) {
            remoteAudioPlayer->stop();
        }
        if (previousNetworkMode == NetworkMode::Client) {
            stopNetworkClientProcessing();
        } else if (previousNetworkMode == NetworkMode::Server && runState == RadioRunState::Running) {
            if (audioProcessor) {
                audioProcessor->setLocalPlaybackEnabled(true);
            }
            if (updateTimer) {
                updateTimer->start();
            }
        }
        if (runState == RadioRunState::Running) {
            updateIqFrameProducerSettings();
        }
        savePersistentSettings();
    });

    connect(requestControlButton, &QPushButton::clicked, &dialog, [=]() {
        if (networkMode != NetworkMode::Client) {
            statusLabel->setText("Switch to Client mode and connect first.");
            return;
        }
        if (!networkController || !networkController->isControlReady()) {
            statusLabel->setText("Control channel is not ready.");
            return;
        }
        if (networkController->clientHasControl()) {
            statusLabel->setText("This client already has control.");
            return;
        }
        if (sendRemoteControlCommand("requestPriority")) {
            statusLabel->setText("Control request sent.");
        } else {
            statusLabel->setText("Control request could not be sent.");
        }
    });

    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    dialog.exec();
    if (networkController && statusConnection) {
        disconnect(statusConnection);
    }
}

void YourClassName::updateNetworkButtonText() {
    if (!networkButton) {
        return;
    }

    const QString base = uiText(QStringLiteral("network"), QStringLiteral("Network"));
    switch (networkMode) {
    case NetworkMode::Server:
        networkButton->setText(isChannelIqProcessingMode()
                                   ? QStringLiteral("%1: Server ChIQ").arg(base)
                                   : (isFullIqProcessingMode() ? QStringLiteral("%1: Server IQ").arg(base)
                                                               : QStringLiteral("%1: Server").arg(base)));
        break;
    case NetworkMode::Client: {
        const QString roleSuffix =
            networkController && networkController->isControlReady()
                ? (networkController->clientHasControl() ? QStringLiteral(" Ctrl") : QStringLiteral(" Obs"))
                : QString();
        networkButton->setText((isChannelIqProcessingMode()
                                    ? QStringLiteral("%1: Client ChIQ").arg(base)
                                    : (isFullIqProcessingMode() ? QStringLiteral("%1: Client IQ").arg(base)
                                                                : QStringLiteral("%1: Client").arg(base))) + roleSuffix);
        break;
    }
    case NetworkMode::Disabled:
    default:
        networkButton->setText(base);
        break;
    }
}

void YourClassName::onNetworkStatusChanged(const QString &status) {
    qDebug() << "[Network]" << status;
    updateNetworkButtonText();
}

void YourClassName::updateAudioFilterLabels() {
    if (audioLowPassLabel) {
        const QString value = pendingSettings.audioLowPassHz > 0.0
                                  ? audioFilterFrequencyText(pendingSettings.audioLowPassHz)
                                  : uiText(QStringLiteral("auto"), QStringLiteral("Auto"));
        audioLowPassLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("audio_lpf"), QStringLiteral("Audio LPF")), value));
    }
    if (audioHighPassLabel) {
        const QString value = pendingSettings.audioHighPassHz > 0.0
                                  ? audioFilterFrequencyText(pendingSettings.audioHighPassHz)
                                  : uiText(QStringLiteral("off"), QStringLiteral("Off"));
        audioHighPassLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("audio_hpf"), QStringLiteral("Audio HPF")), value));
    }
}

void YourClassName::updateHfNoiseCancelControls() {
    const bool enabled = pendingSettings.inputMode == INPUT_HF_NOISE_CANCEL;

    if (hfNoiseCancelDepthLabel) {
        hfNoiseCancelDepthLabel->setText(QStringLiteral("%1: %2%").arg(uiText(QStringLiteral("hf_cancel"), QStringLiteral("HF cancel")))
                                             .arg(hfNoiseCancelDepthToSliderValue(pendingSettings.hfNoiseCancelDepth)));
        hfNoiseCancelDepthLabel->setEnabled(enabled);
    }
    if (hfNoiseCancelRefGainLabel) {
        hfNoiseCancelRefGainLabel->setText(QStringLiteral("%1: %2 dB").arg(uiText(QStringLiteral("ref_gain"), QStringLiteral("Ref gain")))
                                               .arg(clampHfNoiseCancelRefGainDb(pendingSettings.hfNoiseCancelRefGainDb), 0, 'f', 1));
        hfNoiseCancelRefGainLabel->setEnabled(enabled);
    }
    if (hfNoiseCancelRefDelayLabel) {
        hfNoiseCancelRefDelayLabel->setText(QStringLiteral("%1: %2 ns").arg(uiText(QStringLiteral("ref_delay"), QStringLiteral("Ref delay")))
                                                .arg(static_cast<int>(std::lround(clampHfNoiseCancelRefDelayNs(pendingSettings.hfNoiseCancelRefDelayNs)))));
        hfNoiseCancelRefDelayLabel->setEnabled(enabled);
    }
    if (hfNoiseCancelRefTiltLabel) {
        hfNoiseCancelRefTiltLabel->setText(QStringLiteral("%1: %2 dB").arg(uiText(QStringLiteral("ref_tilt"), QStringLiteral("Ref tilt")))
                                               .arg(clampHfNoiseCancelRefTiltDb(pendingSettings.hfNoiseCancelRefTiltDb), 0, 'f', 1));
        hfNoiseCancelRefTiltLabel->setEnabled(enabled);
    }
    if (hfNoiseCancelDepthSlider) {
        hfNoiseCancelDepthSlider->setEnabled(enabled);
        hfNoiseCancelDepthSlider->setToolTip(
            QStringLiteral("Subtract the part of HF2 that stays correlated with HF1. 0% = HF1 only, 100% = normal cancellation."));
    }
    if (hfNoiseCancelRefGainSlider) {
        hfNoiseCancelRefGainSlider->setEnabled(enabled);
        hfNoiseCancelRefGainSlider->setToolTip(
            QStringLiteral("Manual HF2 reference level before subtraction."));
    }
    if (hfNoiseCancelRefDelaySlider) {
        hfNoiseCancelRefDelaySlider->setEnabled(enabled);
        hfNoiseCancelRefDelaySlider->setToolTip(
            QStringLiteral("Manual HF2 reference delay/phase correction before subtraction."));
    }
    if (hfNoiseCancelRefTiltSlider) {
        hfNoiseCancelRefTiltSlider->setEnabled(enabled);
        hfNoiseCancelRefTiltSlider->setToolTip(
            QStringLiteral("Manual HF2 reference slope across the visible HF band."));
    }
    if (hfNoiseCancelFreezeCheckbox) {
        hfNoiseCancelFreezeCheckbox->setEnabled(enabled);
        hfNoiseCancelFreezeCheckbox->setToolTip(
            QStringLiteral("Hold the small adaptive trim added on top of the manual HF2 reference"));
    }
}

void YourClassName::onfftLengthEntered() {
    const int newFftLength = fftComboBox->currentText().toInt();
    applyFftLengthChange(newFftLength, true);
}

bool YourClassName::applyFftLengthChange(int newFftLength, bool notifyRemote) {
    if (newFftLength <= 0) {
        qDebug() << "Invalid FFT length selected.";
        return false;
    }

    if (pendingSettings.fftLength == newFftLength && fftResult) {
        return true;
    }

    qDebug() << "[FFT] applying FFT length"
             << "previous" << pendingSettings.fftLength
             << "new" << newFftLength
             << "state" << runStateName(runState)
             << "networkMode" << static_cast<int>(networkMode)
             << "processingMode" << static_cast<int>(networkProcessingMode);

    pendingSettings.fftLength = newFftLength;
    publishSettingsToGlobals();
    updateSpectrumTimerInterval();
    fftResult = std::make_unique<FFTResult>();

    if (fftComboBox) {
        fftComboBox->blockSignals(true);
        fftComboBox->setCurrentText(QString::number(pendingSettings.fftLength));
        fftComboBox->blockSignals(false);
    }

    if (notifyRemote && isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }

    return true;
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
    if (!sampleBox) {
        return;
    }

    QSignalBlocker sampleBoxBlocker(sampleBox);
    auto addDefaultSampleRates = [this]() {
        if (sampleBox->count() > 0) {
            return;
        }
        const QVector<double> defaultRates = {
            8000000.0, 10000000.0, 12500000.0, 16000000.0, 20000000.0,
            25000000.0, 32000000.0, 40000000.0, 50000000.0, 80000000.0
        };
        for (const double rate : defaultRates) {
            sampleBox->addItem(formatSampleRate(rate), rate);
        }
        const int defaultIndex = sampleBox->findData(pendingSettings.sampleRate);
        if (defaultIndex >= 0) {
            sampleBox->setCurrentIndex(defaultIndex);
        }
        qDebug() << "[FobosDevices] using fallback sample-rate list; no local receiver is required for client control";
    };

    void *sampleRateDevice = activeFobosDevice();
    FobosApiKind sampleRateApiKind = activeFobosApiKind;
    bool openedForSampleRates = false;

    int ret = FOBOS_ERR_OK;
    if (!sampleRateDevice) {
        const FobosDeviceInfo selectedInfo = selectedFobosDeviceInfo();
        sampleRateApiKind = selectedInfo.apiKind;
        if (selectedInfo.apiKind == FobosApiKind::Agile) {
            fobos_sdr_dev_t *openedDevice = nullptr;
            ret = openFobosAgileDeviceSafely(&openedDevice, static_cast<uint32_t>(selectedInfo.nativeIndex));
            sampleRateDevice = openedDevice;
        } else {
            fobos_dev_t *openedDevice = nullptr;
            ret = openFobosDeviceSafely(&openedDevice, static_cast<uint32_t>(selectedInfo.nativeIndex));
            sampleRateDevice = openedDevice;
        }
        openedForSampleRates = (ret == FOBOS_ERR_OK && sampleRateDevice);
    }
    if (ret != FOBOS_ERR_OK) {
        qDebug() << "[FobosDevices] sample-rate list unavailable; using fallback, open result:" << ret;
        addDefaultSampleRates();
        return;
    }
    if (!sampleRateDevice) {
        qDebug() << "Device is not initialized.";
        addDefaultSampleRates();
        return;
    }
    double sampleRates[100];
    unsigned int count = 100;
    int result = sampleRateApiKind == FobosApiKind::Agile
                     ? getFobosAgileSampleRatesSafely(static_cast<fobos_sdr_dev_t*>(sampleRateDevice), sampleRates, &count)
                     : getFobosSampleRatesSafely(static_cast<fobos_dev_t*>(sampleRateDevice), sampleRates, &count);
    if (result != FOBOS_ERR_OK) {
        qDebug() << "Failed to get sample rates, error code:" << result;
        if (openedForSampleRates) {
            if (sampleRateApiKind == FobosApiKind::Agile) {
                closeFobosAgileDeviceSafely(static_cast<fobos_sdr_dev_t*>(sampleRateDevice));
            } else {
                closeFobosDeviceSafely(static_cast<fobos_dev_t*>(sampleRateDevice));
            }
        }
        addDefaultSampleRates();
        return;
    }
    sampleBox->clear();
    for (unsigned int i = 0; i < count; ++i) {
        QString formattedRate = formatSampleRate(sampleRates[i]);
        sampleBox->addItem(formattedRate, sampleRates[i]);
    }
    if (openedForSampleRates) {
        if (sampleRateApiKind == FobosApiKind::Agile) {
            closeFobosAgileDeviceSafely(static_cast<fobos_sdr_dev_t*>(sampleRateDevice));
        } else {
            closeFobosDeviceSafely(static_cast<fobos_dev_t*>(sampleRateDevice));
        }
    }
}

void YourClassName::updateSpectrum() {
    if (!fftResult) {
        return;
    }

    const bool channelIqRecordingOnly =
        isChannelIqRecordingActive() &&
        !(networkMode == NetworkMode::Server && isClientIqProcessingMode());
    if (channelIqRecordingOnly) {
        return;
    }

    const bool traceFrame = spectrumDebugFramesRemaining > 0;
    QElapsedTimer traceTimer;
    if (traceFrame) {
        traceTimer.start();
        qDebug() << "[Spectrum] update begin"
                 << "framesLeft" << spectrumDebugFramesRemaining
                 << "sampleRate" << pendingSettings.sampleRate
                 << "fftLength" << pendingSettings.fftLength
                 << "iqSnapshotFloats" << IqBuffer::size()
                 << "queuedBlocks" << IqBuffer::queuedBlocks();
    }

    auto finishTrace = [&](const char *stage,
                           const std::vector<float> &frequencies,
                           const std::vector<float> &magnitudes) {
        if (!traceFrame) {
            return;
        }
        qDebug() << "[Spectrum] update" << stage
                 << "elapsedMs" << traceTimer.elapsed()
                 << "freqCount" << frequencies.size()
                 << "magCount" << magnitudes.size()
                 << "iqSnapshotFloats" << IqBuffer::size()
                 << "queuedBlocks" << IqBuffer::queuedBlocks();
        --spectrumDebugFramesRemaining;
    };

    if (liveRetuneSettleTimer.isValid()) {
        const qint64 elapsedMs = liveRetuneSettleTimer.elapsed();
        const qint64 settleMs = liveRetuneSettleDurationMs > 0 ? liveRetuneSettleDurationMs : LIVE_RETUNE_SETTLE_MS;
        IqBuffer::clear();
        if (elapsedMs < settleMs) {
            if (traceFrame) {
                qDebug() << "[Spectrum] update retune_settle"
                         << "elapsedMs" << elapsedMs
                         << "settleMs" << settleMs
                         << "queuedBlocks" << IqBuffer::queuedBlocks();
                --spectrumDebugFramesRemaining;
            }
            return;
        }
        liveRetuneSettleTimer.invalidate();
        if (traceFrame) {
            qDebug() << "[Spectrum] update retune_settle_done"
                     << "elapsedMs" << elapsedMs
                     << "settleMs" << settleMs
                     << "queuedBlocks" << IqBuffer::queuedBlocks();
            --spectrumDebugFramesRemaining;
        }
        return;
    }

    //dataq = new float[dataSize];
        //for (int i = 0; i < 8; ++i){
        //int setrf = fobos_rx_set_frequency(device, globalFrequency + globalSampleRate * i, &actualFrequency);
        //memcpy(iqData + i * DEFAULT_BUF_LEN/8, dataq, DEFAULT_BUF_LEN/8 * sizeof(float));
        //}
    std::vector<float> spectrumFrequencies;
    std::vector<float> spectrumMagnitudes;
    std::vector<float> referenceMagnitudes;
    bool haveSpectrum = false;
    RadioSettings spectrumSettings = spectrumProcessingSettings();
    double scanCenterFrequency = currentAgileScanCenterFrequencyHz();
    if (!std::isfinite(scanCenterFrequency)) {
        scanCenterFrequency = currentStandardScanCenterFrequencyHz();
    }
    if (std::isfinite(scanCenterFrequency) && spectrumSettings.inputMode == INPUT_RF) {
        spectrumSettings.centerFrequency = scanCenterFrequency;
        spectrumSettings.actualFrequency = scanCenterFrequency;
    }
    double fullMinFrequency = minFrequency;
    double fullMaxFrequency = maxFrequency;
    if (spectrumSettings.inputMode == INPUT_RF && spectrumSettings.sampleRate > 0.0) {
        fullMinFrequency = spectrumSettings.centerFrequency - spectrumSettings.sampleRate * 0.5;
        fullMaxFrequency = spectrumSettings.centerFrequency + spectrumSettings.sampleRate * 0.5;
    } else if (spectrumSettings.inputMode == INPUT_HF_COMBINED && spectrumSettings.sampleRate > 0.0) {
        fullMinFrequency = -spectrumSettings.sampleRate * 0.5;
        fullMaxFrequency = spectrumSettings.sampleRate * 0.5;
    } else if (spectrumSettings.sampleRate > 0.0) {
        fullMinFrequency = 0.0;
        fullMaxFrequency = spectrumSettings.sampleRate * 0.5;
    }
    if (!std::isfinite(fullMinFrequency) ||
        !std::isfinite(fullMaxFrequency) ||
        fullMaxFrequency <= fullMinFrequency) {
        fullMinFrequency = minFrequency;
        fullMaxFrequency = maxFrequency;
    }

    const double fullSpan = (std::max)(1.0, fullMaxFrequency - fullMinFrequency);
    double visibleSpan = spectrumSettings.sampleRate * (currentScale / 100.0);
    if (!std::isfinite(visibleSpan) || visibleSpan <= 0.0) {
        visibleSpan = fullSpan;
    }
    visibleSpan = (std::clamp)(visibleSpan, 1.0, fullSpan);

    double visibleCenter = pendingSettings.listeningFrequency;
    if (!std::isfinite(visibleCenter) ||
        visibleCenter < fullMinFrequency ||
        visibleCenter > fullMaxFrequency) {
        visibleCenter = std::isfinite(scanCenterFrequency)
                            ? scanCenterFrequency
                            : (fullMinFrequency + fullMaxFrequency) * 0.5;
    }
    visibleCenter = (std::clamp)(visibleCenter, fullMinFrequency, fullMaxFrequency);

    double frameMinFrequency = visibleCenter - visibleSpan * 0.5;
    frameMinFrequency = (std::clamp)(frameMinFrequency,
                                     fullMinFrequency,
                                     fullMaxFrequency - visibleSpan);
    double frameMaxFrequency = frameMinFrequency + visibleSpan;

    try {
        haveSpectrum = fftResult->storeFFTResults(spectrumSettings,
                                                  spectrumFrequencies,
                                                  spectrumMagnitudes,
                                                  &referenceMagnitudes);
    } catch (const std::bad_alloc &error) {
        qCritical() << "[Spectrum] bad_alloc" << error.what()
                    << "sampleRate" << pendingSettings.sampleRate
                    << "fftLength" << pendingSettings.fftLength;
        updateTimer->stop();
        finishTrace("bad_alloc", spectrumFrequencies, spectrumMagnitudes);
        return;
    } catch (const std::exception &error) {
        qCritical() << "[Spectrum] exception" << error.what();
        finishTrace("exception", spectrumFrequencies, spectrumMagnitudes);
        return;
    } catch (...) {
        qCritical() << "[Spectrum] unknown exception";
        finishTrace("unknown_exception", spectrumFrequencies, spectrumMagnitudes);
        return;
    }

    if (!haveSpectrum || spectrumFrequencies.empty() || spectrumMagnitudes.empty()) {
        finishTrace("no_data", spectrumFrequencies, spectrumMagnitudes);
        return;
    }
    updateSpurCalibration(spectrumFrequencies, spectrumMagnitudes, spectrumSettings.centerFrequency);
    applySpurSuppression(spectrumFrequencies, spectrumMagnitudes, spectrumSettings.centerFrequency);
    updateScanMeasurement(spectrumFrequencies, spectrumMagnitudes);

    std::vector<float> displayFrequencies = spectrumFrequencies;
    std::vector<float> displayMagnitudes = spectrumMagnitudes;
    std::vector<float> displayReferenceMagnitudes = referenceMagnitudes;
    std::vector<float> displayMeasurementFrequencies = spectrumFrequencies;
    std::vector<float> dmrHunterFrequencies = spectrumFrequencies;
    std::vector<float> dmrHunterMagnitudes = spectrumMagnitudes;
    std::vector<float> fpvHunterFrequencies = spectrumFrequencies;
    std::vector<float> fpvHunterMagnitudes = spectrumMagnitudes;
    std::vector<float> digitalVideoHunterFrequencies = spectrumFrequencies;
    std::vector<float> digitalVideoHunterMagnitudes = spectrumMagnitudes;
    double displayCenterFrequency = spectrumSettings.centerFrequency;
    double displayMinFrequency = frameMinFrequency;
    double displayMaxFrequency = frameMaxFrequency;
    int displayFftLength = static_cast<int>(displayFrequencies.size());
    QVector<ScanVisualSegment> displayScanSegments;
    const bool agileScanVisualActive =
        agileScanRunning &&
        activeFobosApiKind == FobosApiKind::Agile &&
        spectrumSettings.inputMode == INPUT_RF &&
        activeAgileScanFrequencies.size() > 1;
    const bool standardScanVisualActive =
        standardScanRunning &&
        spectrumSettings.inputMode == INPUT_RF &&
        activeStandardScanFrequencies.size() > 1;
    const bool scanVisualActive = agileScanVisualActive || standardScanVisualActive;
    const QVector<double> &scanVisualFrequencies =
        standardScanVisualActive ? activeStandardScanFrequencies : activeAgileScanFrequencies;
    if (scanVisualActive) {
        int scanVisualBins = 4096;
        if (graphWidget && graphWidget->width() > 0) {
            scanVisualBins = (std::max)(scanVisualBins, graphWidget->width() * 2);
        }
        if (waterfallWidget && waterfallWidget->width() > 0) {
            scanVisualBins = (std::max)(scanVisualBins, waterfallWidget->width() * 2);
        }
        if (scanVisualAssembler.configure(scanVisualFrequencies,
                                          spectrumSettings.sampleRate,
                                          scanVisualBins)) {
            const ScanVisualFrame scanFrame =
                scanVisualAssembler.update(spectrumSettings.centerFrequency,
                                           spectrumFrequencies,
                                           spectrumMagnitudes,
                                           referenceMagnitudes);
            if (scanFrame.valid) {
                displayFrequencies = scanFrame.frequencies;
                displayMagnitudes = scanFrame.magnitudes;
                displayReferenceMagnitudes = scanFrame.referenceMagnitudes;
                displayMeasurementFrequencies = scanFrame.actualFrequencies;
                displayCenterFrequency = scanFrame.centerFrequency;
                displayMinFrequency = scanFrame.minFrequency;
                displayMaxFrequency = scanFrame.maxFrequency;
                displayFftLength = scanFrame.fftLength;
                displayScanSegments = scanFrame.segments;
                if (scanFrame.actualFrequencies.size() == scanFrame.magnitudes.size()) {
                    dmrHunterFrequencies = scanFrame.actualFrequencies;
                    dmrHunterMagnitudes = scanFrame.magnitudes;
                    fpvHunterFrequencies = scanFrame.actualFrequencies;
                    fpvHunterMagnitudes = scanFrame.magnitudes;
                    digitalVideoHunterFrequencies = scanFrame.actualFrequencies;
                    digitalVideoHunterMagnitudes = scanFrame.magnitudes;
                }
            }
        }
    } else {
        scanVisualAssembler.reset();
    }
    updateDmrHunter(dmrHunterFrequencies, dmrHunterMagnitudes);
    updateFpvHunter(fpvHunterFrequencies, fpvHunterMagnitudes);
    updateDigitalVideoHunter(digitalVideoHunterFrequencies, digitalVideoHunterMagnitudes);

    if (spectrumTuningDebugFramesRemaining > 0 &&
        displayFrequencies.size() == displayMagnitudes.size() &&
        !displayFrequencies.empty()) {
        --spectrumTuningDebugFramesRemaining;

        std::array<int, 3> peakIndices = {-1, -1, -1};
        std::array<float, 3> peakLevels = {
            -std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity()};
        int listeningIndex = -1;
        double listeningDelta = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(displayFrequencies.size()); ++i) {
            const double frequency = displayFrequencies[static_cast<std::size_t>(i)];
            const float level = displayMagnitudes[static_cast<std::size_t>(i)];
            if (!std::isfinite(frequency) || !std::isfinite(level)) {
                continue;
            }

            const double delta = std::abs(frequency - pendingSettings.listeningFrequency);
            if (delta < listeningDelta) {
                listeningDelta = delta;
                listeningIndex = i;
            }

            for (int slot = 0; slot < 3; ++slot) {
                if (level <= peakLevels[static_cast<std::size_t>(slot)]) {
                    continue;
                }
                for (int move = 2; move > slot; --move) {
                    peakLevels[static_cast<std::size_t>(move)] = peakLevels[static_cast<std::size_t>(move - 1)];
                    peakIndices[static_cast<std::size_t>(move)] = peakIndices[static_cast<std::size_t>(move - 1)];
                }
                peakLevels[static_cast<std::size_t>(slot)] = level;
                peakIndices[static_cast<std::size_t>(slot)] = i;
                break;
            }
        }

        QStringList peakSummary;
        for (int slot = 0; slot < 3; ++slot) {
            const int index = peakIndices[static_cast<std::size_t>(slot)];
            if (index < 0) {
                continue;
            }
            const double frequency = displayFrequencies[static_cast<std::size_t>(index)];
            const double offset = frequency - displayCenterFrequency;
            peakSummary << QStringLiteral("%1MHz/%2kHz/%3dB")
                               .arg(frequency / 1000000.0, 0, 'f', 6)
                               .arg(offset / 1000.0, 0, 'f', 1)
                               .arg(peakLevels[static_cast<std::size_t>(slot)], 0, 'f', 1);
        }

        const float listeningLevel =
            listeningIndex >= 0
                ? displayMagnitudes[static_cast<std::size_t>(listeningIndex)]
                : std::numeric_limits<float>::quiet_NaN();
        qDebug() << "[SpectrumTune]"
                 << "center" << displayCenterFrequency
                 << "listening" << pendingSettings.listeningFrequency
                 << "actual" << pendingSettings.actualFrequency
                 << "range" << displayMinFrequency << displayMaxFrequency
                 << "sampleRate" << spectrumSettings.sampleRate
                 << "listenOffset" << (pendingSettings.listeningFrequency - displayCenterFrequency)
                 << "listeningBinDelta" << listeningDelta
                 << "listeningLevel" << listeningLevel
                 << "peaks" << peakSummary.join(QStringLiteral(", "));
    }

    if (traceFrame) {
        qDebug() << "[Spectrum] before graph"
                 << "elapsedMs" << traceTimer.elapsed()
                 << "freqCount" << displayFrequencies.size()
                 << "magCount" << displayMagnitudes.size()
                 << "scanVisual" << scanVisualActive;
    }
    const bool suppressLocalVisual =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    if (!suppressLocalVisual) {
        if (scaleWidget) {
            double scaleListening = pendingSettings.listeningFrequency;
            if (!displayScanSegments.isEmpty()) {
                if (!scanListeningLockEnabled &&
                    !actualFrequencyInsideScanSegments(scaleListening, displayScanSegments)) {
                    scaleListening =
                        fallbackActualFrequencyForScanSegments(displayScanSegments, displayCenterFrequency);
                }
            } else if (scaleListening < displayMinFrequency ||
                       scaleListening > displayMaxFrequency) {
                scaleListening = displayCenterFrequency;
            }
            scaleWidget->setScanSegments(displayScanSegments);
            scaleWidget->setTuning(scaleListening,
                                   displayCenterFrequency,
                                   pendingSettings.bandwidth,
                                   pendingSettings.modulationType);
            scaleWidget->setRange(displayMinFrequency, displayMaxFrequency);
        }
        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
        graphWidget->setScanSegments(displayScanSegments);
        graphWidget->setData(displayFrequencies,
                             displayMagnitudes,
                             displayMinFrequency,
                             displayMaxFrequency,
                             displayFftLength,
                             colorf);
        const std::vector<float> &measurementFrequencies =
            displayMeasurementFrequencies.size() == displayMagnitudes.size()
                ? displayMeasurementFrequencies
                : displayFrequencies;
        const std::vector<float> measurementOverlay =
            scanMeasurementOverlay(measurementFrequencies, static_cast<int>(displayMagnitudes.size()));
        graphWidget->setOverlayData(!measurementOverlay.empty() ? measurementOverlay : displayReferenceMagnitudes,
                                    !measurementOverlay.empty() ||
                                        (pendingSettings.inputMode == INPUT_HF_NOISE_CANCEL &&
                                         !displayReferenceMagnitudes.empty()));
        if (traceFrame) {
            qDebug() << "[Spectrum] before waterfall" << "elapsedMs" << traceTimer.elapsed();
        }
        waterfallWidget->setData(displayFrequencies,
                                 displayMagnitudes,
                                 displayMinFrequency,
                                 displayMaxFrequency,
                                 displayFftLength,
                                 secondGraph,
                                 contrast,
                                 sensitivity,
                                 displayLevelMin,
                                 displayLevelMax);
        waterfallWidget->setScanSegments(displayScanSegments);
    } else if (traceFrame) {
        qDebug() << "[Spectrum] local server visual update skipped" << "elapsedMs" << traceTimer.elapsed();
    }
    sendNetworkSpectrumFrame(displayFrequencies,
                             displayMagnitudes,
                             displayReferenceMagnitudes,
                             displayCenterFrequency,
                             displayMinFrequency,
                             displayMaxFrequency,
                             displayScanSegments);
    finishTrace("end", displayFrequencies, displayMagnitudes);
    advanceStandardScanIfNeeded();
    //waterfallWidget->setData(fftFrequencies, fftMagnitudes, minFrequency, maxFrequency, fftLength, secondGraph, contrast, sensitivity);
    //qDebug() << "all took" << timer.elapsed() << "milliseconds";
}

void YourClassName::onSampleRateChanged(int index) {
    qDebug() << "[FobosLifecycle] onSampleRateChanged enter"
             << "index" << index
             << "state" << runStateName(runState)
             << "deviceOpened" << deviceOpened
             << "processorRunning" << (processor && processor->isRunning());

    if (!sampleBox || index < 0) {
        return;
    }

    bool ok = false;
    const double selectedSampleRate = sampleBox->itemData(index).toDouble(&ok);

    if (!ok || selectedSampleRate <= 0.0) {
        qDebug() << "Invalid sample rate selected.";
        return;
    }

    const double previousSampleRate = pendingSettings.sampleRate;
    const bool sampleRateChanged =
        std::abs(previousSampleRate - selectedSampleRate) > 0.5;

    qDebug() << "[FobosLifecycle] sample rate selected"
             << "previous" << pendingSettings.sampleRate
             << "selected" << selectedSampleRate
             << "changed" << sampleRateChanged
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind);

    if (!sampleRateChanged) {
        return;
    }

    pendingSettings.sampleRate = selectedSampleRate;
    normalizeStandardScanCentersUi(false);
    normalizeTuning(pendingSettings);
    publishSettingsToGlobals();
    settingRange();

    if (isNetworkClientMode()) {
        if (isClientIqProcessingMode()) {
            resetNetworkIqReceptionState(false, false, pendingSettings.audioEnabled && !isFullIqProcessingMode());
        }
        scheduleRemoteSettingsCommand();
        return;
    }

    if (!isIdle()) {
        const bool liveAgileRfSampleRate =
            activeFobosApiKind == FobosApiKind::Agile &&
            pendingSettings.inputMode == INPUT_RF &&
            !agileScanEnabled &&
            hasActiveFobosDevice() &&
            processor &&
            processor->isRunning();
        if (liveAgileRfSampleRate) {
            double actualRate = selectedSampleRate;
            qDebug() << "[FobosLifecycle] applying Agile RF sample-rate live"
                     << "previous" << previousSampleRate
                     << "requested" << selectedSampleRate;
            clearLiveSpectrumSnapshot(false);
            const int result = setActiveSampleRateSafely(selectedSampleRate, &actualRate);
            qDebug() << "[FobosLifecycle] Agile RF live sample-rate result"
                     << "result" << result
                     << "actual" << actualRate;
            if (result != FOBOS_ERR_OK) {
                qDebug() << "[FobosLifecycle] Agile RF live sample-rate failed; restoring previous UI state"
                         << "error" << result;
                pendingSettings.sampleRate = previousSampleRate;
                normalizeTuning(pendingSettings);
                publishSettingsToGlobals();
                settingRange();
                return;
            }

            globalSampleRate = actualRate;
            pendingSettings.sampleRate = actualRate;
            appliedSampleRate = actualRate;
            if (hardwareSettingsApplied) {
                appliedHardwareSettings.sampleRate = actualRate;
            }
            sampleRateReopenRequired = false;
            if (processor) {
                processor->setSampleRateHint(actualRate);
            }
            const double autoBandwidthRatio = agileRfAutoBandwidthRatio(actualRate);
            qDebug() << "[FobosLifecycle] refresh Agile auto bandwidth after live sample-rate change"
                     << autoBandwidthRatio;
            const int bandwidthResult = setFobosAgileAutoBandwidthSafely(agileDevice, autoBandwidthRatio);
            qDebug() << "[FobosLifecycle] Agile auto bandwidth after live sample-rate change"
                     << "result" << bandwidthResult;
            updateIqFrameProducerSettings();
            updateSpectrumTimerInterval();
            settingRange();
            clearLiveSpectrumSnapshot(false);
            liveRetuneSettleDurationMs = agileRfLiveSettleMs(actualRate, true);
            liveRetuneSettleTimer.start();
            spectrumTuningDebugFramesRemaining = 32;
            qDebug() << "[FobosLifecycle] Agile RF sample-rate live settle armed"
                     << "settleMs" << liveRetuneSettleDurationMs;
            savePersistentSettings();
            return;
        }
        restartStreamForHardwareChange();
        return;
    }

    if (hasActiveFobosDevice()) {
        const bool selectedRateMatchesOpenSession =
            appliedSampleRate > 0.0 &&
            std::abs(appliedSampleRate - selectedSampleRate) <= 0.5;

        sampleRateReopenRequired = !selectedRateMatchesOpenSession;
    } else {
        sampleRateReopenRequired = false;
    }

    qDebug() << "Sample rate will be applied on the next start.";
}

double YourClassName::fineTuneRangeHz() const {
    double visibleSpan = maxFrequency - minFrequency;
    if (!std::isfinite(visibleSpan) || visibleSpan <= 0.0) {
        visibleSpan = pendingSettings.sampleRate * (currentScale / 100.0);
    }
    if (!std::isfinite(visibleSpan) || visibleSpan <= 0.0) {
        visibleSpan = 100000.0;
    }
    return (std::clamp)(visibleSpan / FINE_TUNE_VISIBLE_RANGE_DIVISOR,
                        FINE_TUNE_MIN_RANGE_HZ,
                        FINE_TUNE_MAX_RANGE_HZ);
}

double YourClassName::fineTuneStepHz() const {
    return fineTuneRangeHz() /
           static_cast<double>((std::max)(std::abs(FINE_TUNE_DIAL_MIN),
                                          std::abs(FINE_TUNE_DIAL_MAX)));
}

void YourClassName::updateFineTuneLabel() {
    if (!fineTuneLabel) {
        return;
    }
    const double range = fineTuneRangeHz();
    if (fineTuneScaleWidget) {
        fineTuneScaleWidget->setRangeHz(range);
    }
    fineTuneLabel->setText(QStringLiteral("%1 +/- %2")
                               .arg(uiText(QStringLiteral("fine_tune"), QStringLiteral("Fine tune")),
                                    audioFilterFrequencyText(range)));
}

void YourClassName::updateFineTuneControlMode() {
    if (!fineTuneStack) {
        return;
    }
    fineTuneControlMode = fineTuneControlMode == FINE_TUNE_MODE_DIAL
                              ? FINE_TUNE_MODE_DIAL
                              : FINE_TUNE_MODE_SCALE;
    if (fineTuneScaleWidget && fineTuneScaleWidget->holdOffsetMode() != fineTuneScaleHoldMode) {
        QSignalBlocker blocker(fineTuneScaleWidget);
        fineTuneScaleWidget->setHoldOffsetMode(fineTuneScaleHoldMode);
    }
    fineTuneStack->setFixedHeight(fineTuneControlMode == FINE_TUNE_MODE_DIAL ? 78 : 58);
    fineTuneStack->setCurrentIndex(fineTuneControlMode == FINE_TUNE_MODE_DIAL ? 1 : 0);
    updateFineTuneScaleModeButton();
    onFineTuneDialReleased();
    updateFineTuneLabel();
}

void YourClassName::updateFineTuneScaleModeButton() {
    if (!fineTuneScaleModeButton) {
        return;
    }

    const bool holdMode = fineTuneScaleWidget ? fineTuneScaleWidget->holdOffsetMode()
                                              : fineTuneScaleHoldMode;
    fineTuneScaleHoldMode = holdMode;
    {
        QSignalBlocker blocker(fineTuneScaleModeButton);
        fineTuneScaleModeButton->setChecked(holdMode);
    }
    fineTuneScaleModeButton->setVisible(fineTuneControlMode == FINE_TUNE_MODE_SCALE);
    fineTuneScaleModeButton->setToolTip(holdMode
                                            ? QStringLiteral("Held fine tune offset. Double-click the scale or click here for temporary mode.")
                                            : QStringLiteral("Temporary fine tune. Double-click the scale or click here for held offset mode."));
    fineTuneScaleModeButton->setStyleSheet(holdMode
                                               ? QStringLiteral("QToolButton { background: #d65050; border: 1px solid #ff9a9a; border-radius: 8px; }"
                                                                "QToolButton:hover { background: #ec6969; }")
                                               : QStringLiteral("QToolButton { background: #3dbb68; border: 1px solid #8af0a8; border-radius: 8px; }"
                                                                "QToolButton:hover { background: #50d47a; }"));
}

void YourClassName::applyListeningFrequencyDelta(double deltaHz, int networkDelayMs) {
    if (!std::isfinite(deltaHz) || std::abs(deltaHz) < 0.01) {
        return;
    }

    const RadioSettings previousSettings = pendingSettings;
    pendingSettings.listeningFrequency += deltaHz;
    normalizeTuning(pendingSettings);
    applyCenterFrequencyToHardwareIfNeeded(previousSettings, "fine tune");
    publishSettingsToGlobals();

    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }
    if (listeningFrequencyControl) {
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
    }

    settingRange();
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand(networkDelayMs);
    }
}

void YourClassName::onFineTuneDialChanged(int value) {
    const int deltaSteps = value - fineTuneDialLastValue;
    fineTuneDialLastValue = value;
    if (deltaSteps == 0) {
        return;
    }

    applyListeningFrequencyDelta(deltaSteps * fineTuneStepHz(), 80);

    if (fineTuneDial && !fineTuneDial->isSliderDown()) {
        QTimer::singleShot(120, this, [this]() {
            onFineTuneDialReleased();
        });
    }
}

void YourClassName::onFineTuneDialReleased() {
    if (!fineTuneDial) {
        return;
    }
    QSignalBlocker blocker(fineTuneDial);
    fineTuneDial->setValue(0);
    fineTuneDialLastValue = 0;
}

void YourClassName::onListeningFrequencyEntered() {
    if (!listeningFrequencyControl) {
        return;
    }

    const RadioSettings previousSettings = pendingSettings;
    pendingSettings.listeningFrequency = listeningFrequencyControl->valueHz();
    normalizeTuning(pendingSettings);
    applyCenterFrequencyToHardwareIfNeeded(previousSettings, "listening control");
    publishSettingsToGlobals();
    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }
    {
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
    }
    qDebug() << "Frequency set to" << listeningFrequency << "Hz";
    settingRange();
    savePersistentSettings();
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}
    
void YourClassName::onFrequencyEntered() {
    const RadioSettings previousSettings = pendingSettings;
    if (pendingSettings.inputMode == INPUT_RF) {
        if (frequencyControl) {
            pendingSettings.centerFrequency = frequencyControl->valueHz();
            normalizeTuning(pendingSettings, true);
            applyCenterFrequencyToHardwareIfNeeded(previousSettings, "center control");
            publishSettingsToGlobals();
            QSignalBlocker frequencyBlocker(frequencyControl);
            frequencyControl->setValueHz(pendingSettings.centerFrequency);
            if (listeningFrequencyControl) {
                QSignalBlocker listeningBlocker(listeningFrequencyControl);
                listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
            }
            qDebug() << "Frequency set to" << globalFrequency << "Hz";
        }
    } else {
        pendingSettings.centerFrequency = 0;
        normalizeTuning(pendingSettings);
        publishSettingsToGlobals();
        if (frequencyControl) {
            QSignalBlocker frequencyBlocker(frequencyControl);
            frequencyControl->setValueHz(pendingSettings.centerFrequency);
        }
        if (listeningFrequencyControl) {
            QSignalBlocker listeningBlocker(listeningFrequencyControl);
            listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
        }
    }
    settingRange();
    savePersistentSettings();
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

QString YourClassName::formatFobosDeviceLabel(const FobosDeviceInfo &info) const {
    const QString serial = info.serial.isEmpty() ? QStringLiteral("unknown SN") : info.serial;
    const QString hw = info.hardwareRevision.isEmpty() ? QStringLiteral("?") : info.hardwareRevision;
    const QString fw = info.firmwareVersion.isEmpty() ? QStringLiteral("?") : info.firmwareVersion;
    return QString("[%1] #%2  SN %3  HW %4  FW %5")
        .arg(fobosApiDisplayName(info.apiKind))
        .arg(info.nativeIndex)
        .arg(serial)
        .arg(hw)
        .arg(fw);
}

void YourClassName::refreshFobosDeviceList(bool recoverUsb) {
    if (recoverUsb) {
        qDebug() << "[FobosDevices] USB recovery refresh requested"
                 << "runState" << static_cast<int>(runState)
                 << "deviceOpened" << deviceOpened
                 << "processorRunning" << (processor && processor->isRunning())
                 << "device" << activeFobosDevice();

        if (isRunningOrTransitioning() || (processor && processor->isRunning())) {
            qDebug() << "[FobosDevices] USB recovery skipped because Fobos is active or transitioning";
        } else {
            bool resetIssued = false;

            if (device) {
                qDebug() << "[FobosDevices] resetting idle standard session" << device;
                const int resetResult = resetFobosDeviceSafely(device);
                qDebug() << "[FobosDevices] idle standard reset result" << resetResult;
                device = nullptr;
                resetIssued = true;
            }
            if (agileDevice) {
                if (agileScanRunning) {
                    const int stopResult = stopFobosAgileScanSafely(agileDevice);
                    qDebug() << "[FobosDevices] idle agile scan stop before reset result" << stopResult;
                    agileScanRunning = false;
                    activeAgileScanFrequencies.clear();
                }
                qDebug() << "[FobosDevices] resetting idle agile session" << agileDevice;
                const int resetResult = resetFobosAgileDeviceSafely(agileDevice);
                qDebug() << "[FobosDevices] idle agile reset result" << resetResult;
                agileDevice = nullptr;
                resetIssued = true;
            }

            if (resetIssued) {
                deviceOpened = false;
                openedDeviceIndex = -1;
                openedNativeDeviceIndex = -1;
                appliedSampleRate = 0.0;
                appliedHardwareSettings = RadioSettings{};
                hardwareSettingsApplied = false;
                sampleRateReopenRequired = false;
                fobosCloseKnownUnsafe = false;
                activeFobosApiKind = FobosApiKind::Standard;
                openedDeviceApiKind = FobosApiKind::Standard;
                QThread::msleep(700);
            }

            const int standardResetCount = getFobosStandardDeviceCountSafely();
            for (int i = 0; i < standardResetCount; ++i) {
                fobos_dev_t *resetDevice = nullptr;
                const int openResult = openFobosDeviceSafely(&resetDevice, static_cast<uint32_t>(i));
                if (openResult != FOBOS_ERR_OK || !resetDevice) {
                    qDebug() << "[FobosDevices] standard recovery open failed"
                             << "index" << i << "result" << openResult;
                    continue;
                }
                const int resetResult = resetFobosDeviceSafely(resetDevice);
                qDebug() << "[FobosDevices] standard recovery reset"
                         << "index" << i << "result" << resetResult;
                resetIssued = true;
            }

            const int agileResetCount = getFobosAgileDeviceCountSafely();
            for (int i = 0; i < agileResetCount; ++i) {
                fobos_sdr_dev_t *resetDevice = nullptr;
                const int openResult = openFobosAgileDeviceSafely(&resetDevice, static_cast<uint32_t>(i));
                if (openResult != FOBOS_ERR_OK || !resetDevice) {
                    qDebug() << "[FobosDevices] agile recovery open failed"
                             << "index" << i << "result" << openResult;
                    continue;
                }
                const int resetResult = resetFobosAgileDeviceSafely(resetDevice);
                qDebug() << "[FobosDevices] agile recovery reset"
                         << "index" << i << "result" << resetResult;
                resetIssued = true;
            }

            if (resetIssued) {
                qDebug() << "[FobosDevices] waiting after USB recovery reset before enumeration";
                QThread::msleep(1200);
            } else {
                qDebug() << "[FobosDevices] no Fobos handles were available for USB recovery reset";
            }
        }
    }

    availableFobosDevices.clear();

    auto addDevice = [this](const FobosDeviceInfo &info) {
        if (!info.serial.isEmpty()) {
            for (int i = 0; i < availableFobosDevices.size(); ++i) {
                FobosDeviceInfo &existing = availableFobosDevices[i];
                if (existing.serial != info.serial) {
                    continue;
                }
                const bool preferAgile =
                    info.apiKind == FobosApiKind::Agile &&
                    (firmwareLooksAgile(info.firmwareVersion) ||
                     firmwareLooksAgile(existing.firmwareVersion));
                if (preferAgile) {
                    existing = info;
                }
                return;
            }
        }
        availableFobosDevices.append(info);
    };

    const int standardCount = getFobosStandardDeviceCountSafely();
    for (int i = 0; i < standardCount; ++i) {
        fobos_dev_t *infoDevice = nullptr;
        const int openResult = openFobosDeviceSafely(&infoDevice, static_cast<uint32_t>(i));
        if (openResult != FOBOS_ERR_OK || !infoDevice) {
            qDebug() << "[FobosDevices] standard open failed" << "index" << i << "result" << openResult;
            continue;
        }

        char hw[256] = {};
        char fw[256] = {};
        char manufacturer[256] = {};
        char product[256] = {};
        char serial[256] = {};
        const int infoResult = getFobosBoardInfoSafely(infoDevice, hw, fw, manufacturer, product, serial);
        closeFobosDeviceSafely(infoDevice);
        if (infoResult != FOBOS_ERR_OK) {
            qDebug() << "[FobosDevices] standard board info failed" << "index" << i << "result" << infoResult;
            continue;
        }

        FobosDeviceInfo info;
        info.apiKind = FobosApiKind::Standard;
        info.nativeIndex = i;
        info.hardwareRevision = QString::fromLocal8Bit(hw).trimmed();
        info.firmwareVersion = QString::fromLocal8Bit(fw).trimmed();
        info.manufacturer = QString::fromLocal8Bit(manufacturer).trimmed();
        info.product = QString::fromLocal8Bit(product).trimmed();
        info.serial = QString::fromLocal8Bit(serial).trimmed();
        info.label = formatFobosDeviceLabel(info);
        addDevice(info);
    }

    const int agileCount = getFobosAgileDeviceCountSafely();
    for (int i = 0; i < agileCount; ++i) {
        fobos_sdr_dev_t *infoDevice = nullptr;
        const int openResult = openFobosAgileDeviceSafely(&infoDevice, static_cast<uint32_t>(i));
        if (openResult != FOBOS_ERR_OK || !infoDevice) {
            qDebug() << "[FobosDevices] agile open failed" << "index" << i << "result" << openResult;
            continue;
        }

        char hw[256] = {};
        char fw[256] = {};
        char manufacturer[256] = {};
        char product[256] = {};
        char serial[256] = {};
        const int infoResult = getFobosAgileBoardInfoSafely(infoDevice, hw, fw, manufacturer, product, serial);
        closeFobosAgileDeviceSafely(infoDevice);
        if (infoResult != FOBOS_ERR_OK) {
            qDebug() << "[FobosDevices] agile board info failed" << "index" << i << "result" << infoResult;
            continue;
        }

        FobosDeviceInfo info;
        info.apiKind = FobosApiKind::Agile;
        info.nativeIndex = i;
        info.hardwareRevision = QString::fromLocal8Bit(hw).trimmed();
        info.firmwareVersion = QString::fromLocal8Bit(fw).trimmed();
        info.manufacturer = QString::fromLocal8Bit(manufacturer).trimmed();
        info.product = QString::fromLocal8Bit(product).trimmed();
        info.serial = QString::fromLocal8Bit(serial).trimmed();
        info.label = formatFobosDeviceLabel(info);
        addDevice(info);
    }

    qDebug() << "[FobosDevices] refreshed"
             << "standardCount" << standardCount
             << "agileCount" << agileCount
             << "usable" << availableFobosDevices.size();
    updateAgileScanControls();
}

QStringList YourClassName::getFobosDevices() {
    if (!deviceOpened && !(processor && processor->isRunning())) {
        refreshFobosDeviceList();
    }

    QStringList deviceList;
    for (const FobosDeviceInfo &info : std::as_const(availableFobosDevices)) {
        deviceList << info.label;
    }
    if (deviceList.isEmpty()) {
        deviceList << uiText(QStringLiteral("no_fobos_devices_detected"),
                             QStringLiteral("No Fobos devices detected"));
    }
    return deviceList;
}

YourClassName::FobosDeviceInfo YourClassName::selectedFobosDeviceInfo() const {
    int selected = pendingSettings.deviceIndex;
    if (comboBox) {
        bool ok = false;
        const int value = comboBox->currentData().toInt(&ok);
        if (ok) {
            selected = value;
        }
    }
    if (selected >= 0 && selected < availableFobosDevices.size()) {
        return availableFobosDevices[selected];
    }

    FobosDeviceInfo fallback;
    fallback.apiKind = FobosApiKind::Standard;
    fallback.nativeIndex = std::max(0, selected);
    fallback.label = QString("Standard device #%1").arg(fallback.nativeIndex);
    return fallback;
}

void YourClassName::listFobosDevices() {
    if (deviceOpened || (processor && processor->isRunning())) {
        QMessageBox::information(this,
                                 uiText(QStringLiteral("devices"), QStringLiteral("Devices")),
                                 uiText(QStringLiteral("stop_processing_before_listing_devices"),
                                        QStringLiteral("Stop processing before listing devices.")));
        return;
    }

    refreshFobosDeviceList();
    char standardLib[256] = {};
    char standardDriver[256] = {};
    char agileLib[256] = {};
    char agileDriver[256] = {};
    getFobosStandardApiInfoSafely(standardLib, standardDriver);
    getFobosAgileApiInfoSafely(agileLib, agileDriver);

    QString deviceInfo = QStringLiteral("%1 %2 (%3)\n%4 %5 (%6)\n\n%7 %8\n")
                             .arg(uiText(QStringLiteral("standard_api"), QStringLiteral("Standard API:")))
                             .arg(standardLib)
                             .arg(standardDriver)
                             .arg(uiText(QStringLiteral("agile_api"), QStringLiteral("Agile API:")))
                             .arg(agileLib)
                             .arg(agileDriver)
                             .arg(uiText(QStringLiteral("detected_devices"), QStringLiteral("Detected devices:")))
                             .arg(availableFobosDevices.size());
    for (int i = 0; i < availableFobosDevices.size(); ++i) {
        const FobosDeviceInfo &info = availableFobosDevices[i];
        deviceInfo += QStringLiteral("%1. %2\n    %3 %4\n    %5 %6\n")
                          .arg(i)
                          .arg(info.label)
                          .arg(uiText(QStringLiteral("manufacturer"), QStringLiteral("manufacturer:")))
                          .arg(info.manufacturer)
                          .arg(uiText(QStringLiteral("product"), QStringLiteral("product:")))
                          .arg(info.product);
    }
    QMessageBox::information(this,
                             uiText(QStringLiteral("fobos_devices"), QStringLiteral("Fobos Devices")),
                             deviceInfo);
}

void YourClassName::onDirectSamplingChanged(int index) {
    Q_UNUSED(index);

    if (!modeBox) {
        return;
    }

    const int value = modeBox->currentData().toInt();

    if (pendingSettings.inputMode == value) {
        return;
    }

    const int previousInputMode = pendingSettings.inputMode;
    pendingSettings.inputMode = value;

    if (value != INPUT_RF) {
        pendingSettings.centerFrequency = 0;
        if (previousInputMode == INPUT_RF ||
            !std::isfinite(pendingSettings.listeningFrequency)) {
            pendingSettings.listeningFrequency = value == INPUT_HF_COMBINED ? 0 : 1250000;
        }
    } else {
        if (!std::isfinite(pendingSettings.listeningFrequency) ||
            pendingSettings.listeningFrequency < RF_MIN_LISTENING_FREQUENCY) {
            pendingSettings.listeningFrequency = 100000000;
        }
        if (!std::isfinite(pendingSettings.centerFrequency) ||
            pendingSettings.centerFrequency < RF_MIN_CENTER_FREQUENCY) {
            pendingSettings.centerFrequency = pendingSettings.listeningFrequency;
        }
    }

    normalizeTuning(pendingSettings);

    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }

    if (listeningFrequencyControl) {
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
    }

    publishSettingsToGlobals();
    settingRange();
    updateHfNoiseCancelControls();

    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
        return;
    }

    if (!isIdle()) {
        restartStreamForHardwareChange();
        return;
    }

    qDebug() << "Input mode will be applied on the next start.";
}
void YourClassName::settingRange() {
    if (!scaleWidget || globalSampleRate <= 0.0) {
        return;
    }

    double newRange = globalSampleRate * (currentScale / 100.0);
    double overallMin = directMinFrequencyForMode(globalMode, globalSampleRate);
    double overallMax = directMaxFrequency(globalSampleRate);

    if (offlineIqPlaybackActive && !offlineIqPlaybackHasMetadata) {
        overallMin = globalFrequency - globalSampleRate / 2.0;
        overallMax = globalFrequency + globalSampleRate / 2.0;
    } else if (globalMode == INPUT_RF) {
        overallMin = (std::max)(RF_MIN_LISTENING_FREQUENCY,
                                globalFrequency - globalSampleRate / 2.0);
        overallMax = (std::max)(overallMin,
                                globalFrequency + globalSampleRate / 2.0);
    }

    if (listeningFrequencyControl) {
        const double controlMin = globalMode == INPUT_RF ? RF_MIN_LISTENING_FREQUENCY : overallMin;
        const double controlMax = globalMode == INPUT_RF ? RF_EXPERIMENTAL_MAX_FREQUENCY : overallMax;
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setRangeHz(controlMin, controlMax);
        listeningFrequencyControl->setValueHz((std::clamp)(pendingSettings.listeningFrequency, controlMin, controlMax));
    }

    const double availableRange = (std::max)(1.0, overallMax - overallMin);
    newRange = (std::clamp)(newRange, 1.0, availableRange);

    double clampedListening = (std::clamp)(pendingSettings.listeningFrequency, overallMin, overallMax);
	double newMin = clampedListening - newRange / 2.0;
    newMin = (std::clamp)(newMin, overallMin, overallMax - newRange);
    double newMax = newMin + newRange;
    minFrequency = newMin;
    maxFrequency = newMax;
    scaleWidget->setTuning(clampedListening, globalFrequency, globalBandwidth, globalModulationType);
    scaleWidget->setRange(minFrequency, maxFrequency);
    updateFineTuneLabel();
}

void YourClassName::onCheckboxStateChanged(int state) {
    QCheckBox *senderCheckbox = qobject_cast<QCheckBox*>(sender());
    if (senderCheckbox) {
        const uint8_t value = currentGpoValue();
        pendingSettings.gpoValue = value;
        qDebug() << "Checkbox state changed. New GPO value:" << value;
        if (hasActiveFobosDevice() && !isIdle() && hardwareSettingsApplied) {
            const int result = setActiveGpoSafely(pendingSettings.gpoValue);
            qDebug() << "[Live] GPO apply result" << result;
            if (result == FOBOS_ERR_OK) {
                appliedHardwareSettings.gpoValue = pendingSettings.gpoValue;
            }
        } else {
            qDebug() << "GPO state will be applied on the next start.";
        }
        if (isNetworkClientMode()) {
            scheduleRemoteSettingsCommand();
        }
    }
}

void YourClassName::onLnaGainChanged(int value) {
    pendingSettings.lnaGain = value;
    lnaGainLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("lna_gain"), QStringLiteral("LNA Gain"))).arg(value));

    if (hasActiveFobosDevice() && !isIdle() && hardwareSettingsApplied) {
        const int result = setActiveLnaGainSafely(static_cast<unsigned int>(value));
        qDebug() << "[Live] LNA gain apply result" << result;

        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.lnaGain = value;
        }
    }

    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

void YourClassName::onVgaGainChanged(int value) {
    pendingSettings.vgaGain = value;
    vgaGainLabel->setText(QStringLiteral("%1: %2").arg(uiText(QStringLiteral("vga_gain"), QStringLiteral("VGA Gain"))).arg(value));

    if (hasActiveFobosDevice() && !isIdle() && hardwareSettingsApplied) {
        const int result = setActiveVgaGainSafely(static_cast<unsigned int>(value));
        qDebug() << "[Live] VGA gain apply result" << result;

        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.vgaGain = value;
        }
    }

    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

void YourClassName::onClkChanged(int index) {
    if (!isIdle()) {
        qDebug() << "Stop processing before changing clock source.";
        revertHardwareControlsToSettings();
        return;
    }

    pendingSettings.clockSource = clkBox->currentData().toInt();
    qDebug() << "Clock source will be applied on the next start.";
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

void YourClassName::startFobosProcessing() {
    if (playbackManager && playbackManager->isPlaying()) {
        stopPlayback();
    }

    if (isChannelIqRecordingActive() &&
        networkMode != NetworkMode::Disabled &&
        isFullIqProcessingMode()) {
        stopRecording(false);
        updateRecordingStatus(QStringLiteral("Recording stopped: Channel IQ cannot run during Full IQ streaming"));
    }

    if (isNetworkClientMode()) {
        if (sendRemoteControlCommand("start")) {
            runState = RadioRunState::Running;
            updateUiForRunState();
            if (isClientIqProcessingMode()) {
                startNetworkClientProcessing();
            } else if (remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
        }
        return;
    }

    const bool watchdogRestart = automaticStreamRestart;
    automaticStreamRestart = false;
    if (!watchdogRestart) {
        streamStartupRetryCount = 0;
        restartAfterStartupWatchdog = false;
    }
    qDebug() << "[FobosLifecycle] Start requested"
             << "state" << runStateName(runState)
             << "deviceOpened" << deviceOpened
             << "processorRunning" << (processor && processor->isRunning())
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind)
             << "appliedSampleRate" << appliedSampleRate
             << "pendingSampleRate" << pendingSettings.sampleRate
             << "sampleRateReopenRequired" << sampleRateReopenRequired
             << "fobosCloseKnownUnsafe" << fobosCloseKnownUnsafe
             << "watchdogRestart" << watchdogRestart
             << "startupRetryCount" << streamStartupRetryCount;
    logMemorySnapshot("before start");
    if (!isIdle() || deviceOpened || (processor && processor->isRunning())) {
        qDebug() << "Warning: Processor is already running!";
        return;
    }

    runState = RadioRunState::Starting;
    qDebug() << "[FobosLifecycle] state changed" << runStateName(runState);
    updateUiForRunState();
    refreshSettingsFromUi();
    if (realignDmrCenterToListening(pendingSettings)) {
        qDebug() << "[FobosLifecycle] DMR start realigned RF center to listening frequency"
                 << "center" << pendingSettings.centerFrequency
                 << "listening" << pendingSettings.listeningFrequency;
        if (frequencyControl) {
            QSignalBlocker blocker(frequencyControl);
            frequencyControl->setValueHz(pendingSettings.centerFrequency);
        }
        if (listeningFrequencyControl) {
            QSignalBlocker blocker(listeningFrequencyControl);
            listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
        }
        settingRange();
    }
    const bool sampleRateDiffersFromOpenSession =
        hasActiveFobosDevice() && appliedSampleRate > 0.0 &&
        std::abs(appliedSampleRate - pendingSettings.sampleRate) > 0.5;
    if (sampleRateDiffersFromOpenSession) {
        sampleRateReopenRequired = true;
        qDebug() << "[FobosLifecycle] sample rate differs from previous run; reopening Fobos session before applying"
                 << "appliedSampleRate" << appliedSampleRate
                 << "pendingSampleRate" << pendingSettings.sampleRate;
    }
    publishSettingsToGlobals();

    const bool agileNormalRfOpenSession =
        hasActiveFobosDevice() &&
        activeFobosApiKind == FobosApiKind::Agile &&
        pendingSettings.inputMode == INPUT_RF &&
        !agileScanEnabled;
    if (agileNormalRfOpenSession && sampleRateReopenRequired) {
        qDebug() << "[FobosLifecycle] Agile RF sample-rate difference will be applied live; keeping session open"
                 << "appliedSampleRate" << appliedSampleRate
                 << "pendingSampleRate" << pendingSettings.sampleRate;
        sampleRateReopenRequired = false;
    }

    if (hasActiveFobosDevice() && sampleRateReopenRequired) {
        if (fobosCloseKnownUnsafe) {
            qDebug() << "[FobosLifecycle] previous Fobos close was unsafe; abandoning stale session pointer before reopen"
                     << activeFobosDevice();
            device = nullptr;
            agileDevice = nullptr;
            agileScanRunning = false;
            activeAgileScanFrequencies.clear();
            activeFobosApiKind = FobosApiKind::Standard;
            openedDeviceIndex = -1;
            openedNativeDeviceIndex = -1;
            appliedSampleRate = 0.0;
            appliedHardwareSettings = RadioSettings{};
            hardwareSettingsApplied = false;
            sampleRateReopenRequired = false;
            fobosCloseKnownUnsafe = false;
        }
    }

    if (hasActiveFobosDevice() && sampleRateReopenRequired) {
        if (processor && !processor->isRunning()) {
            processor->finalizeStopped();
        }
        qDebug() << "[FobosLifecycle] closing Fobos session before sample-rate change"
                 << "device" << activeFobosDevice()
                 << "apiKind" << fobosApiKindName(activeFobosApiKind)
                 << "appliedSampleRate" << appliedSampleRate
                 << "pendingSampleRate" << pendingSettings.sampleRate;
        if (!closeFobosSession(false)) {
            qDebug() << "[FobosLifecycle] Fobos session close returned an error before sample-rate change; trying a fresh open anyway";
        }
        qDebug() << "[FobosLifecycle] Fobos session closed for sample-rate change; waiting before reopen";
        QThread::msleep(500);
    }

    qDebug() << "[FobosLifecycle] opening Fobos session";
    if (!openFobosSession()) {
        qDebug() << "[FobosLifecycle] openFobosSession failed";
        clearLiveSpectrumSnapshot();
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return;
    }

    qDebug() << "[FobosLifecycle] applying Fobos settings";
    if (!applyFobosSettings()) {
        qDebug() << "Start aborted because Fobos settings could not be applied; closing Fobos session before retry.";
        closeFobosSession(true);
        clearLiveSpectrumSnapshot();
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return;
    }
    if (!stabilizeAgileFrequencyBeforeStreaming("start")) {
        qDebug() << "Start aborted because Agile frequency settle failed; closing Fobos session before retry.";
        closeFobosSession(true);
        clearLiveSpectrumSnapshot();
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return;
    }
    clearLiveSpectrumSnapshot(false);
    digitalDecoderGeneration.fetch_add(1, std::memory_order_relaxed);
    pendingDmrDecoderPcm.clear();
    pendingDmrDecoderSampleRate = 48000;
    droppedDigitalDecoderFramesSinceLog.store(0);
    if (digitalDecoder) {
        QMetaObject::invokeMethod(digitalDecoder,
                                  [decoder = digitalDecoder]() {
                                      decoder->reset();
                                  },
                                  Qt::QueuedConnection);
    }
    spectrumDebugFramesRemaining = fobosVerboseLoggingEnabled() ? 12 : 0;
    spectrumTuningDebugFramesRemaining = 32;
    updateSpectrumTimerInterval();
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[FobosLifecycle] clearing IQ buffer before reader start; preserving visual history";
    }

    deviceOpened = true;
    const bool serverIqStreaming = networkMode == NetworkMode::Server && isClientIqProcessingMode();
    const bool serverFullIqStreaming = networkMode == NetworkMode::Server && isFullIqProcessingMode();
    const bool serverChannelIqStreaming = networkMode == NetworkMode::Server && isChannelIqProcessingMode();
    const bool channelIqRecording = isChannelIqRecordingActive();
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    const bool serverAudioStreamingForFullIq = serverFullIqStreaming && pendingSettings.audioEnabled;
    const bool channelIqRecordingOnly = channelIqRecording && !serverIqStreaming;
    const bool serverLocalAudioEnabled =
        pendingSettings.audioEnabled &&
        !channelIqRecordingOnly &&
        (!serverIqStreaming || serverAudioStreamingForFullIq);
    const bool queueAudioBlocks =
        !channelIqRecordingOnly &&
        (!serverIqStreaming || serverAudioStreamingForFullIq);
    const bool publishIqSnapshot = !channelIqRecordingOnly;
    if (audioProcessor) {
        audioProcessor->setLocalPlaybackEnabled(!suppressServerLocalOutput);
    }
    if ((serverIqStreaming || channelIqRecording) && processor) {
        processor->configureNetworkIqStreaming(pendingSettings,
                                               true,
                                               serverChannelIqStreaming || channelIqRecording);
    }
    qDebug() << "[FobosLifecycle] starting DataProcessor"
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind)
             << "sampleRate" << pendingSettings.sampleRate
             << "syncEnabled" << pendingSettings.syncEnabled
             << "queueAudioBlocks" << queueAudioBlocks
             << "publishIqSnapshot" << publishIqSnapshot
             << "serverIqStreaming" << serverIqStreaming
             << "serverChannelIqStreaming" << serverChannelIqStreaming
             << "channelIqRecording" << channelIqRecording;
    processor->startProcessing(activeFobosDevice(),
                               activeFobosApiKind,
                               pendingSettings.syncEnabled,
                               pendingSettings.sampleRate,
                               queueAudioBlocks,
                               publishIqSnapshot,
                               serverIqStreaming || channelIqRecording,
                               agileScanEnabled &&
                                   !standardScanEnabled &&
                                   activeFobosApiKind == FobosApiKind::Agile);
    if (activeFobosApiKind == FobosApiKind::Agile &&
        pendingSettings.inputMode == INPUT_RF &&
        !agileScanEnabled) {
        liveRetuneSettleDurationMs = agileRfLiveSettleMs(pendingSettings.sampleRate, false);
        clearLiveSpectrumSnapshot(false);
        liveRetuneSettleTimer.start();
        qDebug() << "[FobosLifecycle] Agile start settle armed"
                 << "settleMs" << liveRetuneSettleDurationMs;
    }
    pendingAudioStartAfterStreamReady = serverLocalAudioEnabled;
    streamStartCallbackCount = processor ? processor->callbackCount() : 0;
    streamStartElapsedTimer.restart();
    if (streamWatchdogTimer) {
        qDebug() << "[FobosLifecycle] stream startup watchdog armed"
                 << "callbackCount" << streamStartCallbackCount
                 << "retryCount" << streamStartupRetryCount;
        streamWatchdogTimer->start();
    }
    runState = RadioRunState::Running;
    qDebug() << "[FobosLifecycle] state changed" << runStateName(runState)
             << "processorRunning" << (processor && processor->isRunning());
    updateUiForRunState();
    qDebug() << "Fobos fft Started";

    if (serverLocalAudioEnabled) {
        qDebug() << "[FobosLifecycle] audio start deferred until stream produces IQ callbacks";
    } else if (serverChannelIqStreaming) {
        qDebug() << "[NetworkIQ] server is streaming channelized IQ frames; local server audio is disabled";
    } else if (serverFullIqStreaming) {
        qDebug() << "[NetworkIQ] server is streaming full IQ frames; local server audio is disabled";
    }

    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[FobosLifecycle] starting spectrum timer";
    }
    if (serverFullIqStreaming && suppressServerLocalOutput) {
        qDebug() << "[NetworkIQ] server local spectrum timer disabled in full-IQ processing mode";
    } else {
        updateTimer->start();
    }
    applyServerLocalOutputPolicy();
    logMemorySnapshot("after start");
    qDebug() << "[FobosLifecycle] Start sequence complete";
}

void YourClassName::recreateDataProcessor() {
    qDebug() << "[FobosLifecycle] recreateDataProcessor enter"
             << "processor" << processor
             << "processorRunning" << (processor && processor->isRunning());
    if (processor) {
        delete processor;
    }
    processor = new DataProcessor(this);
    connectDataProcessorSignals();
    qDebug() << "[FobosLifecycle] recreateDataProcessor exit" << "processor" << processor;
}

void YourClassName::finishFobosStop(bool forcedRecovery) {
    qDebug() << "[FobosLifecycle] finishFobosStop enter"
             << "forcedRecovery" << forcedRecovery
             << "processorRunning" << (processor && processor->isRunning())
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind);

    if (stopPollTimer) {
        stopPollTimer->stop();
    }
    if (streamWatchdogTimer) {
        streamWatchdogTimer->stop();
    }

    if (processor) {
        processor->finalizeStopped();
    }

    bool closeSucceeded = true;
    if (forcedRecovery) {
        qDebug() << "[FobosLifecycle] forced stop recovery: abandoning Fobos session without close and recreating DataProcessor"
                 << "device" << activeFobosDevice()
                 << "apiKind" << fobosApiKindName(activeFobosApiKind);
        device = nullptr;
        agileDevice = nullptr;
        agileScanRunning = false;
        activeAgileScanFrequencies.clear();
        activeFobosApiKind = FobosApiKind::Standard;
        openedDeviceIndex = -1;
        openedNativeDeviceIndex = -1;
        appliedSampleRate = 0.0;
        appliedHardwareSettings = RadioSettings{};
        hardwareSettingsApplied = false;
        sampleRateReopenRequired = false;
        fobosCloseKnownUnsafe = false;
        if (processor && !processor->isRunning()) {
            recreateDataProcessor();
        }
    } else if (hasActiveFobosDevice()) {
        const bool keepIdleAgileSession =
            activeFobosApiKind == FobosApiKind::Agile &&
            pendingSettings.inputMode == INPUT_RF &&
            !agileScanEnabled &&
            !agileScanRunning;
        if (keepIdleAgileSession) {
            qDebug() << "[FobosLifecycle] clean Agile RF stop: keeping idle session open to avoid unsafe USB close"
                     << activeFobosDevice()
                     << "clearSpectrumAfterStop" << clearSpectrumAfterStop
                     << "sampleRateReopenRequired" << sampleRateReopenRequired;
            sampleRateReopenRequired = false;
        } else {
            qDebug() << "[FobosLifecycle] clean stop: closing Fobos session"
                     << (clearSpectrumAfterStop ? "and clearing IQ snapshot" : "; IQ snapshot remains visible");
            closeSucceeded = closeFobosSession(clearSpectrumAfterStop);
        }
        if (!closeSucceeded) {
            qDebug() << "[FobosLifecycle] clean stop: Fobos close returned an error; stale session pointer was abandoned";
        }
        if (processor && !processor->isRunning()) {
            recreateDataProcessor();
        }
    }

    if (clearSpectrumAfterStop) {
        clearLiveSpectrumSnapshot();
        clearSpectrumAfterStop = false;
    }

    deviceOpened = false;
    runState = RadioRunState::Idle;
    qDebug() << "[FobosLifecycle] state changed" << runStateName(runState);
    updateUiForRunState();
    logMemorySnapshot("after stop");
    qDebug() << "Stop requested: complete.";

    if (restartAfterStartupWatchdog && !forcedRecovery && closeSucceeded) {
        restartAfterStartupWatchdog = false;
        automaticStreamRestart = true;
        hardwareSettingsApplied = false;
        appliedHardwareSettings = RadioSettings{};
        appliedSampleRate = 0.0;
        qDebug() << "[FobosLifecycle] scheduling automatic restart after stream startup watchdog"
                 << "retryCount" << streamStartupRetryCount;
        QTimer::singleShot(350, this, &YourClassName::startFobosProcessing);
    } else if (restartAfterStartupWatchdog && !closeSucceeded) {
        restartAfterStartupWatchdog = false;
        automaticStreamRestart = false;
        qDebug() << "[FobosLifecycle] automatic restart skipped because device close failed";
    }
}

void YourClassName::checkStreamStartup() {
    if (runState != RadioRunState::Running || !processor || !processor->isRunning()) {
        if (streamWatchdogTimer) {
            streamWatchdogTimer->stop();
        }
        return;
    }

    const uint64_t callbackCount = processor->callbackCount();
    if (callbackCount > streamStartCallbackCount) {
        qDebug() << "[FobosLifecycle] stream startup watchdog satisfied"
                 << "callbackCount" << callbackCount
                 << "elapsedMs" << streamStartElapsedTimer.elapsed();
        if (streamWatchdogTimer) {
            streamWatchdogTimer->stop();
        }
        schedulePostStartRetune(QStringLiteral("stream ready"));
        if (pendingAudioStartAfterStreamReady) {
            pendingAudioStartAfterStreamReady = false;
            if (deviceOpened && processor && processor->isRunning()) {
                qDebug() << "[FobosLifecycle] starting deferred audio after IQ stream became active";
                audioProcessor->startDemodulation();
                qDebug() << "Fobos audio Started";
            } else {
                qDebug() << "[FobosLifecycle] deferred audio start skipped because SDR is no longer running";
            }
        }
        return;
    }

    const qint64 elapsedMs = streamStartElapsedTimer.isValid() ? streamStartElapsedTimer.elapsed() : 0;
    if (elapsedMs < 1800) {
        return;
    }

    qDebug() << "[FobosLifecycle] stream startup watchdog: no IQ callbacks after start"
             << "elapsedMs" << elapsedMs
             << "callbackCount" << callbackCount
             << "retryCount" << streamStartupRetryCount
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind)
             << "sampleRate" << pendingSettings.sampleRate;
    if (streamWatchdogTimer) {
        streamWatchdogTimer->stop();
    }
    pendingAudioStartAfterStreamReady = false;
    clearSpectrumAfterStop = true;

    const bool directSamplingStartup = pendingSettings.inputMode != INPUT_RF;
    if (streamStartupRetryCount < 1) {
        ++streamStartupRetryCount;
        restartAfterStartupWatchdog = true;
        qDebug() << "[FobosLifecycle] stream startup watchdog will retry once"
                 << "retryCount" << streamStartupRetryCount
                 << "directSampling" << directSamplingStartup;
    } else {
        restartAfterStartupWatchdog = false;
        qDebug() << "[FobosLifecycle] stream startup watchdog retry already used; leaving receiver stopped";
    }

    stopFobosProcessing();
}

void YourClassName::pollStopCompletion() {
    if (!processor) {
        qDebug() << "[FobosLifecycle] pollStopCompletion: no processor";
        finishFobosStop(false);
        return;
    }

    if (!processor->isRunning()) {
        qDebug() << "[FobosLifecycle] DataProcessor stopped asynchronously"
                 << "elapsedMs" << stopElapsedTimer.elapsed();
        finishFobosStop(false);
        return;
    }

    const qint64 elapsedMs = stopElapsedTimer.isValid() ? stopElapsedTimer.elapsed() : 0;
    if (stopCancelRetryCount < 4 && elapsedMs >= (stopCancelRetryCount + 1) * 1000) {
        ++stopCancelRetryCount;
        qDebug() << "[FobosLifecycle] retrying async cancel"
                 << "retry" << stopCancelRetryCount
                 << "elapsedMs" << elapsedMs;
        processor->requestStop();
        return;
    }

    if (elapsedMs < 6000) {
        return;
    }

    qDebug() << "[FobosLifecycle] DataProcessor did not stop after async cancel; forcing recovery"
             << "elapsedMs" << elapsedMs;
    if (stopPollTimer) {
        stopPollTimer->stop();
    }

    const bool forced = processor->forceStop(1000);
    qDebug() << "[FobosLifecycle] forced recovery result" << forced;
    if (forced) {
        finishFobosStop(true);
        return;
    }

    qDebug() << "[FobosLifecycle] forced recovery failed; UI remains in Stopping to avoid unsafe restart";
    updateUiForRunState();
}
 
void YourClassName::stopFobosProcessing() {
    if (playbackManager && playbackManager->isPlaying()) {
        stopPlayback();
        return;
    }

    if (recordingManager && recordingManager->isRecording()) {
        stopRecording(false);
    }

    if (isNetworkClientMode()) {
        const bool sent = sendRemoteControlCommand("stop");
        if (!sent) {
            qDebug() << "[Network] remote stop could not be sent; stopping local client state";
        }
        if (isClientIqProcessingMode()) {
            stopNetworkClientProcessing();
        } else if (remoteAudioPlayer) {
            remoteAudioPlayer->stop();
        }
        networkClientReconnectPending = false;
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return;
    }

    qDebug() << "[FobosLifecycle] Stop requested"
             << "state" << runStateName(runState)
             << "deviceOpened" << deviceOpened
             << "processorRunning" << (processor && processor->isRunning())
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind);
    logMemorySnapshot("before stop");
    if (runState == RadioRunState::Idle && !deviceOpened && !(processor && processor->isRunning())) {
        qDebug() << "Stop ignored because radio is already idle.";
        return;
    }
    if (runState == RadioRunState::Stopping) {
        qDebug() << "Stop ignored because radio is already stopping.";
        return;
    }

    runState = RadioRunState::Stopping;
    qDebug() << "[FobosLifecycle] state changed" << runStateName(runState);
    updateUiForRunState();
    if (streamWatchdogTimer) {
        streamWatchdogTimer->stop();
    }
    pendingAudioStartAfterStreamReady = false;
    updateTimer->stop();
    qDebug() << "Stop requested: spectrum timer stopped.";

    if (audioProcessor) {
        qDebug() << "Stop requested: stopping AudioProcessor.";
        audioProcessor->stopDemodulation();
        qDebug() << "Stop requested: AudioProcessor stopped.";
    }

    if (processor && (deviceOpened || processor->isRunning())) {
        qDebug() << "Stop requested: requesting DataProcessor stop.";
        stopCancelRetryCount = 0;
        processor->requestStop();
        stopElapsedTimer.restart();
        if (stopPollTimer) {
            stopPollTimer->start();
        }
        return;
    }

    finishFobosStop(false);
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon(QStringLiteral(":/icons/fobosapp.png")));
    installDiagnosticLogger();
    installCrashLogger();
    logFobosApiInfo();
    YourClassName window;
    window.show(); 

    qDebug() << "App started";
#ifdef _WIN32
        SetConsoleOutputCP(CP_UTF8);  // Устанавливаем UTF-8 для вывода
        SetConsoleCP(CP_UTF8);        // Устанавливаем UTF-8 для ввода
#endif
    return app.exec();
}
