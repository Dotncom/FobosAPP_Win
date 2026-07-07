#include "main.h"
#include "appconstants.h"
#include "gnssqthhelpers.h"

#include <QDebug>
#include <QMap>
#include <QPair>
#include <QSignalBlocker>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>
void YourClassName::startSpurCalibration() {
    spurCalibrationBins.clear();
    spurCalibrationFramesDone = 0;
    spurCalibrationTargetFrames = SPUR_CALIBRATION_TARGET_FRAMES;
    spurCalibrationBinHz = 0.0;
    spurCombStepHz = 0.0;
    spurCombSpacingHits = 0;
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
    spurCombStepHz = 0.0;
    spurCombSpacingHits = 0;
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

    spurCombStepHz = 0.0;
    spurCombSpacingHits = 0;
    if (candidates.size() >= 5) {
        constexpr double kMinCombStepHz = 1000.0;
        constexpr double kMaxCombStepHz = 2000000.0;
        constexpr double kCombStepBinHz = 250.0;
        QMap<qint64, int> spacingBins;
        QVector<double> offsets;
        offsets.reserve(candidates.size());
        for (const SpurMaskEntry &candidate : std::as_const(candidates)) {
            offsets.append(candidate.offsetHz);
        }
        std::sort(offsets.begin(), offsets.end());
        const int offsetCount = offsets.size();
        for (int i = 0; i < offsetCount; ++i) {
            for (int j = i + 1; j < offsetCount; ++j) {
                const double spacingHz = std::abs(offsets.at(j) - offsets.at(i));
                if (spacingHz < kMinCombStepHz) {
                    continue;
                }
                if (spacingHz > kMaxCombStepHz) {
                    break;
                }
                const qint64 bin = static_cast<qint64>(std::llround(spacingHz / kCombStepBinHz));
                ++spacingBins[bin];
            }
        }
        for (auto it = spacingBins.constBegin(); it != spacingBins.constEnd(); ++it) {
            if (it.value() > spurCombSpacingHits) {
                spurCombSpacingHits = it.value();
                spurCombStepHz = static_cast<double>(it.key()) * kCombStepBinHz;
            }
        }
        if (spurCombSpacingHits < 6) {
            spurCombStepHz = 0.0;
            spurCombSpacingHits = 0;
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
             << "combStepHz" << spurCombStepHz
             << "combHits" << spurCombSpacingHits
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
    const QString combSuffix =
        spurCombStepHz > 0.0
            ? QStringLiteral(", comb %1k/h%2")
                  .arg(spurCombStepHz / 1000.0, 0, 'f', 2)
                  .arg(spurCombSpacingHits)
            : QString();
    setSpurStatus(
        uiText(QStringLiteral("spur_mask_status"),
               QStringLiteral("Spur mask: %1, %2 offsets [%3%4]"))
            .arg(spurSuppressionEnabled
                     ? uiText(QStringLiteral("on"), QStringLiteral("on"))
                     : uiText(QStringLiteral("off"), QStringLiteral("off")))
            .arg(spurMaskEntries.size())
            .arg(offsets.join(QStringLiteral(", ")))
            .arg(suffix) +
        combSuffix);
}

void YourClassName::updateGnssSpurWatch(const std::vector<float> &frequencies,
                                        const std::vector<float> &magnitudes,
                                        double centerFrequency) {
    if (frequencies.empty() || magnitudes.empty() || frequencies.size() != magnitudes.size()) {
        return;
    }

    const GnssSystemPreset preset = gnssSystemPreset(gnssSystemId);
    const double targetHz = preset.id == QStringLiteral("all_l1")
                                ? GNSS_GPS_L1_HZ
                                : preset.targetHz;
    if (!std::isfinite(targetHz) || targetHz <= 0.0 ||
        !std::isfinite(centerFrequency) || centerFrequency <= 0.0) {
        return;
    }
    const double minHz = static_cast<double>(*std::min_element(frequencies.begin(), frequencies.end()));
    const double maxHz = static_cast<double>(*std::max_element(frequencies.begin(), frequencies.end()));
    if (targetHz < minHz || targetHz > maxHz) {
        return;
    }

    if (!gnssSpurWatchTimer.isValid()) {
        gnssSpurWatchTimer.start();
    }
    if (!gnssSpurLogTimer.isValid()) {
        gnssSpurLogTimer.start();
    }

    constexpr double watchSpanHz = 25000000.0;
    constexpr double binHz = 10000.0;
    QVector<QPair<int, float>> peaks;
    double average = 0.0;
    int averageCount = 0;
    for (int i = 0; i < static_cast<int>(frequencies.size()); ++i) {
        const double frequency = static_cast<double>(frequencies[static_cast<std::size_t>(i)]);
        const float level = magnitudes[static_cast<std::size_t>(i)];
        if (!std::isfinite(frequency) || !std::isfinite(level) ||
            std::abs(frequency - targetHz) > watchSpanHz) {
            continue;
        }
        average += static_cast<double>(level);
        ++averageCount;
    }
    if (averageCount <= 0) {
        return;
    }
    average /= static_cast<double>(averageCount);

    for (int i = 1; i + 1 < static_cast<int>(frequencies.size()); ++i) {
        const double frequency = static_cast<double>(frequencies[static_cast<std::size_t>(i)]);
        const float level = magnitudes[static_cast<std::size_t>(i)];
        if (!std::isfinite(frequency) || !std::isfinite(level) ||
            std::abs(frequency - targetHz) > watchSpanHz ||
            level < average + 8.0f) {
            continue;
        }
        const float prev = magnitudes[static_cast<std::size_t>(i - 1)];
        const float next = magnitudes[static_cast<std::size_t>(i + 1)];
        if (level < prev || level < next) {
            continue;
        }
        peaks.append(qMakePair(i, level));
    }
    std::sort(peaks.begin(), peaks.end(), [](const auto &a, const auto &b) {
        return a.second > b.second;
    });
    while (peaks.size() > 8) {
        peaks.removeLast();
    }

    const qint64 nowMs = gnssSpurWatchTimer.elapsed();
    for (const auto &peak : std::as_const(peaks)) {
        const double frequency = static_cast<double>(frequencies[static_cast<std::size_t>(peak.first)]);
        const qint64 bin = static_cast<qint64>(std::llround((frequency - targetHz) / binHz));
        GnssSpurWatchBin &entry = gnssSpurWatchBins[bin];
        entry.averageDb = entry.hits == 0
                              ? peak.second
                              : static_cast<float>(entry.averageDb * 0.85f + peak.second * 0.15f);
        entry.hits = (std::min)(9999, entry.hits + 1);
        entry.lastSeenMs = nowMs;
    }

    if (gnssSpurLogTimer.elapsed() < 3000) {
        return;
    }
    gnssSpurLogTimer.restart();

    QVector<QPair<qint64, GnssSpurWatchBin>> activeBins;
    for (auto it = gnssSpurWatchBins.begin(); it != gnssSpurWatchBins.end();) {
        if (nowMs - it.value().lastSeenMs > 15000) {
            it = gnssSpurWatchBins.erase(it);
            continue;
        }
        activeBins.append(qMakePair(it.key(), it.value()));
        ++it;
    }
    if (activeBins.isEmpty()) {
        return;
    }
    std::sort(activeBins.begin(), activeBins.end(), [](const auto &a, const auto &b) {
        if (a.second.hits != b.second.hits) {
            return a.second.hits > b.second.hits;
        }
        return a.second.averageDb > b.second.averageDb;
    });

    constexpr int kMaxLoggedSpurBins = 24;
    QStringList summary;
    const int loggedBins = (std::min)(kMaxLoggedSpurBins, activeBins.size());
    summary.reserve(loggedBins);
    for (int i = 0; i < loggedBins; ++i) {
        const auto &entry = activeBins.at(i);
        const double offsetHz = static_cast<double>(entry.first) * binHz;
        summary << QStringLiteral("%1k/%2dB/h%3")
                       .arg(offsetHz / 1000.0, 0, 'f', 1)
                       .arg(entry.second.averageDb, 0, 'f', 1)
                       .arg(entry.second.hits);
    }
    qDebug() << "[GNSS spur watch]"
             << "system" << preset.id
             << "targetHz" << targetHz
             << "centerHz" << centerFrequency
             << "averageDb" << average
             << "trackedBins" << activeBins.size()
             << "loggedBins" << loggedBins
             << "peaks" << summary.join(QStringLiteral(", "));
}
