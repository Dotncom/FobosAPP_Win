#ifndef DMRVOCODER_H
#define DMRVOCODER_H

#include <QByteArray>
#include <QString>

#include <vector>

#include "dmrvoicepayload.h"

class DmrVocoder {
public:
    DmrVocoder();
    ~DmrVocoder();
    DmrVocoder(const DmrVocoder &) = delete;
    DmrVocoder &operator=(const DmrVocoder &) = delete;

    bool isAvailable() const;
    void reset();
    QByteArray decodeSoftFrames(const std::vector<DmrAmbeSoftFrame> &frames,
                                int *decodedFrames = nullptr,
                                int *errorTotal = nullptr,
                                std::vector<int> *perFrameErrors = nullptr);
    QByteArray decodeFrames(const std::vector<QString> &frames,
                            int *decodedFrames = nullptr,
                            int *errorTotal = nullptr);
    QByteArray decodePayloads(const std::vector<DmrAmbePayload> &payloads,
                              int *decodedFrames = nullptr,
                              int *errorTotal = nullptr);
    QByteArray decodePayloads(const std::vector<QString> &payloads,
                              int *decodedFrames = nullptr,
                              int *errorTotal = nullptr);
    QByteArray decodeCanonicalPayloads(const std::vector<DmrAmbePayload> &payloads,
                                       int *decodedFrames = nullptr,
                                       int *errorTotal = nullptr);

private:
    bool initialized = false;
    struct Impl;
    Impl *impl = nullptr;
};

#endif // DMRVOCODER_H
