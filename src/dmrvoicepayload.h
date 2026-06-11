#ifndef DMRVOICEPAYLOAD_H
#define DMRVOICEPAYLOAD_H

#include <QString>

#include <array>
#include <cstdint>

struct DmrAmbePayload {
    QString hex;
    int correctedErrors = 0;
    int rawCorrectedErrors = 0;
};

struct DmrAmbeSoftFrame {
    QString hex;
    std::array<std::uint8_t, 72> reliability = {};
};

#endif // DMRVOICEPAYLOAD_H
