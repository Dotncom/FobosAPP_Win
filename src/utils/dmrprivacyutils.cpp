#include "dmrprivacyutils.h"

#include <QRegularExpression>

QString dmrPrivacyModeId(int mode) {
    switch (mode) {
    case DMR_PRIVACY_ARC4:
        return QStringLiteral("arc4");
    case DMR_PRIVACY_AES256:
        return QStringLiteral("aes256");
    case DMR_PRIVACY_NONE:
    default:
        return QStringLiteral("none");
    }
}

int dmrPrivacyModeFromText(const QString &text) {
    const QString normalized = text.trimmed().toLower();
    if (normalized.contains(QStringLiteral("aes"))) {
        return DMR_PRIVACY_AES256;
    }
    if (normalized.contains(QStringLiteral("arc4")) ||
        normalized.contains(QStringLiteral("rc4"))) {
        return DMR_PRIVACY_ARC4;
    }
    return DMR_PRIVACY_NONE;
}

int normalizedDmrPrivacyMode(int mode) {
    switch (mode) {
    case DMR_PRIVACY_ARC4:
    case DMR_PRIVACY_AES256:
        return mode;
    case DMR_PRIVACY_NONE:
    default:
        return DMR_PRIVACY_NONE;
    }
}

QString normalizedDmrPrivacyKeyHex(const QString &text) {
    QString key = text.trimmed();
    key.remove(QRegularExpression(QStringLiteral("[^0-9A-Fa-f]")));
    key = key.toUpper();
    if (key.size() > 64) {
        key.truncate(64);
    }
    return key;
}

int findDmrPrivacyKeyIndexById(const QVector<DmrPrivacyKeyEntry> &keys, int keyId) {
    for (int i = 0; i < keys.size(); ++i) {
        if (keys.at(i).keyId == keyId) {
            return i;
        }
    }
    return -1;
}

int findDmrPrivacyKeyIndexByModeAndId(const QVector<DmrPrivacyKeyEntry> &keys, int mode, int keyId) {
    const int normalizedMode = normalizedDmrPrivacyMode(mode);
    for (int i = 0; i < keys.size(); ++i) {
        const DmrPrivacyKeyEntry &entry = keys.at(i);
        if (normalizedDmrPrivacyMode(entry.mode) == normalizedMode && entry.keyId == keyId) {
            return i;
        }
    }
    return -1;
}
