#include "main.h"

#include "dmrprivacyutils.h"

#include <QAbstractItemView>
#include <QDebug>
#include <QDialog>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <cstddef>
int YourClassName::selectedDmrPrivacyKeyId() const {
    if (!dmrPrivacyKeyIdCombo) {
        return pendingSettings.dmrPrivacyKeyId;
    }
    const QVariant data = dmrPrivacyKeyIdCombo->currentData();
    bool ok = false;
    const int value = data.toInt(&ok);
    if (ok && value >= 0 && value <= 255) {
        return value;
    }
    return -1;
}

void YourClassName::refreshDmrPrivacyKeyIdCombo() {
    if (!dmrPrivacyKeyIdCombo) {
        return;
    }

    const int mode =
        dmrPrivacyModeCombo
            ? normalizedDmrPrivacyMode(dmrPrivacyModeCombo->currentData().toInt())
            : normalizedDmrPrivacyMode(pendingSettings.dmrPrivacyMode);
    const int preferredKeyId = pendingSettings.dmrPrivacyKeyId;
    QVector<int> keyIds;
    for (const DmrPrivacyKeyEntry &entry : std::as_const(dmrPrivacyKeys)) {
        if (normalizedDmrPrivacyMode(entry.mode) != mode) {
            continue;
        }
        const int keyId = (std::clamp)(entry.keyId, 0, 255);
        if (!keyIds.contains(keyId)) {
            keyIds.append(keyId);
        }
    }

    const QSignalBlocker blocker(dmrPrivacyKeyIdCombo);
    dmrPrivacyKeyIdCombo->clear();
    if (mode == DMR_PRIVACY_NONE) {
        dmrPrivacyKeyIdCombo->addItem(QStringLiteral("-"), -1);
        dmrPrivacyKeyIdCombo->setEnabled(false);
        pendingSettings.dmrPrivacyKeyId = 0;
        pendingSettings.dmrPrivacyKeyHex.clear();
        return;
    }

    dmrPrivacyKeyIdCombo->setEnabled(true);
    if (keyIds.isEmpty()) {
        dmrPrivacyKeyIdCombo->addItem(uiText(QStringLiteral("dmr_privacy_no_keys"),
                                             QStringLiteral("No keys")),
                                      -1);
        pendingSettings.dmrPrivacyKeyHex.clear();
        return;
    }

    for (int keyId : std::as_const(keyIds)) {
        dmrPrivacyKeyIdCombo->addItem(QStringLiteral("KID %1").arg(keyId), keyId);
    }
    int index = dmrPrivacyKeyIdCombo->findData(preferredKeyId);
    if (index < 0) {
        index = 0;
    }
    dmrPrivacyKeyIdCombo->setCurrentIndex(index);
    pendingSettings.dmrPrivacyKeyId = dmrPrivacyKeyIdCombo->currentData().toInt();
}

bool YourClassName::applyDmrPrivacyKeySelection() {
    const int mode =
        dmrPrivacyModeCombo
            ? normalizedDmrPrivacyMode(dmrPrivacyModeCombo->currentData().toInt())
            : normalizedDmrPrivacyMode(pendingSettings.dmrPrivacyMode);
    pendingSettings.dmrPrivacyMode = mode;
    if (mode == DMR_PRIVACY_NONE) {
        pendingSettings.dmrPrivacyKeyId = 0;
        pendingSettings.dmrPrivacyKeyHex.clear();
        return false;
    }

    const int keyId = selectedDmrPrivacyKeyId();
    if (keyId < 0) {
        pendingSettings.dmrPrivacyKeyId = 0;
        pendingSettings.dmrPrivacyKeyHex.clear();
        return false;
    }
    pendingSettings.dmrPrivacyKeyId = (std::clamp)(keyId, 0, 255);
    const int index = findDmrPrivacyKeyIndexByModeAndId(dmrPrivacyKeys,
                                                        mode,
                                                        pendingSettings.dmrPrivacyKeyId);
    if (index < 0) {
        pendingSettings.dmrPrivacyKeyHex.clear();
        return false;
    }

    const DmrPrivacyKeyEntry entry = dmrPrivacyKeys.at(index);
    pendingSettings.dmrPrivacyKeyHex = normalizedDmrPrivacyKeyHex(entry.keyHex);

    qDebug() << "[DMR privacy] selected key from table"
             << "keyId" << entry.keyId
             << "mode" << dmrPrivacyModeId(entry.mode)
             << "keyHexLen" << normalizedDmrPrivacyKeyHex(entry.keyHex).size();
    return true;
}

void YourClassName::showDmrPrivacyKeyDialog() {
    QDialog dialog(this);
    dialog.setWindowTitle(uiText(QStringLiteral("dmr_privacy_keys_title"),
                                 QStringLiteral("DMR privacy keys")));
    dialog.resize(760, 420);

    auto *layout = new QVBoxLayout(&dialog);
    auto *hint = new QLabel(uiText(QStringLiteral("dmr_privacy_keys_hint"),
                                   QStringLiteral("The active key is selected by Privacy mode + KID. ARC4 and AES-256 keys are stored in separate tables.")),
                            &dialog);
    hint->setWordWrap(true);
    layout->addWidget(hint);

    auto *tabs = new QTabWidget(&dialog);
    layout->addWidget(tabs, 1);

    const auto createTable = [&dialog]() {
        auto *table = new QTableWidget(&dialog);
        table->setColumnCount(3);
        table->setHorizontalHeaderLabels(QStringList()
                                         << QStringLiteral("KID")
                                         << QStringLiteral("Key hex")
                                         << QStringLiteral("Note"));
        table->horizontalHeader()->setStretchLastSection(true);
        table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
        table->horizontalHeader()->setSortIndicatorShown(true);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setAlternatingRowColors(true);
        table->setSortingEnabled(false);
        return table;
    };

    auto *arc4Table = createTable();
    auto *aesTable = createTable();
    tabs->addTab(arc4Table, QStringLiteral("ARC4"));
    tabs->addTab(aesTable, QStringLiteral("AES-256"));

    const auto tableForMode = [arc4Table, aesTable](int mode) {
        return normalizedDmrPrivacyMode(mode) == DMR_PRIVACY_AES256 ? aesTable : arc4Table;
    };
    const auto modeForTable = [arc4Table](const QTableWidget *table) {
        return table == arc4Table ? DMR_PRIVACY_ARC4 : DMR_PRIVACY_AES256;
    };
    const auto appendRow = [](QTableWidget *table, const DmrPrivacyKeyEntry &entry) {
        const bool sortingEnabled = table->isSortingEnabled();
        table->setSortingEnabled(false);
        const int row = table->rowCount();
        table->insertRow(row);

        auto *kidItem = new QTableWidgetItem(QString::number((std::clamp)(entry.keyId, 0, 255)));
        kidItem->setData(Qt::EditRole, (std::clamp)(entry.keyId, 0, 255));
        table->setItem(row, 0, kidItem);

        table->setItem(row, 1, new QTableWidgetItem(normalizedDmrPrivacyKeyHex(entry.keyHex)));
        table->setItem(row, 2, new QTableWidgetItem(entry.note));
        table->setSortingEnabled(sortingEnabled);
    };

    for (const DmrPrivacyKeyEntry &entry : std::as_const(dmrPrivacyKeys)) {
        const int mode = normalizedDmrPrivacyMode(entry.mode);
        if (mode == DMR_PRIVACY_ARC4 || mode == DMR_PRIVACY_AES256) {
            appendRow(tableForMode(mode), entry);
        }
    }
    const auto sortByKid = [](QTableWidget *table) {
        if (!table) {
            return;
        }
        for (int row = 0; row < table->rowCount(); ++row) {
            QTableWidgetItem *item = table->item(row, 0);
            if (!item) {
                continue;
            }
            bool ok = false;
            const int keyId = item->text().trimmed().toInt(&ok);
            if (ok) {
                item->setData(Qt::EditRole, (std::clamp)(keyId, 0, 255));
            }
        }
        table->sortItems(0, Qt::AscendingOrder);
        table->horizontalHeader()->setSortIndicator(0, Qt::AscendingOrder);
    };
    connect(arc4Table->horizontalHeader(), &QHeaderView::sectionClicked, &dialog, [arc4Table, sortByKid](int section) {
        if (section == 0) {
            sortByKid(arc4Table);
        }
    });
    connect(aesTable->horizontalHeader(), &QHeaderView::sectionClicked, &dialog, [aesTable, sortByKid](int section) {
        if (section == 0) {
            sortByKid(aesTable);
        }
    });
    if (dmrPrivacyModeCombo) {
        const int mode = normalizedDmrPrivacyMode(dmrPrivacyModeCombo->currentData().toInt());
        tabs->setCurrentWidget(tableForMode(mode));
    }

    auto *buttonLayout = new QHBoxLayout();
    auto *addButton = new QPushButton(uiText(QStringLiteral("add"), QStringLiteral("Add")), &dialog);
    auto *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), &dialog);
    auto *moveUpButton = new QToolButton(&dialog);
    moveUpButton->setText(QStringLiteral("↑"));
    moveUpButton->setToolTip(uiText(QStringLiteral("move_up"), QStringLiteral("Move up")));
    auto *moveDownButton = new QToolButton(&dialog);
    moveDownButton->setText(QStringLiteral("↓"));
    moveDownButton->setToolTip(uiText(QStringLiteral("move_down"), QStringLiteral("Move down")));
    auto *useSelectedButton = new QPushButton(uiText(QStringLiteral("dmr_privacy_use_selected"),
                                                     QStringLiteral("Use selected KID")),
                                              &dialog);
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(removeButton);
    buttonLayout->addWidget(moveUpButton);
    buttonLayout->addWidget(moveDownButton);
    buttonLayout->addWidget(useSelectedButton);
    buttonLayout->addStretch();
    layout->addLayout(buttonLayout);

    auto *dialogButtons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    layout->addWidget(dialogButtons);

    const auto nextFreeKeyId = [](const QTableWidget *table) {
        std::array<bool, 256> used{};
        for (int row = 0; row < table->rowCount(); ++row) {
            bool ok = false;
            const int keyId = table->item(row, 0) ? table->item(row, 0)->text().trimmed().toInt(&ok) : -1;
            if (ok && keyId >= 0 && keyId <= 255) {
                used[static_cast<size_t>(keyId)] = true;
            }
        }
        for (int i = 0; i <= 255; ++i) {
            if (!used[static_cast<size_t>(i)]) {
                return i;
            }
        }
        return 255;
    };
    const auto moveSelectedRow = [](QTableWidget *table, int delta) {
        if (!table) {
            return;
        }
        const int row = table->currentRow();
        const int targetRow = row + delta;
        if (row < 0 || targetRow < 0 || targetRow >= table->rowCount()) {
            return;
        }
        const bool sortingEnabled = table->isSortingEnabled();
        table->setSortingEnabled(false);
        QVector<QTableWidgetItem *> currentItems;
        QVector<QTableWidgetItem *> targetItems;
        currentItems.reserve(table->columnCount());
        targetItems.reserve(table->columnCount());
        for (int column = 0; column < table->columnCount(); ++column) {
            currentItems.append(table->takeItem(row, column));
            targetItems.append(table->takeItem(targetRow, column));
        }
        for (int column = 0; column < table->columnCount(); ++column) {
            table->setItem(row, column, targetItems.at(column));
            table->setItem(targetRow, column, currentItems.at(column));
        }
        table->selectRow(targetRow);
        table->setSortingEnabled(sortingEnabled);
    };

    connect(addButton, &QPushButton::clicked, &dialog, [tabs, modeForTable, appendRow, nextFreeKeyId]() {
        auto *table = qobject_cast<QTableWidget *>(tabs->currentWidget());
        if (!table) {
            return;
        }
        DmrPrivacyKeyEntry entry;
        entry.mode = modeForTable(table);
        entry.keyId = nextFreeKeyId(table);
        appendRow(table, entry);
        table->selectRow(table->rowCount() - 1);
    });
    connect(removeButton, &QPushButton::clicked, &dialog, [tabs]() {
        auto *table = qobject_cast<QTableWidget *>(tabs->currentWidget());
        if (!table) {
            return;
        }
        const int row = table->currentRow();
        if (row >= 0) {
            table->removeRow(row);
        }
    });
    connect(moveUpButton, &QToolButton::clicked, &dialog, [tabs, moveSelectedRow]() {
        moveSelectedRow(qobject_cast<QTableWidget *>(tabs->currentWidget()), -1);
    });
    connect(moveDownButton, &QToolButton::clicked, &dialog, [tabs, moveSelectedRow]() {
        moveSelectedRow(qobject_cast<QTableWidget *>(tabs->currentWidget()), 1);
    });
    connect(useSelectedButton, &QPushButton::clicked, &dialog, [this, tabs, modeForTable]() {
        auto *table = qobject_cast<QTableWidget *>(tabs->currentWidget());
        if (!table) {
            return;
        }
        const int row = table->currentRow();
        if (row < 0 || !dmrPrivacyKeyIdCombo) {
            return;
        }
        bool ok = false;
        const int keyId = table->item(row, 0) ? table->item(row, 0)->text().trimmed().toInt(&ok) : 0;
        if (!ok) {
            return;
        }
        if (dmrPrivacyModeCombo) {
            const int comboIndex = dmrPrivacyModeCombo->findData(modeForTable(table));
            if (comboIndex >= 0) {
                dmrPrivacyModeCombo->setCurrentIndex(comboIndex);
            }
        }
        pendingSettings.dmrPrivacyMode = modeForTable(table);
        pendingSettings.dmrPrivacyKeyId = (std::clamp)(keyId, 0, 255);
    });
    connect(dialogButtons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(dialogButtons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    QVector<DmrPrivacyKeyEntry> parsedKeys;
    const auto parseTable = [&parsedKeys, modeForTable, this](QTableWidget *table) {
        const int mode = modeForTable(table);
        for (int row = 0; row < table->rowCount(); ++row) {
            bool ok = false;
            const int keyId =
                table->item(row, 0) ? table->item(row, 0)->text().trimmed().toInt(&ok) : -1;
            if (!ok || keyId < 0 || keyId > 255) {
                continue;
            }

            const QString keyHex =
                normalizedDmrPrivacyKeyHex(table->item(row, 1) ? table->item(row, 1)->text() : QString());
            const QString note = table->item(row, 2) ? table->item(row, 2)->text().trimmed() : QString();
            if (keyHex.isEmpty() && note.isEmpty()) {
                continue;
            }

            const int existing = findDmrPrivacyKeyIndexByModeAndId(parsedKeys, mode, keyId);
            DmrPrivacyKeyEntry entry;
            entry.active =
                pendingSettings.dmrPrivacyKeyId == keyId &&
                dmrPrivacyModeCombo &&
                normalizedDmrPrivacyMode(dmrPrivacyModeCombo->currentData().toInt()) == mode;
            entry.mode = mode;
            entry.keyId = keyId;
            entry.keyHex = keyHex;
            entry.note = note;
            if (existing >= 0) {
                parsedKeys[existing] = entry;
            } else {
                parsedKeys.append(entry);
            }
        }
    };
    parseTable(arc4Table);
    parseTable(aesTable);

    dmrPrivacyKeys = parsedKeys;
    refreshDmrPrivacyKeyIdCombo();
    applyDmrPrivacyKeySelection();
    updateDsdNeoBridgeSettings();
    updateGopherTrunkBridgeSettings();
    updateDigitalDecoderMode();
    savePersistentSettings();
}
