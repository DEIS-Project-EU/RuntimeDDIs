#pragma once

#include "rte_monitor/RtEList.h"

#include <QWidget>
#include <QLabel>
#include <QHBoxLayout>
#include <QListWidget>
#include <QListWidgetItem>

#include <string>
#include <map>

class QtVisualizer : public QWidget {
    Q_OBJECT
    public:
        QtVisualizer(QWidget* parent = nullptr, unsigned bufferCount = 3):
            QWidget(parent) {
                
                list = new QListWidget();
                auto testItem = new QListWidgetItem(tr("Sample Item"), list);
                list->setStyleSheet("border: 1px solid blue");

                statusLabel = new QLabel("<center>Variable status</center><br>"
                    "Variable name: Sample<br>"
                    "Variable type: Float Bounds Check<br>"
                    "Variable output: False, 1.0");            

                statusLabel->setStyleSheet("background-color: grey; border: 1px solid black");
                
                hBoxLayout = new QHBoxLayout();
                hBoxLayout->addWidget(list);
                hBoxLayout->addWidget(statusLabel);
                setLayout(hBoxLayout);

                SetBufferElements(bufferCount);

                adjustSize();
                setFixedSize(size());

                connect(list, SIGNAL(currentItemChanged(QListWidgetItem*, QListWidgetItem*)), 
                    this, SLOT(OnSelectedVariable(QListWidgetItem*)));
        }

        std::vector<std::string> GetVariableList() const {
            std::vector<std::string> result;
            for (unsigned idx = 0; idx < list->count(); ++idx)
                result.push_back(list->item(idx)->text().toStdString());
            return result;
        }

        void SetBufferElements(unsigned count) {
            for (const auto& label : bufferLabels) {
                hBoxLayout->removeWidget(label);
                delete label;
            }
            bufferLabels.clear();

            for (unsigned idx = 0; idx < count; ++idx) {
                    auto text = QString("<center>Buffer at t-%1 status</center><br>"
                    "tag: Sample<br>"
                    "float value: 0.0<br>"
                    "boolean value: False").arg(idx);
                    auto newBufferLabel = new QLabel(text);
                    newBufferLabel->setStyleSheet("background-color: grey; border: 1px solid black");
                    hBoxLayout->addWidget(newBufferLabel);
                    bufferLabels.push_back(newBufferLabel);
                }

            setFixedSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
            adjustSize();
            setFixedSize(size());
        }

    signals:

    public slots:

        void OnAddVariable(const std::string& varName) {
            auto newVar = new QListWidgetItem(varName.c_str(), list);
        }

        void OnRemoveVariable(const std::string& varName) {
            for (unsigned idx = 0; idx < list->count(); ++idx) {
                if (list->item(idx)->text().toStdString() == varName) {
                    list->takeItem(idx);
                    return;
                }
            }
        }

        void UpdateRtEList(const rte_monitor::RtEList& rteList) { 
            this->rteList = rteList; 
        }

        void OnSelectedVariable(QListWidgetItem* current);

        void OnUpdateSelectedVariable();

    private:
        QListWidget* list = nullptr;
        QLabel* statusLabel = nullptr;
        QHBoxLayout* hBoxLayout = nullptr;
        std::vector<QLabel*> bufferLabels;
        rte_monitor::RtEList rteList;

        void UpdateBufferVisualisation(unsigned targetIdx, 
            const std::string& tag, const std::string& color, 
            float floatValue, bool boolValue) {
            if (targetIdx < 0 || targetIdx > bufferLabels.size()-1)
                return;
            auto targetText = 
                QString("<center>Buffer at t-%1 status</center><br>"
                        "tag: %2<br>"
                        "float value: %3<br>"
                        "boolean value: %4")
                    .arg(targetIdx)
                    .arg(tag.c_str())
                    .arg(floatValue)
                    .arg(boolValue);
            bufferLabels[targetIdx]->setText(targetText);
            auto style = 
                QString("background-color:%1; border: 1px solid black").
                    arg(color.c_str());
            bufferLabels[targetIdx]->setStyleSheet(style);
        }
};