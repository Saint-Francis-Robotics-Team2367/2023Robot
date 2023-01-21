#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <vector>
#include "UIWidget.h"

UIWidget::UIWidget(nt::GenericEntry *entry, std::string tab, std::string name) {
    this->name = name;
    this->tab = tab;
    this->entry = entry;
}

nt::GenericEntry *UIWidget::GetEntry() {
    return this->entry;
}

void UIWidget::SetValue(double newVal) {
    this->GetEntry()->SetDouble(newVal);
}

std::string UIWidget::GetName() {
    return this->name;
}

std::string UIWidget::GetTab() {
    return this->tab;
}