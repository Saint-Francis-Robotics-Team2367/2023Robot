#include "ShuffleUI.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <vector>
#include <iostream>

std::vector<UIWidget *> ShuffleUI::widgetList;

// makes a widget and returns a pointer to its entry. If widget with that name already exists, updates its value and returns that entry's pointer instead
nt::GenericEntry *ShuffleUI::MakeWidget(std::string name, std::string tab, int value=0) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetInteger(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value).GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

// makes a widget and returns a pointer to its entry. If widget with that name already exists, updates its value and returns that entry's pointer instead
nt::GenericEntry *ShuffleUI::MakeWidget(std::string name, std::string tab, double value=0.0) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetDouble(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value).GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

// makes a widget and returns a pointer to its entry. If widget with that name already exists, updates its value and returns that entry's pointer instead
nt::GenericEntry *ShuffleUI::MakeWidget(std::string name, std::string tab, float value) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetFloat(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value).GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

nt::GenericEntry *ShuffleUI::MakeWidget(std::string name, std::string tab, bool value) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetBoolean(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value)
        .WithWidget(frc::BuiltInWidgets::kBooleanBox)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}
/*
nt::GenericEntry *ShuffleUI::MakeWidget(std::string name, std::string tab, std::string value) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetString(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}*/

nt::GenericEntry *ShuffleUI::MakeWidgetPos(std::string name, std::string tab, int value, int posX, int posY) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetInteger(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value)
        .WithPosition(posX, posY)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

nt::GenericEntry *ShuffleUI::MakeWidgetPos(std::string name, std::string tab, double value, int posX, int posY) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetDouble(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value)
        .WithPosition(posX, posY)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

nt::GenericEntry *ShuffleUI::MakeWidgetPos(std::string name, std::string tab, float value, int posX, int posY) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetFloat(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value)
        .WithPosition(posX, posY)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

nt::GenericEntry *ShuffleUI::MakeWidgetPos(std::string name, std::string tab, bool value, int posX, int posY) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetBoolean(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value)
        .WithWidget(frc::BuiltInWidgets::kBooleanBox)
        .WithPosition(posX, posY)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

nt::GenericEntry *ShuffleUI::MakeWidgetPos(std::string name, std::string tab, std::string value, int posX, int posY) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetString(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value)
        .WithPosition(posX, posY)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

// makes a widget with number slider and returns a pointer to its entry. If widget with that name already exists, returns that entry's pointer instead
nt::GenericEntry *ShuffleUI::MakeSlider(std::string name, std::string tab, double min, double max, double defaultVal=0.0) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, defaultVal)
        .WithWidget(frc::BuiltInWidgets::kNumberSlider)
        .WithProperties({
            {"min", nt::Value::MakeDouble(min)},
            {"max", nt::Value::MakeDouble(max)}
        })
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
} 

void ShuffleUI::AddEntry(UIWidget *widget) {
    ShuffleUI::widgetList.push_back(widget);
}

// finds an Entry by name & tab, returns NULL if it doesn't exist
nt::GenericEntry *ShuffleUI::GetEntry(std::string name, std::string tab) {
    for (UIWidget *w: ShuffleUI::widgetList) {
        if (w->GetName() == name && w->GetTab() == tab) {
            return w->GetEntry();
        }
    }
    return NULL;
}

// finds double entry and returns value
double ShuffleUI::GetDouble(std::string name, std::string tab, double defaultVal) {
    if (GetEntry(name, tab) != NULL) {
        return ShuffleUI::GetEntry(name, tab)->GetDouble(defaultVal);
    } else {
        return defaultVal;
    }
}

// finds int entry and returns value
int ShuffleUI::GetInt(std::string name, std::string tab, int defaultVal) {
    if (GetEntry(name, tab) != NULL) {
        return ShuffleUI::GetEntry(name, tab)->GetInteger(defaultVal);
    } else {
        return defaultVal;
    }
}

// finds bool entry and returns value
bool ShuffleUI::GetBool(std::string name, std::string tab, bool defaultVal) {
    if (GetEntry(name, tab) != NULL) {
        return ShuffleUI::GetEntry(name, tab)->GetBoolean(defaultVal);
    } else {
        return defaultVal;
    }
}

//finds float entry and returns value
float ShuffleUI::GetFloat(std::string name, std::string tab, float defaultVal) {
    if (GetEntry(name, tab) != NULL) {
        return ShuffleUI::GetEntry(name, tab)->GetFloat(defaultVal);
    } else {
        return defaultVal;
    }
}

//prints the name of all widgets in the widget list
void ShuffleUI::PrintWidgetList() {
    for (UIWidget *w: ShuffleUI::widgetList) {
        std::cout << w->GetName() << std::endl;
    }
}

// makes a widget with a graph and returns a pointer to its entry. If widget with that name already exists, returns that entry's pointer instead
nt::GenericEntry *ShuffleUI::MakeGraph(std::string name, std::string tab, double value) {
    if (GetEntry(name, tab) != NULL) { 
        //std::cout << "widget already exists" << std::endl;
        try {
            GetEntry(name, tab)->SetDouble(value);
        } catch(...) {}
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, value)
        .WithWidget(frc::BuiltInWidgets::kGraph)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

nt::GenericEntry *ShuffleUI::MakeButton(std::string name, std::string tab, bool defaultState) {
    if (GetEntry(name, tab) != NULL) { 
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, defaultState)
        .WithWidget(frc::BuiltInWidgets::kToggleButton)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}

nt::GenericEntry *ShuffleUI::MakeButtonPos(std::string name, std::string tab, bool defaultState, int posX, int posY) {
    if (GetEntry(name, tab) != NULL) { 
        return GetEntry(name, tab);
    }
    nt::GenericEntry *entry = frc::Shuffleboard::GetTab(tab).Add(name, defaultState)
        .WithWidget(frc::BuiltInWidgets::kToggleButton)
        .WithPosition(posX, posY)
        .GetEntry();
    UIWidget *widget = new UIWidget(entry, tab, name);
    ShuffleUI::AddEntry(widget);
    return entry;
}