#include "ShuffleUI.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <vector>
#include <iostream>

std::vector<UIWidget *> ShuffleUI::widgetList;

int currentGrid[3][9] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0}
};

std::string ShuffleUI::decimalToBinary(int n)  {
    int binaryNum[32];
    int i = 0;
    while (n > 0) {
    binaryNum[i] = n % 2;
    n = n / 2;
    i++;
    }

    std::string str = "";

    for (int j = i - 1; j >= 0; j--) {
    str += std::to_string(binaryNum[j]);
    }

    return str;
}

void ShuffleUI::printGridArray(int arr[3][9]) {
    for (int row = 0; row < 3; row++) {
        std::cout << "[ ";
        for (int col = 0; col < 9; col++) {
            std::cout << arr[row][col];
            if (col != 8) {
                std::cout << ",";
            }
            std::cout << " ";
        }
        std::cout << "]" << std::endl;
    }
}

void ShuffleUI::printGridArray(double arr[3][9]) {
    for (int row = 0; row < 3; row++) {
        std::cout << "[ ";
        for (int col = 0; col < 9; col++) {
            std::cout << arr[row][col];
            if (col != 8) {
                std::cout << ",";
            }
            std::cout << " ";
        }
        std::cout << "]" << std::endl;
    }
}

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

void ShuffleUI::OptimalScorePositions() {
    //these don't change. The first three have no name on the sheet.
    int numbers1[] = {9, 8, 7, 6, 5, 4, 3, 2, 1};
    int numbers2[] = {324, 152, 163};
    int numbers3[] = {128, 36, 68};
    int pointPotential[3][9] = {{5, 5, 5, 5, 5, 5, 5, 5, 5}, {3, 3, 3, 3, 3, 3, 3, 3, 3}, {2, 2, 2, 2, 2, 2, 2, 2, 2}};
    int difficultyScore[3][9] = {{7, 7, 8, 9, 10, 9, 8, 7, 7}, {4, 4, 5, 6, 7, 6, 5, 4, 4}, {1, 2, 2, 3, 4, 3, 2, 2, 1}};
    
    //all of these update
    double dynamicLocationScore[3][9];
    int dynamicLinkLocation[3][9];
    int dynamicCoopBonus[3][9];
    double nextGridSpotScore[3][9];

    //dynamicLocationScore and dynamicLinkLocation
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 9; col++) {
            //for dynamicLocationScore - works
            double tot1 = 0;
            if (row == 0 && col == 0 && currentGrid[row][col] > 0) { // for some reason this check only happens with one cell on the sheet?
                tot1 = 0;
            } else if (col == 0) {
                tot1 = currentGrid[row][col + 1] + 0.5*currentGrid[row][col + 2];
            } else if (col == 1) {
                tot1 = currentGrid[row][col + 1] + currentGrid[row][col - 1] + 0.5*currentGrid[row][col + 2];
            } else if (col == 7) {
                tot1 = currentGrid[row][col + 1] + currentGrid[row][col - 1] + 0.5*currentGrid[row][col - 2];
            } else if (col == 8) {
                tot1 = currentGrid[row][col - 1] + 0.5*currentGrid[row][col - 2];
            } else {
                tot1 = currentGrid[row][col + 1] + currentGrid[row][col - 1] + 0.5*(currentGrid[row][col + 2] + currentGrid[row][col - 2]);
            }
            dynamicLocationScore[row][col] = tot1;

            // for dynamicLinkLocation - works
            int tot2 = 0;
            int tempInt;
            if (decimalToBinary(numbers3[row]).length() < numbers1[col]) {
                tot2 = 0;
            } else {
                std::string temp = decimalToBinary(numbers3[row]);
                tot2 = int(temp[temp.length() - numbers1[col]])-48;
            }
            std::cout << tot2 << std::endl;
            dynamicLinkLocation[row][col] = tot2;
            //ok so this is actually static, even on the sheet? the sheet may not be the most recent version.
        }
    }

    //for dynamicCoopBonus - works
    int tot3 = 0;
    for (int row = 0; row < 3; row++) {
        for (int col = 3; col < 6; col++) {
            tot3 += currentGrid[row][col];
        }
    }
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 9; col++) {
            if (col > 2 && col < 6) {
                dynamicCoopBonus[row][col] = int(tot3 < 3);
            } else {
                dynamicCoopBonus[row][col] = 0;
            }
        }
    }

    //for nextGridSpotScore - works
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 9; col++) {
            if (currentGrid[row][col] > 0) {
                nextGridSpotScore[row][col] = 0;
            } else {
                // a lot of these numbers come from the "weights" section of the sheet
                nextGridSpotScore[row][col] = (0.5 * pointPotential[row][col]) + (0.75/difficultyScore[row][col]) +
                (1 * dynamicLocationScore[row][col]) + (0.25 * dynamicCoopBonus[row][col]) + (1 * dynamicLinkLocation[row][col]);
            }
        }
    }

    //testing purposes: prints values to console
    /*std::cout << "Current Grid" << std::endl;
    printGridArray(currentGrid);
    std::cout << "Dynamic Location Score" << std::endl;
    printGridArray(dynamicLocationScore);
    std::cout << "Dynamic Link Location" << std::endl;
    printGridArray(dynamicLinkLocation);
    std::cout << "Dynamic Cooperation Bonus" << std::endl;
    printGridArray(dynamicCoopBonus);
    std::cout << "Next Grid Spot Score" << std::endl;
    printGridArray(nextGridSpotScore);*/

    //putting it onto shuffleboardm- to-do
    //see if there's a way to change button labels without changing widget name
}