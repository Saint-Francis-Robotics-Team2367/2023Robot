#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <vector>
#include "UIWidget.h"

// A static class for handling UIWidgets
class ShuffleUI {
  public:
    static std::vector<UIWidget *> widgetList;
    //static int number;

    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, int value);
    
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, double value);
    
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, float value);

    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, bool value);
    
    static nt::GenericEntry *MakeSlider(std::string name, std::string tab, double min, double max, double defaultVal);

    static void Update();

    static nt::GenericEntry *GetEntry(std::string name, std::string tab);

    static void AddEntry(UIWidget *widget);

    static void PrintWidgetList();

    static nt::GenericEntry *MakeGraph(std::string name, std::string tab, double value);
};