#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <vector>

// for use in the ShuffleUI class
class UIWidget {
  private:
    nt::GenericEntry *entry;
    std::string name;
    std::string tab;

  public:
    UIWidget(nt::GenericEntry *entry, std::string tab, std::string name);
    nt::GenericEntry * GetEntry();
    void SetValue(double NewVal);
    std::string GetName();
    std::string GetTab();
};