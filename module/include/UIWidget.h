#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <vector>

// for use in the ShuffleUI class
class UIWidget {
  private:
    //pointer to widget's entry
    nt::GenericEntry *entry;
    //widget's name
    std::string name;
    //widget's tab
    std::string tab;

  public:
    /**
     * UIWidget constructor
     *
     * @param entry pointer to widget's entry
     * @param tab Widget's tab
     * @param name Widget's name
     */
    UIWidget(nt::GenericEntry *entry, std::string tab, std::string name);
    
    /**
     * Get's a UIWidget's entry
     * @return pointer to the widget's entry
     */
    nt::GenericEntry * GetEntry();

    /**
     * Get's a UIWidget's name
     * @return widget's name
     */
    std::string GetName();

    /**
     * Get's a UIWidget's tab
     * @return widget's tab
     */
    std::string GetTab();
};