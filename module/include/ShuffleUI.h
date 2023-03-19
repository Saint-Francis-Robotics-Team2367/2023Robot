#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <vector>
#include "UIWidget.h"

// A static class for making Shuffleboard easier to use
class ShuffleUI {
  private:
    // static vector to store info on created widgets, using the UIWidget class.
    static std::vector<UIWidget *> widgetList;

    /**
     * Adds a UIWidget to a list of created widgets
     *
     * @param widget pointer to a UIWidget
     */
    static void AddEntry(UIWidget *widget);


  public:
    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, int value);
    
    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, double value);
    
    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, float value);

    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, bool value);

    /**
     * Creates a new widget, or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidget(std::string name, std::string tab, std::string value);

    /**
     * Creates a new widget at a specified position (0,0 being the top left), or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @param posX X position
     * @param posY Y position
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidgetPos(std::string name, std::string tab, int value, int posX, int posY);

    /**
     * Creates a new widget at a specified position (0,0 being the top left), or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @param posX X position
     * @param posY Y position
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidgetPos(std::string name, std::string tab, double value, int posX, int posY);

    /**
     * Creates a new widget at a specified position (0,0 being the top left), or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @param posX X position
     * @param posY Y position
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidgetPos(std::string name, std::string tab, float value, int posX, int posY);

    /**
     * Creates a new widget at a specified position (0,0 being the top left), or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @param posX X position
     * @param posY Y position
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidgetPos(std::string name, std::string tab, bool value, int posX, int posY);

    /**
     * Creates a new widget at a specified position (0,0 being the top left), or updates an existing widget's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @param posX X position
     * @param posY Y position
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeWidgetPos(std::string name, std::string tab, std::string value, int posX, int posY);

    /**
     * Creates a new widget with a slider.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param min Slider lower bound
     * @param max Slider upper bound
     * @param defaultVal Slider's default value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeSlider(std::string name, std::string tab, double min, double max, double defaultVal);

    /**
     * Finds an existing widget.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @return pointer to the widget's entry, or NULL if widget does not exist
     */
    static nt::GenericEntry *GetEntry(std::string name, std::string tab);

    /**
     * Finds an existing widget storing a double and returns its value.
     * If entry does not exist or is a different type, returns defaultVal.
     * 
     * @param name Widget's name
     * @param tab Widget's tab
     * @param defaultVal Value to return if error
     * @return widget's value, or defaultVal if error getting the value
    */
    static double GetDouble(std::string name, std::string tab, double defaultVal);

    /**
     * Finds an existing widget storing a int and returns its value.
     * If entry does not exist or is a different type, returns defaultVal.
     * 
     * @param name Widget's name
     * @param tab Widget's tab
     * @param defaultVal Value to return if error
     * @return widget's value, or defaultVal if error getting the value
    */
    static int GetInt(std::string name, std::string tab, int defaultVal);

    /**
     * Finds an existing widget storing a bool and returns its value. 
     * If entry does not exist or is a different type, returns defaultVal.
     * 
     * @param name Widget's name
     * @param tab Widget's tab
     * @param defaultVal Value to return if error
     * @return widget's value, or defaultVal if error getting the value
    */
    static bool GetBool(std::string name, std::string tab, bool defaultVal);

    /**
     * Finds an existing widget storing a float and returns its value.
     * If entry does not exist or is a different type, returns defaultVal.
     * 
     * @param name Widget's name
     * @param tab Widget's tab
     * @param defaultVal Value to return if error
     * @return widget's value, or defaultVal if error getting the value
    */
    static float GetFloat(std::string name, std::string tab, float defaultVal);

    /**
     * Creates a new widget with a graph, or updates an existing graph's value.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param value Widget's value
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeGraph(std::string name, std::string tab, double value);

    /**
     * Creates a new widget with a button.
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param defaultState Widget's default state
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeButton(std::string name, std::string tab, bool defaultState);

    /**
     * Creates a new widget with a button at a specific position.
     * (0,0) is the top left
     *
     * @param name Widget's name
     * @param tab Widget's tab
     * @param defaultState Widget's default state
     * @param posX X position
     * @param posY Y position
     * @return pointer to the widget's entry
     */
    static nt::GenericEntry *MakeButtonPos(std::string name, std::string tab, bool defaultState, int posX, int posY);

    /**
     * Prints the names of all widgets.
     */
    static void PrintWidgetList();
};