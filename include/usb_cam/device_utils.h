#include <map>
#include <string>

/**
 * Type alias for map of <std::string, std::string>
 */
using str_map = std::map<std::string, std::string>;

/**
 * A function that uses libudev to find all connected usb_devices and serial
 * number.
 * This function save a map <std::string, std::string> where the serial number
 * is the key of the map.
 * 
 * @return map of <std::string, std::string> where the serial number is the key
 */
str_map get_serial_dev_info();
