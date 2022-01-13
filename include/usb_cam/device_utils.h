#include <map>
#include <string>

/**
 * Type alias for map of <std::string, std::string>
 */
using str_map = std::map<std::string, std::string>;

/**
 * @brief A function that uses libudev to find all connected usb_devices and
 * serial number.
 * This function save a map <std::string, std::string> where the device file
 * path is the key of the map.
 * 
 * @return map of <std::string, std::string> where the device file path is the
 * key.
 */
str_map get_serial_dev_info();

/**
 * @brief A function that removes the devices file path from map that doesn't
 * support the expected pixel_format.
 * 
 * @param maps reference of map of <std::string, std::string> with the device
 * file path and the device's serial number. The function will erase the device
 * file path that doesn't match the criteria.
 * @param pixel_format  the selected pixel format.
 */
void clear_unsupported_devices(str_map& maps, std::string pixel_format);

/**
 * @brief A function that create a map that associate the pixel format name
 * used by usb_cam and V4L.
 * This function save a map <std::string, std::string> where the usb_cam pixel
 * format name is the key of the map.
 * 
 * @return map of <std::string, std::string> where the usb_cam pixel format
 * name is the key.
 */
str_map get_pixel_format_map();

/**
 * @brief A function that get a pixel format name that works for V4L.
 * 
 * @param maps reference of map of <std::string, std::string> with the pixel
 * format name used by usb_cam and V4L.
 * @param pixel_format  the pixel format name used at usb_cam.
 * 
 * @return std::string  the pixel format name used at V4L.
 */
std::string get_pixel_format_v4l(str_map& maps, std::string pixel_format);
