/// @brief Errors related to the options of the UsbCam library
#ifndef USB_CAM__OPTIONS__ERRORS_HPP_
#define USB_CAM__OPTIONS__ERRORS_HPP_
#include <map>
#include <exception>
#include <functional>
#include <string>
#include <iostream>


namespace usb_cam
{

typedef std::string error_details_t;

enum class OptionError {
    NO_OPTIONS_PROVIDED,
    UNSUPPORTED,
    OUT_OF_BOUNDS,
    INVALID
};

class UsbCamError : public std::exception {};

class NoOptionsError : public UsbCamError {
public:
    const char* what() const noexcept override {
        return "No options provided";
    }
};

class UnsupportedError : public std::exception {
public:
    UnsupportedError(const std::string & message)
    : m_msg("Unsupported option provided: " + message) {}

    const char* what() const noexcept override {
        return m_msg.c_str();
    }

private:
    std::string m_msg;
};

class MissingOptionValueError : public std::exception {
public:
    MissingOptionValueError(const std::string & message)
    : m_msg("Option is missing a value: " + message) {}

    const char* what() const noexcept override {
        return m_msg.c_str();
    }

private:
    std::string m_msg;
};

// const error_details_t fmt_unsupported_error(const error_details_t & detail_str) {
//     return "Unsupported option provided: " + detail_str;
// }

// const error_details_t fmt_out_of_bounds_error(const error_details_t & detail_str) {
//     return "Given option is out of bounds: " + detail_str;
// }

// const error_details_t fmt_invalid_error(const error_details_t & detail_str) {
//     return "Given option is invalid: " + detail_str;
// }

}  // namespace usb_cam

#endif  // USB_CAM__OPTIONS__ERRORS_HPP_
