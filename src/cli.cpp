/// @brief A pure C++ v4l2 driver using the UsbCam library
/// @param argc 
/// @param argv 
/// @return status code
#include <string>
#include <vector>
#include <iostream>
#include <map>
#include <optional>
#include <algorithm>

#include "usb_cam/cli.hpp"
#include "usb_cam/options/errors.hpp"


namespace usb_cam
{

Cli::Cli(input_args_t & args)
: m_settings(parse_args(args)),
  m_device(m_settings["--device"].c_str())
{}

Cli::~Cli() {}

options_with_values_t Cli::parse_args(input_args_t & args)
{
    if (args.empty()) {
        throw NoOptionsError();
    }

    options_with_values_t parsed_args;
    for (size_t i = 0; i < args.size(); ++i) {
        auto & option = args[i];
        auto found_option_at = std::find(UsbCamOptions.begin(), UsbCamOptions.end(), option);
        if (found_option_at == UsbCamOptions.end()) {
            throw UnsupportedError(option);
        }
        // Next argument should be the option value
        i += 1;
        if (args.size() <= i) {
            throw MissingOptionValueError(option);
        }
        auto & value = args[i];
        parsed_args[option] = value;
    }
    return parsed_args;
}

} // namespace usb_cam



int main(int argc, char* argv[])
{
    auto result = 0;
    usb_cam::input_args_t args(argv + 1, argv + argc);

    try {
        usb_cam::Cli cli(args);
    } catch (const usb_cam::UsbCamError & err) {
        std::cerr << err.what() << std::endl;
        result = 1;
    } catch (const std::exception & err) {
        std::cerr << err.what() << std::endl;
        result = 1;
    }

    return result;
}	
