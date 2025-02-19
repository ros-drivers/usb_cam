/// @brief A CLI used for testing of the V4L library
#ifndef V4L__CLI_HPP
#define V4L__CLI_HPP

#include "v4l/errors.hpp"
#include "v4l/device.hpp"

namespace v4l
{

typedef std::string option_flag_t;
typedef std::vector<option_flag_t> options_t;
typedef std::vector<std::string> input_args_t;
typedef std::map<option_flag_t, std::string> options_with_values_t;

const options_t CLIOptions = {
  "--device"
};

class Cli {
public:
  Cli(input_args_t & args);
  ~Cli();

private:
  options_with_values_t parse_args(input_args_t & args);

  options_with_values_t m_settings;
  v4l::Device m_device;
};

} // namespace v4l


#endif  // V4L__CLI_HPP
