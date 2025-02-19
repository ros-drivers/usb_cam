/// @brief A CLI used for testing of the V4L library
#include <string>
#include <vector>
#include <iostream>
#include <map>
#include <optional>
#include <algorithm>

#include "v4l/cli.hpp"
#include "v4l/errors.hpp"


namespace v4l
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
    auto found_option_at = std::find(CLIOptions.begin(), CLIOptions.end(), option);
    if (found_option_at == CLIOptions.end()) {
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

} // namespace v4l


int main(int argc, char * argv[])
{
  auto result = 0;
  v4l::input_args_t args(argv + 1, argv + argc);

  try {
    v4l::Cli cli(args);
  } catch (const v4l::V4LError & err) {
    std::cerr << "[V4LError] " << err.what() << std::endl;
    result = 1;
  } catch (const std::exception & err) {
    std::cerr << err.what() << std::endl;
    result = 1;
  }

  return result;
}
