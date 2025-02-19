/// @brief Errors related to the options of the UsbCam library
#ifndef V4L__OPTIONS__ERRORS_HPP_
#define V4L__OPTIONS__ERRORS_HPP_
#include <map>
#include <exception>
#include <functional>
#include <string>
#include <iostream>
#include <cstring>

namespace v4l
{

typedef std::string error_details_t;

enum class OptionError
{
  NO_OPTIONS_PROVIDED,
  UNSUPPORTED,
  OUT_OF_BOUNDS,
  INVALID
};

class V4LError : public std::exception
{
public:
  V4LError(const std::string & message)
  : m_error_msg(message)
  {
    m_error_msg += "\n" + std::string(strerror(errno));
  }

  const char * what() const noexcept override
  {
    return m_error_msg.c_str();
  }

protected:
  std::string m_error_msg;
};

class QueryError : public V4LError {
public:
  QueryError()
  : V4LError("Query error:") {}
};


class FailedToOpen : public V4LError {
public:
  FailedToOpen(const std::string & message)
  : V4LError("Failed to open V4L2 device: " + message) {}
};

class NoOptionsError : public V4LError {
public:
  NoOptionsError()
  : V4LError("No options provided") {}
};

class UnsupportedError : public V4LError {
public:
  UnsupportedError(const std::string & message)
  : V4LError("Unsupported option provided: " + message) {}
};

class MissingOptionValueError : public V4LError {
public:
  MissingOptionValueError(const std::string & message)
  : V4LError("Option is missing a value: " + message) {}
};

class NoDeviceCapabilities : public V4LError {
public:
  NoDeviceCapabilities(const std::string & message)
  : V4LError("Could not retrieve device capabilities: " + message) {}
};

class VideoInputError : public V4LError {
public:
  VideoInputError(const std::string & message)
  : V4LError(std::string("Unable to get video input: test " + message)) {}
};

class VideoOutputError : public V4LError {
public:
  VideoOutputError(const std::string & message)
  : V4LError("Unable to get video output: " + message) {}
};

}  // namespace v4l

#endif  // V4L__OPTIONS__ERRORS_HPP_
