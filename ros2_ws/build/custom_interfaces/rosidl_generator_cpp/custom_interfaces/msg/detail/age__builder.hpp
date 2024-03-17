// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/Age.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__AGE__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__AGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/age__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_Age_day
{
public:
  explicit Init_Age_day(::custom_interfaces::msg::Age & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::Age day(::custom_interfaces::msg::Age::_day_type arg)
  {
    msg_.day = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::Age msg_;
};

class Init_Age_month
{
public:
  explicit Init_Age_month(::custom_interfaces::msg::Age & msg)
  : msg_(msg)
  {}
  Init_Age_day month(::custom_interfaces::msg::Age::_month_type arg)
  {
    msg_.month = std::move(arg);
    return Init_Age_day(msg_);
  }

private:
  ::custom_interfaces::msg::Age msg_;
};

class Init_Age_year
{
public:
  Init_Age_year()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Age_month year(::custom_interfaces::msg::Age::_year_type arg)
  {
    msg_.year = std::move(arg);
    return Init_Age_month(msg_);
  }

private:
  ::custom_interfaces::msg::Age msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::Age>()
{
  return custom_interfaces::msg::builder::Init_Age_year();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__AGE__BUILDER_HPP_
