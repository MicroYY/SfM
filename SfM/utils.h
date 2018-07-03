#pragma once

#include <string>

std::string right(std::string const& str, std::size_t chars);


inline std::string
right(std::string const& str, std::size_t chars)
{
	return str.substr(str.size() - std::min(str.size(), chars));
}