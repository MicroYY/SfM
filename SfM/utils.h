#pragma once

#include <string>


#define MATH_PI         3.14159265358979323846264338327950288   // pi

#define MATH_RAD2DEG(x) ((x) * (180.0 / MATH_PI))
#define MATH_DEG2RAD(x) ((x) * (MATH_PI / 180.0))




std::string right(std::string const& str, std::size_t chars);


inline std::string
right(std::string const& str, std::size_t chars)
{
	return str.substr(str.size() - std::min(str.size(), chars));
}




template <typename T>
T const&
clamp(T const& v, T const& min = T(0), T const& max = T(1))
{
	return (v < min ? min : (v > max ? max : v));
}



template <typename T>
void
vector_clean(std::vector<int> const& delete_list, std::vector<T>* vector)
{
	typename std::vector<T>::iterator vr = vector->begin();
	typename std::vector<T>::iterator vw = vector->begin();
	typename std::vector<int>::const_iterator dr = delete_list.begin();

	while (vr != vector->end() && dr != delete_list.end())
	{
		if (*dr++)
		{
			vr++;
			continue;
		}
		if (vw != vr)
			*vw = *vr;
		vw++;
		vr++;
	}
	vector->erase(vw, vector->end());
}