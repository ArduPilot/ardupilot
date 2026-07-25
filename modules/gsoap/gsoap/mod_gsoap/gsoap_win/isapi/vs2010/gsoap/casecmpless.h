/** Interface for the casecmpless struct
    @file casecmpless.h
  */
#ifndef CASECMPLESS_H
#define CASECMPLESS_H

/** helper for case-insensitive less<> operator for the map<> template.*/
struct casecmpless {
	bool operator()(const std::string &s1, const std::string &s2) const { 
#ifdef _WIN32
		return _stricmp(s1.c_str(), s2.c_str()) < 0;
#else
		return strcasecmp(s1.c_str(), s2.c_str()) < 0;
#endif
	};
};

#endif //CASECMPLESS_H
