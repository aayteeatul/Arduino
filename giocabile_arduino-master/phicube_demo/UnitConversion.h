#ifndef UNIT_CONVERSION
#define UNIT_CONVERSION

inline double ConvertMsToS(double val) {return val / 1000.;}
inline double ConvertUsToS(double val) {return val / 1000000.;}
inline double ConvertSToMs(double val) {return val * 1000.;}
inline double ConvertSToUs(double val) {return val * 1000000.;}

#endif
