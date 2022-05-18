/*! @file UnitConversion.h
    @author Matteo Malosio (matteo.malosio@itia.cnr.it)
    @version Revision 0.1
    @brief A set of functions to facilitate unit conversions.
    @details The function name format is \code convertAToB \endcode
      where
      \code A \endcode is the source unit
      \code B \endcode is the destination unit according to unit symbols <br>
    @date September 20, 2014
    \todo: Substitute Convert... with convert...
*/


#ifndef UNIT_CONVERSION
#define UNIT_CONVERSION

inline double ConvertMsToS(double val) {
  return val / 1000.;
}


inline double ConvertUsToS(double val) {
  return val / 1000000.;
}


inline double ConvertSToMs(double val) {
  return val * 1000.;
}


inline double ConvertSToUs(double val) {
  return val * 1000000.;
}


#endif
