#ifndef COMMON_CIB_UTILS_H_
#define COMMON_CIB_UTILS_H_
extern "C"
{
  #include <inttypes.h>
}
#include <cstdint>
#ifndef CIB_DUNEDAQ
#define CIB_DUNEDAQ 1
#endif
#include <cib_data_fmt.h>

namespace dunedaq
{
  namespace cibmodules
  {

    // some helper functions outside of the class
    // taken from the cib_data_utils
    namespace util
    {
      uint32_t bitmask(uint32_t highbit, uint32_t lowbit);
      int32_t cast_to_signed(const uint32_t reg, const uint32_t mask); 
      int32_t get_m1(dunedaq::cib::daq::iols_trigger_t &t); 
      int32_t get_m2(dunedaq::cib::daq::iols_trigger_t &t); 
      int32_t get_m3(dunedaq::cib::daq::iols_trigger_t &t);
      bool parse_hex(std::string_view s, std::uint32_t &out);

    } // namespace util
  } // namespace cibmodules
} // namespace dunedaq
#endif /* COMMON_CIB_UTILS_H_ */