#include <cstddef>
#include <charconv>
#include <cib_utilities.h>

namespace dunedaq::cibmodules
{

  // some helper functions outside of the class
  // taken from the cib_data_utils
  namespace util
  {

    uint32_t bitmask(uint32_t highbit, uint32_t lowbit)
    {
      // sort the bit order or this fails miserably
      if (highbit < lowbit)
      {
        uint32_t tmp = lowbit;
        lowbit = highbit;
        highbit = tmp;
      }

      uint32_t i = ~0U;
      return ~(i << highbit << 1) & (i << lowbit);
    }

    // converts a masked unsigned value into a signed
    // the mask is always assumed to start at 0, so the value has to be shifted right until the lsb aligns with 0
    int32_t cast_to_signed(const uint32_t reg, const uint32_t mask)
    {
      // first find the msb in the mask. That will be the signed bit
      uint32_t msb = 0;
      int32_t res = 0;
      for (size_t bit = 31; bit > 0; bit--)
      {
        if ((1U << bit) & mask)
        {
          msb = bit;
          break;
        }
      }
      //  check the msb of the register. That is the sign bit
      if ((1U << msb) & reg)
      {
        // spdlog::trace("MSB of the mask is {0}",msb);

        res = bitmask(31, msb + 1); // set all bits to 1 above the mask
        res = res | (reg & mask);
        // it is a negative value. Set the msb in the result
      }
      else
      {
        // it is a positive value. No need to set the sign bit, but still need to
        // apply the mask or we're carrying out the other bits that may be outside the mask
        res = (reg & mask);
      }
      return res;
    }

    int32_t get_m1(dunedaq::cib::daq::iols_trigger_t &t)
    {
      return cast_to_signed(t.pos_m1, t.bitmask_m1);
    }

    int32_t get_m2(dunedaq::cib::daq::iols_trigger_t &t)
    {
      uint32_t m2_lsb = t.pos_m2_lsb;
      uint32_t m2_msb = t.pos_m2_msb;
      uint32_t m2 = (m2_msb << 15) | m2_lsb;
      // the bitmask is the same
      return cast_to_signed(m2, t.bitmask_m2);
    }

    int32_t get_m3(dunedaq::cib::daq::iols_trigger_t &t)
    {
      return cast_to_signed(t.pos_m3, t.bitmask_m3);
    }

    bool parse_hex(std::string_view s, std::uint32_t &out)
    {
      // Optional 0x / 0X prefix
      if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
        s.remove_prefix(2);

      // Empty after stripping?
      if (s.empty())
        return false;

      auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out, 16);

      // ec=={} means parse OK; ptr at end means no trailing garbage
      return ec == std::errc{} && ptr == s.data() + s.size();
    }

  } // namespace util
} // namespace dunedaq::cibmodules