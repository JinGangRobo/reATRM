// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__CRC_HPP_
#define RM_SERIAL_DRIVER__CRC_HPP_

#include <cstdint>
#include <vector>

namespace crc8
{
extern uint8_t get_CRC8_check_sum(uint8_t * pchMessage, unsigned int dwLength, uint8_t ucCRC8);

extern uint32_t verify_CRC8_check_sum(uint8_t * pchMessage, unsigned int dwLength);

extern void append_CRC8_check_sum(uint8_t * pchMessage, unsigned int dwLength);
}  // namespace crc8

namespace crc16
{
/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */
uint32_t Verify_CRC16_Check_Sum(const uint8_t * pchMessage, uint32_t dwLength);

/**
  * @brief Append CRC16 value to the end of the buffer
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return none
  */
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);
extern bool verify_CRC16_check_sum(std::vector<uint8_t> & pchMessage);

}  // namespace crc16

#endif  // RM_SERIAL_DRIVER__CRC_HPP_
