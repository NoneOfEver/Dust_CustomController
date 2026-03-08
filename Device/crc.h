/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       crc8_crc16.c/h
  * @brief      crc8 and crc16 calculate function, verify function, append function.
  *             crc8��crc16���㺯��,У�麯��,���Ӻ���
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef CRC8_CRC16_H
#define CRC8_CRC16_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief          ����CRC8
 * @param[in]      pch_message: ����
 * @param[in]      dw_length: ���ݺ�У��ĳ���
 * @param[in]      ucCRC8:��ʼCRC8
 * @retval         �������CRC8
 */
extern uint8_t get_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);

/**
 * @brief          CRC8У�麯��
 * @param[in]      pch_message: ����
 * @param[in]      dw_length: ���ݺ�У��ĳ���
 * @retval         ����߼�
 */
extern uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

/**
 * @brief          ����CRC8�����ݵĽ�β
 * @param[in]      pch_message: ����
 * @param[in]      dw_length: ���ݺ�У��ĳ���
 * @retval         none
 */
extern void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

/**
 * @brief          ����CRC16
 * @param[in]      pch_message: ����
 * @param[in]      dw_length: ���ݺ�У��ĳ���
 * @param[in]      wCRC:��ʼCRC16
 * @retval         �������CRC16
 */
extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

/**
 * @brief          CRC16У�麯��
 * @param[in]      pch_message: ����
 * @param[in]      dw_length: ���ݺ�У��ĳ���
 * @retval         ����߼�
 */
extern uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

/**
 * @brief          ����CRC16�����ݵĽ�β
 * @param[in]      pch_message: ����
 * @param[in]      dw_length: ���ݺ�У��ĳ���
 * @retval         none
 */
extern void append_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

#ifdef __cplusplus
}
#endif
#endif
