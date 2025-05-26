/*
 * security.h
 *
 *  Created on: May 10, 2025
 *      Author: widos
 */

#ifndef INC_SECURITY_H_
#define INC_SECURITY_H_

#include "stdint.h"


uint8_t checksum_115(uint8_t i,uint8_t * data);
uint8_t sum_nibbles(uint8_t *data) ;
uint8_t checksum_115(uint8_t next_algo, uint8_t *data) ;
uint8_t checksum_17d(uint8_t next_algo, uint8_t *data) ;
uint8_t checksum_24f(uint8_t next_algo, uint8_t *data) ;
uint8_t checksum_166(uint8_t next_algo, uint8_t *data) ;
uint8_t checksum_1e2(uint8_t next_algo, uint8_t *data) ;
void key_27_05(uint8_t *seed, uint8_t *result) ;



#endif /* INC_SECURITY_H_ */
