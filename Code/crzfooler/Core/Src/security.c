/*
 * security.c
 *
 *  Created on: May 10, 2025
 *      Author: widos
 */
#include "security.h"

uint8_t sum_nibbles(uint8_t *data) {
	uint8_t sum = 0;
	for (uint8_t i = 0; i < 7; i++) {
		sum += (data[i] & 0xf) + ((data[i] >> 4) & 0xf);
	}
	return sum;
}

uint8_t checksum_115(uint8_t next_algo, uint8_t *data) {
	uint8_t sum = sum_nibbles(data);

	switch (next_algo) {
	case 1:
		sum += 0x0F;
		break;
	case 2:
		sum += 0;
		break;
	case 3:
		sum += 0x01;
		break;
	case 0:
	default:
		sum += 0x0E;
		break;
	}
	sum = ~sum;
	sum &= 0xf;
	sum = next_algo << 4 | sum;
	return sum;
}
uint8_t checksum_17d(uint8_t next_algo, uint8_t *data) {
	uint8_t sum = sum_nibbles(data);

	sum &= 0xf;
	sum = ~sum;
	switch (next_algo) {
	case 1:
		sum += 3;
		break;
	case 2:
		sum += 2;
		break;
	case 3:
		sum += 1;
		break;
	case 0:
	default:
		sum += 4;
		break;
	}

	sum = next_algo << 4 | sum;
	return sum;
}
uint8_t checksum_24f(uint8_t next_algo, uint8_t *data) {
	return checksum_17d(next_algo, data);
}

uint8_t checksum_166(uint8_t next_algo, uint8_t *data) {
	uint8_t sum = sum_nibbles(data);

	switch (next_algo) {
	case 1:
		sum += 5;
		break;
	case 2:
		sum += 6;
		break;
	case 3:
		sum += 7;
		break;
	case 0:
	default:
		sum += 4;
		break;
	}
	sum &= 0xf;
	sum = ~sum;

	sum = next_algo << 4 | sum;
	return sum;
}
uint8_t checksum_1e2(uint8_t next_algo, uint8_t *data) {
	uint8_t sum = sum_nibbles(data);

	switch (next_algo) {
	case 1:
		sum += 0x09;
		break;
	case 2:
		sum += 0x0A;
		break;
	case 3:
		sum += 0x0B;
		break;
	case 0:
	default:
		sum += 0x08;
		break;
	}
	sum &= 0xf;
	sum = ~sum;

	sum = next_algo << 4 | sum;
	return sum;
}
void key_27_05(uint8_t *seed, uint8_t *result) {
	uint16_t k1;
	uint16_t k2;
	uint8_t k3;
	uint8_t k4;
	if (seed[2] == 0x69) {
		k1 = 62603;
		k2 = 7632;
		k3 = 14;
		k4 = 15;
	} else if (seed[2] == 0x6b) {
		k1 = 46336;
		k2 = 32973;
		k3 = 1;
		k4 = 3;
	}

	const uint16_t KEY1 = k1;
	const uint16_t KEY3 = k2;

	uint16_t W1 = (seed[0] << 8) | seed[1];
	uint16_t W2 = W1 + KEY1;
	W2 = (W2 >> k3) | (W2 << (16 - k3));
	uint16_t rotated_W1 = (W1 << k4) | (W1 >> (16 - k4));
	uint16_t final_result = (rotated_W1 * W2) & 0xFFFF;
	final_result = (final_result + KEY3) & 0xFFFF;

	result[0] = (final_result >> 8) & 0xFF;
	result[1] = final_result & 0xFF;
	result[2] = seed[3];
}

