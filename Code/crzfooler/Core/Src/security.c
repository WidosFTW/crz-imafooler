/*
 * security.c
 *
 *  Created on: May 10, 2025
 *      Author: widos
 */
#include "security.h"

uint8_t sum_nibbles(uint8_t *data){
	uint8_t sum=0;
	for (uint8_t i=0; i<7;i++){
		sum+=(data[i]&0xf)+(data[i]>>4)&0xf;
		printf("%d:%d data=%d\n",i,sum,data[i]);
	}
	return sum;
}

uint8_t checksum_115(uint8_t next_algo,uint8_t *data)
{
	uint8_t sum=sum_nibbles(data);

	switch (next_algo) {
		case 1:
			sum+=0x0F;
			break;
		case 2:
			sum+=0;
			break;
		case 3:
			sum+=0x01;
			break;
		case 0:
		default:
			sum+=0x0E;
			break;
	}
	sum=~sum;
	sum&=0xf;
	sum=next_algo<<4|sum;
	return sum;
}
uint8_t checksum_17d(uint8_t next_algo,uint8_t *data)
{
	uint8_t sum=sum_nibbles(data);

	sum&=0xf;
	sum=~sum;
	switch (next_algo) {
		case 1:
			sum+=3;
			break;
		case 2:
			sum+2;
			break;
		case 3:
			sum+=1;
			break;
		case 0:
		default:
			sum+=4;
			break;
	}

	sum=next_algo<<4|sum;
	return sum;
}
uint8_t checksum_24f(uint8_t next_algo,uint8_t *data)
{
	return checksum_17d(next_algo, data);
}

uint8_t checksum_166(uint8_t next_algo,uint8_t *data)
{
	uint8_t sum=sum_nibbles(data);


	switch (next_algo) {
		case 1:
			sum+=5;
			break;
		case 2:
			sum+6;
			break;
		case 3:
			sum+=7;
			break;
		case 0:
		default:
			sum+=4;
			break;
	}
	sum&=0xf;
	sum=~sum;

	sum=next_algo<<4|sum;
	return sum;
}
uint8_t checksum_1e2(uint8_t next_algo,uint8_t *data)
{
	uint8_t sum=sum_nibbles(data);


	switch (next_algo) {
		case 1:
			sum+=0x09;
			break;
		case 2:
			sum+0x0A;
			break;
		case 3:
			sum+=0x0B;
			break;
		case 0:
		default:
			sum+=0x08;
			break;
	}
	sum&=0xf;
	sum=~sum;

	sum=next_algo<<4|sum;
	return sum;
}

