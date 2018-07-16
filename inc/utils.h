/*
 * utils.h
 *
 *  Created on: 20 de jun. de 2018
 *      Author: Matute
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "stdint.h"

#define MENSAJE_BIENVENIDA "Bienvenidos!"

void initHardware(void);
void vPrintString(char *);
void vPrintNumber(int32_t);
void vPrintStringAndNumber(char *, int32_t);
void format(float, char *, uint8_t);


#endif /* UTILS_H_ */
