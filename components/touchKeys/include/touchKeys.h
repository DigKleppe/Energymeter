/*
 * touchKeys.h
 *
 *  Created on: Jan 25, 2023
 *      Author: dig
 */

#ifndef COMPONENTS_TOUCHKEYS_INCLUDE_TOUCHKEYS_H_
#define COMPONENTS_TOUCHKEYS_INCLUDE_TOUCHKEYS_H_

void startTouchKeys(void);
extern volatile bool keyIn;
extern volatile uint16_t touchValue;


#endif /* COMPONENTS_TOUCHKEYS_INCLUDE_TOUCHKEYS_H_ */
