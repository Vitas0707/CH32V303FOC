#ifndef __MENU_H__
#define __MENU_H__

#include "key.h"
#include "foc.h"  // FOC_Mode, foc_mode/target/actual_* 现在定义在foc.h/foc.c

void Menu_Init(void);
void Menu_Run(KEY_Event evt);

#endif // __MENU_H__
