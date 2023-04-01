/*
 * main.h
 *
Copyright (c) 2017 EVTV / Collin Kidder

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include "CANUtil.h"

#ifndef MAIN_H_
#define MAIN_H_

void loadSettings();
void alarmupdate();
void gaugeupdate();
void printbmsstat();
void getcurrent();
void updateSOC();
void SOCcharged(int);
void Prechargecon();
void contcon();
void calcur();
void VEcan();
void BMVmessage();
void menu();
void canread();
void CAB300();
void CAB500();
void currentlimit();
void inputdebug();
void outputdebug();
void balancing();
void sendcommand();
void resetwdog();
void pwmcomms();
void dashupdate();
void chargercomms();
void resetbalancedebug();
void handleVictronLynx();
void isrCP();
void low_voltage_isr();
int pgnFromCANId(int canId);
uint8_t getcheck(CAN_message_t &msg, int id);
void Rx309();

#endif /* MAIN_H_ */
