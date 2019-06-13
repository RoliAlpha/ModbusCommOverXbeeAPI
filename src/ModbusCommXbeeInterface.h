#pragma once
#include <Arduino.h>

#ifndef _ModbusCommXbeeInterface_h
#define _ModbusCommXbeeInterface_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
//不同板子关于“ModbusRTUComm(MySerial * port, long baud, byte slaveId);”中“port”的定义不同；
//mega2560定义的是HardwareSerial；lenoard 定义的是 Serial_
#if defined (USBCON)
#define  MySerial Serial_
#else
#define  MySerial HardwareSerial 
#endif

#include <CommunicationBase.h>
#include "dataConvertion.h"
#include <stdlib.h>
#include "XBee.h"
class ModbusXbeeComm:public CommunicationBase
{
public:
	ModbusXbeeComm(MySerial * _Freeport, HardwareSerial * _Xbeeport, long baud, byte slaveId, XBeeAddress64 &addr64, uint16_t addr16,
		int tx_statusLed=13, int tx_errorLed=13, int rx_statusLed = 12, int rx_errorLed = 12);
	~ModbusXbeeComm();
	
	int HandleCommunication(Host* host);
	void rx_Comm(XBee&xbee);
	void tx_Comm(Host * host, XBee&xbee);
	uint8_t getDataFrame();


private:
	XBee xbee;
	XBeeAddress64 addr64;
	uint16_t addr16;
	byte modbusSlaveId;
	MySerial * Freeport;
	HardwareSerial * Xbeeport;
	// create reusable response objects for responses we expect to handle 
	ZBRxResponse rx;
	ModemStatusResponse msr;
	//create a byte array storage the data from received API message
	uint8_t* dataFrame ;
	uint8_t dataFrameLength ;
	//set TX indicator LED at Pin 13 to show TX status and TX response status
	int tx_statusLed ;
	int tx_errorLed ;
	//set RX indicator LED at Pin 13 to show RX status and RX response status
	int rx_statusLed ;
	int rx_errorLed ;
	void flashLed(int pin, int times, int wait);
	bool checkSumOk(byte * frame);
	void readResponse(Host*_host, uint8_t*_dataFrame);
	void writeResponse(Host*_host, uint8_t*_dataFrame);
	void sendResultAndCheck(byte u8ModbusTransmitADUSize, byte* u8ModbusTransmitADU);
};








#endif