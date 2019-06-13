#include"ModbusCommXbeeInterface.h"

ModbusXbeeComm::ModbusXbeeComm(MySerial * _Freeport, HardwareSerial * _Xbeeport, long baud, byte slaveId, XBeeAddress64 &_addr64, uint16_t _addr16,
	int _tx_statusLed = 13, int _tx_errorLed = 13, int _rx_statusLed = 12, int _rx_errorLed = 12)
{
	this->Freeport = _Freeport;
	(*_Freeport).begin(baud);
	this->Xbeeport = _Xbeeport;
	(*_Xbeeport).begin(baud);
	//set _Xbeeport to xbee module
	xbee.setSerial(*_Xbeeport);
	modbusSlaveId = slaveId;
	this->addr64 = _addr64;
	this->addr16 = _addr16;
	this->tx_statusLed = _tx_statusLed;
	this->tx_errorLed = _tx_errorLed;
	this->rx_statusLed = _rx_statusLed;
	this->rx_errorLed = _rx_errorLed;
	this->dataFrame = NULL;
	this->dataFrameLength = 0;
}

ModbusXbeeComm::~ModbusXbeeComm()
{
}

int ModbusXbeeComm::HandleCommunication(Host * host)
{
	//receive RX packet or MODEM status response packet
	rx_Comm(xbee);
	//send inquire result packet
	tx_Comm(host,xbee);
	return 0;
}

void ModbusXbeeComm::rx_Comm(XBee & xbee)
{
	//receive RX packet
	xbee.readPacket();

	if (xbee.getResponse().isAvailable()) {
		//Serial.println("GET");
		// got something

		if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
			// got a zb rx packet
			//Serial.write(device_xbee.getResponse().getApiId());
			// now fill our zb rx class
			xbee.getResponse().getZBRxResponse(rx);

			if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED) {
				// the sender got an ACK
				//Serial.write(rx.getOption());
				flashLed(rx_statusLed, 10, 10);
			}
			else {
				// we got it (obviously) but sender didn't get an ACK
				flashLed(rx_errorLed, 2, 20);
			}
			// get the data from RX packet
			dataFrameLength = getDataFrame();
			/*for (size_t i = 0; i < dataFrameLength; i++)
			{
			Serial.write(dataFrame[i]);
			}*/
		}
		else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
			xbee.getResponse().getModemStatusResponse(msr);
			// the local XBee sends this response on certain events, like association/dissociation

			if (msr.getStatus() == ASSOCIATED) {
				// this is great.  flash led
				flashLed(rx_statusLed, 10, 10);
			}
			else if (msr.getStatus() == DISASSOCIATED) {
				// this is awful.. flash led to show our discontent
				flashLed(rx_errorLed, 10, 10);
			}
			else {
				// another status
				flashLed(rx_statusLed, 5, 10);
			}
		}
		else {
			// not something we were expecting
			flashLed(rx_errorLed, 1, 25);
		}
	}
	else if (xbee.getResponse().isError()) {
		Serial.print("Error reading packet.  Error code: ");
		Serial.println(xbee.getResponse().getErrorCode());
	}
}

void ModbusXbeeComm::tx_Comm(Host * host, XBee & xbee)
{
	//recognize the data if right inquire order
	//check address
	if (dataFrame[0] != modbusSlaveId)
	{
	return;
	}
	
	//check check sum
	if (dataFrame == NULL)
	{
		return;
	}
	if (!checkSumOk(dataFrame))
	{
		return;
	}
	//check function code
	if (dataFrame[1] != 3 && dataFrame[1] != 16)
	{
		return;
	}
	//determin command type
	switch (dataFrame[1])
	{
		//run the command
	case 3:
	{
		readResponse(host, dataFrame);
		dataFrame = NULL;
	}
	break;
	case 16:
	{
		writeResponse(host, dataFrame);
		dataFrame = NULL;
	}
	}
	return;
}

uint8_t ModbusXbeeComm::getDataFrame()
{
	dataFrame = rx.getData();
	return rx.getDataLength();
}

void ModbusXbeeComm::flashLed(int pin, int times, int wait)
{
	for (int i = 0; i < times; i++)
	{
		digitalWrite(pin, HIGH);
		delay(wait);
		digitalWrite(pin, LOW);

		if (i + 1 < times)
		{
			delay(wait);
		}
	}
}

bool ModbusXbeeComm::checkSumOk(byte * frame)
{
	unsigned int calcCheckSum = 0;
	for (byte i = 0; i < dataFrameLength - 2; i++)
	{
		calcCheckSum += frame[i];
	}
	unsigned int receivedCRC;
	unsigned char  _checkSumBytes[2];
	/*for (byte i = _frameLength - 2; i < _frameLength; i++)
	{
	_checkSumBytes[0] = frame[i];
	}*/
	_checkSumBytes[1] = frame[dataFrameLength - 2];
	_checkSumBytes[0] = frame[dataFrameLength - 1];

	Byte_to_Uint(&receivedCRC, _checkSumBytes);
	if (calcCheckSum != receivedCRC)
	{
		return false;
	}
	return true;
}

void ModbusXbeeComm::readResponse(Host * _host, uint8_t * _dataFrame)
{
	unsigned int _u16ReadChannelAddress;
	unsigned int _u16ReadChannelQty;
	//get start channel, and channel numbers to read
	unsigned int _u16RequestBuffer[2];
	for (byte i = 0; i < 2; i++)
	{
		_u16RequestBuffer[i] = word(_dataFrame[2 * i + 2], _dataFrame[2 * i + 3]);
	}
	_u16ReadChannelAddress = _u16RequestBuffer[0]; //use _u16RequestBuffer[0] storage 16bit ChannelAddress
	_u16ReadChannelQty = _u16RequestBuffer[1];     //use _u16RequestBuffer[1] storage 16bit ChannelQty
												   //make a reply frame
	float _32TransmitBuffer[64];
	for (byte i = 0; i < _u16ReadChannelQty; i++)
	{
		/* use _32TransmitBuffer[i] storage channelsamples[i] for preparing
		subsequent disassembling channelsample to transmit
		*/
		_32TransmitBuffer[i] = _host->ChannelSamples[_u16ReadChannelAddress + i];

	}
	byte u8ModbusTransmitADU[256];
	byte u8ModbusTransmitADUSize = 0;
	u8ModbusTransmitADU[0] = 2;
	u8ModbusTransmitADU[1] = 3;
	u8ModbusTransmitADU[2] = 4 * _u16ReadChannelQty;
	u8ModbusTransmitADUSize = u8ModbusTransmitADUSize + 3;

	//disassembling channelsample from float to byte[] and fill into u8ModbusTransmitADU[]
	for (size_t i = 0; i <_u16ReadChannelQty; i++)
	{
		unsigned char channelsampleByte[4];
		Float_to_Byte(_32TransmitBuffer[i], channelsampleByte);
		//Highbyte forward!!
		u8ModbusTransmitADU[4 * i + 3] = channelsampleByte[3];
		u8ModbusTransmitADU[4 * i + 4] = channelsampleByte[2];
		u8ModbusTransmitADU[4 * i + 5] = channelsampleByte[1];
		u8ModbusTransmitADU[4 * i + 6] = channelsampleByte[0];
		u8ModbusTransmitADUSize = u8ModbusTransmitADUSize + 4;
	}

	//Serial.println("sendready");
	sendResultAndCheck(u8ModbusTransmitADUSize, u8ModbusTransmitADU);
}

void ModbusXbeeComm::writeResponse(Host * _host, uint8_t * _dataFrame)
{
	unsigned int _u16WriteChannelAddress;
	unsigned int _u16WriteChannelQty;
	uint8_t u8Qty;
	//for each channel to write
	unsigned int _u16RequestBuffer[2];
	for (byte i = 0; i < 2; i++)
	{
		_u16RequestBuffer[i] = word(_dataFrame[2 * i + 2], _dataFrame[2 * i + 3]);//Highbyte foward
	}
	_u16WriteChannelAddress = _u16RequestBuffer[0]; //set WriteChannelAddress and WriteChannelQty
	_u16WriteChannelQty = _u16RequestBuffer[1];
	u8Qty = _dataFrame[6];
	//disassemble Payload
	for (byte i = 0; i < _u16WriteChannelQty; i++)
	{
		unsigned char chanNo[50];
		float chanSample[50];
		unsigned char sampleByte[4];
		float* sample;
		chanNo[i] = _dataFrame[7 + 5 * i];
		//Highbyte forward!
		sampleByte[3] = _dataFrame[8 + 5 * i];
		sampleByte[2] = _dataFrame[9 + 5 * i];
		sampleByte[1] = _dataFrame[10 + 5 * i];
		sampleByte[0] = _dataFrame[11 + 5 * i];
		Byte_to_Float(sample, sampleByte);
		chanSample[i] = *sample;
		//write channelSample to corresponding channel in host
		_host->ChannelSamples[chanNo[i]] = chanSample[i];

		/*Serial.println(chanSample[i]);*/
	}
	//make a reply frame
	byte u8ModbusTransmitADU[256];
	byte u8ModbusTransmitADUSize = 0;
	u8ModbusTransmitADU[0] = 2;
	u8ModbusTransmitADU[1] = 16;
	u8ModbusTransmitADUSize = u8ModbusTransmitADUSize + 2;
	u8ModbusTransmitADU[2] = _dataFrame[2];         //WriteChannelAddress
	u8ModbusTransmitADU[3] = _dataFrame[3];
	u8ModbusTransmitADUSize = u8ModbusTransmitADUSize + 2;
	u8ModbusTransmitADU[4] = _dataFrame[4];        //WriteChannelQty
	u8ModbusTransmitADU[5] = _dataFrame[5];
	u8ModbusTransmitADUSize = u8ModbusTransmitADUSize + 2;
	sendResultAndCheck(u8ModbusTransmitADUSize, u8ModbusTransmitADU);
}

void ModbusXbeeComm::sendResultAndCheck(byte u8ModbusTransmitADUSize, byte * u8ModbusTransmitADU)
{
	// calculate CRC
	unsigned int transCRC = 0;
	for (byte i = 0; i < u8ModbusTransmitADUSize; i++)
	{
		transCRC += u8ModbusTransmitADU[i];
	}
	//fill  CRC into u8ModbusTransmitADU[]
	unsigned char crcByte[2];
	Uint_to_Byte(transCRC, crcByte);

	u8ModbusTransmitADU[u8ModbusTransmitADUSize++] = crcByte[1];//Highbyte forward!!
	u8ModbusTransmitADU[u8ModbusTransmitADUSize++] = crcByte[0];
	/*Serial.write(u8ModbusTransmitADUSize);*/
	/*for (size_t i = 0; i < u8ModbusTransmitADUSize; i++)
	{
	Serial.write(u8ModbusTransmitADU[i]);
	}*/
	// send TX packet including inquire result***************
	//create a TX request ,config parameters of the destination node
	//ZBTxRequest zbTx = ZBTxRequest(addr64, addr16, broadcastRadius, option,u8ModbusTransmitADU, u8ModbusTransmitADUSize, frameId);
	ZBTxRequest zbTx = ZBTxRequest(addr64, u8ModbusTransmitADU, u8ModbusTransmitADUSize);
	// create a TX response for loading the response of TX request------to get the status of TX messege
	ZBTxStatusResponse txStatus = ZBTxStatusResponse();

	//send the TX request message
	xbee.send(zbTx);

	for (size_t i = 0; i < u8ModbusTransmitADUSize; i++)
	{
		Serial1.write(u8ModbusTransmitADU[i]);
	}
	// flash TX indicator LED to show having already send the TX request message
	flashLed(tx_statusLed, 1, 100);

	// after sending a tx request, we expect a status response
	// wait up to half second for the status response
	if (xbee.readPacket(500))
	{
		// got a response!

		// should be a znet tx status            	
		if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE)
		{
			xbee.getResponse().getZBTxStatusResponse(txStatus);

			// get the delivery status, the fifth byte
			if (txStatus.getDeliveryStatus() == SUCCESS)
			{
				// success!!---  time to celebrate
				flashLed(tx_statusLed, 5, 50);
			}
			else
			{
				// failure£¡£¡£¡---the remote XBee did not receive our packet. is it powered on?
				flashLed(tx_errorLed, 3, 500);
			}
		}
	}
	else if (xbee.getResponse().isError())
	{
		Serial.print("Error reading packet.  Error code: ");
		Serial.println(xbee.getResponse().getErrorCode());
	}
	else
	{
		// local XBee did not provide a timely TX Status Response -- should not happen
		flashLed(tx_errorLed, 2, 50);
	}

	delay(1000);
}
