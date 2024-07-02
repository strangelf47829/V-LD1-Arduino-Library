#include <VLD1.h>
#include "Arduino.h"

//extern UART Serial2;

// The maximum amount of bytes allowed to be allocated for a single variable.
#define MAX_STACK_ALLOC 280

VLD1::VLD1(uint32_t baudrate)
{
	// Process the desired baudrate.
	this->baudrate = baudrate;
	switch(baudrate)
	{
		case 115200:
			init(0);
			break;
		case 460800:
			init(1);
			break;
		case 921600:
			init(2);
			break;
		case 2000000:
			init(3);
			break;
		default:
			setError(false, "Invalid Baudrate");
			// Error: Baudrate not supported.
			// Return here, since nothing else makes sense.
			return;
	}

	// set config defaults
	config.distanceRange = 0;
	config.thresholdOffset = 60;
	config.minimumRangeFilter = 5;
	config.maximumRangeFilter = 460;
	config.distanceAverageCount = 5;
	config.targetFilter = 1;
	config.distancePrecision = 0;
	config.txPower = 31;
	config.chirpIntegrationCount = 1;
	config.shortRangeDistanceFilter = 0;
	setStatus(false, "Configured");
}

bool VLD1::connect()
{
	// Set status -- connect
	setStatus(false, "Attempting to connect");

	// Check if baudrate is valid.
	if (baudrate != 115200 && baudrate != 460800 && baudrate != 921600 && baudrate != 2000000)
		return setError(true, "invalid Baud setting");

	// Baud rate valid, start serial.
	VLD1_RXTXUART.begin(baudrate, SERIAL_8E1);
	setStatus(false, "OK");

	// Init should have been called from constructor.
	// Check if init is loaded command
	if (!isTXCommand("INIT"))
		return setError(true, "INIT Command not loaded");

	setStatus(false, "Initialized comms");
	delay(3000);
	setStatus(false, "Initializing radar");

	// Clear out any data
	while(VLD1_RXUART.available())
		VLD1_RXUART.read();

	// Send init command
	if (sendPayload())
		return setError(true, "Payload error");

	setStatus(false, "Payload sent");

	// Await RESP response
	if (awaitReceivePayload(1000))
		return true;

	setStatus(false, "Received RESP");

	// Read response
	if (readRESP())
		return true;

	setStatus(false, "Processed RESP");

	// Await VERS response
	if (awaitReceivePayload(1000))
		return true;

	//Serial.println("Version received");
	setStatus(false, "VERS received");

	if (readVERS())
		return true;

	setStatus(false, "VERS read");

	// Connected successfully.
	return false;
}

bool VLD1::disconnect()
{
	// Send GBYE
	gbye();

	// Send GBYE
	if (sendPayload())
		return true;

	// Read RESP
	return awaitReceivePayload(1000);
}

uint16_t VLD1::Distance()
{
	return distance;
}

uint16_t VLD1::Magnitude()
{
	return magnitude;
}

bool VLD1::getFrame()
{
	// Write GNFD
	if(gnfd(0x04))
		return setError(true, "Error setting GNFD header");

	// Send command
	if (sendPayload())
		return setError(true, "Error sending GNFD frame");

	// await response and print.
	if (awaitReceivePayload(1000))
		return setError(true, "Error reading GNFD frame response: Expected RESP");

	if (readRESP())
		return setError(true, "Error reading RESP frame");

	if (awaitReceivePayload(1000))
		return setError(true, "Error reading GNFD frame response: Expected PDAT");

	// Read PDAT
	if (readPDAT())
		return setError(true, "Error reading PDAT frame");

	return false;
}

bool VLD1::setRange(uint16_t minimum, uint16_t maximum)
{
	// Write MIRA
	if (mira(minimum))
		return setError(true, "Invalid MIRA setting");

	// Send command
	if (sendPayload())
		return setError(true, "Error sending MIRA frame");

	// Await response
	if (awaitReceivePayload(1000))
		return setError(true, "Expected RESP");

	// If invalid RESP, return error
	if (readRESP())
		return setError(true, "MIRA response error");

	// Send MARA
	if (mara(maximum))
		return setError(true, "Invalid MARA setting");

	// Send command
	if (sendPayload())
		return setError(true, "Error sending MARA frame");

	// Await response
	if (awaitReceivePayload(1000))
		return setError(true, "Expected RESP");

	// Read resp
	if (readRESP())
		return setError(true, "MARA response error");

	// No error :)
	return false;
}

bool VLD1::isTXCommand(const char *str)
{
        bool a = (str[0] == txheader[0]);
	bool b = (str[1] == txheader[1]);
	bool c = (str[2] == txheader[2]);
	bool d = (str[3] == txheader[3]);

	return a && b && c && d;
}

bool VLD1::isRXCommand(const char *str)
{
	bool a = (str[0] == rxheader[0]);
	bool b = (str[1] == rxheader[1]);
	bool c = (str[2] == rxheader[2]);
	bool d = (str[3] == rxheader[3]);

	return a && b && c && d;
}

bool VLD1::lowPrecision()
{
	// Set low precision
	if (prec(0))
		return setError(true, "Low precision error");

	// Send payload
	if(sendPayload())
		return setError(true, "Payload error");

	return false;
}

bool VLD1::highPrecision()
{
	// Set high precision
	if (prec(1))
		return setError(true, "High precision error");

	// Send payload
	if (sendPayload())
		return setError(true, "Payload error");

	return false;
}

bool VLD1::init(uint8_t rate)
{
	// Check validity of rate
	if (rate > 3)
		return true;
	setTxHeader("INIT");
	setTxSize(1);
	txpayload[0] = rate;
	return false;
}

bool VLD1::gnfd(uint8_t bitfield)
{
	setTxHeader("GNFD");
	setTxSize(1);
	txpayload[0] = bitfield;
	return false;
}

bool VLD1::grps()
{
	setTxHeader("GRPS");
	setTxSize(0);
	return false;
}

bool VLD1::srps(VLD1Config const&cfg)
{
	setTxHeader("SRPS");
	setTxSize(43);

	// Set firmware version
	for (unsigned int i = 0; i < 19; i++)
		txpayload[i] = cfg.firmware[i];

	// Set other values
	txpayload[31] = cfg.distanceRange;
	txpayload[32] = cfg.thresholdOffset;
	txpayload[33] = cfg.minimumRangeFilter;
	txpayload[34] = cfg.minimumRangeFilter >> 8;
	txpayload[35] = cfg.maximumRangeFilter;
	txpayload[36] = cfg.maximumRangeFilter >> 8;
	txpayload[37] = cfg.distanceAverageCount;
	txpayload[38] = cfg.targetFilter;
	txpayload[39] = cfg.distancePrecision;
	txpayload[40] = cfg.txPower;
	txpayload[41] = cfg.chirpIntegrationCount;
	txpayload[42] = cfg.shortRangeDistanceFilter;

	return false;
}

bool VLD1::rfse()
{
	setTxHeader("RFSE");
	setTxSize(0);
	return false;
}

bool VLD1::gbye()
{
	setTxHeader("GBYE");
	setTxSize(0);
	return false;
}

bool VLD1::rrai(uint8_t range)
{
	// check validity
	if (range > 1)
		return true;
	setTxHeader("RRAI");
	setTxSize(1);
	txpayload[0] = range;
	config.distanceRange = range;
	return false;
}

bool VLD1::thof(uint8_t offset)
{
	// check validity
	if (offset < 20 || offset > 90)
		return true;
	setTxHeader("THOF");
	setTxSize(1);
	txpayload[0] = offset;
	config.thresholdOffset = offset;
	return false;
}

bool VLD1::mira(uint16_t bin)
{
	// Check validity
	if (bin < 1 || bin > 510)
		return true;
	setTxHeader("MIRA");
	setTxSize(2);
	txpayload[0] = bin;
	txpayload[1] = bin >> 8;
	config.minimumRangeFilter = bin;
	return false;
}

bool VLD1::mara(uint16_t bin)
{
	// check validity
	if (bin < 2 || bin > 511)
		return true;
	setTxHeader("MARA");
	setTxSize(2);
	txpayload[0] = bin;
	txpayload[1] = bin >> 8;
	config.maximumRangeFilter = bin;
	return false;
}

bool VLD1::ravg(uint8_t avg)
{
	// Check validity
	if (avg == 0)
		return true;
	setTxHeader("RAVG");
	setTxSize(1);
	txpayload[0] = avg;
	config.distanceAverageCount = avg;
	return false;
}

bool VLD1::tgfi(uint8_t filter)
{
	// Check validity
	if (filter > 2)
		return true;
	setTxHeader("TGFI");
	setTxSize(1);
	txpayload[0] = filter;
	config.targetFilter = filter;
	return false;
}

bool VLD1::prec(uint8_t precision)
{
	// Check validity
	if (precision > 1)
		return true;
	setTxHeader("PREC");
	setTxSize(1);
	txpayload[0] = precision;
	config.distancePrecision = precision;
	return false;
}

bool VLD1::txpw(uint8_t pw)
{
	// Check validity
	if (pw > 31)
		return true;
	setTxHeader("TXPW");
	setTxSize(1);
	txpayload[0] = pw;
	config.txPower = pw;
	return false;
}

bool VLD1::intn(uint8_t count)
{
	// Check validity
	if (count == 0 || count > 100)
		return true;
	setTxHeader("INTN");
	setTxSize(1);
	txpayload[0] = count;
	config.chirpIntegrationCount = count;
	return false;
}

bool VLD1::srdf(uint8_t filter)
{
	// Check validity
	if (filter > 1)
		return true;
	setTxHeader("SRDF");
	setTxSize(1);
	txpayload[0] = filter;
	config.shortRangeDistanceFilter = filter;
	return false;
}


uint8_t VLD1::readRESP()
{
	//Serial.println("Validating RESP");
	// Check if payload is actually RESP
	if (!isRXCommand("RESP"))
		return 8;
	//com4.println("Validating size");
	// Check if data is available
	if (rxPayloadSize != 1)
		return 9;
	//Serial.println("Validating data");
	//Serial.println(rxpayload[0]);

	// Return resp
	return rxpayload[0];
}

bool VLD1::readVERS()
{
	// Check if payload is actually VERS
	if (!isRXCommand("VERS"))
		return true;
	// Check payload size
	if (rxPayloadSize != 19)
		return true;
	// Copy data
	for (unsigned int i = 0; i < 19; i++)
		config.firmware[i] = rxpayload[i];
	return false;
}

bool VLD1::readRADC()
{
	// TODO: implement
	return true;
}

bool VLD1::readRFFT()
{
	// TODO: implement
	return true;
}

bool VLD1::readPDAT()
{
	// Check if payload is actually PDAT
	if (!isRXCommand("PDAT"))
		return true;

	// Check if something is available
	if (rxPayloadSize == 0)
	{
		// Read was successful, however nothing found.
		magnitude = 0;
		distance = 0;
		return false;
	}

	// Check if payload size is invalid
	if (rxPayloadSize != 6)
		return true;

	// Declare 32 bits
	uint32_t rawData = 0;

	// Convert 4 bytes into float.
	for (int idx = 0; idx < 4; idx++)
	{
		// Promote 8 bit into 32 bit
		uint32_t promoted = rxpayload[idx];

		// Bit shift
		promoted = promoted << (idx * 8);

		// Then add result to rawData
		rawData |= promoted;
	}

	//Serial.print("Raw: ");
	//Serial.println(rawData, HEX);

	// Create pointer to float
	float *distPtr = (float *)(&rawData);

	// Then do some multiplications
	uint32_t mult = (uint32_t)(*distPtr * 1000.0);

	// Set distance
	distance = mult;

	// read magnitude
	magnitude = (rxpayload[5] << 8) | (rxpayload[4]);

	return false;
}

uint32_t VLD1::readDONE()
{
	// Check if payload is actually DONE
	if (!isRXCommand("DONE"))
		return true;
	// Check if payload length is valid
	if (rxPayloadSize != 4)
		return true;
	// Read frame
	uint32_t frame = (rxpayload[3] << 24) | (rxpayload[2] << 16) | (rxpayload[1] << 8) | (rxpayload[0]);

	return frame;
}

bool VLD1::readRPST()
{
	// TODO: implement
	return true;
}




void VLD1::setTxHeader(const char *str)
{
	txheader[0] = str[0];
	txheader[1] = str[1];
	txheader[2] = str[2];
	txheader[3] = str[3];
	txheader[4] = 0;
}

void VLD1::setRxHeader(const char *str)
{
	rxheader[0] = str[0];
	rxheader[1] = str[1];
	rxheader[2] = str[2];
	rxheader[3] = str[3];
	rxheader[4] = 0;
}

void VLD1::setTxSize(uint32_t size)
{
	txPayloadSize = size;
}

void VLD1::setRxSize(uint32_t size)
{
	rxPayloadSize = size;
}

bool VLD1::sendPayload()
{
	// Check if size is allowed
	if (txPayloadSize > 2048)
		return true;

	// Stack allocated!
	//static_assert(sizeof(txheader) + sizeof(txpayload) <= MAX_STACK_ALLOC, "Size of payload buffer exceeds maximum allowed stack allocation size.");
	uint8_t data[sizeof(txheader) + sizeof(txpayload)];

	// Copy header data into data
	for (unsigned int i = 0; i < 4; i++)
		data[i] = txheader[i];

	// Copy size data
	for (unsigned int i = 0; i < 4; i++)
		data[i + 4] = (txPayloadSize >> i * 8);

	// Copy payload data
	for (unsigned int i = 0; i < txPayloadSize; i++)
		data[i + 8] = txpayload[i];

	// Send away!
	VLD1_TXUART.write(data, 8 + txPayloadSize);
	return false;
}

bool VLD1::receivePayload()
{
	// Check if bytes are available
	if (VLD1_RXUART.available() >= 8)
	{
		// Get first byte
		char firstByte = VLD1_RXUART.read();
		//Serial.print("first Byte: ");
		//Serial.println(firstByte, HEX);

		// Ugly hack. TODO: Find source of this hack.
		if (firstByte >= 0x7F)
			rxheader[0] = VLD1_RXUART.read();
		else
			rxheader[0] = firstByte;

		// Read remaining 3 bytes
		rxheader[1] = VLD1_RXUART.read();
		rxheader[2] = VLD1_RXUART.read();
		rxheader[3] = VLD1_RXUART.read();

		// Read next four bytes
		rxPayloadSize = 0;
		rxPayloadSize |= (VLD1_RXUART.read() << 0);
		rxPayloadSize |= (VLD1_RXUART.read() << 8);
		rxPayloadSize |= (VLD1_RXUART.read() << 16);
		rxPayloadSize |= (VLD1_RXUART.read() << 24);

		//Serial.print("Received command: ");
		//Serial.println((const char *)rxheader);

		//Serial.print("Size: ");
		//Serial.println(rxPayloadSize);

		// If payload is too large, return true since error
//		if (rxPayloadSize >= 7)
//			return true;

		// wait until bytes are available
		while (VLD1_RXUART.available() < rxPayloadSize);

		// Bytes avaialable, read them
		for (unsigned int i = 0; i < rxPayloadSize; i++)
			rxpayload[i] = VLD1_RXUART.read();

		// Return false
		return false;
	}
	// No bytes available, return true.
	return true;
}

bool VLD1::awaitReceivePayload(uint16_t timeout)
{
	// Wait until now + timeout
	long int until = millis() + timeout;

	// While stuff is available
	//while(VLD1_RXUART.available())
	//	VLD1_RXUART.read();

	// Wait until bytes available, or until timeout has passed
	while (VLD1_RXUART.available() < 8 && millis() < until)
	{
		//Serial.print(VLD1_RXUART.available());
		//Serial.print(", ");
	}

	//Serial.println();
	//Serial.print(VLD1_RXUART.available());
	//Serial.println(" Bytes available");

	// Return result
	return receivePayload();
}


bool VLD1::setError(bool r, const char *err)
{
	//Serial.print("ERROR: ");
	//Serial.println(err);
	for (unsigned int i = 0; i < 32; i++)
	{
		char c = err[i];
		errormsg[i] = c;
		if (c == 0)
			break;
	}

	return r;
}

bool VLD1::setStatus(bool r, const char *status)
{
	//Serial.print("STATUS: ");
	//Serial.println(status);
	for (unsigned int i = 0; i < 32; i++)
	{
		char c = status[i];
		statusmsg[i] = c;
		if (c == 0)
			break;
	}

	return r;
}

const char *VLD1::getError(){
	return (const char *)errormsg;
}

const char *VLD1::getStatus()
{
	return (const char *)statusmsg;
}







