#include <arduino.h>

#ifndef VLD1_H
#define VLD1_H

// Set this macro to '0' if you want the RX and TX uart to be seperate
#ifndef VLD1_UARTMODE
#define VLD1_UARTMODE 1
#endif // VLD1_UARTMODE

// If VLD1_UARTMODE is set to '1' (the default), both VLD1_RXUART and VLD1_TXUART will be the same.
// The value that VLD1_RXUART and VLD1_TXUART will be set to is VLD1_RXTXUART.
// If this macro is not defined, it will equal 'Serial' by default.

#ifndef VLD1_RXTXUART
#define VLD1_RXTXUART Serial2
#endif // VLD1_RXTXUART

#if VLD1_UARTMODE == 1
#define VLD1_RXUART VLD1_RXTXUART
#define VLD1_TXUART VLD1_RXTXUART
#elif VLD1_UARTMODE == 0
#ifndef VLD1_RXUART
#error "Please define VLD1_RXUART"
#endif // VLD1_RXUART
#ifndef VLD1_TXUART
#error "Please define VLD1_TXUART"
#endif // VLD1_TXUART
#endif

struct VLD1Config
{
	// Zero terminated strings.
	char firmware[19]; //Format: V-LD1_APP-RFB-YYXX
	char uniqueID[12]; //Format: L1234n12345

	// Distance range setting. "0" = "20m", "1" = "50m".
	uint8_t distanceRange;

	// Threshold setting [db]. Must be between "20" and "90".
	uint8_t thresholdOffset;

	// Minimum range filter [bin]. Must be between "1" and "510".
	uint16_t minimumRangeFilter;

	// Maximum range filter [bin]. Must be between "2" and "511".
	uint16_t maximumRangeFilter;

	// The amount of times a measurement is taken. Must be between "1" and "255".
	uint8_t distanceAverageCount;

	// Sets the filtering mode for the targets. "0" = "Strongest", "1" = "Nearest", "2" = "Furthest".
	uint8_t targetFilter;

	// Precision setting. "0" = "Low precision", "1" = "High precision".
	uint8_t distancePrecision;

	// The strength to use when emitting. Must be between "0" and "31".
	uint8_t txPower;

	// Chirp integration count. TODO: Fill in docs
	uint8_t chirpIntegrationCount;

	// Enables or disables the short range filter. "0" = "off", "1" = "on"
	uint8_t shortRangeDistanceFilter;

};

class VLD1
{
	public:

		// Constructors
		VLD1(uint32_t);
		VLD1() : VLD1(115200)
	{}

		// Attempt to connect to the radar. "0" = "success", "1" otherwise.
		bool connect();
		bool disconnect();

		// Read the distance.
		bool getFrame();

		// Set range of radar (in bins)
		bool setRange(uint16_t, uint16_t);

		// Get values
		uint16_t Distance();
		uint16_t Magnitude();

		// Get strings
		const char *getError();
		const char *getStatus();

		// Set precision modes
		bool lowPrecision();
		bool highPrecision();

	private:

		// Status strings
		char errormsg[64];
		char statusmsg[64];


		bool setError(bool, const char *);
		bool setStatus(bool, const char *);

		// Current radar configuration
		VLD1Config config;
		uint32_t baudrate; // Stored as actual baud rate (115200, not 0).

		bool isRXCommand(const char *);
		bool isTXCommand(const char *);

		// These commands are used to actually send data to the radar
		bool init(uint8_t baudrate);
		bool gnfd(uint8_t bitfield);
		bool grps();
		bool srps(VLD1Config const&);
		bool rfse();
		bool gbye();
		bool rrai(uint8_t);
		bool thof(uint8_t);
		bool mira(uint16_t);
		bool mara(uint16_t);
		bool ravg(uint8_t);
		bool tgfi(uint8_t);
		bool prec(uint8_t);
		bool txpw(uint8_t);
		bool intn(uint8_t);
		bool srdf(uint8_t);
		bool jbtl();

		// These commands are used to read data from the radar
		uint8_t readRESP();
		bool readVERS();
		bool readRADC();
		bool readRFFT();
		bool readPDAT();
		uint32_t readDONE();
		bool readRPST();

		// Registers to store incoming data
		uint16_t magnitude;
		uint16_t distance;

		// This array stores header data
		uint8_t txheader[5];
		uint8_t rxheader[5];

		// These values store payload data
		uint32_t txPayloadSize;
		uint32_t rxPayloadSize;

		// These arrays store the data of the payload
		uint8_t rxpayload[2048];
		uint8_t txpayload[2048];

		// Helper functions to ease command setting/receiving
		void setTxHeader(const char *);
		void setRxHeader(const char *);

		void setTxSize(uint32_t);
		void setRxSize(uint32_t);

		// These two functions are used to send and receive payload data.
		// These are private, because these functions shouldn't be called directly.
		// Returns false on success, true otherwise.
		bool sendPayload();
		bool receivePayload();

		// Waits until response received, or timeout.
		bool awaitReceivePayload(uint16_t timeout);
};







#endif // VLD1_H
