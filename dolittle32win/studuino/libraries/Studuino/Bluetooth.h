#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

// States while receiving data via Bluetooth.
// Data block contains 13 bytes.
// [HEADER][HEADER][Data0][Data1]...[Data09][FOOTER]
enum ReceiveStatus {
  RecvHeader = 0x0,   // Header
  RecvData,           // Data
  RecvFooter          // Footer
};

// Protocol
#define HEADER		(0xed)		// Data header
#define FOOTER		(0xde)		// Data footer
#define HEADER_RECV_TIME	(2)	// Num of header

#define BT_ID_01  0x00
#define BT_ID_02  0x01
#define BT_ID_03  0x02
#define BT_ID_04  0x03
#define BT_ID_05  0x04
#define BT_ID_06  0x05
#define BT_ID_07  0x06
#define BT_ID_08  0x07
#define BT_ID_09  0x08
#define BT_ID_10  0x09

#define BT_ID_ACCEL 0x10

// Data structure for storing values.
typedef struct {
  unsigned int flagButton;
  unsigned int flagOther;
  //unsigned int accX, accY, accZ;
  int8_t accX, accY, accZ;
}bt_param_t;

#endif // _BLUETOOTH_H_
