#include <canfix.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <Wire.h>

#define CAN0_INT 2
#define MESSAGE_SEND_INTERVAL 200
MCP_CAN can0(10);

CanFix cf(0x80);

unsigned long now;
unsigned long lastParameterTimer[5];

void setup() {
  Serial.begin(115200);

  cf.setDeviceId(0x80);
  cf.setModel(0x12346);
  cf.setFwVersion(3);
  cf.set_write_callback(can_write_callback);

  delay(250);
  Wire.begin();

  while(can0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("Unable to begin can0");
    delay(1000);
  }

  can0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);

  now = millis();
}

void loop() {
  unsigned long rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8];

  now = millis();

  // Check to see if the can0 interrupt pin is low (meaning there's a message waiting)
  if (!digitalRead(CAN0_INT)) {
    // Read the buffered message and assign it to a couple of variables
    can0.readMsgBuf(&rxId, &len, rxBuf);

    // Verify that the message is not an extended type
    if ((rxId & 0x80000000) != 0x80000000) {
      // Turn the message into a CanFixFrame
      CanFixFrame frame;
      frame.id = rxId;
      frame.length = len;
      memcpy(frame.data, rxBuf, len);  // Use the actual length returned
      cf.exec(frame);

      send_canfix_frame_to_serial(frame);
    }
  }
}

/*********************************************************************
  This function converts CanFix messages to Serial and sends them over
  to the Android device connected on the USB port.
**********************************************************************/
void send_canfix_frame_to_serial(CanFixFrame frame) {
  byte message[] = {
    (byte)(frame.id & 0xFF),
    (byte)((frame.id >> 8) & 0xFF),
    (byte)((frame.id >> 16) & 0xFF),
    frame.data[0],
    frame.data[1],
    frame.data[2],
    frame.data[3],
    frame.data[4],
    frame.data[5],
    frame.data[6],
    frame.data[7]
  };

  Serial.write(message, 11);
}

/*********************************************************************
  This function is a callback function that is used by the CanFix object
  for sending CAN frames on the network.
**********************************************************************/
void can_write_callback(CanFixFrame frame) {
  can0.sendMsgBuf(frame.id, 0, frame.length, frame.data);
}
