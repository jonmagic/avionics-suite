#include "canfix.h"

/*  CAN-FIX Interface library for the Arduino
 *  (License text omitted for brevity)
 */

void CFParameter::setMetaData(byte meta) {
  fcb &= 0x0F;
  fcb |= (meta << 4);
}

byte CFParameter::getMetaData(void) {
  return fcb >> 4;
}

void CFParameter::setFlags(byte flags) {
  fcb &= 0xF0;
  fcb |= (flags & 0x0F);
}

byte CFParameter::getFlags(void) {
  return fcb & 0x0F;
}

/* Constructor for the CanFix Class. */
CanFix::CanFix(byte id) {
  deviceid = id;
  nodeid   = id;
  model    = 0L;
  fw_version = 0;

  // Removed unused variable 'bitrate'
  // Initialize all callbacks to NULL
  __write_callback            = NULL;
  __report_callback           = NULL;
  __twoway_callback           = NULL;
  __config_callback           = NULL;
  __query_callback            = NULL;
  __param_callback            = NULL;
  __alarm_callback            = NULL;
  __stream_callback           = NULL;
  __bitrate_set_callback      = NULL;
  __node_set_callback         = NULL;
  __parameter_enable_callback = NULL;
  __parameter_query_callback  = NULL;
}

/* This is a wrapper for the write frame callback function. */
byte CanFix::writeFrame(CanFixFrame frame) {
  if (__write_callback != NULL) {
    __write_callback(frame);
    return 0;  // Return success
  }
  return 0;
}

// This function deals with enabling/disabling parameters.
void CanFix::parameterEnable(CanFixFrame frame) {
  int x;
  CanFixFrame rframe; // Response frame

  rframe.data[0] = frame.data[0];
  rframe.data[1] = frame.id - NSM_START;
  rframe.id      = NSM_START + nodeid;

  x = frame.data[2];
  x |= frame.data[3] << 8;
  rframe.length = 3;

  /* Check Range - send error if out of range */
  if (x < 256 || x > 1759) {
    rframe.data[2] = 0x01;
    writeFrame(rframe);
    return;
  }

  if (__parameter_enable_callback != NULL) {
    // NSM_DISABLE is 3, NSM_ENABLE is 4; subtracting yields 0 or 1
    rframe.data[2] = __parameter_enable_callback((word)x, frame.data[1] - NSM_DISABLE);
  } else {
    rframe.data[2] = 0x01; // Error if no callback defined
  }
  writeFrame(rframe);
}

void CanFix::handleNodeSpecific(CanFixFrame frame) {
  int x;
  byte length;
  CanFixFrame rframe; // Response frame

  // Prepare a generic response
  rframe.data[0] = frame.data[0];
  rframe.data[1] = frame.id - NSM_START;
  rframe.id = NSM_START + nodeid;

  switch(frame.data[0]) {
    case NSM_ID: // Node Identify Message
      if (frame.data[1] == nodeid || frame.data[0] == 0) {
        rframe.data[2] = 0x01;
        rframe.data[3] = deviceid;
        rframe.data[4] = fw_version;
        rframe.data[5] = (byte)model;
        rframe.data[6] = (byte)(model >> 8);
        rframe.data[7] = (byte)(model >> 16);
        rframe.length  = 8;
        break;
      } else {
        return;
      }

    case NSM_BITRATE:
      if (frame.data[1] == nodeid || frame.data[0] == 0) {
        if (frame.data[2] == 1) {
          x = 125;
        } else if (frame.data[2] == 2) {
          x = 250;
        } else if (frame.data[2] == 3) {
          x = 500;
        } else if (frame.data[2] == 4) {
          x = 1000;
        } else {
          rframe.data[2] = 0x01;
          rframe.length  = 3;
          writeFrame(rframe);
          return;
        }
        if (__bitrate_set_callback != NULL) {
          __bitrate_set_callback(x);
        }
      } else {
        return;
      }
      // Intentional fallthrough to NSM_NODE_SET if needed
      [[fallthrough]];

    case NSM_NODE_SET: // Node Set Message
      if (frame.data[1] == nodeid || frame.data[0] == 0) {
        if (frame.data[2] != 0x00) { // Ignore broadcast
          nodeid = frame.data[2];
          if (__node_set_callback != NULL) {
            __node_set_callback(nodeid);
          }
          rframe.id      = NSM_START + nodeid; // Update frame ID
          rframe.data[2] = 0x00;
        } else {
          rframe.data[2] = 0x01;
        }
        rframe.length = 3;
        break;
      } else {
        return;
      }

    case NSM_DISABLE:
      if (frame.data[1] == nodeid || frame.data[1] == 0) {
        parameterEnable(frame);
      }
      return;

    case NSM_ENABLE:
      if (frame.data[1] == nodeid) {
        parameterEnable(frame);
      }
      return;

    case NSM_REPORT:
      if (frame.data[1] == nodeid || frame.data[1] == 0) {
        if (__report_callback) __report_callback();
      }
      return;

    case NSM_FIRMWARE: // Not implemented yet
      if (frame.data[1] == nodeid) {
        // Firmware logic would go here
      }
      return;

    case NSM_TWOWAY:
      if (frame.data[1] == nodeid) {
        if (__twoway_callback && frame.data[0] != 0x00) {
          if (__twoway_callback(frame.data[2], *((word *)(&frame.data[3]))) == 0) {
            rframe.data[2] = 0x00;
            rframe.length  = 3;
            writeFrame(rframe);
            break;
          }
        }
      } else {
        return;
      }
      break;  // Ensure no accidental fall-through

    case NSM_CONFSET:
      if (frame.data[1] == nodeid) {
        if (__config_callback) {
          rframe.data[2] = __config_callback(*((word*)(&frame.data[2])), &frame.data[4]);
        } else {
          rframe.data[2] = 1;
        }
        rframe.length = 3;
        break;
      } else {
        return;
      }

    case NSM_CONFGET:
      if (frame.data[1] == nodeid) {
        if (__query_callback) {
          rframe.data[2] = __query_callback(*((word*)(&frame.data[2])), &rframe.data[3], &length);
        } else {
          rframe.data[2] = 1;
        }
        if (rframe.data[2] == 0) {
          rframe.length = 3 + length;
        } else {
          rframe.length = 3;
        }
        break;
      } else {
        return;
      }

    default:
      return;
  }
  writeFrame(rframe);
}

void CanFix::handleFrame(CanFixFrame frame) {
  if (frame.id == 0x00) {
    return;  // Ignore ID 0
  }
  else if (frame.id < 256) {
    // Node Alarms
    if (__alarm_callback) {
      __alarm_callback(frame.id, *((word *)(&frame.data[0])),
                       &frame.data[2], frame.length - 2);
    }
  }
  else if (frame.id < 0x6E0) {
    // Parameters
    if (__param_callback) {
      CFParameter par;
      par.type   = frame.id;
      par.node   = frame.data[0];
      par.index  = frame.data[1];
      par.fcb    = frame.data[2];
      par.length = frame.length - 3;
      for (byte n = 0; n < par.length; n++) {
        par.data[n] = frame.data[3 + n];
      }
      __param_callback(par);
    }
  }
  else if (frame.id < 0x7E0) {
    // Node Specific Message
    handleNodeSpecific(frame);
  }
  else {
    // Communication Channel – not implemented
  }
}

/* Main entry point for each incoming CAN frame. */
void CanFix::exec(CanFixFrame frame) {
  handleFrame(frame);
}

void CanFix::setNodeNumber(byte id) {
  nodeid = id;
}

void CanFix::setDeviceId(byte id) {
  deviceid = id;
}

void CanFix::setModel(unsigned long m) {
  model = m;
}

void CanFix::setFwVersion(byte v) {
  fw_version = v;
}

/* Sends a Node Status Information Message. */
void CanFix::sendStatus(word type, byte *data, byte length) {
  CanFixFrame frame;
  frame.id = NSM_START + nodeid;
  frame.data[0] = NSM_STATUS;
  frame.data[1] = (byte) type;
  frame.data[2] = (byte)(type >> 8);

  for (byte n = 0; n < length && n < 5; n++) {
    frame.data[3 + n] = data[n];
  }
  frame.length = length + 3;
  writeFrame(frame);
}

// Used to send the parameter given by p.
void CanFix::sendParam(CFParameter p) {
  if (__parameter_query_callback != NULL) {
    if (__parameter_query_callback(p.type) == 0x00) {
      return;
    }
  }

  CanFixFrame frame;
  frame.id = p.type;
  frame.data[0] = nodeid;
  frame.data[1] = p.index;
  frame.data[2] = p.fcb;
  frame.length  = p.length + 3;

  for (byte n = 0; (n < p.length) && (n < 5); n++) {
    frame.data[3 + n] = p.data[n];
  }
  writeFrame(frame);
}

void CanFix::sendAlarm(word type, byte *data, byte length) {
  // Not fully implemented
}

// Callback setters:
void CanFix::set_write_callback(void (*f)(CanFixFrame)) {
  __write_callback = f;
}

void CanFix::set_report_callback(void (*f)(void)) {
  __report_callback = f;
}

void CanFix::set_twoway_callback(byte (*f)(byte, word)) {
  __twoway_callback = f;
}

void CanFix::set_config_callback(byte (*f)(word, byte *)) {
  __config_callback = f;
}

void CanFix::set_query_callback(byte (*f)(word, byte *, byte *)) {
  __query_callback = f;
}

void CanFix::set_param_callback(void (*f)(CFParameter)) {
  __param_callback = f;
}

void CanFix::set_alarm_callback(void (*f)(byte, word, byte*, byte)) {
  __alarm_callback = f;
}

void CanFix::set_stream_callback(void (*f)(byte, byte *, byte)) {
  __stream_callback = f;
}

void CanFix::set_bitrate_callback(void (*f)(byte)) {
  __bitrate_set_callback = f;
}

void CanFix::set_node_callback(void (*f)(byte)) {
  __node_set_callback = f;
}

void CanFix::set_parameter_enable_callback(byte (*f)(word, byte)) {
  __parameter_enable_callback = f;
}

void CanFix::set_parameter_query_callback(byte (*f)(word)) {
  __parameter_query_callback = f;
}
