// Interprets the next four bytes from off onwards as a network byte order
// 32 bit unsigned integer
function readInt16(bytes, off) {
  return bytes[off] + (bytes[off + 1] << 8);
}

// Decodes a variable byte encoded integer at pos off
// @returns [int, newOff] The value and the offset of the next byte after the
// integer.
function readIntVbe(bytes, off, discarded_bits) {
  var i = 0;
  var b = 0;
  while(true) {
    i += (bytes[off] & 127) << b;
    if (bytes[off] > 127 || off >= bytes.length) {
      off += 1;
      break;
    }
    b += 7;
    off += 1;
  }
  i = i << discarded_bits;
  return [i, off];
}

function Decoder(bytes, port) {
  // CONFIG
  // Please ensure these variables match those used for the microcontroller.
  var NUM_MEASUREMENTS = 10;
  var DISCARDED_BITS = 2;
  var LORA_DATA_PORT = 1;

  var decoded = {};

  if (port === LORA_DATA_PORT) {
    /**
     * The message has the following format:
     *  - 2 bytes timestamp in s
     *  - n bytes of NUM_MEASUREMENTS measurements encoded using a variable
     *    bytes encoding. 
     */
    decoded.timestamp = readInt16(bytes, 0); 
    decoded.measurements = [];
    var off = 4;
    for (var i = 0; i < NUM_MEASUREMENTS; i++) {
      l = readIntVbe(bytes, off, DISCARDED_BITS);
      decoded.measurements.push(l[0]);
      off = l[1];
    }
  }

  return decoded;
}
