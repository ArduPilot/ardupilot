/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  GCS Protocol
 *
 *  4	Ardupilot Header
 *  D
 *  5	Payload length
 *  1	Message ID
 *  1	Message Version
 *  9	Payload byte 1
 *  8	Payload byte 2
 *  7	Payload byte 3
 *  A	Checksum byte 1
 *  B	Checksum byte 2
 *
 */

/*
 * #if GCS_PORT == 3
 # define SendSerw		Serial3.write
 # define SendSer		Serial3.print
 ##else
 # define SendSerw		Serial.write
 # define SendSer		Serial.print
 ##endif
 #
 #  byte mess_buffer[60];
 #  byte buff_pointer;
 #
 #  // Unions for getting byte values
 #  union f_out{
 #       byte bytes[4];
 #       float value;
 #  } floatOut;
 #
 #  union i_out {
 #       byte bytes[2];
 #       int16_t value;
 #  } intOut;
 #
 #  union l_out{
 #       byte bytes[4];
 #       int32_t value;
 #  } longOut;
 #
 #  // Add binary values to the buffer
 #  void write_byte(byte val)
 #  {
 #       mess_buffer[buff_pointer++] = val;
 #  }
 #
 #  void write_int(int16_t val )
 #  {
 #       intOut.value = val;
 #       mess_buffer[buff_pointer++] = intOut.bytes[0];
 #       mess_buffer[buff_pointer++] = intOut.bytes[1];
 #  }
 #
 #  void write_float(float val)
 #  {
 #       floatOut.value = val;
 #       mess_buffer[buff_pointer++] = floatOut.bytes[0];
 #       mess_buffer[buff_pointer++] = floatOut.bytes[1];
 #       mess_buffer[buff_pointer++] = floatOut.bytes[2];
 #       mess_buffer[buff_pointer++] = floatOut.bytes[3];
 #  }
 #
 #  void write_long(int32_t val)
 #  {
 #       longOut.value = val;
 #       mess_buffer[buff_pointer++] = longOut.bytes[0];
 #       mess_buffer[buff_pointer++] = longOut.bytes[1];
 #       mess_buffer[buff_pointer++] = longOut.bytes[2];
 #       mess_buffer[buff_pointer++] = longOut.bytes[3];
 #  }
 #
 #  void flush(byte id)
 #  {
 #       byte mess_ck_a = 0;
 #       byte mess_ck_b = 0;
 #       byte i;
 #
 #       SendSer("4D");             // This is the message preamble
 #       SendSerw(buff_pointer);    // Length
 #       SendSerw(2);               // id
 #
 #       for (i = 0; i < buff_pointer; i++) {
 #               SendSerw(mess_buffer[i]);
 #       }
 #
 #       buff_pointer = 0;
 #  }
 */