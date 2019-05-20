#pragma once

/*
  return the number of bytes to send for a packetised connection
*/
uint16_t mavlink_packetise(ByteBuffer &writebuf, uint16_t n);

