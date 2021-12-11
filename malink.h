#ifndef _malink_h_
#define _malink_h_

#include <stdint.h>

// MALink
// ramhdi 13/02/2020

// ----------------------------------------------------------------------------
// Definitions
#define MALINK_MAX_PAYLOAD_LEN 4
#define MALINK_NON_CHECKSUM_LEN 4
#define MALINK_CHECKSUM_LEN 1
#define MALINK_NON_PAYLOAD_LEN (MALINK_NON_CHECKSUM_LEN + MALINK_CHECKSUM_LEN)
#define MALINK_MAX_PACKET_LEN (MALINK_MAX_PAYLOAD_LEN + MALINK_NON_PAYLOAD_LEN)
#define MALINK_STX 254
#define MALINK_CRC_POLY 0x91

// ----------------------------------------------------------------------------
// Data Types
typedef enum __malink_parse_state {
  MALINK_PARSE_STATE_IDLE,
  MALINK_PARSE_STATE_GOT_STX,
  MALINK_PARSE_STATE_GOT_LEN,
  MALINK_PARSE_STATE_GOT_SEQ,
  MALINK_PARSE_STATE_GOT_MSGID,
  MALINK_PARSE_STATE_GOT_PAYLOAD,
} malink_parse_state_t;

typedef enum __malink_msg_id {
  MALINK_MSG_ID_UNKNOWN,
  MALINK_MSG_ID_AREAD,
  MALINK_MSG_ID_TOGGLE
} malink_message_id_t;

typedef struct __malink_message {
  uint8_t stx; // start of packet
  uint8_t len; // packet length
  uint8_t seq; // packet sequence
  uint8_t msgid; // message identifier
  uint8_t payload[MALINK_MAX_PAYLOAD_LEN]; // payload data
  uint8_t checksum; // checksum
} malink_message_t;

typedef struct __malink_status {
  //uint8_t buffer_overrun;             ///< Number of buffer overruns
  //uint8_t parse_error;                ///< Number of parse errors
  //uint16_t packet_rx_success_count;   ///< Received packets
  //uint16_t packet_rx_drop_count;      ///< Number of packet drops
  uint8_t msg_received;               ///< Number of received messages
  uint8_t current_rx_seq;             ///< Sequence number of last packet received
  uint8_t current_tx_seq;             ///< Sequence number of last packet sent
  uint8_t packet_idx;                 ///< Index in current packet
  malink_parse_state_t parse_state;  ///< Parsing state machine
} malink_status_t;

// ----------------------------------------------------------------------------
// Cyclic Redundancy Check
uint8_t crc_calculate(malink_message_t msg, uint8_t length)
{
  uint8_t msgarr[MALINK_MAX_PACKET_LEN];
  msgarr[0] = msg.stx;
  msgarr[1] = msg.len;
  msgarr[2] = msg.seq;
  msgarr[3] = msg.msgid;
  for (int i = 0; i < MALINK_MAX_PAYLOAD_LEN; i++) {
    msgarr[MALINK_NON_CHECKSUM_LEN + i] = msg.payload[i];
  }
  uint8_t i, j, crc = 0;

  for (i = 0; i < length; i++) {
    crc ^= msgarr[i];
    for (j = 0; j < 8; j++) {
      if (crc & 1) crc ^= MALINK_CRC_POLY;
      crc >>= 1;
    }
  }
  return crc;
}

// ----------------------------------------------------------------------------
// MALink status initialization
void malink_status_init (malink_status_t* status) {
  status->msg_received = 0;
  status->current_rx_seq = 0;
  status->current_tx_seq = 0;
  status->packet_idx = 0;
  status->parse_state = MALINK_PARSE_STATE_IDLE;
}

// ----------------------------------------------------------------------------
// MALink aread message pack
// aread has size of 2-bytes
// the data is splited into two separate bytes
// the first 8-bits from LSB is put into the first payload
// the last 8-bits from LSB is put into the second payload
void malink_msg_aread_pack (malink_message_t* msg, malink_status_t* status, uint16_t aread) {
  msg->stx = MALINK_STX;
  msg->len = 2;
  msg->seq = status->current_tx_seq; (status->current_tx_seq)++;
  msg->msgid = MALINK_MSG_ID_AREAD;
  msg->payload[0] = aread;
  msg->payload[1] = aread >> 8;
  msg->checksum = crc_calculate(*msg, MALINK_NON_CHECKSUM_LEN + msg->len);
}

// ----------------------------------------------------------------------------
// MALink aread message decode
// the second payload is shifted 8 bits to left
// and then added with the first payload
void malink_msg_aread_decode (malink_message_t* msg, uint16_t* aread) {
  *aread = (msg->payload[1] << 8) + msg->payload[0];
}

// ----------------------------------------------------------------------------
// MALink toggle message pack
// toggle has size of 1-byte
// the data is put directly into the first payload
void malink_msg_toggle_pack (malink_message_t* msg, malink_status_t* status, uint8_t toggle) {
  msg->stx = MALINK_STX;
  msg->len = 1;
  msg->seq = status->current_tx_seq; (status->current_tx_seq)++;
  msg->msgid = MALINK_MSG_ID_TOGGLE;
  msg->payload[0] = toggle;
  msg->checksum = crc_calculate(*msg, MALINK_NON_CHECKSUM_LEN + msg->len);
}

// ----------------------------------------------------------------------------
// MALink toggle message decode
// the data is taken directly from the first payload
void malink_msg_toggle_decode (malink_message_t* msg, uint8_t* toggle) {
  *toggle = msg->payload[0];
}

// ----------------------------------------------------------------------------
// Send message to a buffer (array of bytes)
// Returns the length of the buffer slot occupied by the message
uint8_t malink_msg_to_send_buffer (uint8_t* buf, malink_message_t* msg) {
  buf[0] = msg->stx;
  buf[1] = msg->len;
  buf[2] = msg->seq;
  buf[3] = msg->msgid;
  for (int i = 0; i < MALINK_MAX_PAYLOAD_LEN; i++) {
    buf[MALINK_NON_CHECKSUM_LEN + i] = msg->payload[i];
  }
  buf[MALINK_NON_CHECKSUM_LEN + msg->len] = msg->checksum;
  return (MALINK_NON_PAYLOAD_LEN + msg->len);
}

// ----------------------------------------------------------------------------
// Checks whether a packet with a certain message-id
// carries the correct payload length
// Returns true or false
uint8_t malink_check_packet_length(uint8_t len, malink_message_id_t msgid) {
  uint8_t res = 0;
  switch (msgid) {
    case MALINK_MSG_ID_AREAD:
      res = (len == 2 ? 1 : 0);
      break;
    case MALINK_MSG_ID_TOGGLE:
      res = (len == 1 ? 1 : 0);
      break;
    default:
      break;
  }
  return res;
}

// ----------------------------------------------------------------------------
// Parses a MALink message from a buffer
// Checks the integrity of the packet using CRC
uint8_t malink_parse_char (uint8_t c, uint8_t* rxbuf, malink_message_t* msg, malink_status_t* status) {
  uint8_t res = 0;
  switch (status->parse_state) {
    case MALINK_PARSE_STATE_IDLE:
      //msg->msgid = MALINK_MSG_ID_UNKNOWN;
      // ambil stx
      if (c == MALINK_STX) {
        status->packet_idx = 0;
        rxbuf[status->packet_idx] = c;
        status->parse_state = MALINK_PARSE_STATE_GOT_STX;
      }
      break;

    case MALINK_PARSE_STATE_GOT_STX:
      // ambil len
      status->packet_idx = 1;
      rxbuf[status->packet_idx] = c;
      status->parse_state = MALINK_PARSE_STATE_GOT_LEN;
      break;

    case MALINK_PARSE_STATE_GOT_LEN:
      // ambil seq;
      status->packet_idx = 2;
      rxbuf[status->packet_idx] = c;
      status->parse_state = MALINK_PARSE_STATE_GOT_SEQ;
      break;

    case MALINK_PARSE_STATE_GOT_SEQ:
      // ambil msgid
      if (malink_check_packet_length(rxbuf[1], c)) {
        status->packet_idx = 3;
        rxbuf[status->packet_idx] = c;
        status->parse_state = MALINK_PARSE_STATE_GOT_MSGID;
      } else {
        status->parse_state = MALINK_PARSE_STATE_IDLE;
      }
      break;

    case MALINK_PARSE_STATE_GOT_MSGID:
      // ambil payload
      if (status->packet_idx == rxbuf[1] + MALINK_NON_CHECKSUM_LEN - 1) {
        (status->packet_idx)++;
        rxbuf[status->packet_idx] = c;
        status->parse_state = MALINK_PARSE_STATE_GOT_PAYLOAD;
      } else {
        (status->packet_idx)++;
        rxbuf[status->packet_idx] = c;
      }
      break;

    case MALINK_PARSE_STATE_GOT_PAYLOAD:
      msg->stx = rxbuf[0];
      msg->len = rxbuf[1];
      msg->seq = rxbuf[2];
      msg->msgid = rxbuf[3];
      for (int i = 0; i < MALINK_MAX_PAYLOAD_LEN; i++) {
        msg->payload[i] = rxbuf[MALINK_NON_CHECKSUM_LEN + i];
      }
      msg->checksum = rxbuf[MALINK_NON_CHECKSUM_LEN + msg->len];
      // checksum
      uint8_t rxcrc = rxbuf[MALINK_NON_CHECKSUM_LEN + msg->len];
      uint8_t msgcrc = crc_calculate(*msg, MALINK_NON_CHECKSUM_LEN + msg->len);
      if (rxcrc == msgcrc) {
        res = 1;
        status->current_rx_seq = msg->seq;
      }
      (status->msg_received)++;
      status->packet_idx = 0;
      status->parse_state = MALINK_PARSE_STATE_IDLE;
      break;

    default:
      break;
  }
  return res;
}

#endif
