#include <SoftwareSerial.h>
#include "malink.h"

#define FREQ_SEND_TOGGLE 2
#define FREQ_TOGGLE_LED 10

malink_status_t stat;
uint32_t time_now = 0;
uint32_t last_send_toggle = 0;
uint32_t last_toggle_led = 0;
uint8_t toggle_tx = 0;
uint8_t toggle_rx = 0;
SoftwareSerial MalinkSerial(11, 12); // RX,TX


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  MalinkSerial.begin(9600);
  pinMode(3, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  malink_status_init(&stat);
}

void loop() {
  // put your main code here, to run repeatedly:
  malink_message_t msg;
  uint8_t rxbuf[MALINK_MAX_PACKET_LEN];
  uint8_t p = 0;
  while (MalinkSerial.available() > 0 || stat.parse_state != MALINK_PARSE_STATE_IDLE) {
    uint8_t c = MalinkSerial.read();
    p = malink_parse_char(c, rxbuf, &msg, &stat);
    Serial.println(p);
    if (p) {
      switch (msg.msgid) {
        case MALINK_MSG_ID_TOGGLE:
          malink_msg_toggle_decode(&msg, &toggle_rx);
          Serial.println("----------------");
          Serial.print("TOGGLE_RX = ");
          Serial.println(toggle_rx);
          Serial.print("MSG_RCV = ");
          Serial.println(stat.msg_received);
          Serial.print("CUR_RX_SEQ = ");
          Serial.println(stat.current_rx_seq);
          break;
        default:
          Serial.println("----------------");
          Serial.println("Gagal parse");
          break;
      }
    }
  }

  time_now = millis();
  if (time_now - last_toggle_led >= 1000 / FREQ_TOGGLE_LED) {
    digitalWrite(LED_BUILTIN, toggle_rx);
    last_toggle_led = time_now;
  }

  ///*
  time_now = millis();
  if (time_now - last_send_toggle >= 1000 / FREQ_SEND_TOGGLE) {
    toggle_tx = !toggle_tx;
    Serial.println("----------------");
    Serial.print("TOGGLE_TX = ");
    Serial.println(toggle_tx);
    Serial.print("CUR_TX_SEQ = ");
    Serial.println(stat.current_tx_seq);

    malink_message_t msg;
    uint8_t buf[MALINK_MAX_PACKET_LEN];
    uint8_t len;
    malink_msg_toggle_pack(&msg, &stat, toggle_tx);
    len = malink_msg_to_send_buffer(buf, &msg);

    MalinkSerial.write(buf, len);

    last_send_toggle = time_now;
  }
  //*/

}
