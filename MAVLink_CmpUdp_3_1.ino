#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <MAVLink.h>
#include <DFRobot_QMC5883.h>

// Змінні з данними компаса 
float compassHeading = 36.29;
int16_t roll_;
int16_t pitch_;
int16_t yaw_;

 
WiFiUDP udp;                            //  ЮДП
DFRobot_QMC5883 compass(&Wire, 0x0D);   //  Компас


void setup() {
  Serial.begin(57600);
  WiFi.softAP("MAVLink", "12345678");
  udp.begin(14550);


  while (!compass.begin())
  {
    Serial.println("Could not find a valid 5883 sensor, check wiring!");
    delay(500);
  }

if(compass.isQMC())
  {
     compass.setRange(QMC5883_RANGE_2GA);
     compass.setMeasurementMode(QMC5883_CONTINOUS);
     compass.setDataRate(QMC5883_DATARATE_50HZ);
     compass.setSamples(QMC5883_SAMPLES_8);
  }
}

void loop() {
  sendMAVLink();
  receiveMAVLink();

   float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI); // -- Поправка відхилення кута
  compass.setDeclinationAngle(declinationAngle);
  sVector_t mag = compass.readRaw();
  compass.getHeadingDegrees();

  // ----------- Передаємо значення компаса напрямок X,Y,Z в відповідні змінні.
  compassHeading = mag.HeadingDegress;
  roll_ = mag.XAxis;
  pitch_ = mag.YAxis;
  yaw_ = mag.ZAxis;

  // ------------------- Перевірка даних компаса (опціонально)
  Serial.print("X:");
  Serial.print(mag.XAxis);
  Serial.print(" Y:");
  Serial.print(mag.YAxis);
  Serial.print(" Z:");
  Serial.println(mag.ZAxis);
  Serial.print("Degress = ");
  Serial.println(mag.HeadingDegress);

  //delay(500);

}

void sendMAVLink() {
  static uint32_t lastSent = 0;
  if (millis() - lastSent < 1000) return; // Send every second








    // HEARTBEAT message buffer ------------------------------------------------------------ HEARTBEAT
  mavlink_message_t heartbeatMsg;
  uint8_t heartbeatBuf[MAVLINK_MAX_PACKET_LEN];
    // Формуємо пакет Hertbeat  
  mavlink_msg_heartbeat_pack(255, MAV_COMP_ID_USER1, &heartbeatMsg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 1, MAV_STATE_STANDBY);
  uint16_t heartbeatLen = mavlink_msg_to_send_buffer(heartbeatBuf, &heartbeatMsg);

    // SYS_STATUS message buffer ---------------------------------------------------------- SYS_STATUS
  mavlink_message_t sysStatusMsg;
  uint8_t sysStatusBuf[MAVLINK_MAX_PACKET_LEN];
  
  // Формуємо пакет системних данних 
  mavlink_msg_sys_status_pack(255, MAV_COMP_ID_USER1, &sysStatusMsg,  MAV_SYS_STATUS_SENSOR_3D_MAG, 1, 1, 25, 1, 27, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1);
  uint16_t sysStatusLen = mavlink_msg_to_send_buffer(sysStatusBuf, &sysStatusMsg);

    // ATTITUDE message buffer ------------------------------------------------------------- ATTITUDE
  mavlink_message_t attitudeMsg;
  uint8_t attitudeBuf[MAVLINK_MAX_PACKET_LEN];
  
  // ФОрмуємо пакет з даними компаса
  mavlink_msg_attitude_pack(255, MAV_COMP_ID_USER1, &attitudeMsg, millis(), 0, 0, compassHeading, 0, 0, 0); //(,millis(),roll,pich,yaw,roll-speed,pich-speed,yaw-speed)
  uint16_t attitudeLen = mavlink_msg_to_send_buffer(attitudeBuf, &attitudeMsg);


  // ------------------------------- Send HEARTBEAT 
  udp.beginPacket("255.255.255.255", 14550);
  udp.write(heartbeatBuf, heartbeatLen);
  udp.endPacket();

  // ------------------------------- Send SYS_STATUS 
  udp.beginPacket("255.255.255.255", 14550);
  udp.write(sysStatusBuf, sysStatusLen);
  udp.endPacket();

  // -------------------------------- Send ATTITUDE 
  udp.beginPacket("255.255.255.255", 14550);
  udp.write(attitudeBuf, attitudeLen);
  udp.endPacket();

  lastSent = millis();
}

void receiveMAVLink() {
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  // Read UDP packet
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  udp.read(buf, MAVLINK_MAX_PACKET_LEN);

  // Parse MAVLink message
  mavlink_message_t msg;
  mavlink_status_t status;
  for (int i = 0; i < packetSize; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          Serial.println("Received HEARTBEAT");
          break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL: // Моніторинг керування для налагодження та контролю
          mavlink_manual_control_t manualControl;   
          mavlink_msg_manual_control_decode(&msg, &manualControl);
          //Serial.print("Received MANUAL_CONTROL:");   // Вивід данних на монітор порта
          //Serial.print(" x=");
          //Serial.print(manualControl.x);
          //Serial.print(" y=");
          //Serial.print(manualControl.y);
          //Serial.print(" z=");
          //Serial.print(manualControl.z);
          //Serial.print(" r=");
          //Serial.println(manualControl.r);
          break;
        default:
          Serial.print("Received message with ID ");
          Serial.println(msg.msgid);
          break;
      }
    }
  }
}
