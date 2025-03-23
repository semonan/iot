
#ifndef LED_BUILTIN_1
#define LED_BUILTIN_1 PA0
#endif

#define DATA_SIZE      (21)

#define PACKET_START1  (0xAA)
#define PACKET_START2  (0x55)
#define PACKET_END1    (0x0D)
#define PACKET_END2    (0x0D)

#define LIVINGROOM     (0x0001)
#define ROOM1          (0x0101)
#define ROOM2          (0x0201)
#define ROOM3          (0x0301)
#define KITCHEN        (0x0401)


#include <Matter.h>
#include <MatterLightbulb.h>


MatterLightbulb device_livingroom_1;
MatterLightbulb device_livingroom_2;
MatterLightbulb device_kitchen_1;
MatterLightbulb device_kitchen_2;
MatterLightbulb device_room1_1;
MatterLightbulb device_room1_2;
MatterLightbulb device_room2;
MatterLightbulb device_room3;


bool status_livingroom_1 = false;
bool status_livingroom_2 = false;
bool status_kitchen_1 = false;
bool status_kitchen_2 = false;
bool status_room1_1 = false;
bool status_room1_2 = false;
bool status_room2 = false;
bool status_room3 = false;


void setup()
{
  Serial.begin(115200);  
  Serial1.begin(9600);
  Matter.begin();

  device_livingroom_1.begin();
  device_livingroom_2.begin();
  device_kitchen_1.begin();
  device_kitchen_2.begin();
  device_room1_1.begin();
  device_room1_2.begin();
  device_room2.begin();
  device_room3.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
  pinMode(LED_BUILTIN_1, OUTPUT);
  digitalWrite(LED_BUILTIN_1, LED_BUILTIN_INACTIVE);

  pinMode(BTN_BUILTIN, INPUT_PULLUP);

  Serial.println("Matter multiple lightbulbs");

  if (!Matter.isDeviceCommissioned()) {
    Serial.println("Matter device is not commissioned");
    Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
    Serial.printf("Manual pairing code: %s\n", Matter.getManualPairingCode().c_str());
    Serial.printf("QR code URL: %s\n", Matter.getOnboardingQRCodeUrl().c_str());
  }
  while (!Matter.isDeviceCommissioned()) {
    delay(200);
  }

  Serial.println("Waiting for Thread network...");

  while (!Matter.isDeviceThreadConnected()) {
    delay(200);
    decommission_handler();
  }

  Serial.println("Connected to Thread network");
  Serial.println("Waiting for Matter device discovery...");

  while (
       !device_livingroom_1.is_online() 
    || !device_livingroom_2.is_online()
    || !device_kitchen_1.is_online()
    || !device_kitchen_2.is_online()
    || !device_room1_1.is_online()
    || !device_room1_2.is_online()
    || !device_room2.is_online()
    || !device_room3.is_online()
  ) {
    delay(200);
    decommission_handler();
  }

  for (uint8_t i = 0u; i < 5u; i++) {
    digitalWrite(LED_BUILTIN, !(digitalRead(LED_BUILTIN)));
    delay(200);
  }
  digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);

  Serial.println("Matter devices are now online");

  light_control(LIVINGROOM, false, false);
  delay(200);
  light_control(ROOM1, false, false);
  delay(200);
  light_control(ROOM2, false, false);
  delay(200);
  light_control(ROOM3, false, false);
  delay(200);
  light_control(KITCHEN, false, false);
  delay(200);

  device_livingroom_1.set_onoff(false);
  device_livingroom_2.set_onoff(false);
  device_room1_1.set_onoff(false);
  device_room1_2.set_onoff(false);
  device_room2.set_onoff(false);
  device_room3.set_onoff(false);
  device_kitchen_1.set_onoff(false);
  device_kitchen_2.set_onoff(false);

  serial1_buffer_clear();
}


void loop()
{
  decommission_handler();

  for (int i = 0; i < 128; i++)
  {
    refresh_light_status();
  }

  livingroom_handle();
  room1_handle();
  room2_handle();
  room3_handle();
  kitchen_handle();

  delay(1);
}


void serial1_buffer_clear()
{
    while (Serial1.available() > 0)
    {
        Serial1.read();
    }
}


unsigned char get_checksum(unsigned char* data)
{
  int sum = 0;
  for (int i = 0; i < 16; i++)
  {
    sum += data[i];
  }
  return sum % 256;
}


unsigned short read_rx_buffer(bool* light_1, bool* light_2)
{
  *light_1 = false;
  *light_2 = false;
  unsigned char rx[DATA_SIZE] = {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0};
  int idx = 0;
  bool packetReceived = false;
  while (Serial1.available() > 0)
  {
    delay(2);
    unsigned char onebyte = Serial1.read();
    
    if (idx == 0 && onebyte == PACKET_START1)
    {
      rx[idx++] = onebyte;
      continue;
    }
    if (idx == 1 && rx[idx-1] == PACKET_START1 && onebyte == PACKET_START2)
    {
      rx[idx++] = onebyte;
      continue;  
    }
    if (idx == DATA_SIZE-2 && onebyte == PACKET_END1)
    {
      rx[idx++] = onebyte;
      continue;
    }
    if (idx == DATA_SIZE-1 && rx[idx-1] == PACKET_END1 && onebyte == PACKET_END2)
    {
      rx[idx++] = onebyte;
      if (get_checksum(&rx[2]) != rx[18])
      {
        Serial.println("Checksum error");
        return 0x00;
      }
      Serial.println("Packet received");
      packetReceived = true;
      break;
    }
    if (idx >= 2 && idx < DATA_SIZE)
    {
      rx[idx++] = onebyte;
      continue;
    }
  } // while end

  if (packetReceived)
  {  // AA55 30DC 000E SITE 0000 DATA
    if (rx[2] != 0x30 || rx[3] != 0xDC || 
        rx[4] != 0x00 || rx[5] != 0x0E || 
        rx[8] != 0x00 || rx[9] != 0x00)
    {
      return 0x00;
    }
    if (rx[10] == 0xFF)
    {
      *light_1 = true;
    }
    if (rx[11] == 0xFF)
    {
      *light_2 = true;
    }
    if ( (((unsigned short)rx[6])<<8 | rx[7]) == LIVINGROOM )
    {
      return LIVINGROOM;
    }
    else if ( (((unsigned short)rx[6])<<8 | rx[7]) == ROOM1 )
    {
      return ROOM1;
    }
    else if ( (((unsigned short)rx[6])<<8 | rx[7]) == ROOM2 )
    {
      return ROOM2;
    }
    else if ( (((unsigned short)rx[6])<<8 | rx[7]) == ROOM3 )
    {
      return ROOM3;
    }
    else if ( (((unsigned short)rx[6])<<8 | rx[7]) == KITCHEN )
    {
      return KITCHEN;
    }
  }

  return 0x00;
}


void refresh_light_status()
{
  bool light_1, light_2;
  unsigned short site = read_rx_buffer(&light_1, &light_2);
  if (site == LIVINGROOM)
  {
    Serial.println("from Living room");
    device_livingroom_1.set_onoff(light_1);
    device_livingroom_2.set_onoff(light_2);
    status_livingroom_1 = light_1;
    status_livingroom_2 = light_2;
  }
  else if (site == ROOM1)
  {
    Serial.println("from Main room");
    device_room1_1.set_onoff(light_1);
    device_room1_2.set_onoff(light_2);
    status_room1_1 = light_1;
    status_room1_2 = light_2;
  }
  else if (site == ROOM2)
  {
    Serial.println("from Room2");
    device_room2.set_onoff(light_1);
    status_room2 = light_1;
  }
  else if (site == ROOM3)
  {
    Serial.println("from Room3");
    device_room3.set_onoff(light_1);
    status_room3 = light_1;
  }
  else if (site == KITCHEN)
  {
    Serial.println("from Kitchen");
    device_kitchen_1.set_onoff(light_1);
    device_kitchen_2.set_onoff(light_2);
    status_kitchen_1 = light_1;
    status_kitchen_2 = light_2;
  }
}


void decommission_handler()
{
  if (digitalRead(BTN_BUILTIN) != LOW || !Matter.isDeviceCommissioned()) {
    return;
  }
  uint32_t start_time = millis();
  while (digitalRead(BTN_BUILTIN) == LOW) {
    uint32_t elapsed_time = millis() - start_time;
    if (elapsed_time < 5000u) {
      yield();
      continue;
    }
    for (uint8_t i = 0u; i < 10u; i++) {
      digitalWrite(LED_BUILTIN, !(digitalRead(LED_BUILTIN)));
      delay(100);
    }
    Serial.println("Starting decommissioning process, device will reboot...");
    Serial.println();
    digitalWrite(LED_BUILTIN, LED_BUILTIN_INACTIVE);
    Matter.decommission();
  }
}


void light_control(unsigned short site, bool light_1, bool light_2)
{
  //AA55  30BC  000E  0001  0000  0000000000000000  FB  0D0D
  unsigned char tx[DATA_SIZE] = {0xAA,0x55, 0x30,0xBC, 0x00,0x0E, 0x00,0x01, 0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, 0xFF, 0x0D,0x0D};
  tx[6] = site>>8;
  tx[7] = site;
  if (light_1)
  {
    tx[10] = 0xFF;
  }
  if (light_2)
  {
    tx[11] = 0xFF;
  }
  tx[18] = get_checksum(&tx[2]);

  Serial1.write(tx, sizeof(tx));
  delay(200);
  serial1_buffer_clear();
}


void livingroom_handle()
{
  bool light_1 = device_livingroom_1.get_onoff();
  bool light_2 = device_livingroom_2.get_onoff();
  if (status_livingroom_1 != light_1 || status_livingroom_2 != light_2)
  {
    status_livingroom_1 = light_1;
    status_livingroom_2 = light_2;
    light_control(LIVINGROOM, light_1, light_2);
    Serial.println("Light control : LIVINGROOM");
  }
}

void room1_handle()
{
  bool light_1 = device_room1_1.get_onoff();
  bool light_2 = device_room1_2.get_onoff();
  if (status_room1_1 != light_1 || status_room1_2 != light_2)
  {
    status_room1_1 = light_1;
    status_room1_2 = light_2;
    light_control(ROOM1, light_1, light_2);
    Serial.println("Light control : ROOM1");
  }
}

void room2_handle()
{
  bool light_1 = device_room2.get_onoff();
  if (status_room2 != light_1)
  {
    status_room2 = light_1;
    light_control(ROOM2, light_1, false);
    Serial.println("Light control : ROOM2");
  }
}

void room3_handle()
{
  bool light_1 = device_room3.get_onoff();
  if (status_room3 != light_1)
  {
    status_room3 = light_1;
    light_control(ROOM3, light_1, false);
    Serial.println("Light control : ROOM3");
  }
}

void kitchen_handle()
{
  bool light_1 = device_kitchen_1.get_onoff();
  bool light_2 = device_kitchen_2.get_onoff();
  if (status_kitchen_1 != light_1 || status_kitchen_2 != light_2)
  {
    status_kitchen_1 = light_1;
    status_kitchen_2 = light_2;
    light_control(KITCHEN, light_1, light_2);
    Serial.println("Light control : KITCHEN");
  }
}



