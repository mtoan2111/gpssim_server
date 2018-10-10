#include <TinyGPS++.h>

int state = 0;
double oLat6 = 0.0;
double oLat7 = 0.0;
double oLon6 = 0.0;
double oLon7 = 0.0;
double oAlt6 = 0.0;
double oAlt7 = 0.0;
const unsigned char ublox_bRate[] = {
  //change baudrate to 115200 - Protocol in/out only accepts NMEA messages
  0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,
  0x01,0x00,0x02,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0xBA,0x50
};
const unsigned char ublox_rate[] = {
  //change rate to 60ms
  0xB5,0x62,0x06,0x08,0x06,0x00,0x3C,0x00,0x01,0x00,0x01,0x00,0x52,0x22
  //change rate to 80ms
//  0xB5,0x62,0x06,0x08,0x06,0x00,0x50,0x00,0x01,0x00,0x01,0x00,0x66,0x9A
  //change rate to 100ms
//  0xB5,0x62,0x06,0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00,0x0B,0x77
};
const unsigned char ublox_cfg[] = {
  //disable,NMEA,GSV
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38,
  //disable,NMEA,VTG
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46,
  //disable,NMEA,GSA
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x31
};
const unsigned char ublox_GNSS[] = {
  //disable GPS, QZSS, SBAS, enable GLONASS
  0xB5,0x62,0x06,0x3E,0x24,0x00,0x00,0x00,0x16,0x04,0x00,0x04,0xFF,0x00,0x00,0x00,0x00,
  0x01,0x01,0x01,0x03,0x00,0x00,0x00,0x00,0x01,0x05,0x00,0x03,0x00,0x00,0x00,0x00,0x01,
  0x06,0x08,0xFF,0x00,0x01,0x00,0x00,0x01,0xA4,0x0D
};

int handshake()
{
  digitalWrite(12,1);
  state = !state;
  if (SerialUSB.available() > 0)
  {
    if (SerialUSB.read() == 'U')
    {
      SerialUSB.write(0x55);
      digitalWrite(12,0);
      return 1;
    }
    return 0;
  }
}

void cfg_ublox()
{
  int bRate = sizeof(ublox_bRate);
  int uRate = sizeof(ublox_rate);
  int uSize = sizeof(ublox_cfg);
  int gSize = sizeof(ublox_GNSS);
  int isOK = -1;
//  Config NMEA data rate   
  for (int i = 0; i < uRate; i++)
  {
    Serial1.write(ublox_rate[i]);
    Serial2.write(ublox_rate[i]);
  }
  delay(300);
  
//  Disable some specific messages on NMEA package
  for (int i = 0; i < uSize; i++)
  {
    Serial1.write(ublox_cfg[i]);
    Serial2.write(ublox_rate[i]);
  }
//  delay(300);
//  for (int i = 0; i < gSize; i++)
//  {
//    Serial2.write(ublox_GNSS[i]);
//  }
  delay(300);
  //Config baudrate
  for (int i = 0; i < bRate; i++)
  {
    Serial1.write(ublox_bRate[i]);
    Serial2.write(ublox_bRate[i]);
    isOK = 1;
  }
  delay(200);
  if (isOK)
  {
    //Re-config baudrate SerialPort1/2
    Serial1.end();
    Serial2.end();
    delay(200);
    Serial1.begin(115200);
    Serial2.begin(115200);
  }
}


void setup() {
  SerialUSB.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);
  pinMode(1,OUTPUT);
  pinMode(12,OUTPUT);
  while(!handshake()){}
  cfg_ublox();
}
  TinyGPSPlus gps6;
  TinyGPSPlus gps7;
  unsigned char isData = 0x00;
void loop() {
  while (Serial1.available() > 0)
  {
    if (gps6.encode(Serial1.read()))
    {
      if (gps6.location.isValid())
      {
        isData |= 0x01;
      }
    }
  }
  while (Serial2.available() > 0)
  {
    if (gps7.encode(Serial2.read()))
    {
      if (gps7.location.isValid())
      {
        isData |= 0x02;
      }
    }
  }
  if (isData == 0x03)
  {
    isData == 0x00;
    double lat6 = 0.00;
    double lon6 = 0.00;
    double alt6 = 0.00;
    double lat7 = 0.00;
    double lon7 = 0.00;
    double alt7 = 0.00;
    if (gps6.location.isValid())
    {
      lat6 = gps6.location.lat();
      lon6 = gps6.location.lng();
      alt6 = gps6.altitude.meters();
    }
    if (gps7.location.isValid())
    {
      lat7 = gps7.location.lat();
      lon7 = gps7.location.lng();
      alt7 = gps7.altitude.meters();
    }
    if (((lat6 != oLat6) || (lon6 != oLon6)) &&
        ((lat7 != oLat7) || (lon7 != oLon7)) &&
        ((alt6 != oAlt6) || (alt7 != oAlt7)))
    {
      oLat6 = lat6;
      oLat7 = lat7;
      oLon6 = lon6;
      oLon7 = lon7;
      digitalWrite(1,(state) ? 0 : 1);
      state = !state;
      String tmp = String(lat6,6) + "," + String(lon6,6) + "," + String(alt6,6) + "," + String(lat7,6) + "," + String(lon7,6) + "," + String(alt7,6);
      int clen = tmp.length();
      SerialUSB.print(clen);
      delay(10);
      SerialUSB.print(tmp);
    }
  }
}

