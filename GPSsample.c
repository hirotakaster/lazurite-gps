#include "GPSsample_ide.h"      // Additional Header
#include "lazurite.h"
#include "SPI.h"
#include "Wire.h"

#include <stdlib.h>
#include <string.h>


#define SEND_OK_LED_PIN  26
#define SEND_NG_LED_PIN  10
#define SUBGHZ_CH       36          // channel number (frequency)
#define SUBGHZ_PANID    0xabcd      // panid
#define HOST_ADDRESS    0xC001      // distination address

enum GPS_INVALID_STATE {
    GPS_INVALID_AGE = 0xFFFFFFFF,    
    GPS_INVALID_DATE = 0,
    GPS_INVALID_TIME = 0xFFFFFFFF,
    GPS_INVALID_FIX_TIME = 0xFFFFFFFF,
    GPS_INVALID_SATELLITES = 0xFF,
    GPS_INVALID_HDOP = 0xFFFFFFFF
};
#define GPS_INVALID_ANGLE    999999999
#define GPS_INVALID_ALTITUDE 999999999
#define GPS_INVALID_SPEED    999999999
enum {_GPS_SENTENCE_GPGGA, _GPS_SENTENCE_GPRMC, _GPS_SENTENCE_OTHER};


static const float GPS_INVALID_F_ANGLE = 1000.0;
static const float GPS_INVALID_F_ALTITUDE = 1000000.0;
static const float GPS_INVALID_F_SPEED = -1.0;

#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"

unsigned long _encoded_characters;
unsigned short _good_sentences;
unsigned short _failed_checksum;
unsigned short _passed_checksum;
  
byte _parity;
bool _is_checksum_term;
char _term[15];
byte _sentence_type;
byte _term_number;
byte _term_offset;
bool _gps_data_good;

unsigned long _time, _new_time;
unsigned long _date, _new_date;
long _latitude, _new_latitude;
long _longitude, _new_longitude;
long _altitude, _new_altitude;
unsigned long  _speed, _new_speed;
unsigned long  _course, _new_course;
unsigned long  _hdop, _new_hdop;
unsigned short _numsats, _new_numsats;
unsigned long _last_time_fix, _new_time_fix;
unsigned long _last_position_fix, _new_position_fix;

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

static bool encode(char c);
static bool term_complete();
static unsigned long parse_decimal();
static int gpsstrcmp(const char *str1, const char *str2);
static int from_hex(char a) ;
static long gpsatol(const char *str);
static unsigned long parse_degrees();
static bool gpsisdigit(char c);
static void get_position(long *latitude, long *longitude, unsigned long *fix_age);
static void f_get_position(float *latitude, float *longitude, unsigned long *fix_age);
static void sendresult(uint8_t rssi, int status);


void f_get_position(float *latitude, float *longitude, unsigned long *fix_age)
{
  long lat, lon;
  get_position(&lat, &lon, fix_age);
  *latitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (lat / 1000000.0);
  *longitude = lat == GPS_INVALID_ANGLE ? GPS_INVALID_F_ANGLE : (lon / 1000000.0);
}

static void get_position(long *latitude, long *longitude, unsigned long *fix_age)
{
  if (latitude) *latitude = _latitude;
  if (longitude) *longitude = _longitude;
  if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ? 
   GPS_INVALID_AGE : millis() - _last_position_fix;
}

static bool gpsisdigit(char c) {
    return c >= '0' && c <= '9';
}

static int gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2)
    ++str1, ++str2;
  return *str1;
}

static unsigned long parse_degrees()
{
  char *p;
  unsigned long left_of_decimal = gpsatol(_term);
  unsigned long hundred1000ths_of_minute = (left_of_decimal % 100UL) * 100000UL;
  for (p=_term; gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 10000;
    while (gpsisdigit(*++p))
    {
      hundred1000ths_of_minute += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left_of_decimal / 100) * 1000000 + (hundred1000ths_of_minute + 3) / 6;
}

static long gpsatol(const char *str)
{
  long ret = 0;
  while (gpsisdigit(*str))
    ret = 10 * ret + *str++ - '0';
  return ret;
}

static int from_hex(char a) 
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

static unsigned long parse_decimal()
{
  char *p = _term;
  bool isneg = *p == '-';
  unsigned long ret = 100UL * gpsatol(p);
  if (isneg) ++p;
  while (gpsisdigit(*p)) ++p;
  if (*p == '.')
  {
    if (gpsisdigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (gpsisdigit(p[2]))
        ret += p[2] - '0';
    }
  }
  return isneg ? -ret : ret;
}

static bool encode(char c)
{
  bool valid_sentence = false;

  ++_encoded_characters;
  switch(c)
  {
  case ',': // term terminators
    _parity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (_term_offset < sizeof(_term))
    {
      _term[_term_offset] = 0;
      valid_sentence = term_complete();
    }
    ++_term_number;
    _term_offset = 0;
    _is_checksum_term = c == '*';
    return valid_sentence;

  case '$': // sentence begin
    _term_number = _term_offset = 0;
    _parity = 0;
    _sentence_type = _GPS_SENTENCE_OTHER;
    _is_checksum_term = false;
    _gps_data_good = false;
    return valid_sentence;
  }

  // ordinary characters
  if (_term_offset < sizeof(_term) - 1)
    _term[_term_offset++] = c;
  if (!_is_checksum_term)
    _parity ^= c;

  return valid_sentence;
}


static bool term_complete() {
  if (_is_checksum_term)
  {
    byte checksum = 16 * from_hex(_term[0]) + from_hex(_term[1]);
    if (checksum == _parity)
    {
      if (_gps_data_good)
      {
#ifndef _GPS_NO_STATS
        ++_good_sentences;
#endif
        _last_time_fix = _new_time_fix;
        _last_position_fix = _new_position_fix;

        switch(_sentence_type)
        {
        case _GPS_SENTENCE_GPRMC:
          _time      = _new_time;
          _date      = _new_date;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _speed     = _new_speed;
          _course    = _new_course;
          break;
        case _GPS_SENTENCE_GPGGA:
          _altitude  = _new_altitude;
          _time      = _new_time;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _numsats   = _new_numsats;
          _hdop      = _new_hdop;
          break;
        }

        return true;
      }
    }

#ifndef _GPS_NO_STATS
    else
      ++_failed_checksum;
#endif
    return false;
  }

  // the first term determines the sentence type
  if (_term_number == 0)
  {
    if (!gpsstrcmp(_term, _GPRMC_TERM))
      _sentence_type = _GPS_SENTENCE_GPRMC;
    else if (!gpsstrcmp(_term, _GPGGA_TERM))
      _sentence_type = _GPS_SENTENCE_GPGGA;
    else
      _sentence_type = _GPS_SENTENCE_OTHER;
    return false;
  }

  if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0])
    switch(COMBINE(_sentence_type, _term_number))
  {
    case COMBINE(_GPS_SENTENCE_GPRMC, 1): // Time in both sentences
    case COMBINE(_GPS_SENTENCE_GPGGA, 1):
      _new_time = parse_decimal();
      _new_time_fix = millis();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 2): // GPRMC validity
      _gps_data_good = _term[0] == 'A';
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 3): // Latitude
    case COMBINE(_GPS_SENTENCE_GPGGA, 2):
      _new_latitude = parse_degrees();
      _new_position_fix = millis();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 4): // N/S
    case COMBINE(_GPS_SENTENCE_GPGGA, 3):
      if (_term[0] == 'S')
        _new_latitude = -_new_latitude;
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 5): // Longitude
    case COMBINE(_GPS_SENTENCE_GPGGA, 4):
      _new_longitude = parse_degrees();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 6): // E/W
    case COMBINE(_GPS_SENTENCE_GPGGA, 5):
      if (_term[0] == 'W')
        _new_longitude = -_new_longitude;
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 7): // Speed (GPRMC)
      _new_speed = parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 8): // Course (GPRMC)
      _new_course = parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPRMC, 9): // Date (GPRMC)
      _new_date = gpsatol(_term);
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
      _gps_data_good = _term[0] > '0';
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA)
      _new_numsats = (unsigned char)atoi(_term);
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 8): // HDOP
      _new_hdop = parse_decimal();
      break;
    case COMBINE(_GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
      _new_altitude = parse_decimal();
      break;
  }

  return false;
}

unsigned char counter;
void setup() {
    Serial.begin(115200);   // USB Serial
    Serial1.begin(9600);    // GPS(pin 11:tx, 12:rx)

    SubGHz.init();
    counter = 0xFF;
    pinMode(SEND_OK_LED_PIN, OUTPUT);
    pinMode(SEND_NG_LED_PIN, OUTPUT);
}

void loop() {
    unsigned long starttime = millis();
    long lon = 0, lat = 0;
    unsigned char buffer[10];
    unsigned long fix_age;
    unsigned int sendsize = 0;
    SUBGHZ_MSG msg;

    // get GPS data
    do {
        while (Serial1.available())
            encode(Serial1.read());
    } while (millis() - starttime < 1000);

    delay(1000);
    get_position(&lat, &lon, &fix_age);
    
    // set sequence no, GPS data(lon, lat) to buffer
    memset(buffer, 0, sizeof(buffer));
    if (++counter == 0) counter++;  
    buffer[0] = counter;  sendsize += sizeof(counter);
    memcpy(&buffer[sendsize], &lat, sizeof(lat)); sendsize += sizeof(lat);
    memcpy(&buffer[sendsize], &lon, sizeof(lon)); sendsize += sizeof(lon);

    // send data to 920MHz
    SubGHz.begin(SUBGHZ_CH, SUBGHZ_PANID,  SUBGHZ_50KBPS, SUBGHZ_PWR_20MW);
    msg = SubGHz.send(SUBGHZ_PANID, HOST_ADDRESS, &buffer, sendsize, NULL);
    if (msg == SUBGHZ_OK) {
        digitalWrite(SEND_OK_LED_PIN, LOW);
        digitalWrite(SEND_NG_LED_PIN, LOW);
        Serial.println("Success");
    } else {
        digitalWrite(SEND_OK_LED_PIN, HIGH);
        digitalWrite(SEND_NG_LED_PIN, HIGH);
        Serial.println("Fail");
        SubGHz.msgOut(msg);
    }
    SubGHz.close();

}
