#include <SimpleTimer.h>    // https://github.com/marcelloromani/Arduino-SimpleTimer
#include <SerialCommands.h> // https://github.com/ppedro74/Arduino-SerialCommands/
#include <EEPROM.h>

char serial_command_buffer_[64];

SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\n", " ");

SimpleTimer timer;
//------------------------------------------------------------------------------

//------------------//
// GLOBAL VARIABLES //
//------------------//
char fwVersion[4] = "0.2";
int target_temp;
int pidP;
int pidI;
int pidD;
int pidB;
int minrpm;
int maxrpm;
int rpm;
int ontime;
int offtime;
int restime;
int sensor;                     //which sensor to use for the PID controller (0...2; 3 means external temperature)
int filter_threshold;           //value in mBar
int sensor_temps[4];  //values in °C
int pressure;                   //value in mBar of pressure difference
int external_reference;
int mode;                       //0 = gpu, 1 = asic
const int MAX_MINERS = 120;
int miners[MAX_MINERS];             //-1 = disabled/not present, 0 = off, 1 = on
bool filter_ok = true;
bool status = true;
//------------------------------------------------------------------------------

int addr_marker = -4;
int addr(int offset) { addr_marker += offset; return addr_marker; }
int addr() { return addr(4); }

//------------------//
// EEPROM ADDRESSES //
//------------------//
const int ADDR_INIT_MARKER = addr(); //int size 4 byte
const int ADDR_TARGET_TEMP = addr();
const int ADDR_PIDP = addr();
const int ADDR_PIDI = addr();
const int ADDR_PIDD = addr();
const int ADDR_PIDB = addr();
const int ADDR_MINRPM = addr();
const int ADDR_MAXRPM = addr();
const int ADDR_ONTIME = addr();
const int ADDR_OFFTIME = addr();
const int ADDR_RESTIME = addr();
const int ADDR_SENSOR = addr();
const int ADDR_THRESHOLD = addr();
const int ADDR_EXTERNAL = addr();
const int ADDR_MODE = addr();
const int ADDR_TEMPS = addr(); //int[4] size = 16 byte
const int ADDR_VERSION = addr(sizeof(sensor_temps)); //String size = 12 byte
const int ADDR_MINERS = addr(sizeof(fwVersion)); //int[120] size = 480 byte

//------------------//
// HELPER FUNCTIONS //
//------------------//
void returnOK(SerialCommands* sender) { echo(sender, "OK"); }
void returnOK(SerialCommands* sender, String msg) { echo(sender, "OK - " + msg); }
void returnERR(SerialCommands* sender) { echo(sender, "ERR"); }
void returnERR(SerialCommands* sender, String msg) { echo(sender, "ERR - " + msg); }

//This is the default handler, and gets called when no other command matches. 
void cmd_unrecognized(SerialCommands* sender, const char* cmd) {
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

/*
 * Checks whether the param is NULL, empty or a non-digit.
 * Returns true if a valid number is given.
 */
bool check_param(SerialCommands* sender, char* param) {
  if (param == NULL) {
    returnERR(sender, "no argument given");
    return false;
  }
  for (int i = 0; i < sizeof(param)/sizeof(char); ++i) {
    if (!isDigit(param[i]) and param[i] != '\0') { // if no digit and not empty
      returnERR(sender, "invalid ID given");
      return false;
    } 
  }
  return true;
}

/*
 * Checks the param (see check_param) internally and sets the resulting int
 * as the referenced variable value.
 */
void setArgument(SerialCommands* sender, int& injectedVar) {
  char* temp = sender->Next();
  if (!check_param(sender, temp)) {
    return;
  }
  injectedVar = atoi(temp);
}

/*
 * The set method also returns the value to ease the process of syncronization. eg. int var1 = setEEPROM(ADDR, 12345)
 */
int setEEPROMval(int address, int value) { EEPROM.update(address, value); return value; }
void setEEPROMval(int address, char *value) { 
  for(int i=0; i < sizeof(value)/sizeof(char); ++i) {
    EEPROM.update(address + i * sizeof(char), value[i]);
  }
}
int getEEPROMval(int address) { return EEPROM.read(address); }

void getEEPROMval(int address, char *store) {
  for (int i = 0; i < sizeof(store)/sizeof(char); ++i) {
    store[i] = EEPROM.read(address + i * sizeof(char));
  }
}

// Theoretically this clears the EEPROM, but I recommend against executing this because it is
// incredibly slow and it hurts the lifetime of the flash memory of the nucleo board (as it has no real EEPROM)
void clearEEPROM() {
  for (int i = 0 ; i < EEPROM.length() ; ++i) {
    EEPROM.write(i, 0);
    Serial.println(i);
  }
}

bool EEPROMisEmpty() {
  return getEEPROMval(ADDR_INIT_MARKER) == 255;
}

// TODO call this somewhere
unsigned long eeprom_crc(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0 ; index < EEPROM.length()  ; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

// Prints the variable to the senders serial interface
void echo(SerialCommands* sender, int var) { sender->GetSerial()->println(var); }
void echo(SerialCommands* sender, String var) { sender->GetSerial()->println(var); }

//------------------------------------------------------------------------------

//----------------------------//
// USER INTERACTION FUNCTIONS //
//----------------------------//

void printFWVersion(SerialCommands* sender) { echo(sender, fwVersion); } //?fw
void getPidP(SerialCommands* sender) { echo(sender, pidP); } //?pidp
void getPidI(SerialCommands* sender) { echo(sender, pidI); } //?pidi
void getPidD(SerialCommands* sender) { echo(sender, pidD); } //?pidd
void getPidB(SerialCommands* sender) { echo(sender, pidB); } //?pidb
void getTargetTemperature(SerialCommands* sender) { echo(sender, target_temp); } //?targettemp
void getPress(SerialCommands* sender) { echo(sender, pressure); } //?pressure
void getPressThreshold(SerialCommands* sender) { echo(sender, filter_threshold); } //?threshold
void getFilter(SerialCommands* sender) { echo(sender, filter_ok); } //?filter
void getSensor(SerialCommands* sender) { echo(sender, sensor); } //?sensor
void getMinRPM(SerialCommands* sender) { echo(sender, minrpm); } //?minrpm
void getMaxRPM(SerialCommands* sender) { echo(sender, maxrpm); } //?maxrpm
void getRPM(SerialCommands* sender) { echo(sender, rpm); } //?rpm
void getOnTime(SerialCommands* sender) { echo(sender, ontime); } //?ontime
void getOffTime(SerialCommands* sender) { echo(sender, offtime); } //?offtime
void getResTime(SerialCommands* sender) { echo(sender, restime); } //?restime
void getExternalReference(SerialCommands* sender) { echo(sender, restime); } //?external
void getMode(SerialCommands* sender) { echo(sender, mode); } //?mode

//?temps
void getTemps(SerialCommands* sender) {
  String temp = "";
  for (int i = 0; i < sizeof(sensor_temps)/sizeof(int); ++i) {
    temp += i;
    temp += ":";
    temp += sensor_temps[i];
    temp += ", ";
  }
  temp.remove(temp.length() - 2); //Removing the last ,_
  echo(sender, temp);
}

//?miner <id>
void getMiner(SerialCommands* sender) {
  String raw_id = String(sender->Next());
  if (raw_id == "") {
    returnERR(sender, "no argument given");
    return;
  }
  // TODO invalid argument given

  int id = raw_id.toInt();
  if (id < 0 or id >= MAX_MINERS) {
    returnERR(sender, "invalid ID given");
    return;
  }
  echo(sender, miners[id]);
}

void setTargetTemperature(SerialCommands* sender) {
  setArgument(sender, target_temp);
  setEEPROMval(ADDR_TARGET_TEMP, target_temp);
} //!targettemp
void setSensor(SerialCommands* sender) { setArgument(sender, sensor); } //!sensor
void setExternalReference(SerialCommands* sender) { setArgument(sender, external_reference); } //!external
void setPressureThreshold(SerialCommands* sender) { setArgument(sender, filter_threshold); } //!threshold
void setMinRPM(SerialCommands* sender) { setArgument(sender, minrpm); } //!minrpm
void setMaxRPM(SerialCommands* sender) { setArgument(sender, maxrpm); } //!maxrpm
void setMode(SerialCommands* sender) { setArgument(sender, mode); } //!mode
void setOntime(SerialCommands* sender) { setArgument(sender, ontime); } //!ontime
void setOfftime(SerialCommands* sender) { setArgument(sender, offtime); } //!offtime
void setRestime(SerialCommands* sender) { setArgument(sender, restime); } //!restime
void setPidP(SerialCommands* sender) { setArgument(sender, pidP); } //!pidp
void setPidI(SerialCommands* sender) { setArgument(sender, pidI); } //!pidi
void setPidD(SerialCommands* sender) { setArgument(sender, pidD); } //!pidd
void setPidB(SerialCommands* sender) { setArgument(sender, pidB); } //!pidb

//!miner <id> <action>
void setMiner(SerialCommands* sender) {
  String raw_id = String(sender->Next());
  if (raw_id == "") {
    returnERR(sender, "no argument given");
    return;
  }
  // TODO invalid argument given

  int id = raw_id.toInt();
  String action = String(sender->Next());
  if (action == NULL) {
    returnERR(sender, "no argument given");
    return;
  } 
  if (action != "on" and action != "off" and action != "register" and action != "deregister") {
    returnERR(sender, "invalid argument given");
    return;
  }
  if (action == "on" or action =="register") {
    miners[id] = 1;
  } else if (action == "off") {
    miners[id] = 0;
  } else if (action == "deregister") {
    miners[id] = -1;
  }
}

//------------------------------------------------------------------------------

//-------------------//
// SERVICE FUNCTIONS //
//-------------------//

void check_filter_status() {
  filter_ok = pressure < filter_threshold;
}

void init_values() {
  target_temp = setEEPROMval(ADDR_TARGET_TEMP, 50);
  pidP = setEEPROMval(ADDR_PIDP, 1);
  pidI = setEEPROMval(ADDR_PIDI, 3);
  pidD = setEEPROMval(ADDR_PIDD, 4);
  pidB = setEEPROMval(ADDR_PIDB, 2);
  minrpm = setEEPROMval(ADDR_MINRPM, 5);
  maxrpm = setEEPROMval(ADDR_MAXRPM, 80);
  ontime = setEEPROMval(ADDR_ONTIME, 100);
  offtime = setEEPROMval(ADDR_OFFTIME, 85);
  restime = setEEPROMval(ADDR_RESTIME, 125);
  sensor = setEEPROMval(ADDR_SENSOR, 1);
  filter_threshold = setEEPROMval(ADDR_THRESHOLD, 1400);
  external_reference = setEEPROMval(ADDR_EXTERNAL, 55);
  for (int i = 0; i < sizeof(miners)/sizeof(int); ++i) {
    if (random(0, 100) < 3) {
      miners[i] = -1;
    } else if (random(0, 100) < 10) {
      miners[i] = 0;
    } else {
      miners[i] = 1;
    }
    setEEPROMval(ADDR_MINERS + i * sizeof(int), miners[i]);
  }
}

void load_values() {
  target_temp = getEEPROMval(ADDR_TARGET_TEMP);
  pidP = getEEPROMval(ADDR_PIDP);
  pidI = getEEPROMval(ADDR_PIDI);
  pidD = getEEPROMval(ADDR_PIDD);
  pidB = getEEPROMval(ADDR_PIDB);
  minrpm = getEEPROMval(ADDR_MINRPM);
  maxrpm = getEEPROMval(ADDR_MAXRPM);
  ontime = getEEPROMval(ADDR_ONTIME);
  offtime = getEEPROMval(ADDR_OFFTIME);
  restime = getEEPROMval(ADDR_RESTIME);
  sensor = getEEPROMval(ADDR_SENSOR);
  filter_threshold = getEEPROMval(ADDR_THRESHOLD);
  external_reference = getEEPROMval(ADDR_EXTERNAL);
  for (int i = 0; i < sizeof(miners)/sizeof(int); ++i) {
    miners[i] = getEEPROMval(ADDR_MINERS + i * sizeof(int));
  }
}

void mock_changes() {
  rpm = random(minrpm + 10, maxrpm - 10);
  pressure = random(1200, 1500);
  for (int i = 0; i < sizeof(sensor_temps)/sizeof(int); ++i) {
    sensor_temps[i] = random(30, 80);
  }
}

//------------------------------------------------------------------------------

//-----------------//
// TIMER FUNCTIONS //
//-----------------//
void mainTimer() {
  mock_changes();

  // blinking green status LED
  digitalWrite(PB0, status);
  status = !status;

  // flashing blue LED if filter needs cleaning
  check_filter_status();
  digitalWrite(PB7, !filter_ok);
}

//------------------------------------------------------------------------------

//-----------------//
// SERIAL COMMANDS //
//-----------------//
SerialCommand cmd_fw("?fw", printFWVersion);
SerialCommand cmd_getpidp("?pidp", getPidP);
SerialCommand cmd_getpidi("?pidi", getPidI);
SerialCommand cmd_getpidd("?pidd", getPidD);
SerialCommand cmd_getpidb("?pidb", getPidB);
SerialCommand cmd_gettemps("?temps", getTemps);
SerialCommand cmd_gettargettemp("?targettemp", getTargetTemperature);
SerialCommand cmd_getpress("?pressure", getPress);
SerialCommand cmd_getpressthreshold("?threshold", getPressThreshold);
SerialCommand cmd_getfilter("?filter", getFilter);
SerialCommand cmd_getsensor("?sensor", getSensor);
SerialCommand cmd_getminrpm("?minrpm", getMinRPM);
SerialCommand cmd_getmaxrpm("?maxrpm", getMaxRPM);
SerialCommand cmd_getrpm("?rpm", getRPM);
SerialCommand cmd_getontime("?ontime", getOnTime);
SerialCommand cmd_getofftime("?offtime", getOffTime);
SerialCommand cmd_getrestime("?restime", getResTime);
SerialCommand cmd_getexternal("?external", getExternalReference);
SerialCommand cmd_getmode("?mode", getMode);
SerialCommand cmd_getminer("?miner", getMiner);
SerialCommand cmd_settargettemp("!targettemp", setTargetTemperature);
SerialCommand cmd_setexternalreference("!external", setExternalReference);
SerialCommand cmd_setpressurethreshold("!threshold", setPressureThreshold);
SerialCommand cmd_setmaxrpm("!maxrpm", setMaxRPM);
SerialCommand cmd_setminrpm("!minrpm", setMinRPM);
SerialCommand cmd_setmode("!mode", setMode);
SerialCommand cmd_setontime("!ontime", setOntime);
SerialCommand cmd_setofftime("!offtime", setOfftime);
SerialCommand cmd_setrestime("!restime", setRestime);
SerialCommand cmd_setpidp("!pidp", setPidP);
SerialCommand cmd_setpidi("!pidi", setPidI);
SerialCommand cmd_setpidd("!pidd", setPidD);
SerialCommand cmd_setpidb("!pidb", setPidB);
SerialCommand cmd_setminer("!miner", setMiner);

void add_serial_commands() {
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&cmd_fw);
  serial_commands_.AddCommand(&cmd_getpidp);
  serial_commands_.AddCommand(&cmd_getpidi);
  serial_commands_.AddCommand(&cmd_getpidd);
  serial_commands_.AddCommand(&cmd_getpidb);
  serial_commands_.AddCommand(&cmd_gettemps);
  serial_commands_.AddCommand(&cmd_gettargettemp);
  serial_commands_.AddCommand(&cmd_getpress);
  serial_commands_.AddCommand(&cmd_getpressthreshold);
  serial_commands_.AddCommand(&cmd_getfilter);
  serial_commands_.AddCommand(&cmd_getsensor);
  serial_commands_.AddCommand(&cmd_getminrpm);
  serial_commands_.AddCommand(&cmd_getmaxrpm);
  serial_commands_.AddCommand(&cmd_getrpm);
  serial_commands_.AddCommand(&cmd_getontime);
  serial_commands_.AddCommand(&cmd_getofftime);
  serial_commands_.AddCommand(&cmd_getrestime);
  serial_commands_.AddCommand(&cmd_getexternal);
  serial_commands_.AddCommand(&cmd_getmode);
  serial_commands_.AddCommand(&cmd_getminer);
  serial_commands_.AddCommand(&cmd_settargettemp);
  serial_commands_.AddCommand(&cmd_setexternalreference);
  serial_commands_.AddCommand(&cmd_setpressurethreshold);
  serial_commands_.AddCommand(&cmd_setminrpm);
  serial_commands_.AddCommand(&cmd_setmaxrpm);
  serial_commands_.AddCommand(&cmd_setmode);
  serial_commands_.AddCommand(&cmd_setontime);
  serial_commands_.AddCommand(&cmd_setofftime);
  serial_commands_.AddCommand(&cmd_setrestime);
  serial_commands_.AddCommand(&cmd_setpidp);
  serial_commands_.AddCommand(&cmd_setpidi);
  serial_commands_.AddCommand(&cmd_setpidd);
  serial_commands_.AddCommand(&cmd_setpidb);
  serial_commands_.AddCommand(&cmd_setminer);
}

//------------------------------------------------------------------------------

//--------------------------//
// SETUP CODE AND MAIN LOOP //
//--------------------------//
void setup() {
  pinMode(PB0, OUTPUT);
  pinMode(PB7, OUTPUT);
  pinMode(PB14, OUTPUT);
  
  Serial.begin(9600);

  // the following line is only used for testing/debugging. straight-out-of-factory EEPROMs
  // have 255 written all over the memory
  // setEEPROMval(ADDR_INIT_MARKER, 255);
  if (EEPROMisEmpty()) {
    Serial.println("Initializing values. This may take a minute or two.");
    init_values();
  } else {
    // TODO
    setEEPROMval(ADDR_INIT_MARKER, 1);
    load_values();
  }
  
  timer.setInterval(1000, mainTimer);
  add_serial_commands();
}

void loop() {
  serial_commands_.ReadSerial();
  timer.run();
}
