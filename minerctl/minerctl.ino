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
char fw_version[4] = "0.2";
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

int addr_marker = -sizeof(int);
int addr(int offset) { addr_marker += offset; return addr_marker; }
int addr() { return addr(sizeof(int)); }

//------------------//
// EEPROM ADDRESSES //
//------------------//
const int ADDR_INIT_MARKER = addr();
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
const int ADDR_MINERS = addr(sizeof(fw_version)); //int[120] size = 480 byte

//------------------//
// HELPER FUNCTIONS //
//------------------//
void return_OK(SerialCommands* sender) { echo(sender, "OK"); }
void return_OK(SerialCommands* sender, String msg) { echo(sender, "OK - " + msg); }
void return_ERR(SerialCommands* sender) { echo(sender, "ERR"); }
void return_ERR(SerialCommands* sender, String msg) { echo(sender, "ERR - " + msg); }

/*
 * This is the default handler, and gets called when no other command matches.
 */
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
    return_ERR(sender, "no argument given");
    return false;
  }
  for (int i = 0; i < sizeof(param)/sizeof(char); ++i) {
    if (!isDigit(param[i]) and param[i] != '\0') { // if no digit and not empty
      return_ERR(sender, "invalid ID given");
      return false;
    } 
  }
  return true;
}

/*
 * Checks the param (see check_param) internally and sets the resulting int
 * as the referenced variable value.
 */
void set_arg(SerialCommands* sender, int& injectedVar) {
  char* temp = sender->Next();
  if (!check_param(sender, temp)) {
    return;
  }
  injectedVar = atoi(temp);
}

void set_arg(SerialCommands* sender, int& injectedVar, int address) {
  set_arg(sender, injectedVar);
  set_EEPROM(address, injectedVar);
}

/*
 * The set method also returns the value to ease the process of syncronization. eg. int var1 = setEEPROM(ADDR, 12345)
 */
int set_EEPROM(int address, int value) { EEPROM.update(address, value); return value; }
void set_EEPROM(int address, char *value) { 
  for(int i=0; i < sizeof(value)/sizeof(char); ++i) {
    EEPROM.update(address + i * sizeof(char), value[i]);
  }
}
int get_EEPROM(int address) { return EEPROM.read(address); }

void get_EEPROM(int address, char *store) {
  for (int i = 0; i < sizeof(store)/sizeof(char); ++i) {
    store[i] = EEPROM.read(address + i * sizeof(char));
  }
}

/*
 * Theoretically this clears the EEPROM, but I recommend against executing this because it is
 * incredibly slow and it hurts the lifetime of the flash memory of the nucleo board (as it has no real EEPROM)
 */
void clear_EEPROM() {
  for (int i = 0 ; i < EEPROM.length() ; ++i) {
    EEPROM.write(i, 0);
    Serial.println(i);
  }
}

bool EEPROM_is_empty() {
  // TODO remove marker and exchange for crc
  return get_EEPROM(ADDR_INIT_MARKER) == 255;
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

void get_fw_version(SerialCommands* sender) { echo(sender, fw_version); } //?fw
void get_pidp(SerialCommands* sender) { echo(sender, pidP); } //?pidp
void get_pidi(SerialCommands* sender) { echo(sender, pidI); } //?pidi
void get_pidd(SerialCommands* sender) { echo(sender, pidD); } //?pidd
void get_pidb(SerialCommands* sender) { echo(sender, pidB); } //?pidb
void get_target_temp(SerialCommands* sender) { echo(sender, target_temp); } //?targettemp
void get_pressure(SerialCommands* sender) { echo(sender, pressure); } //?pressure
void get_pressure_threshold(SerialCommands* sender) { echo(sender, filter_threshold); } //?threshold
void get_filter(SerialCommands* sender) { echo(sender, filter_ok); } //?filter
void get_sensor(SerialCommands* sender) { echo(sender, sensor); } //?sensor
void get_minrpm(SerialCommands* sender) { echo(sender, minrpm); } //?minrpm
void get_maxrpm(SerialCommands* sender) { echo(sender, maxrpm); } //?maxrpm
void get_rpm(SerialCommands* sender) { echo(sender, rpm); } //?rpm
void get_ontime(SerialCommands* sender) { echo(sender, ontime); } //?ontime
void get_offtime(SerialCommands* sender) { echo(sender, offtime); } //?offtime
void get_restime(SerialCommands* sender) { echo(sender, restime); } //?restime
void get_external_reference(SerialCommands* sender) { echo(sender, restime); } //?external
void get_mode(SerialCommands* sender) { echo(sender, mode); } //?mode

//?temps
void get_temps(SerialCommands* sender) {
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
void get_miner(SerialCommands* sender) {
  String raw_id = String(sender->Next());
  if (raw_id == "") {
    return_ERR(sender, "no argument given");
    return;
  }
  // TODO invalid argument given

  int id = raw_id.toInt();
  if (id < 0 or id >= MAX_MINERS) {
    return_ERR(sender, "invalid ID given");
    return;
  }
  echo(sender, miners[id]);
}

void set_target_temp(SerialCommands* sender) { set_arg(sender, target_temp, ADDR_TARGET_TEMP); } //!targettemp <int>
void set_sensor(SerialCommands* sender) { set_arg(sender, sensor, ADDR_SENSOR); } //!sensor <int>
void set_external_reference(SerialCommands* sender) { set_arg(sender, external_reference, ADDR_EXTERNAL); } //!external <int>
void set_pressure_threshold(SerialCommands* sender) { set_arg(sender, filter_threshold, ADDR_THRESHOLD); } //!threshold <int>
void set_minrpm(SerialCommands* sender) { set_arg(sender, minrpm, ADDR_MINRPM); } //!minrpm <int>
void set_maxrpm(SerialCommands* sender) { set_arg(sender, maxrpm, ADDR_MAXRPM); } //!maxrpm <int>
void set_mode(SerialCommands* sender) { set_arg(sender, mode, ADDR_MODE); } //!mode <int>
void set_ontime(SerialCommands* sender) { set_arg(sender, ontime, ADDR_ONTIME); } //!ontime <int>
void set_offtime(SerialCommands* sender) { set_arg(sender, offtime, ADDR_OFFTIME); } //!offtime <int>
void set_restime(SerialCommands* sender) { set_arg(sender, restime, ADDR_RESTIME); } //!restime <int>
void set_pidp(SerialCommands* sender) { set_arg(sender, pidP, ADDR_PIDP); } //!pidp <int>
void set_pidi(SerialCommands* sender) { set_arg(sender, pidI, ADDR_PIDI); } //!pidi <int>
void set_pidd(SerialCommands* sender) { set_arg(sender, pidD, ADDR_PIDD); } //!pidd <int>
void set_pidb(SerialCommands* sender) { set_arg(sender, pidB, ADDR_PIDB); } //!pidb <int>

//!miner <id> <action>
void set_miner(SerialCommands* sender) {
  String raw_id = String(sender->Next());
  if (raw_id == "") {
    return_ERR(sender, "no argument given");
    return;
  }
  // TODO invalid argument given

  int id = raw_id.toInt();
  String action = String(sender->Next());
  if (action == NULL) {
    return_ERR(sender, "no argument given");
    return;
  } 
  if (action != "on" and action != "off" and action != "register" and action != "deregister") {
    return_ERR(sender, "invalid argument given");
    return;
  }
  if (action == "on" or action =="register") {
    miners[id] = 1;
  } else if (action == "off") {
    miners[id] = 0;
  } else if (action == "deregister") {
    miners[id] = -1;
  }
  set_EEPROM(ADDR_MINERS + id * sizeof(miners[id]), miners[id]);
}

//------------------------------------------------------------------------------

//-------------------//
// SERVICE FUNCTIONS //
//-------------------//

void check_filter_status() {
  filter_ok = pressure < filter_threshold;
}

void init_values() {
  target_temp = set_EEPROM(ADDR_TARGET_TEMP, 50);
  pidP = set_EEPROM(ADDR_PIDP, 1);
  pidI = set_EEPROM(ADDR_PIDI, 3);
  pidD = set_EEPROM(ADDR_PIDD, 4);
  pidB = set_EEPROM(ADDR_PIDB, 2);
  minrpm = set_EEPROM(ADDR_MINRPM, 5);
  maxrpm = set_EEPROM(ADDR_MAXRPM, 80);
  ontime = set_EEPROM(ADDR_ONTIME, 100);
  offtime = set_EEPROM(ADDR_OFFTIME, 85);
  restime = set_EEPROM(ADDR_RESTIME, 125);
  sensor = set_EEPROM(ADDR_SENSOR, 1);
  filter_threshold = set_EEPROM(ADDR_THRESHOLD, 1400);
  external_reference = set_EEPROM(ADDR_EXTERNAL, 55);
  for (int i = 0; i < sizeof(miners)/sizeof(int); ++i) {
    if (random(0, 100) < 3) {
      miners[i] = -1;
    } else if (random(0, 100) < 10) {
      miners[i] = 0;
    } else {
      miners[i] = 1;
    }
    set_EEPROM(ADDR_MINERS + i * sizeof(int), miners[i]);
  }
}

void load_values() {
  target_temp = get_EEPROM(ADDR_TARGET_TEMP);
  pidP = get_EEPROM(ADDR_PIDP);
  pidI = get_EEPROM(ADDR_PIDI);
  pidD = get_EEPROM(ADDR_PIDD);
  pidB = get_EEPROM(ADDR_PIDB);
  minrpm = get_EEPROM(ADDR_MINRPM);
  maxrpm = get_EEPROM(ADDR_MAXRPM);
  ontime = get_EEPROM(ADDR_ONTIME);
  offtime = get_EEPROM(ADDR_OFFTIME);
  restime = get_EEPROM(ADDR_RESTIME);
  sensor = get_EEPROM(ADDR_SENSOR);
  filter_threshold = get_EEPROM(ADDR_THRESHOLD);
  external_reference = get_EEPROM(ADDR_EXTERNAL);
  for (int i = 0; i < sizeof(miners)/sizeof(int); ++i) {
    miners[i] = get_EEPROM(ADDR_MINERS + i * sizeof(int));
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
void main_timer() {
  //mock_changes();

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
SerialCommand cmd_fw("?fw", get_fw_version);
SerialCommand cmd_getpidp("?pidp", get_pidp);
SerialCommand cmd_getpidi("?pidi", get_pidi);
SerialCommand cmd_getpidd("?pidd", get_pidd);
SerialCommand cmd_getpidb("?pidb", get_pidb);
SerialCommand cmd_gettemps("?temps", get_temps);
SerialCommand cmd_gettargettemp("?targettemp", get_target_temp);
SerialCommand cmd_getpress("?pressure", get_pressure);
SerialCommand cmd_getpressthreshold("?threshold", get_pressure_threshold);
SerialCommand cmd_getfilter("?filter", get_filter);
SerialCommand cmd_getsensor("?sensor", get_sensor);
SerialCommand cmd_getminrpm("?minrpm", get_minrpm);
SerialCommand cmd_getmaxrpm("?maxrpm", get_maxrpm);
SerialCommand cmd_getrpm("?rpm", get_rpm);
SerialCommand cmd_getontime("?ontime", get_ontime);
SerialCommand cmd_getofftime("?offtime", get_offtime);
SerialCommand cmd_getrestime("?restime", get_restime);
SerialCommand cmd_getexternal("?external", get_external_reference);
SerialCommand cmd_getmode("?mode", get_mode);
SerialCommand cmd_getminer("?miner", get_miner);
SerialCommand cmd_settargettemp("!targettemp", set_target_temp);
SerialCommand cmd_setexternalreference("!external", set_external_reference);
SerialCommand cmd_setpressurethreshold("!threshold", set_pressure_threshold);
SerialCommand cmd_setmaxrpm("!maxrpm", set_maxrpm);
SerialCommand cmd_setminrpm("!minrpm", set_minrpm);
SerialCommand cmd_setmode("!mode", set_mode);
SerialCommand cmd_setontime("!ontime", set_ontime);
SerialCommand cmd_setofftime("!offtime", set_offtime);
SerialCommand cmd_setrestime("!restime", set_restime);
SerialCommand cmd_setpidp("!pidp", set_pidp);
SerialCommand cmd_setpidi("!pidi", set_pidi);
SerialCommand cmd_setpidd("!pidd", set_pidd);
SerialCommand cmd_setpidb("!pidb", set_pidb);
SerialCommand cmd_setminer("!miner", set_miner);

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
  // set_EEPROM(ADDR_INIT_MARKER, 255);
  if (EEPROM_is_empty()) {
    Serial.println("Initializing values. This may take a minute or two.");
    init_values();
    Serial.println("Initialization complete.");
  } else {
    set_EEPROM(ADDR_INIT_MARKER, 1);
    load_values();
  }
  
  timer.setInterval(1000, main_timer);
  add_serial_commands();
}

void loop() {
  serial_commands_.ReadSerial();
  timer.run();
}
