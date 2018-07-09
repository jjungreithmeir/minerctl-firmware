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
String fwVersion = "0.1";
int target_temp = 0;
int pidP = 0;
int pidI = 0;
int pidD = 0;
int pidB = 0;
int minrpm = 0;
int maxrpm = 0;
int rpm = 0;
int ontime = 0;
int offtime = 0;
int restime = 0;
int sensor = 0;                     //which sensor to use for the PID controller (0...2; 3 means external temperature)
int filter_threshold = 0;           //value in mBar
int sensor_temps[] = {0, 0, 0, 0};  //values in °C
int pressure = 0;                   //value in mBar of pressure difference
int external_reference = 0;
int mode = 0;                       //0 = gpu, 1 = asic
const int MAX_MINERS = 120;
int miners[MAX_MINERS];             //-1 = disabled/not present, 0 = off, 1 = on
bool filter_ok = true;
bool status = true;
//------------------------------------------------------------------------------

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

void setArgument(SerialCommands* sender, int& injectedVar) {
  char* temp = sender->Next();
  if (!check_param(sender, temp)) {
    return;
  }
  injectedVar = atoi(temp);
}

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
  if (id < 0 or id >= MAX_MINERS)
  {
    returnERR(sender, "invalid ID given");
    return;
  }
  echo(sender, miners[id]);
}

void setTargetTemperature(SerialCommands* sender) { setArgument(sender, target_temp); } //!targettemp
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
  target_temp = 50;
  pidP = 2;
  pidI = 3;
  pidD = 4;
  pidB = 1;
  minrpm = 5;
  maxrpm = 75;
  ontime = 100;
  offtime = 85;
  restime = 250;
  sensor = 1;
  filter_threshold = 1400;
  external_reference = 55;
  for (int i = 0; i < sizeof(miners)/sizeof(int); ++i) {
    if (random(0, 100) < 3) {
      miners[i] = -1;
    } else if (random(0, 100) < 10) {
      miners[i] = 0;
    } else {
      miners[i] = 1;
    }
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

void add_serial_commands()
{
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

  timer.setInterval(1000, mainTimer);
  
  add_serial_commands();
  init_values();
}

void loop() {
  serial_commands_.ReadSerial();
  timer.run();
}
