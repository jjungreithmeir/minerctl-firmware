#include <SimpleTimer.h>    // https://github.com/marcelloromani/Arduino-SimpleTimer
#include <SerialCommands.h> // https://github.com/ppedro74/Arduino-SerialCommands/
#include <EEPROM.h>

String fwVersion = "0.1";

char serial_command_buffer_[64];

SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\n", " ");

SimpleTimer timer;
//------------------------------------------------------------------------------

//------------------//
// GLOBAL VARIABLES //
//------------------//
int target_temp = 0;
int pidP = 0;
int pidI = 0;
int PidD = 0;
int PidB = 0;
int minrpm = 0;
int maxrpm = 0;
int rpm = 0;
int ontime = 0;
int offtime = 0;
int restime = 0;
int sensor = 0;                     //which sensor to use for the PID controller (0...2; 3 means external temperature)
int filter_threshold = 0;           //value in mBar
int sensorTemps[] = {0, 0, 0, 0};   //values in °C
int pressure = 0;                   //value in mBar of pressure difference

//------------------------------------------------------------------------------

//------------------//
// HELPER FUNCTIONS //
//------------------//
void returnOK(SerialCommands* sender)
{
  sender->GetSerial()->println("OK");
}

void returnOK(SerialCommands* sender, String msg)
{
  sender->GetSerial()->println("OK - " + msg);
}

void returnERR(SerialCommands* sender)
{
  sender->GetSerial()->println("ERR");
}

void returnERR(SerialCommands* sender, String msg)
{
  sender->GetSerial()->println("ERR - " + msg);
}

//------------------------------------------------------------------------------

//----------------------------//
// USER INTERACTION FUNCTIONS //
//----------------------------//
//?fw
void printFWVersion(SerialCommands* sender)
{
  sender->GetSerial()->println(fwVersion);
}

//?pidp
void getPidP(SerialCommands* sender)
{
  
}

//?pidi
void getPidI(SerialCommands* sender)
{
  
}

//?pidD
void getPidD(SerialCommands* sender)
{
  
}

//?pidb
void getPidB(SerialCommands* sender)
{
  
}

//?temps
void getTemps(SerialCommands* sender)
{
  
}

//?tartemp
void getTargetTemperature(SerialCommands* sender)
{

}

//?press
void getPress(SerialCommands* sender)
{
  
}

//?filter
void getFilter(SerialCommands* sender)
{
  
}

//?sensor
void getSensor(SerialCommands* sender)
{
  
}

//?minrpm
void getMinRPM(SerialCommands* sender)
{
  
}

//?maxrpm
void getMaxRPM(SerialCommands* sender)
{
  
}

//?rpm
void getRPM(SerialCommands* sender)
{
  
}

//?ontime
void getOnTime(SerialCommands* sender)
{
  
}

//?offtime
void getOffTime(SerialCommands* sender)
{
  
}

//?restime
void getResTime(SerialCommands* sender)
{
  
}
//------------------------------------------------------------------------------

//-----------------//
// TIMER FUNCTIONS //
//-----------------//
void dummyTimer()
{
  
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
SerialCommand cmd_gettargettemp("?tartemp", getTargetTemperature);
SerialCommand cmd_getpress("?press", getPress);
SerialCommand cmd_getfilter("?filter", getFilter);
SerialCommand cmd_getsensor("?sensor", getSensor);
SerialCommand cmd_getminrpm("?minrpm", getMinRPM);
SerialCommand cmd_getmaxrpm("?maxrpm", getMaxRPM);
SerialCommand cmd_getrpm("?rpm", getRPM);
SerialCommand cmd_getontime("?ontime", getOnTime);
SerialCommand cmd_getofftime("?offtime", getOffTime);
SerialCommand cmd_getrestime("?restime", getResTime);

void add_serial_commands()
{
  serial_commands_.AddCommand(&cmd_fw);
  serial_commands_.AddCommand(&cmd_getpidp);
  serial_commands_.AddCommand(&cmd_getpidi);
  serial_commands_.AddCommand(&cmd_getpidd);
  serial_commands_.AddCommand(&cmd_getpidb);
  serial_commands_.AddCommand(&cmd_gettemps);
  serial_commands_.AddCommand(&cmd_gettargettemp);
  serial_commands_.AddCommand(&cmd_getpress);
  serial_commands_.AddCommand(&cmd_getfilter);
  serial_commands_.AddCommand(&cmd_getsensor);
  serial_commands_.AddCommand(&cmd_getminrpm);
  serial_commands_.AddCommand(&cmd_getmaxrpm);
  serial_commands_.AddCommand(&cmd_getrpm);
  serial_commands_.AddCommand(&cmd_getontime);
  serial_commands_.AddCommand(&cmd_getofftime);
  serial_commands_.AddCommand(&cmd_getrestime);
}

//------------------------------------------------------------------------------

//--------------------------//
// SETUP CODE AND MAIN LOOP //
//--------------------------//
void setup() {
  Serial.begin(9600);

  timer.setInterval(1000, dummyTimer);
  
  add_serial_commands();
}

void loop() {
  serial_commands_.ReadSerial();
  timer.run();
}
