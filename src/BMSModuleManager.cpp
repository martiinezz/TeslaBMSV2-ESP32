#include "config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"

extern EEPROMSettings settings;

BMSModuleManager::BMSModuleManager()
{
  for (int i = 1; i <= MAX_MODULE_ADDR; i++) {
    modules[i].setExists(false);
    modules[i].setAddress(i);
  }
  lowestPackVolt = 1000.0f;
  highestPackVolt = 0.0f;
  lowestPackTemp = 200.0f;
  highestPackTemp = -100.0f;
  isFaulted = false;
}

void BMSModuleManager::clearmodules()
{
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      modules[y].clearmodule();
    }
  }
}

int BMSModuleManager::seriescells()
{
  spack = 0;
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      spack = spack + modules[y].getscells();
    }
  }
  return spack;
}

void BMSModuleManager::balanceCells(int duration, int debug)
{
  uint8_t payload[4];
  uint8_t buff[30];
  uint8_t balance = 0;//bit 0 - 5 are to activate cell balancing 1-6
  CellsBalancing = 0;
  if (debug == 1)
  {
    Serial.println();
  }
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting() == 1)
    {
      balance = 0;
      for (int i = 0; i < 6; i++)
      {
        if (getLowCellVolt() < modules[y].getCellVoltage(i))
        {
          balance = balance | (1 << i);
        }
      }
      if (debug == 1)
      {
        Serial.print(y);
        Serial.print(" - ");
        Serial.print(balance, BIN);
        Serial.print(" | ");
      }
      if (balance != 0) //only send balance command when needed
      {
        payload[0] = y << 1;
        payload[1] = REG_BAL_TIME;
        payload[2] = duration; //5 second balance limit, if not triggered to balance it will stop after 5 seconds
        BMSUtil::sendData(payload, 3, true);
        delay(2);
        BMSUtil::getReply(buff, 30);
        if (debug == 1)
        {
          for (int z = 0; z < 4; z++)
          {
            Serial.print(buff[z], HEX);
            Serial.print("-");
          }
          Serial.print(" | ");
        }
        payload[0] = y << 1;
        payload[1] = REG_BAL_CTRL;
        payload[2] = balance; //write balance state to register
        BMSUtil::sendData(payload, 3, true);
        delay(2);
        BMSUtil::getReply(buff, 30);
        if (debug == 1)
        {
          for (int z = 0; z < 4; z++)
          {
            Serial.print(buff[z], HEX);
            Serial.print("-");
          }
        }
        CellsBalancing = CellsBalancing + balance;
      }
      if (debug == 1)
      {
        Serial.println();
      }
    }
  }
}

/*
   Try to set up any unitialized boards. Send a command to address 0 and see if there is a response. If there is then there is
   still at least one unitialized board. Go ahead and give it the first ID not registered as already taken.
   If we send a command to address 0 and no one responds then every board is inialized and this routine stops.
   Don't run this routine until after the boards have already been enumerated.\
   Note: The 0x80 conversion it is looking might in theory block the message from being forwarded so it might be required
   To do all of this differently. Try with multiple boards. The alternative method would be to try to set the next unused
   address and see if any boards respond back saying that they set the address.
*/
void BMSModuleManager::setupBoards()
{
  uint8_t payload[3];
  uint8_t buff[10];
  int retLen;

  payload[0] = 0;
  payload[1] = 0;
  payload[2] = 1;

  while (1 == 1)
  {
    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 1;
    retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
    if (retLen == 4)
    {
      if (buff[0] == 0x80 && buff[1] == 0 && buff[2] == 1)
      {
        Logger::debug("00 found");
        //look for a free address to use
        for (int y = 1; y < 63; y++)
        {
          if (!modules[y].isExisting())
          {
            payload[0] = 0;
            payload[1] = REG_ADDR_CTRL;
            payload[2] = y | 0x80;
            BMSUtil::sendData(payload, 3, true);
            delay(3);
            if (BMSUtil::getReply(buff, 10) > 2)
            {
              if (buff[0] == (0x81) && buff[1] == REG_ADDR_CTRL && buff[2] == (y + 0x80))
              {
                modules[y].setExists(true);
                numFoundModules++;
                Logger::debug("Address assigned");
              }
            }
            break; //quit the for loop
          }
        }
      }
      else break; //nobody responded properly to the zero address so our work here is done.
    }
    else break;
  }
}

/*
   Iterate through all 62 possible board addresses (1-62) to see if they respond
*/
void BMSModuleManager::findBoards()
{
  uint8_t payload[3];
  uint8_t buff[8];

  numFoundModules = 0;
  payload[0] = 0;
  payload[1] = 0; //read registers starting at 0
  payload[2] = 1; //read one byte
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    modules[x].setExists(false);
    payload[0] = x << 1;
    BMSUtil::sendData(payload, 3, false);
    delay(20);
    if (BMSUtil::getReply(buff, 8) > 4)
    {
      if (buff[0] == (x << 1) && buff[1] == 0 && buff[2] == 1 && buff[4] > 0) {
        modules[x].setExists(true);
        numFoundModules++;
      }
    }
    delay(5);
  }
}


/*
   Force all modules to reset back to address 0 then set them all up in order so that the first module
   in line from the master board is 1, the second one 2, and so on.
*/
void BMSModuleManager::renumberBoardIDs()
{
  uint8_t payload[3];
  uint8_t buff[8];
  int attempts = 1;

  for (int y = 1; y < 63; y++)
  {
    modules[y].setExists(false);
    numFoundModules = 0;
  }

  while (attempts < 3)
  {
    payload[0] = 0x3F << 1; //broadcast the reset command
    payload[1] = 0x3C;//reset
    payload[2] = 0xA5;//data to cause a reset
    BMSUtil::sendData(payload, 3, true);
    delay(100);
    BMSUtil::getReply(buff, 8);
    if (buff[0] == 0x7F && buff[1] == 0x3C && buff[2] == 0xA5 && buff[3] == 0x57) break;
    attempts++;
  }

  setupBoards();
}

/*
  After a RESET boards have their faults written due to the hard restart or first time power up, this clears thier faults
*/
void BMSModuleManager::clearFaults()
{
  uint8_t payload[3];
  uint8_t buff[8];
  payload[0] = 0x7F; //broadcast
  payload[1] = REG_ALERT_STATUS;//Alert Status
  payload[2] = 0xFF;//data to cause a reset
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  payload[0] = 0x7F; //broadcast
  payload[2] = 0x00;//data to clear
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  payload[0] = 0x7F; //broadcast
  payload[1] = REG_FAULT_STATUS;//Fault Status
  payload[2] = 0xFF;//data to cause a reset
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  payload[0] = 0x7F; //broadcast
  payload[2] = 0x00;//data to clear
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

  isFaulted = false;
}

/*
  Puts all boards on the bus into a Sleep state, very good to use when the vehicle is a rest state.
  Pulling the boards out of sleep only to check voltage decay and temperature when the contactors are open.
*/

void BMSModuleManager::sleepBoards()
{
  uint8_t payload[3];
  uint8_t buff[8];
  payload[0] = 0x7F; //broadcast
  payload[1] = REG_IO_CTRL;//IO ctrl start
  payload[2] = 0x04;//write sleep bit
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);
}

/*
  Wakes all the boards up and clears thier SLEEP state bit in the Alert Status Registery
*/

void BMSModuleManager::wakeBoards()
{
  uint8_t payload[3];
  uint8_t buff[8];
  payload[0] = 0x7F; //broadcast
  payload[1] = REG_IO_CTRL;//IO ctrl start
  payload[2] = 0x00;//write sleep bit
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);

  payload[0] = 0x7F; //broadcast
  payload[1] = REG_ALERT_STATUS;//Fault Status
  payload[2] = 0x04;//data to cause a reset
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);
  payload[0] = 0x7F; //broadcast
  payload[2] = 0x00;//data to clear
  BMSUtil::sendData(payload, 3, true);
  delay(2);
  BMSUtil::getReply(buff, 8);
}

void BMSModuleManager::StopBalancing()
{
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      modules[x].stopBalance();
    }
  }
}

void BMSModuleManager::getAllVoltTemp()
{
  packVolt = 0.0f;
  //lowTemp = 999.0f;
  //highTemp = -999.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      modules[x].stopBalance();
    }
  }

  if (numFoundModules < 8)
  {
    delay(200);
  }
  else
  {
    delay(50);
  }
  /*
    delay(200);
  */
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      Logger::debug("");
      Logger::debug("Module %i exists. Reading voltage and temperature values", x);
      modules[x].readModuleValues();
      Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
      Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
      Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));
      packVolt += modules[x].getModuleVoltage();
      if (modules[x].getLowTemp() < lowestPackTemp && modules[x].getLowTemp() > -70) lowestPackTemp = modules[x].getLowTemp();
      if (modules[x].getHighTemp() > highestPackTemp && modules[x].getLowTemp() > -70) highestPackTemp = modules[x].getHighTemp();
    }
  }

  packVolt = packVolt / Pstring;
  if (packVolt > highestPackVolt) highestPackVolt = packVolt;
  if (packVolt < lowestPackVolt) lowestPackVolt = packVolt;

  if (digitalRead(11) == LOW) {
    if (!isFaulted) Logger::error("One or more BMS modules have entered the fault state!");
    isFaulted = true;
  }
  else
  {
    if (isFaulted) Logger::info("All modules have exited a faulted state");
    isFaulted = false;
  }
  HighCellVolt = 0.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getHighCellV() >  HighCellVolt)  HighCellVolt = modules[x].getHighCellV();
    }
  }
  LowCellVolt = 5.0;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getLowCellV() <  LowCellVolt)  LowCellVolt = modules[x].getLowCellV();
    }
  }
}

int BMSModuleManager::getBalancing()
{
  return CellsBalancing;
}

float BMSModuleManager::getLowCellVolt()
{
  return LowCellVolt;
}

int BMSModuleManager::getNumModules()
{
  return numFoundModules;
}

float BMSModuleManager::getHighCellVolt()
{
  return HighCellVolt;
}

float BMSModuleManager::getPackVoltage()
{
  return packVolt;
}

float BMSModuleManager::getLowVoltage()
{
  return lowestPackVolt;
}

float BMSModuleManager::getHighVoltage()
{
  return highestPackVolt;
}

void BMSModuleManager::setBatteryID(int id)
{
  batteryID = id;
}

void BMSModuleManager::setPstrings(int Pstrings)
{
  Pstring = Pstrings;
}

void BMSModuleManager::setSensors(int sensor, float Ignore)
{
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      modules[x].settempsensor(sensor);
      modules[x].setIgnoreCell(Ignore);
    }
  }
}

float BMSModuleManager::getAvgTemperature()
{
  float avg = 0.0f;
  lowTemp = 999.0f;
  highTemp = -999.0f;
  int y = 0; //counter for modules below -70 (no sensors connected)
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting())
    {
      if (modules[x].getAvgTemp() > -70)
      {
        avg += modules[x].getAvgTemp();
        if (modules[x].getHighTemp() > highTemp)
        {
          highTemp = modules[x].getHighTemp();
        }
        if (modules[x].getLowTemp() < lowTemp)
        {
          lowTemp = modules[x].getLowTemp();
        }
      }
      else
      {
        y++;
      }
    }
  }
  avg = avg / (float)(numFoundModules - y);

  return avg;
}

float BMSModuleManager::getHighTemperature()
{
  return highTemp;
}

float BMSModuleManager::getLowTemperature()
{
  return lowTemp;
}

float BMSModuleManager::getAvgCellVolt()
{
  float avg = 0.0f;
  for (int x = 1; x <= MAX_MODULE_ADDR; x++)
  {
    if (modules[x].isExisting()) avg += modules[x].getAverageV();
  }
  avg = avg / (float)numFoundModules;

  return avg;
}

void BMSModuleManager::printPackSummary()
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;

  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i    Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules,
                  getPackVoltage(), getAvgCellVolt(), getAvgTemperature());
  Logger::console("");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      Logger::console("                               Module #%i", y);

      Logger::console("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(),
                      modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());
      if (faults > 0)
      {
        Logger::console("  MODULE IS FAULTED:");
        if (faults & 1)
        {
          SERIALCONSOLE.print("    Overvoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 6; i++)
          {
            if (COV & (1 << i))
            {
              SERIALCONSOLE.print(i + 1);
              SERIALCONSOLE.print(" ");
            }
          }
          SERIALCONSOLE.println();
        }
        if (faults & 2)
        {
          SERIALCONSOLE.print("    Undervoltage Cell Numbers (1-6): ");
          for (int i = 0; i < 6; i++)
          {
            if (CUV & (1 << i))
            {
              SERIALCONSOLE.print(i + 1);
              SERIALCONSOLE.print(" ");
            }
          }
          SERIALCONSOLE.println();
        }
        if (faults & 4)
        {
          Logger::console("    CRC error in received packet");
        }
        if (faults & 8)
        {
          Logger::console("    Power on reset has occurred");
        }
        if (faults & 0x10)
        {
          Logger::console("    Test fault active");
        }
        if (faults & 0x20)
        {
          Logger::console("    Internal registers inconsistent");
        }
      }
      if (alerts > 0)
      {
        Logger::console("  MODULE HAS ALERTS:");
        if (alerts & 1)
        {
          Logger::console("    Over temperature on TS1");
        }
        if (alerts & 2)
        {
          Logger::console("    Over temperature on TS2");
        }
        if (alerts & 4)
        {
          Logger::console("    Sleep mode active");
        }
        if (alerts & 8)
        {
          Logger::console("    Thermal shutdown active");
        }
        if (alerts & 0x10)
        {
          Logger::console("    Test Alert");
        }
        if (alerts & 0x20)
        {
          Logger::console("    OTP EPROM Uncorrectable Error");
        }
        if (alerts & 0x40)
        {
          Logger::console("    GROUP3 Regs Invalid");
        }
        if (alerts & 0x80)
        {
          Logger::console("    Address not registered");
        }
      }
      if (faults > 0 || alerts > 0) SERIALCONSOLE.println();
    }
  }
}

void BMSModuleManager::printPackDetails(int digits)
{
  uint8_t faults;
  uint8_t alerts;
  uint8_t COV;
  uint8_t CUV;
  int cellNum = 0;

  Logger::console("");
  Logger::console("");
  Logger::console("");
  Logger::console("Modules: %i Cells: %i Strings: %i  Voltage: %fV   Avg Cell Voltage: %fV  Low Cell Voltage: %fV   High Cell Voltage: %fV Delta Voltage: %zmV   Avg Temp: %fC ", numFoundModules, seriescells(),
                  Pstring, getPackVoltage(), getAvgCellVolt(), LowCellVolt, HighCellVolt, (HighCellVolt - LowCellVolt) * 1000, getAvgTemperature());
  Logger::console("");
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      faults = modules[y].getFaults();
      alerts = modules[y].getAlerts();
      COV = modules[y].getCOVCells();
      CUV = modules[y].getCUVCells();

      SERIALCONSOLE.print("Module #");
      SERIALCONSOLE.print(y);
      if (y < 10) SERIALCONSOLE.print(" ");
      SERIALCONSOLE.print("  ");
      SERIALCONSOLE.print(modules[y].getModuleVoltage(), digits);
      SERIALCONSOLE.print("V");
      for (int i = 0; i < 6; i++)
      {
        if (cellNum < 10) SERIALCONSOLE.print(" ");
        SERIALCONSOLE.print("  Cell");
        SERIALCONSOLE.print(cellNum++);
        SERIALCONSOLE.print(": ");
        SERIALCONSOLE.print(modules[y].getCellVoltage(i), digits);
        SERIALCONSOLE.print("V");
      }
      SERIALCONSOLE.print("  Neg Term Temp: ");
      SERIALCONSOLE.print(modules[y].getTemperature(0));
      SERIALCONSOLE.print("C  Pos Term Temp: ");
      SERIALCONSOLE.print(modules[y].getTemperature(1));
      SERIALCONSOLE.println("C");

    }
  }
}

String BMSModuleManager::htmlPackDetails(float current, int SOC)
{
    uint8_t faults;
    uint8_t alerts;
    uint8_t COV;
    uint8_t CUV;
    int cellNum = 0;

    String ptr = "<!DOCTYPE html> <head> <title> ESP32 SimpBMS like</title> <meta http-equiv=\"refresh\" content=\"8\"> </head> <html>\n";

    ptr += "   <h2>Modules: " + String(numFoundModules);
    ptr += "   <br>Voltage: " + String(getPackVoltage());
    ptr += "   <br>SOC: " + String(SOC) +"%";
    ptr += "   <br><b>Current: </b>" + String(current) + " A";
    ptr += "   <br>Avg Cell Voltage: " + String( getAvgCellVolt());
    ptr += "   <br>Low Cell Voltage: " + String(LowCellVolt);
    ptr += "   <br>High Cell Voltage: " + String(HighCellVolt);
    ptr += "   <br>Delta: " + String((HighCellVolt-LowCellVolt)*1000) + " mV";
    ptr += "   <br>Avg Temp: " + String(getAvgTemperature());
    ptr += "   </h2> <h3>";

    bool isModuleColor1 = true; // switch between two colors
    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting())
        {
            faults = modules[y].getFaults();
            alerts = modules[y].getAlerts();
            COV = modules[y].getCOVCells();
            CUV = modules[y].getCUVCells();

            if (isModuleColor1) {
                ptr += " <br> <span style=\"background-color: #f5ad42\">"; // first color
            }
            else {
                ptr += " <br> <span style=\"background-color: #42e0f5\">"; // second color
            }
            ptr += "<br>Module #" + String(y < 10 ? "0" : "") + String(y) + " ";
            ptr += String(modules[y].getModuleVoltage()) + "V";

            for (int i = 0; i < 12; i++)
            {
                ptr += "   Cell";
                ptr += String(cellNum < 10 ? "0" : "") + String(cellNum++) + ": ";
                ptr += String(modules[y].getCellVoltage(i)) + "V  ";

            }
            ptr += "    Temp 1: ";
            ptr += String(modules[y].getTemperature(0));
            ptr += "    Temp 2: ";
            ptr += String(modules[y].getTemperature(1));

            isModuleColor1 = !isModuleColor1; // toggle color
        }
    }
    ptr += "</h3</html>";
    return ptr;
}


void BMSModuleManager::printAllCSV(unsigned long timestamp, float current, int SOC, int delim)
{
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      SERIALCONSOLE.print(timestamp);
      if (delim == 1)
      {
        SERIALCONSOLE.print(" ");
      }
      else
      {
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(current, 0);
      if (delim == 1)       {
        SERIALCONSOLE.print(" ");
      }       else       {
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(SOC);
      if (delim == 1)       {
        SERIALCONSOLE.print(" ");
      }       else       {
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(y);
      if (delim == 1)       {
        SERIALCONSOLE.print(" ");
      }       else       {
        SERIALCONSOLE.print(",");
      }
      for (int i = 0; i < 8; i++)
      {
        SERIALCONSOLE.print(modules[y].getCellVoltage(i));
        if (delim == 1)       {
          SERIALCONSOLE.print(" ");
        }       else       {
          SERIALCONSOLE.print(",");
        }
      }
      SERIALCONSOLE.print(modules[y].getTemperature(0));
      if (delim == 1)       {
        SERIALCONSOLE.print(" ");
      }       else       {
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(modules[y].getTemperature(1));
      if (delim == 1)       {
        SERIALCONSOLE.print(" ");
      }       else       {
        SERIALCONSOLE.print(",");
      }
      SERIALCONSOLE.print(modules[y].getTemperature(2));
      SERIALCONSOLE.println();
    }
  }
  for (int y = 1; y < 63; y++)
  {
    if (modules[y].isExisting())
    {
      Serial2.print(timestamp);
      if (delim == 1)
      {
        Serial2.print(" ");
      }
      else
      {
        Serial2.print(",");
      }
      Serial2.print(current, 0);
      if (delim == 1)       {
        Serial2.print(" ");
      }       else       {
        Serial2.print(",");
      }
      Serial2.print(SOC);
      if (delim == 1)       {
        Serial2.print(" ");
      }       else       {
        Serial2.print(",");
      }
      Serial2.print(y);
      if (delim == 1)       {
        Serial2.print(" ");
      }       else       {
        Serial2.print(",");
      }
      for (int i = 0; i < 8; i++)
      {
        Serial2.print(modules[y].getCellVoltage(i));
        if (delim == 1)       {
          Serial2.print(" ");
        }       else       {
          Serial2.print(",");
        }
      }
      Serial2.print(modules[y].getTemperature(0));
      if (delim == 1)       {
        Serial2.print(" ");
      }       else       {
        Serial2.print(",");
      }
      Serial2.print(modules[y].getTemperature(1));
      if (delim == 1)       {
        Serial2.print(" ");
      }       else       {
        Serial2.print(",");
      }
      Serial2.print(modules[y].getTemperature(2));
      Serial2.println();
    }
  }
}

uint16_t BMSModuleManager::getcellvolt(int modid, int cellid)
{
  return (modules[modid].getCellVoltage(cellid) * 1000);
}

uint16_t BMSModuleManager::gettemp(int modid, int tempid)
{
  return (modules[modid].getTemperature(tempid));
}
