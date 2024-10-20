#include "timetable.h"
#include "config.h"
#include "robot.h"
#include "op/op.h"


TimeTable::TimeTable()
{
  autostartNow = false;
  autostopNow = false;
  mowingCompletedInCurrentTimeFrame = false;
  nextCheckTime = 0;
  lastMowingAllowedState = false;
  autostartTriggered = false;
  autostopTriggered = false;
  autostopTime.dayOfWeek = NOT_SET;
  autostartTime.dayOfWeek = NOT_SET;
  timetable.enable = false;

  timetable.hours[0] = 0;
  timetable.hours[1] = 0;
  timetable.hours[2] = 0;
  timetable.hours[3] = 0;
  timetable.hours[4] = 0;
  timetable.hours[5] = 0;
  timetable.hours[6] = 0;
  timetable.hours[7] = 0;
  timetable.hours[8] = 0;
  timetable.hours[9] = 127; // if mowing allowed, mask is set for that day
  timetable.hours[10] = 127;
  timetable.hours[11] = 127;
  timetable.hours[12] = 127;
  timetable.hours[13] = 127;
  timetable.hours[14] = 127;
  timetable.hours[15] = 127;
  timetable.hours[16] = 127;
  timetable.hours[17] = 127;
  timetable.hours[18] = 127;
  timetable.hours[19] = 127;
  timetable.hours[20] = 0;
  timetable.hours[21] = 0;
  timetable.hours[22] = 0;
  timetable.hours[23] = 0;
}

void TimeTable::setMowingCompletedInCurrentTimeFrame(bool completed)
{
  mowingCompletedInCurrentTimeFrame = completed;
}


// set current UTC time
void TimeTable::setCurrentTime(int16_t hour, int16_t min, int16_t dayOfWeek)
{
  currentTime.hour = hour;
  currentTime.min = min;
  currentTime.dayOfWeek = dayOfWeek;
  wxLogMessage("GPS time (UTC): ");
  dumpWeekTime(currentTime);
}

void TimeTable::dumpWeekTime(weektime_t time)
{
  wxLogMessage("dayOfWeek=");
  String s;
  switch (time.dayOfWeek)
    {
    case 0:
      s = "mon";
      break;
    case 1:
      s = "tue";
      break;
    case 2:
      s = "wed";
      break;
    case 3:
      s = "thu";
      break;
    case 4:
      s = "fri";
      break;
    case 5:
      s = "sat";
      break;
    case 6:
      s = "sun";
      break;
    }
  wxLogMessage("%s hour= %i", s.c_str(), time.hour);
}


void TimeTable::dump()
{
  wxString table("timetable (UTC times)    ");
  for (int16_t hour=0; hour < 24; hour++)
    {
      String s;
      if (hour < 10) s += "0";
      s += hour;
      table.Append(s.c_str());
      table.Append(" ");
    }
  wxLogMessage(table);

  for (int16_t day=0; day < 7; day++)
    {
      table.clear();
      table.append("timetable (UTC times) ");
      String s;
      switch (day)
        {
        case 0:
          s = "mon";
          break;
        case 1:
          s = "tue";
          break;
        case 2:
          s = "wed";
          break;
        case 3:
          s = "thu";
          break;
        case 4:
          s = "fri";
          break;
        case 5:
          s = "sat";
          break;
        case 6:
          s = "sun";
          break;
        }
      table.Append(s.c_str());
      int16_t mask = (1 << day);
      for (int16_t hour=0; hour < 24; hour++)
        {
          String s = "   ";
          if (timetable.hours[hour] & mask) s = " * ";
          table.Append(s.c_str());
        }
      wxLogMessage(table);
    }
  wxLogMessage("* means mowing allowed");
  wxLogMessage("current GPS UTC weektime: ");
  dumpWeekTime(currentTime);
  wxLogMessage("timetable enabled: %b", timetable.enable);
  wxLogMessage("mowing allowed: %b", mowingAllowed());
}


void TimeTable::clear()
{
  for (int16_t i=0; i  < 24; i++)
    {
      timetable.hours[i] = 0;
    }
}

int16_t TimeTable::crc()
{
  int16_t crc = 0;
  for (int16_t i=0; i  < 24; i++)
    {
      crc += i * timetable.hours[i];
    }
  crc += ((uint8_t)timetable.enable);
  return crc;
}

// set day mask for hour
bool TimeTable::setDayMask(int16_t hour, daymask_t mask)
{
  if ((hour < 0) || (hour > 23)) return false;
  //wxLogMessage("setDayMask hour=");
  //wxLogMessage(hour);
  //wxLogMessage("  mask=");
  //wxLogMessage(mask);
  timetable.hours[hour] = mask;
  return true;
}

void TimeTable::setEnabled(bool flag)
{
  timetable.enable = flag;
}


bool TimeTable::mowingAllowed(weektime_t time)
{
  if (!timetable.enable) return true; // timetable not enabled => mowing allowed
  int16_t hour = time.hour;
  if ((hour < 0) || (hour > 23)) return false;
  int16_t mask = (1 << time.dayOfWeek);

  bool allowed = ( (timetable.hours[hour] & mask) != 0); // if mowing allowed, mask is set for that day
  return allowed;
}

bool TimeTable::mowingAllowed()
{
  return mowingAllowed(currentTime);
}


bool TimeTable::findAutostopTime(weektime_t &time)
{
  time.dayOfWeek = NOT_SET;
  if (!timetable.enable)
    {
      wxLogMessage("AUTOSTOP: timetable is disabled");
      return false;
    }
  bool autostop = false;
  bool triggered = autostopTriggered;
  weektime_t checktime = currentTime;
  bool checkstate = true;

  // check timetable
  for (int16_t hour =0; hour < 24 * 7; hour++)
    {
      bool allowed = mowingAllowed(checktime);
      if (allowed != checkstate)
        {
          if ( (!allowed ) && (!triggered) )
            {
              // timetable status transition
              wxLogMessage("AUTOSTOP: timetable transition ");
              time = checktime;
              dumpWeekTime(time);
              autostop = true;
              break;
            }
          triggered = false;
          checkstate = allowed;
        }
      // continue to next hour in timetable
      checktime.hour++;
      if (checktime.hour > 23)
        {
          checktime.dayOfWeek++;
          if (checktime.dayOfWeek > 6)
            {
              checktime.dayOfWeek = 0;
            }
          checktime.hour=0;
        }
    }
  if (!autostop) wxLogMessage("AUTOSTOP: no time found");
  return autostop;
}


bool TimeTable::findAutostartTime(weektime_t &time)
{
  time.dayOfWeek = NOT_SET;
  if ( !DOCKING_STATION )
    {
      wxLogMessage("AUTOSTART: not defined DOCKING_STATION");
      return false;
    }
  if (!DOCK_AUTO_START)  // automatic continue mowing allowed?
    {
      wxLogMessage("AUTOSTART: not defined DOCK_AUTO_START");
      return false;
    }
  if ( !battery.isDocked() )   // robot is in dock?
    {
      wxLogMessage("AUTOSTART: not docked automatically (use DOCK command first)");
      return false;
    }
  bool autostart = false;
  weektime_t checktime = currentTime;
  bool checkstate = false;
  bool triggered = autostartTriggered;
  uint32_t waitmillis = millis();

  // check timetable and rain timeouts
  for (int16_t hour =0; hour < 24 * 7; hour++)
    {
      if ( (!dockOp.dockReasonRainTriggered) || (waitmillis > dockOp.dockReasonRainAutoStartTime) )    // raining timeout
        {
          if (!timetable.enable)   // timetable disabled
            {
              if ((!dockOp.initiatedByOperator) && (maps.mowPointsIdx > 0))   // mowing not completed yet
                {
                  wxLogMessage("AUTOSTART: mowing not completed yet ");
                  time = checktime;
                  dumpWeekTime(time);
                  autostart = true;
                  break;
                }
            }
          else       // timetable enabled
            {
              bool allowed = mowingAllowed(checktime);
              if (allowed != checkstate)
                {
                  if ((allowed) && (!triggered))
                    {
                      // timetable status transition
                      wxLogMessage("AUTOSTART: timetable transition ");
                      time = checktime;
                      dumpWeekTime(time);
                      autostart = true;
                      break;
                    }
                  triggered = false;
                  checkstate = allowed;
                }
              if (allowed)    // timetable status
                {
                  if ( (!dockOp.initiatedByOperator) && ((!mowingCompletedInCurrentTimeFrame) || (maps.mowPointsIdx > 0)) )
                    {
                      wxLogMessage("AUTOSTART: timetable state ");
                      time =checktime;
                      dumpWeekTime(time);
                      autostart = true;
                      break;
                    }
                }
            }
        }
      // continue to next hour in timetable
      checktime.hour++;
      waitmillis += 1000 * 60 * 60; // 1 hour
      if (checktime.hour > 23)
        {
          checktime.dayOfWeek++;
          if (checktime.dayOfWeek > 6)
            {
              checktime.dayOfWeek = 0;
            }
          checktime.hour=0;
        }
    }
  if (!autostart) wxLogMessage("AUTOSTART: no time found");
  return autostart;
}

// called from charge operation
bool TimeTable::shouldAutostartNow()
{
  if (autostartNow)
    {
      autostartTriggered = true; // remember trigger
      return true;
    }
  return false;
}


// called from mow operation
bool TimeTable::shouldAutostopNow()
{
  if (autostopNow)
    {
      autostopTriggered = true;  // remember trigger
      return true;
    }
  return false;
}

// called every 30s in robot
void TimeTable::run()
{
  //if (millis() < nextCheckTime) return;
  //nextCheckTime = millis() + 30000;

  // reset triggers on timetable changes
  bool allowed = mowingAllowed(currentTime);
  if (allowed != lastMowingAllowedState)
    {
      lastMowingAllowedState = allowed;
      autostartTriggered = false; // reset trigger
      autostopTriggered = false; // reset trigger
    }

  autostopNow = false;
  if (findAutostopTime(autostopTime))
    {
      if (autostopTime.dayOfWeek == currentTime.dayOfWeek)
        {
          if (autostopTime.hour == currentTime.hour)
            {
              autostopNow = true;
            }
        }
    }

  autostartNow = false;
  if (findAutostartTime(autostartTime))
    {
      if (autostartTime.dayOfWeek == currentTime.dayOfWeek)
        {
          if (autostartTime.hour == currentTime.hour)
            {
              autostartNow = true;
            }
        }
    }
}


// calc dayOfWeek(0=Monday) for given UTC date (untested and not used!)
// not needed as GPS signal gives us millis since start of a week
int16_t TimeTable::calcDayOfWeek(int16_t year, int16_t month, int16_t day)
{
  int16_t a, b, c, d;
  int32_t e;
  a = day;
  b = month;
  c = year;

  for (d = 1, e = (365 * (c - 1)) + (a - 1) + ((c / 4) - (c / 100) + (c / 400)); d < b; d++)
    e += (d == 2) ? (28) : ((d == 4 || d == 6 || d == 9 || d == 11) ? (30) : (31));
  int16_t dayOfWeek = (e % 7);
  return dayOfWeek;
}


