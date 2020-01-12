enum LogLevel {
  SILENT,
  FATAL,
  ERROR,
  WARNING,
  NOTICE,
  INFO,
  DEBUG
};

class BlynkLogger
{
    protected:
        WidgetTerminal* terminal;
        char logBuffer[1024];
        LogLevel serialLogLevel = LogLevel::DEBUG;
        LogLevel blynklLogLevel = LogLevel::INFO; 
        void _log(char *message, LogLevel level);

    public:
        BlynkLogger(WidgetTerminal *terminal): terminal(terminal){};
        void fatal(char *message);
        void error(char *message);
        void warning(char *message);
        void notice(char *message);
        void info(char *message);
        void debug(char *message);
        void setSerialLogLevel(LogLevel level);
        void setBlynklLogLevel(LogLevel level);
};

void BlynkLogger::fatal(char *message)
{
    _log(message, LogLevel::FATAL);
}

void BlynkLogger::error(char *message)
{
    _log(message, LogLevel::ERROR);
}

void BlynkLogger::warning(char *message)
{
    _log(message, LogLevel::WARNING);
}

void BlynkLogger::notice(char *message)
{
    _log(message, LogLevel::NOTICE);
}

void BlynkLogger::info(char *message)
{
    _log(message, LogLevel::INFO);
}

void BlynkLogger::debug(char *message)
{
    _log(message, LogLevel::DEBUG);
}

void BlynkLogger::setSerialLogLevel(LogLevel level)
{
    serialLogLevel = level;
}

void BlynkLogger::setBlynklLogLevel(LogLevel level)
{
    blynklLogLevel = level;
}


void BlynkLogger::_log(char *message, LogLevel level)
{
    char timeBuf[100];
    bool hasTime  = false;
    time_t now = time(NULL);

    if (now > 28900) {
      struct tm * timeStruct = localtime(&now);
      strftime(timeBuf, 100, "%b %d %T " , timeStruct);
      hasTime = true;
    }
    
    if (serialLogLevel >= level) {
        if (hasTime) {
          Serial.print(timeBuf);
        }
        Serial.println(message);
    }

    if (blynklLogLevel >= level && Blynk.connected() && strlen(logBuffer)) {
        terminal->println(logBuffer);
        terminal->flush();
        strcpy(logBuffer, "");
    }

    if (strlen(logBuffer) + strlen(message) > sizeof(logBuffer)) {
        strcpy(logBuffer, "");
    }

     
    if (blynklLogLevel >= level) {
        if (Blynk.connected()) {
            if (hasTime) {
              terminal->print(timeBuf);
            }
            terminal->println(message);
            terminal->flush();
        } else {
            if (hasTime) {
              strcat(logBuffer, timeBuf);
            }
            strcat(logBuffer, message);
            strcat(logBuffer, "\n");
        }
    } 
}
