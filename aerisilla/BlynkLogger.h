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
    if (serialLogLevel >= level) {
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
            terminal->println(message);
            terminal->flush();
        } else 
            strcat(logBuffer, message);{
        }
    } 
}
