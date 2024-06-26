#include "Logger.h"

#if defined(ARDUINO_ARCH_AVR)
  #include <avr/pgmspace.h>
#endif

// There appears to be an incompatibility with ESP8266 2.3.0.
#if defined(ESP8266)
  #define MEM_TYPE
#else
  #define MEM_TYPE PROGMEM
#endif

const char LEVEL_VERBOSE[] MEM_TYPE = "VERBOSE";
const char LEVEL_NOTICE[] MEM_TYPE = "NOTICE";
const char LEVEL_WARNING[] MEM_TYPE = "WARNING";
const char LEVEL_ERROR[] MEM_TYPE = "ERROR";
const char LEVEL_FATAL[] MEM_TYPE = "FATAL";
const char LEVEL_SILENT[] MEM_TYPE = "SILENT";

const char* const LOG_LEVEL_STRINGS[] MEM_TYPE =
{
    LEVEL_VERBOSE,
    LEVEL_NOTICE,
    LEVEL_WARNING,
    LEVEL_ERROR,
    LEVEL_FATAL,
    LEVEL_SILENT
};


Logger::Logger():
    _level(WARNING),
    _loggerOutputFunction(0)
{
}


void Logger::setLogLevel(Level level)
{
    getInstance()._level = level;
}


Logger::Level Logger::getLogLevel()
{
    return getInstance()._level;
}


void Logger::verbose(const char* message)
{
    log(VERBOSE, message);
}

void Logger::verbose(const String message)
{
    log(VERBOSE, message);
}

void Logger::notice(const char* message)
{
    log(NOTICE, message);
}

void Logger::notice(const String message)
{
    log(NOTICE, message);
}


void Logger::warning(const char* message)
{
    log(WARNING, message);
}

void Logger::warning(const String message)
{
    log(WARNING, message);
}


void Logger::error(const char* message)
{
    log(ERROR, message);
}

void Logger::error(const String message)
{
    log(ERROR, message);
}


void Logger::fatal(const char* message)
{
    log(FATAL, message);
}

void Logger::fatal(const String message)
{
    log(FATAL, message);
}


void Logger::verbose(const char* module, const char* message)
{
    log(VERBOSE, module, message);
}

void Logger::verbose(const String module, const String message)
{
    log(VERBOSE, module, message);
}


void Logger::notice(const char* module, const char* message)
{
    log(NOTICE, module, message);
}

void Logger::notice(const String module, const String message)
{
    log(NOTICE, module, message);
}


void Logger::warning(const char* module, const char* message)
{
    log(WARNING, module, message);
}

void Logger::warning(const String module, const String message)
{
    log(WARNING, module, message);
}


void Logger::error(const char* module, const char* message)
{
    log(ERROR, module, message);
}

void Logger::error(const String module, const String message)
{
    log(ERROR, module, message);
}


void Logger::fatal(const char* module, const char* message)
{
    log(FATAL, module, message);
}

void Logger::fatal(const String module, const String message)
{
    log(FATAL, module, message);
}


void Logger::log(Level level, const char* message)
{
    log(level, "", message);
}

void Logger::log(Level level, const String message)
{
    log(level, "", message);
}


void Logger::log(Level level, const char* module, const char* message)
{
    if (level >= getLogLevel())
    {
        if (getInstance()._loggerOutputFunction)
        {
            getInstance()._loggerOutputFunction(level, module, message);
        }
        else
        {
            getInstance().defaultLog(level, module, message);
        }
    }
}

void Logger::log(Level level, const String module, const String message)
{
	log(level, module.c_str(), message.c_str());
}


void Logger::setOutputFunction(LoggerOutputFunction loggerOutputFunction)
{
    getInstance()._loggerOutputFunction = loggerOutputFunction;
}


Logger& Logger::getInstance()
{
    static Logger logger;
    return logger;
}


const char* Logger::asString(Level level)
{
    return LOG_LEVEL_STRINGS[level];
}


void Logger::defaultLog(Level level, const char* module, const char* message)
{
    Serial.print(F("["));

    Serial.print(asString(level));

    Serial.print(F("] "));

    if (strlen(module) > 0)
    {
        Serial.print(F(": "));
        Serial.print(module);
        Serial.print(F(" "));
    }

    Serial.println(message);
}


// typedef Logger_<SerialOutput> SerialLogger;
