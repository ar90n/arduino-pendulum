#pragma once

#define DEBUG

#ifdef DEBUG
#define PRINT(str) Serial.print(str)
#else
#define PRINT(str)
#endif