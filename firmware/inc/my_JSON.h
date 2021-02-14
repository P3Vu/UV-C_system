/**
  ******************************************************************************
  * @file       main.c
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       10-December-2020
  ******************************************************************************
  */


#include <stdbool.h>

#define DAYS  7
#define HOURS  24
#define MINUTES 60

void testData(void);
void UpdateCalendar(char *http_buf, bool Calendar[DAYS][HOURS][MINUTES]);
bool SwitchInfo(char *http_buf);
