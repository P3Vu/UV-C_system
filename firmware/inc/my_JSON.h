/**
  ******************************************************************************
  * @file       main.c
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       10-December-2020
  ******************************************************************************
  */


#include <stdbool.h>

#define number_of_days  7
#define number_of_time_sections  24

void testData(void);
void UpdateCalendar(char *http_buf, bool Calendar[number_of_days][number_of_time_sections], int ID);
