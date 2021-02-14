/**
  ******************************************************************************
  * @file       my_JSON.c
  * @author     Pawel Wojciechowski
  * @version    V.1.0.0
  * @date       10-December-2020
  ******************************************************************************
  */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include "cJSON.h"
#include "my_JSON.h"

enum week {Mon, Tue, Wed, Thur, Fri, Sat, Sun};

/** Get the number of the day by given data string*/
int CalculateDay(char *date){

    int dayNum = -1;

    //printf("\n Date is : %s ", date);
    char ch_year[5] = {0}, ch_month[3] = {0}, ch_day[3] = {0};
    int i_year = 0, i_month = 0, i_day = 0;

    // 2020-12-28
    strncpy(ch_year, date, 4);
    strncpy(ch_month, date+5, 2);
    strncpy(ch_day, date+8, 2);
    //printf("Splitted : Year %s | Month %s | Day %s\n", year, month, day);

    i_year = atoi(ch_year);
    i_month = atoi(ch_month);
    i_day = atoi(ch_day);

    // Now Find the number of the day - Gregorian
    //1. Take the last 2 digits of the year
    char buffer[3] = {0};
    strncpy(buffer, date+2, 2);
    int LastTwoDigits = atoi(buffer);

    int buf = LastTwoDigits;

    //2. Divide by 4, discarding any fraction
    buf /= 4;

    //3. Add the day of the month
    buf += i_day;

    //4. Add the month's key value:
    switch (i_month)
    {
        // January
        case 1:
            buf += 1;
            break;

        //February
        case 2:
            buf += 4;
            break;

        //March
        case 3:
            buf += 4;
            break;

        // April
        case 4:
            buf += 0;
            break;

        //May
        case 5:
            buf += 2;
            break;

        //June
        case 6:
            buf += 5;
            break;

        //July
        case 7:
            buf += 0;
            break;

        //August
        case 8:
            buf += 3;
            break;

        //September
        case 9:
            buf += 6;
            break;

        //October
        case 10:
            buf += 1;
            break;

        //November
        case 11:
            buf += 4;
            break;

        //December
        case 12:
            buf += 6;
            break;

        default:
            printf("Wrong month number!\n");
            break;
    }

    //5. Substract 1 Janury or February of a leap year. Why January?
    if(i_year % 4 == 0 && (i_month == 1 || i_month == 2 )) buf -= 1;

    //6. For a Gregorian date, add 0 for 1900's, 6 for 2000's, 4 for 1700's,
    //  2 for 1800's; for other years, add or subtract multiples of 400.
    buf += 6;

    //7. Add last 2 digits of the year
    buf += LastTwoDigits;

    //8. Divide by 7 and the the reminder
    buf %= 7;

    //9. Parse buf, couse in this algorithm, Sunday is 1 and in our calendar Monday is 0.
    if (buf == 1) dayNum = Sun;
    else if (buf == 2) dayNum = Mon;
    else if (buf == 3) dayNum = Tue;
    else if (buf == 4) dayNum = Wed;
    else if (buf == 5) dayNum = Thur;
    else if (buf == 6) dayNum = Fri;
    else if (buf == 0) dayNum = Sat;

    //printf("dayNum is %d\n", dayNum);
    return dayNum;
}

/** Parsing input calendar */
void UpdateCalendar(char *http_buf, bool Calendar[][HOURS][MINUTES])
{
    const cJSON *calendar       = NULL;
    const cJSON *single_data    = NULL;

    cJSON *string = cJSON_Parse(http_buf);

    char Date[20] = {};
    char Timeslot[20] = {};

    calendar = cJSON_GetObjectItemCaseSensitive(string, "calendar");

    cJSON_ArrayForEach(single_data, calendar)
    {
        cJSON *date     = cJSON_GetObjectItemCaseSensitive(single_data, "date");
        cJSON *timeslot = cJSON_GetObjectItemCaseSensitive(single_data, "timeslot");

        if(!cJSON_IsString(date) || !cJSON_IsString(timeslot)){
            //status = 0;
            goto end;
        }

        if(cJSON_IsString(date) && date->valuestring != NULL){
            strcpy(Date, date->valuestring);
            //printf(Date);
            //printf("\n");
        }

        if(cJSON_IsString(timeslot) && timeslot->valuestring != NULL){
            strcpy(Timeslot, timeslot->valuestring);
            //printf(Timeslot);
            //printf("\n");
        }

        int day = -1;
        int hour_start = -1, hour_stop = -1;
        int minute_start = -1, minute_stop = -1;

        char buf_hour_start[3] = {};
        char buf_hour_stop[3] = {};

        char buf_minute_start[3] = {};
        char buf_minute_stop[3] = {};

        strncpy(buf_hour_start, Timeslot, 2);
        hour_start = atoi(buf_hour_start);

        strncpy(buf_hour_stop, Timeslot+6, 2);
        hour_stop = atoi(buf_hour_stop);

        strncpy(buf_minute_start, Timeslot+3, 2);
        minute_start = atoi(buf_minute_start);

        strncpy(buf_minute_stop, Timeslot+9, 2);
        minute_stop = atoi(buf_minute_stop);

        day = CalculateDay(Date);

        //printf("h_start = %d m_start = %d - h_stop = %d m_stop = %d\n", hour_start, minute_start, hour_stop, minute_stop);

        if(day >= 0 && hour_start >= 0 && hour_start <= 59 && minute_start >= 0 && minute_start <= 59
           && hour_stop >= 0 && hour_stop <= 59 && minute_stop >= 0 && minute_stop <= 59)
           {
            int hour = -1, min = -1;

            hour = hour_start;
            min = minute_start;

            while(1)
            {
                Calendar[day][hour][min] = 1;
                //printf("update\n");
                min++;

                if(min >= 60){
                    min = 0;
                    hour++;
                }

                if(hour >= 24){
                    hour = 0;
                    day++;
                }

                if(day > Sun){
                    day = Mon;
                }

                if(hour == hour_stop && min == minute_stop){
                    //printf("end\n");
                    break;
                }
            }
        }
    }

    end:
        cJSON_Delete(string);
        return;
}

/** Parsing info about clicked switch */
bool SwitchInfo(char *http_buf)
{
    const cJSON *SwitchStatus       = NULL;

    cJSON *string = cJSON_Parse(http_buf);

    bool switchStatus = false;

    SwitchStatus = cJSON_GetObjectItemCaseSensitive(string, "SwitchStatus");
    if(!cJSON_IsBool(SwitchStatus)) goto end;
    else{
        switchStatus = SwitchStatus->valueint;
    }

    end:
        cJSON_Delete(string);

    return switchStatus;
}
