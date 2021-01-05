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
void UpdateCalendar(char *http_buf, bool Calendar[][number_of_time_sections], int device_ID)
{
    const cJSON *calendar       = NULL;
    const cJSON *single_data    = NULL;

    cJSON *string = cJSON_Parse(http_buf);

    int ID_param = 0;
    char Date[20] = {};
    char Timeslot[20] = {};

    //printf("pre ID_param = %d\n", ID_param);

    calendar = cJSON_GetObjectItemCaseSensitive(string, "calendar");
    cJSON_ArrayForEach(single_data, calendar)
    {
        cJSON *id       = cJSON_GetObjectItemCaseSensitive(single_data, "room_id");
        cJSON *date     = cJSON_GetObjectItemCaseSensitive(single_data, "date");
        cJSON *timeslot = cJSON_GetObjectItemCaseSensitive(single_data, "timeslot");

        if(!cJSON_IsNumber(id) || !cJSON_IsString(date) || !cJSON_IsString(timeslot)){
            //status = 0;
            goto end;
        }

        if (cJSON_IsNumber(id)){
            ID_param = id->valueint;
            //printf("post ID_param = %d\n", ID_param);
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

        if(device_ID == ID_param){
            int hour = -1, day = -1;
            char buf[2] = {};
            buf[0] = Timeslot[0];
            buf[1] = Timeslot[1];
            hour = atoi(buf);

            day = CalculateDay(Date);

            if(day > -1) Calendar[day][hour] = 1;
        }
    }

    end:
        cJSON_Delete(string);
        return;
}
