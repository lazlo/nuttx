/* DCF77 message structure */

#pragma once

struct dcf77msg_s
{
  /*--- page 1 (0:19) ---*/

  uint64_t start        : 1; /* 0: start of minute */

  /* civil warning bits (1:14) */

  uint64_t              : 1;  /* 1 */
  uint64_t              : 1;  /* 2 */
  uint64_t              : 1;  /* 3 */
  uint64_t              : 1;  /* 4 */
  uint64_t              : 1;  /* 5 */
  uint64_t              : 1;  /* 6 */
  uint64_t              : 1;  /* 7 */
  uint64_t              : 1;  /* 8 */
  uint64_t              : 1;  /* 9 */
  uint64_t              : 1;  /* 10 */
  uint64_t              : 1;  /* 11 */
  uint64_t              : 1;  /* 12 */
  uint64_t              : 1;  /* 13 */
  uint64_t              : 1;  /* 14 */

  /* call bit/summer time/CEST/CET/leap second (15:19) */

  uint64_t r            : 1;  /* 15 call bit. abnormal transmitter operation */
  uint64_t a1           : 1;  /* 16 summer time announcment. set during hour before change */
  uint64_t z1           : 1;  /* 17 set to 1 when CEST is in effect */
  uint64_t z2           : 1;  /* 18 set to 1 when CET is in effect */
  uint64_t a2           : 1;  /* 19 set during hour before leap second */

  /*--- page 2 (20:39) ---*/

  uint64_t start_time   : 1;  /* 20 - start of encoded time (always 1) */

  /* minutes + parity bit (21:28) */

  uint64_t m1           : 1;  /* 21 */
  uint64_t m2           : 1;  /* 22 */
  uint64_t m4           : 1;  /* 23 */
  uint64_t m8           : 1;  /* 24 */
  uint64_t m10          : 1;  /* 25 */
  uint64_t m20          : 1;  /* 26 */
  uint64_t m40          : 1;  /* 27 */
  uint64_t p1           : 1;  /* 28 - even parity over minute bits 21-28 */

  /* hours + parity bit (29:35) */

  uint64_t h1           : 1;  /* 29 */
  uint64_t h2           : 1;  /* 30 */
  uint64_t h4           : 1;  /* 31 */
  uint64_t h8           : 1;  /* 32 */
  uint64_t h10          : 1;  /* 33 */
  uint64_t h20          : 1;  /* 34 */
  uint64_t p2           : 1;  /* 35 - even parity over hour bits 29-35 */

  /* day of month (36:39) */

  uint64_t dm1          : 1; /* 36 */
  uint64_t dm2          : 1; /* 37 */
  uint64_t dm4          : 1; /* 38 */
  uint64_t dm8          : 1; /* 39 */

  /*--- page 3 (40:59) ---*/

  /* day of month (40:41) */

  uint64_t dm10         : 1; /* 40 */
  uint64_t dm20         : 1; /* 41 */

  /* day of week (1=Monday,...,7=Sunday) (42:44) */

  uint64_t dw1          : 1; /* 42 */
  uint64_t dw2          : 1; /* 43 */
  uint64_t dw4          : 1; /* 44 */

  /* month number (1-12) (45:49) */

  uint64_t mn1          : 1; /* 45 */
  uint64_t mn2          : 1; /* 46 */
  uint64_t mn4          : 1; /* 47 */
  uint64_t mn8          : 1; /* 48 */
  uint64_t mn10         : 1; /* 49 */

  /* year within century (0-99) (50:57) */

  uint64_t y1           : 1; /* 50 */
  uint64_t y2           : 1; /* 51 */
  uint64_t y4           : 1; /* 52 */
  uint64_t y8           : 1; /* 53 */
  uint64_t y10          : 1; /* 54 */
  uint64_t y20          : 1; /* 55 */
  uint64_t y40          : 1; /* 56 */
  uint64_t y80          : 1; /* 57 */
  uint64_t p3           : 1; /* 58 - even parity over bits 36-58 */
  uint64_t end          : 1; /* 59 end of minute mark */

  /* END OF DCF77 message */

  uint64_t pad          : 4; /* 60:64 */
};

union dcf77msg_u {
	uint64_t raw;
	struct dcf77msg_s msg;
};

/*
 * TODO Check if the parity function work from the right direction with respect to the bits inserted into the receive buffer
 * data		uint64_t used as string of bits
 * offset	start from this bit offset
 * nbits	number of bits to check
 * even		true for even, false for odd parity
 */
inline int check_parity(const uint64_t data, const int offset, const int nbits,
				bool even)
{
	const int len = 8 * sizeof(uint64_t);
	int pos;
	int count = 0;
	bool set;

	printf("check bits %d:%d for %s parity", offset, offset+nbits,
					even ? "even" : "odd");

	printf(" start:%2d {", offset);

	/* Count the number of bits set */
	for (pos = offset; pos < offset+nbits; pos++)
	{
		set = (data & ((uint64_t)1 << pos)) ? true : false;
		if (set) count++;

		printf("%d", set ? 1 : 0);
	}
	printf("} end:%d (%d bits set)\n", pos, count);

	/* Calculate parity on our own */
	return (count % 2 == (int)!even);
}

/* Checks for the respective bits that make a valid DCF77 message */
inline bool dcf77msg_valid(const struct dcf77msg_s m)
{
	bool valid = false;

	if (m.start_time == 1)
		valid = true;

	return valid;
}

inline int dcf77msg_minute(const struct dcf77msg_s m)
{
	int minute = 0;

	if (m.m1) { minute += 1; }
	if (m.m2) { minute += 2; }
	if (m.m4) { minute += 4; }
	if (m.m8) { minute += 8; }
	if (m.m10) { minute += 10; }
	if (m.m20) { minute += 20; }
	if (m.m40) { minute += 40; }

	return minute;
}

inline int dcf77msg_hour(const struct dcf77msg_s m)
{
	int hour = 0;

	if (m.h1)  { hour += 1; }
	if (m.h2)  { hour += 2; }
	if (m.h4)  { hour += 4; }
	if (m.h8)  { hour += 8; }
	if (m.h10) { hour += 10; }
	if (m.h20) { hour += 20; }

	return hour;
}

inline int dcf77msg_day(const struct dcf77msg_s m)
{
	int day = 0;

	day += m.dm1 ? 1 : 0;
	day += m.dm2 ? 2 : 0;
	day += m.dm4 ? 4 : 0;
	day += m.dm8 ? 8 : 0;
	day += m.dm10 ? 10 : 0;
	day += m.dm20 ? 20 : 0;

	return day;
}

inline int dcf77msg_weekday(const struct dcf77msg_s m)
{
	int weekday = 0;

	weekday += m.dw1 ? 1 : 0;
	weekday += m.dw2 ? 2 : 0;
	weekday += m.dw4 ? 4 : 0;

	return weekday;
}

inline int dcf77msg_month(const struct dcf77msg_s m)
{
	int month = 0;

	month += m.mn1 ? 1 : 0;
	month += m.mn2 ? 2 : 0;
	month += m.mn4 ? 4 : 0;
	month += m.mn8 ? 8 : 0;
	month += m.mn10 ? 10 : 0;

	return month;
}

inline int dcf77msg_year(const struct dcf77msg_s m)
{
	int year = 0;

	year += m.y1 ? 1 : 0;
	year += m.y2 ? 2 : 0;
	year += m.y4 ? 4 : 0;
	year += m.y8 ? 8 : 0;
	year += m.y10 ? 10 : 0;
	year += m.y20 ? 20 : 0;
	year += m.y40 ? 40 : 0;
	year += m.y80 ? 80 : 0;

	return year;
}

inline void dcf77msg_dump(struct dcf77msg_s m)
{
	union dcf77msg_u u = {.msg = m};
	struct datatime {
		int minute;
		int hour;
		int day;
		int weekday;
		int month;
		int year;
	} i = {
		.minute		= dcf77msg_minute(u.msg),
		.hour		= dcf77msg_hour(u.msg),
		.day		= dcf77msg_day(u.msg),
		.weekday	= dcf77msg_weekday(u.msg),
		.month		= dcf77msg_month(u.msg),
		.year		= dcf77msg_year(u.msg),
	};

	if (u.msg.p1 != check_parity(u.raw, 21, 7, true))
		printf("minute parity error (p1)\n");
	if (u.msg.p2 != check_parity(u.raw, 29, 6, true))
		printf("hour parity error (p2)\n");
	if (u.msg.p3 != check_parity(u.raw, 36, 22, true))
		printf("date parity error (p3)\n");

	printf("\n");
	printf("DCF77 Message: %4d-%02d-%02d (%d) %02d:%02d\n",
		i.year + 2000, i.month, i.day,
		i.weekday,
		i.hour, i.minute);
	printf("\n");
}
