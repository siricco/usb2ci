/*
 * wintv-ci-en50221.c : WinTV-CI - USB2 Common Interface driver
 *
 * Copyright (C) 2017 Helmut Binder (cco@aon.at)
 #
 * (+HB+) 2017-08-13
 * (+HB+) 2018-12-09
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * To obtain the license, point your browser to
 * http://www.gnu.org/copyleft/gpl.html
 */


/*
 *  E N 5 0 2 2 1   T P D U - I N F O
 */

#include "wintv-ci.h"

/* fake caid for testing - this forces TS-streaming on channesls
 * without matching CAM/Card (no descrambling) - applies only
 * if CA_INFO-tpdu is parsed with dump_io_tpdu()
 */

#define CAID_S2 0x648
#define CAID_T2 0x69C

#define PATCH_CAID 0 /* 0/1 */
#define FORCED_CAID CAID_T2

/* en50221: A.4.1.13 List of transport tags */
enum
{
	TT_SB		= 0x80,
	TT_RCV		= 0x81,
	TT_CREATE_TC	= 0x82,
	TT_CTC_REPLY	= 0x83,
	TT_DELETE_TC	= 0x84,
	TT_DTC_REPLY	= 0x85,
	TT_REQUEST_TC	= 0x86,
	TT_NEW_TC	= 0x87,
	TT_TC_ERROR	= 0x88,
	TT_DATA_LAST	= 0xA0,
	TT_DATA_MORE	= 0xA1
};

/* en50221: 7.2.7 Coding of the session tags */
enum
{
	ST_SESSION_NUMBER	= 0x90,
	ST_OPEN_SESSION_REQU	= 0x91,
	ST_OPEN_SESSION_RESP	= 0x92,
	ST_CREATE_SESSION	= 0x93,
	ST_CREATE_SESSION_RESP	= 0x94,
	ST_CLOSE_SESSION_REQU	= 0x95,
	ST_CLOSE_SESSION_RESP	= 0x96
};

/* en50221: Table 7: Open Session Status values */
enum
{
	SPDU_STATUS_OPENED	= 0x00,
	SPDU_STATUS_NOT_EXISTS	= 0xF0
};

/* en50221: 8.8.1 Resource Identifiers */
enum
{
	RI_RESOURCE_MANAGER		= 0x00010041,
	RI_APPLICATION_INFORMATION	= 0x00020041,
	RI_CONDITIONAL_ACCESS_SUPPORT	= 0x00030041,
	RI_HOST_CONTROL			= 0x00200041,
	RI_DATE_TIME			= 0x00240041,
	RI_MMI				= 0x00400041
};

/* en50221: Table 58: Application object tag values */
enum
{
	AOT_PROFILE_ENQ			= 0x9F8010,
	AOT_PROFILE			= 0x9F8011,
	AOT_PROFILE_CHANGE		= 0x9F8012,

	AOT_APPLICATION_INFO_ENQ	= 0x9F8020,
	AOT_APPLICATION_INFO		= 0x9F8021,
	AOT_ENTER_MENU			= 0x9F8022,

	AOT_CA_INFO_ENQ			= 0x9F8030,
	AOT_CA_INFO			= 0x9F8031,
	AOT_CA_PMT			= 0x9F8032,
	AOT_CA_PMT_REPLY		= 0x9F8033,
	AOT_CA_UPDATE			= 0x9F8034,

	AOT_TUNE			= 0x9F8400,
	AOT_REPLACE			= 0x9F8401,
	AOT_CLEAR_REPLACE		= 0x9F8402,
	AOT_ASK_RELEASE			= 0x9F8403,

	AOT_DATE_TIME_ENQ		= 0x9F8440,
	AOT_DATE_TIME			= 0x9F8441,

	AOT_CLOSE_MMI			= 0x9F8800,
	AOT_DISPLAY_CONTROL		= 0x9F8801,
	AOT_DISPLAY_REPLY		= 0x9F8802,
	AOT_TEXT_LAST			= 0x9F8803,
	AOT_TEXT_MORE			= 0x9F8804,
	AOT_KEYPAD_CONTROL		= 0x9F8805,
	AOT_KEYPRESS			= 0x9F8806,
	AOT_ENQ				= 0x9F8807,
	AOT_ANSW			= 0x9F8808,
	AOT_MENU_LAST			= 0x9F8809,
	AOT_MENU_MORE			= 0x9F880A,
	AOT_MENU_ANSW			= 0x9F880B,
	AOT_LIST_LAST			= 0x9F880C,
	AOT_LIST_MORE			= 0x9F880D,
	AOT_SUBTITLE_SEGMENT_LAST	= 0x9F880E,
	AOT_SUBTITLE_SEGMENT_MORE	= 0x9F880F,
	AOT_DISPLAY_MESSAGE		= 0x9F8810,
	AOT_SCENE_END_MARK		= 0x9F8811,
	AOT_SCENE_DONE			= 0x9F8812,
	AOT_SCENE_CONTROL		= 0x9F8813,
	AOT_SUBTITLE_DOWNLOAD_LAST	= 0x9F8814,
	AOT_SUBTITLE_DOWNLOAD_MORE	= 0x9F8815,
	AOT_FLUSH_DOWNLOAD		= 0x9F8816,
	AOT_DOWNLOAD_REPLY		= 0x9F8817,

	AOT_COMMS_CMD			= 0x9F8C00,
	AOT_CONNECTION_DESCRIPTOR	= 0x9F8C01,
	AOT_COMMS_REPLY			= 0x9F8C02,
	AOT_COMMS_SEND_LAST		= 0x9F8C03,
	AOT_COMMS_SEND_MORE		= 0x9F8C04,
	AOT_COMMS_RCV_LAST		= 0x9F8C05,
	AOT_COMMS_RCV_MORE		= 0x9F8C06
};

#define CASE_T_STR(t) case t: p = #t; break

static char * ttag2str(u8 ttag) {
    char * p = "T_unknown";

    switch (ttag) {
    CASE_T_STR(TT_SB);
    CASE_T_STR(TT_RCV);
    CASE_T_STR(TT_CREATE_TC);
    CASE_T_STR(TT_CTC_REPLY);
    CASE_T_STR(TT_DELETE_TC);
    CASE_T_STR(TT_DTC_REPLY);
    CASE_T_STR(TT_REQUEST_TC);
    CASE_T_STR(TT_NEW_TC);
    CASE_T_STR(TT_TC_ERROR);
    CASE_T_STR(TT_DATA_LAST);
    CASE_T_STR(TT_DATA_MORE);
    }
    return p;
}

static char * stag2str(u8 stag) {
    char * p = "ST_unknown";

    switch (stag) {
    CASE_T_STR(ST_SESSION_NUMBER);
    CASE_T_STR(ST_OPEN_SESSION_REQU);
    CASE_T_STR(ST_OPEN_SESSION_RESP);
    CASE_T_STR(ST_CREATE_SESSION);
    CASE_T_STR(ST_CREATE_SESSION_RESP);
    CASE_T_STR(ST_CLOSE_SESSION_REQU);
    CASE_T_STR(ST_CLOSE_SESSION_RESP);
    }
    return p;
}

#define CASE_A_STR(t) case (t & 0xFF) : p = #t; break

static char * atag2str(u32 atag) {

    char def_str[16];
    char *p = def_str;

    u32 agrp = atag >> 8;

    snprintf(def_str, sizeof(def_str)-1,"%X", atag);

    if (agrp == 0x9F80) {
	switch (atag & 0xFF) {
	CASE_A_STR(AOT_PROFILE_ENQ);
	CASE_A_STR(AOT_PROFILE);
	CASE_A_STR(AOT_PROFILE_CHANGE);

	CASE_A_STR(AOT_APPLICATION_INFO_ENQ);
	CASE_A_STR(AOT_APPLICATION_INFO);
	CASE_A_STR(AOT_ENTER_MENU);

	CASE_A_STR(AOT_CA_INFO_ENQ);
	CASE_A_STR(AOT_CA_INFO);
	CASE_A_STR(AOT_CA_PMT);
	CASE_A_STR(AOT_CA_PMT_REPLY);
	CASE_A_STR(AOT_CA_UPDATE);
	}
    }
    else if (agrp == 0x9F84) {
	switch (atag & 0xFF) {
	CASE_A_STR(AOT_DATE_TIME_ENQ);
	CASE_A_STR(AOT_DATE_TIME);
	}
    }
    return p;
}

#define SIZE_INDICATOR 0x80

static int rsn_1_dec(u8 *buf, u16 *size) {
	u8 size_flag = buf[0];

	if (size_flag < SIZE_INDICATOR) {
		*size = size_flag;
		return 1;
	}
	else if (size_flag == (SIZE_INDICATOR | 0x1)) {
		*size = buf[1];
		return 2;
	}
	else if (size_flag == (SIZE_INDICATOR | 0x2)) {
		*size = (buf[1] << 8) | buf[2];
		return 3;
	}
	else {
		*size = 0;
		return 0;
	}
}

void dump_io_tpdu( u8 *buf, size_t count, const char *func, int dir_in) {

	u32   tag_num = 0;
	char *tag_str = NULL;

	int lsize = 0;
	u16 st_len = 0;

	/*
	 # buf[0] = slot
	 # buf[1] = tcid
	 # buf[2...n] = TPDU-Tag, rsn-size, TPDU-body ....
	 */

	u8 slot = buf[0];
	u8 tcid = buf[1];
	u32 atag = 0;
#if PATCH_CAID
	u16 caid_old;
#endif
	int i = 2;

	tag_num = buf[i++]; 

	if (tag_num == TT_SB || tag_num == TT_RCV )
//	if (tag_num == TT_SB )
	    return;

	if (tag_num != TT_DATA_LAST) {
	    tag_str = ttag2str(tag_num);
	}
	else {
	    lsize = rsn_1_dec(buf+i, &st_len);
	    i += lsize;

	    if ( st_len > 1 ) {
		i++; // skip tcid

		tag_num = buf[i++]; 
		if (tag_num != ST_SESSION_NUMBER) {
		    tag_str = stag2str(tag_num);
		}
		else {
		    lsize = rsn_1_dec(buf+i, &st_len);
		    i += lsize;

		    if (st_len == 2) {

			tag_num = buf[i] << 8 | buf[i+1]; // session-number
			atag    = buf[i+2] << 16 | buf[i+3] << 8 | buf[i+4]; // app-obj-tag
			tag_str = atag2str(atag);
#if PATCH_CAID
			/* patch CAID to force TS-streaming */
			if (atag == AOT_CA_INFO) {
			    if (buf[i+5] >= 2) { /* len */
				i += 6;
				caid_old = buf[i] << 8 | buf[i+1]; /* patch first caid */
				buf[i]   = FORCED_CAID >> 8;
				buf[i+1] = FORCED_CAID & 0xFF;
				pr_info("CAID-patch: 0x%X -> 0x%X\n", caid_old, FORCED_CAID);
			    }
			    else
				pr_info("CAID-patch: no CAIDs in CAM ?\n");
			}
#endif
		    }
		    else {
			tag_num = st_len;
			tag_str = "(AOT_ERROR)";
		    }
		}
	    }
	    else if (st_len == 1)
		//tag_str = "(POLL)";
		tag_str = NULL;
	    else
		tag_str = "(ST_ERROR)";
	}

	if (tag_str) {
		pr_info("[%-4s] %s (%d)(%d)[%2zu] *** (%X) %s ***\n",
					(dir_in) ? "CAM" : "HOST",
					func, slot, tcid, count,
					tag_num,tag_str);

		print_hex_dump(KERN_DEBUG, " TPDU : ", DUMP_PREFIX_OFFSET, 16, 1,
						buf+2, min(count-2, (size_t)0x20), 1);
	}
}
