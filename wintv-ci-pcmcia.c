/*
 * wintv-ci-pcmcia.c : WinTV-CI - USB2 Common Interface driver
 *
 * Copyright (C) 2017 Helmut Binder (cco@aon.at)
 #
 * (+HB+) 2017-08-13
 * (+HB+) 2019-02-19
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
 *  C I S  -  C A R D   I N F O R M A T I O N   S T R U C T U R E
 */

#include "wintv-ci.h"

#define CIS_DEBUG		0

#define CIS_MAX_IF		4
#define CIS_MAX_V1_STRINGS	4
#define CIS_MAX_V1_STRLEN	32

struct cis_vers_1 {
    unsigned int vmajor;
    unsigned int vminor;
    char vstr[CIS_MAX_V1_STRINGS][CIS_MAX_V1_STRLEN+1];
};

#define ENV_ID_STR "DVB_CI_V1.00"

static int parse_cis_vstrings(unsigned char *data, int len, struct cis_vers_1 *c_v1) {

	int i, n;

	for (i = 0, n = 0; i < len && data[i] != 0xFF && n < CIS_MAX_V1_STRINGS; n++, i++) {
		int j = i;
		int l = 0;
		
		while (i < len && data[i]) {
			i++; l++;
		}
		if (l > CIS_MAX_V1_STRLEN)
			l = CIS_MAX_V1_STRLEN;
		if (l)
			memcpy(&c_v1->vstr[n], data+j, l);
		c_v1->vstr[n][l] = 0;
		if (l)
			pr_info("%s : CFG_V1_STR %d: %s\n",__func__, n, c_v1->vstr[n]);
	}
	return n;
}

int parse_cis(unsigned char *cis, int size, struct slot_info *s_info) {

	struct cis_vers_1 c_v1;
	u8 tpl_code		= 0;
	u8 tpl_link		= 0;
	u8 cc_index 		= 0;

	int i, i_todo, i_next;

	s_info->config_base	= 0;
	s_info->config_option	= 0;
	s_info->cis_valid	= 0;

	memset(&c_v1,0,sizeof(c_v1));

//	print_hex_dump(KERN_DEBUG, " CIS : ", DUMP_PREFIX_OFFSET, 16, 1,
//			cis, size, 1);

	for (i = 0, i_todo = size; i_todo; i = i_next) {

		tpl_code = cis[i++];
		i_todo--;
		if (tpl_code == 0xFF)	/* table end */
			break;

		tpl_link = cis[i++];
		i_todo--;
		if (tpl_link > i_todo)
			goto error_inv;	/* invalid cis-structure */

		i_todo -= tpl_link;
		i_next = i + tpl_link;
#if CIS_DEBUG
		pr_info("cis: tuple %02X, len=%d, todo=%d\n",
			tpl_code, tpl_link, i_todo);
		print_hex_dump(KERN_DEBUG, " CIS : ",
			DUMP_PREFIX_OFFSET, 16, 1, cis+i, tpl_link, 1);
#endif
		/*** CISTPL_VERS_1 - prduct info ***/
		if (tpl_code == 0x15) { 
			c_v1.vmajor = cis[i];
			c_v1.vminor = cis[i+1];
			/* strings start at cis[i+2] */
			parse_cis_vstrings(cis+i+2, tpl_link-2, &c_v1);
		}
		/*** CISTPL_CONFIG - card-config ==> 
		CONFIG-REGISTER BASE-ADDRESS (in attribute memory) ***/
		else if (tpl_code == 0x1A) {
			u16 cfg_base = 0;
			int j, j_todo, j_next;
			int n;

			u8 cc_rasz = cis[i] & 0x3;
			u8 cc_rmsz = (cis[i] >> 2) & 0xF; /* up to 16 bytes (128 bits) for Cfg-Reg-Present */
			u8 cc_rfsz = (cis[i] >> 6) & 0x3; /* reserved size 0..3 bytes */
			i += 2;

			/* [4] RADR */
			for (n = 0; n <= cc_rasz; n++) {
				cfg_base |= cis[i++] << (n<<3);
			}
			pr_info("%s : CFG_BASE: 0x%X (Cfg.Reg[0] in Attrib.Memory)\n",
					__func__, cfg_base);
			s_info->config_base = cfg_base;

			for (n = 0; n <= cc_rmsz; n++, i++) {
			    pr_info("%s : CFG_REGS present [%d-%d] = 0x%02X\n",
					__func__, n<<3, ((n+1)<<3)-1, cis[i]);
			}

			i += cc_rfsz;/* skip up to 3 reserved bytes */

			/* check sub-tuples for valid custom-interface with if-code 0x241 (DVB CI) */
			cc_index = 0;
			for (j = i, j_todo = i_next - i; j_todo; j = j_next, cc_index++) {
				u8 st_code, st_link;

				st_code = cis[j++];
				j_todo--;
				if (st_code == 0xFF)	/* table end */
					break;

				st_link = cis[j++];
				j_todo--;
				if (st_link > j_todo)
					goto error_inv;	/* invalid cis-structure */

				j_todo -= st_link;
				j_next = j + st_link;

				//pr_info("%s : sub-tuple %02X\n",__func__, st_code);

				/* CCSTPL_CIF - custom interface */
				if (st_code == 0xC0) {
					u16 if_code = (cis[j+1]<<8) | cis[j];
					u8  ifn_len = 1 + ((cis[j] >> 6) & 0x3); /* len 2..4 bytes */

					if (ifn_len != 2 || if_code != 0x241)
						continue;	/* wrong if-code */

					/* interface description strings follow - chk only first */
					j += ifn_len;
					if (strncmp(cis+j, ENV_ID_STR, 12) == 0) {
						pr_info("%s : IF-CODE[%d] 0x%04X '%s' matched\n",
									__func__, cc_index, if_code, ENV_ID_STR);
						s_info->cis_valid = 1;
						break;
					}
				}
			}
			if (!s_info->cis_valid)
				break; /* no vaild CI-card interface found */
		}
		/*** CISTPL_CFTABLE_ENTRY ==> CONFIG_OPTION - use custom-interface from 0x1A */
		else if (tpl_code == 0x1B) {
			u8 tpce_idx = cis[i];
			u8 cfg_option, if_type;

			if (!s_info->cis_valid)
				break; /* no vaild CI-card interface found  (or 0x1A not parsed) */
			if ((tpce_idx & 0x80) == 0)
				continue; /* no cfg-byte */

			if_type = cis[i+1] & 0xF;
			cfg_option = tpce_idx & 0x3F;

			if (if_type != (cc_index+4) || 		/* custom interfaces start at 4 */
			   (cfg_option & 0x5) != 0x5) {		/* only if enabled and use IRGs */
				pr_info("%s : skip Interface description for IF-index %d, cfg-options 0x%X\n",
							__func__, if_type, cfg_option);
				continue;
			}

			if (!s_info->config_option || (tpce_idx & 0x40)) { /* store new or last default */
				pr_info("%s : CFG-OPTIONS: 0x%X\n",
							__func__, cfg_option);
				s_info->config_option = cfg_option;
			}
			else
				pr_info("%s : CFG-OPTIONS: 0x%X *** IGNORE ***\n",
						__func__, cfg_option);
		}
	}

	if (tpl_code != 0xFF || i_todo != 0)
		pr_warn("%s : short CIS data : tuple 0x%02X, remaining data_len %d\n",
						__func__, tpl_code, i_todo);

	if (s_info->config_option && s_info->config_base && s_info->cis_valid) {
		pr_info("DVB-CI-Module detected\n");
		return 0;
	}
	else {
		pr_info("*** NO supported DVB-CI-Module detected\n");
		goto no_cam;
	}

error_inv:
	pr_err("%s : Invalid CIS Structure\n",__func__);
no_cam:
	s_info->cis_valid = 0;
	return -EINVAL;
}
