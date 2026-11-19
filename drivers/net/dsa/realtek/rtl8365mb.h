/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _RTL8365MB_H
#define _RTL8365MB_H

#include "realtek.h"

enum rtl8365mb_family {
	RTL8365MB_FAMILY_C,
	RTL8365MB_FAMILY_D,
};

struct rtl8365mb_jam_tbl_entry;
struct rtl8365mb_l2_uc;
struct rtl8365mb_l2_mc;

struct rtl8365mb_family_info {
	enum rtl8365mb_family family;
	unsigned int num_ports;
	const struct rtl8365mb_jam_tbl_entry *jam_table;
	size_t jam_size;
	size_t l2_entry_size;
	void (*l2_data_to_uc)(const u16 *data, struct rtl8365mb_l2_uc *uc);
	void (*l2_uc_to_data)(const struct rtl8365mb_l2_uc *uc, u16 *data);
	void (*l2_data_to_mc)(const u16 *data, struct rtl8365mb_l2_mc *mc);
	void (*l2_mc_to_data)(const struct rtl8365mb_l2_mc *mc, u16 *data);
};

const struct rtl8365mb_family_info *
rtl8365mb_get_family_info(struct realtek_priv *priv);

static inline bool rtl8365mb_is_d(struct realtek_priv *priv)
{
	return rtl8365mb_get_family_info(priv)->family == RTL8365MB_FAMILY_D;
}

#endif /* _RTL8365MB_H */
