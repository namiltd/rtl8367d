#include <linux/module.h>
#include <linux/export-internal.h>
#include <linux/compiler.h>

MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.arch = MODULE_ARCH_INIT,
};

MODULE_INFO(intree, "Y");

KSYMTAB_FUNC(rtl83xx_lock, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(rtl83xx_unlock, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(rtl83xx_setup_user_mdio, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(rtl83xx_probe, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(rtl83xx_register_switch, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(rtl83xx_unregister_switch, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(rtl83xx_shutdown, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(rtl83xx_remove, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(realtek_mdio_probe, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(realtek_mdio_remove, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(realtek_mdio_shutdown, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(realtek_smi_probe, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(realtek_smi_remove, "_gpl", "REALTEK_DSA");
KSYMTAB_FUNC(realtek_smi_shutdown, "_gpl", "REALTEK_DSA");

MODULE_INFO(depends, "dsa_core");

