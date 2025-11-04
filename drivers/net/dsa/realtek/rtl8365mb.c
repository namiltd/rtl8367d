// SPDX-License-Identifier: GPL-2.0
/* Realtek SMI subdriver for the Realtek RTL8365MB-VC ethernet switch.
 *
 * Copyright (C) 2021 Alvin Šipraga <alsi@bang-olufsen.dk>
 * Copyright (C) 2021 Michael Rasmussen <mir@bang-olufsen.dk>
 *
 * The RTL8365MB-VC is a 4+1 port 10/100/1000M switch controller. It includes 4
 * integrated PHYs for the user facing ports, and an extension interface which
 * can be connected to the CPU - or another PHY - via either MII, RMII, or
 * RGMII. The switch is configured via the Realtek Simple Management Interface
 * (SMI), which uses the MDIO/MDC lines.
 *
 * Below is a simplified block diagram of the chip and its relevant interfaces.
 *
 *                          .-----------------------------------.
 *                          |                                   |
 *         UTP <---------------> Giga PHY <-> PCS <-> P0 GMAC   |
 *         UTP <---------------> Giga PHY <-> PCS <-> P1 GMAC   |
 *         UTP <---------------> Giga PHY <-> PCS <-> P2 GMAC   |
 *         UTP <---------------> Giga PHY <-> PCS <-> P3 GMAC   |
 *                          |                                   |
 *     CPU/PHY <-MII/RMII/RGMII--->  Extension  <---> Extension |
 *                          |       interface 1        GMAC 1   |
 *                          |                                   |
 *     SMI driver/ <-MDC/SCL---> Management    ~~~~~~~~~~~~~~   |
 *        EEPROM   <-MDIO/SDA--> interface     ~REALTEK ~~~~~   |
 *                          |                  ~RTL8365MB ~~~   |
 *                          |                  ~GXXXC TAIWAN~   |
 *        GPIO <--------------> Reset          ~~~~~~~~~~~~~~   |
 *                          |                                   |
 *      Interrupt  <----------> Link UP/DOWN events             |
 *      controller          |                                   |
 *                          '-----------------------------------'
 *
 * The driver uses DSA to integrate the 4 user and 1 extension ports into the
 * kernel. Netdevices are created for the user ports, as are PHY devices for
 * their integrated PHYs. The device tree firmware should also specify the link
 * partner of the extension port - either via a fixed-link or other phy-handle.
 * See the device tree bindings for more detailed information. Note that the
 * driver has only been tested with a fixed-link, but in principle it should not
 * matter.
 *
 * NOTE: Currently, only the RGMII interface is implemented in this driver.
 *
 * The interrupt line is asserted on link UP/DOWN events. The driver creates a
 * custom irqchip to handle this interrupt and demultiplex the events by reading
 * the status registers via SMI. Interrupts are then propagated to the relevant
 * PHY device.
 *
 * The EEPROM contains initial register values which the chip will read over I2C
 * upon hardware reset. It is also possible to omit the EEPROM. In both cases,
 * the driver will manually reprogram some registers using jam tables to reach
 * an initial state defined by the vendor driver.
 *
 * This Linux driver is written based on an OS-agnostic vendor driver from
 * Realtek. The reference GPL-licensed sources can be found in the OpenWrt
 * source tree under the name rtl8367c. The vendor driver claims to support a
 * number of similar switch controllers from Realtek, but the only hardware we
 * have is the RTL8365MB-VC. Moreover, there does not seem to be any chip under
 * the name RTL8367C. Although one wishes that the 'C' stood for some kind of
 * common hardware revision, there exist examples of chips with the suffix -VC
 * which are explicitly not supported by the rtl8367c driver and which instead
 * require the rtl8367d vendor driver. With all this uncertainty, the driver has
 * been modestly named rtl8365mb. Future implementors may wish to rename things
 * accordingly.
 *
 * In the same family of chips, some carry up to 8 user ports and up to 2
 * extension ports. Where possible this driver tries to make things generic, but
 * more work must be done to support these configurations. According to
 * documentation from Realtek, the family should include the following chips:
 *
 *  - RTL8363NB
 *  - RTL8363NB-VB
 *  - RTL8363SC
 *  - RTL8363SC-VB
 *  - RTL8364NB
 *  - RTL8364NB-VB
 *  - RTL8365MB-VC
 *  - RTL8366SC
 *  - RTL8367RB-VB
 *  - RTL8367SB
 *  - RTL8367S
 *  - RTL8370MB
 *  - RTL8310SR
 *
 * Some of the register logic for these additional chips has been skipped over
 * while implementing this driver. It is therefore not possible to assume that
 * things will work out-of-the-box for other chips, and a careful review of the
 * vendor driver may be needed to expand support. The RTL8365MB-VC seems to be
 * one of the simpler chips.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/if_bridge.h>
#include <linux/if_vlan.h>

#include "realtek.h"
#include "realtek-smi.h"
#include "realtek-mdio.h"
#include "rtl83xx.h"

/* Family-specific data and limits */
#define RTL8365MB_PHYADDRMAX		7
#define RTL8365MB_NUM_PHYREGS		32
#define RTL8365MB_PHYREGMAX		(RTL8365MB_NUM_PHYREGS - 1)
#define RTL8365MB_MAX_NUM_PORTS		11
#define RTL8365MB_MAX_NUM_EXTINTS	3
#define RTL8365MB_LEARN_LIMIT_MAX	2112

/* Chip identification registers */
#define RTL8365MB_CHIP_ID_REG		0x1300

#define RTL8365MB_CHIP_VER_REG		0x1301

#define RTL8365MB_MAGIC_REG		0x13C2
#define   RTL8365MB_MAGIC_VALUE		0x0249

/* Chip reset register */
#define RTL8365MB_CHIP_RESET_REG	0x1322
#define RTL8365MB_CHIP_RESET_SW_MASK	0x0002
#define RTL8365MB_CHIP_RESET_HW_MASK	0x0001

/* Interrupt polarity register */
#define RTL8365MB_INTR_POLARITY_REG	0x1100
#define   RTL8365MB_INTR_POLARITY_MASK	0x0001
#define   RTL8365MB_INTR_POLARITY_HIGH	0
#define   RTL8365MB_INTR_POLARITY_LOW	1

/* Interrupt control/status register - enable/check specific interrupt types */
#define RTL8365MB_INTR_CTRL_REG			0x1101
#define RTL8365MB_INTR_STATUS_REG		0x1102
#define   RTL8365MB_INTR_SLIENT_START_2_MASK	0x1000
#define   RTL8365MB_INTR_SLIENT_START_MASK	0x0800
#define   RTL8365MB_INTR_ACL_ACTION_MASK	0x0200
#define   RTL8365MB_INTR_CABLE_DIAG_FIN_MASK	0x0100
#define   RTL8365MB_INTR_INTERRUPT_8051_MASK	0x0080
#define   RTL8365MB_INTR_LOOP_DETECTION_MASK	0x0040
#define   RTL8365MB_INTR_GREEN_TIMER_MASK	0x0020
#define   RTL8365MB_INTR_SPECIAL_CONGEST_MASK	0x0010
#define   RTL8365MB_INTR_SPEED_CHANGE_MASK	0x0008
#define   RTL8365MB_INTR_LEARN_OVER_MASK	0x0004
#define   RTL8365MB_INTR_METER_EXCEEDED_MASK	0x0002
#define   RTL8365MB_INTR_LINK_CHANGE_MASK	0x0001
#define   RTL8365MB_INTR_ALL_MASK                      \
		(RTL8365MB_INTR_SLIENT_START_2_MASK |  \
		 RTL8365MB_INTR_SLIENT_START_MASK |    \
		 RTL8365MB_INTR_ACL_ACTION_MASK |      \
		 RTL8365MB_INTR_CABLE_DIAG_FIN_MASK |  \
		 RTL8365MB_INTR_INTERRUPT_8051_MASK |  \
		 RTL8365MB_INTR_LOOP_DETECTION_MASK |  \
		 RTL8365MB_INTR_GREEN_TIMER_MASK |     \
		 RTL8365MB_INTR_SPECIAL_CONGEST_MASK | \
		 RTL8365MB_INTR_SPEED_CHANGE_MASK |    \
		 RTL8365MB_INTR_LEARN_OVER_MASK |      \
		 RTL8365MB_INTR_METER_EXCEEDED_MASK |  \
		 RTL8365MB_INTR_LINK_CHANGE_MASK)

/* Per-port interrupt type status registers */
#define RTL8365MB_PORT_LINKDOWN_IND_REG		0x1106
#define   RTL8365MB_PORT_LINKDOWN_IND_MASK	0x07FF

#define RTL8365MB_PORT_LINKUP_IND_REG		0x1107
#define   RTL8365MB_PORT_LINKUP_IND_MASK	0x07FF

/* PHY indirect access registers */
#define RTL8365MB_INDIRECT_ACCESS_CTRL_REG			0x1F00
#define   RTL8365MB_INDIRECT_ACCESS_CTRL_RW_MASK		0x0002
#define   RTL8365MB_INDIRECT_ACCESS_CTRL_RW_READ		0
#define   RTL8365MB_INDIRECT_ACCESS_CTRL_RW_WRITE		1
#define   RTL8365MB_INDIRECT_ACCESS_CTRL_CMD_MASK		0x0001
#define   RTL8365MB_INDIRECT_ACCESS_CTRL_CMD_VALUE		1
#define RTL8365MB_INDIRECT_ACCESS_STATUS_REG			0x1F01
#define RTL8365MB_INDIRECT_ACCESS_ADDRESS_REG			0x1F02
#define   RTL8365MB_INDIRECT_ACCESS_ADDRESS_OCPADR_5_1_MASK	GENMASK(4, 0)
#define   RTL8365MB_INDIRECT_ACCESS_ADDRESS_PHYNUM_MASK		GENMASK(7, 5)
#define   RTL8365MB_INDIRECT_ACCESS_ADDRESS_OCPADR_9_6_MASK	GENMASK(11, 8)
#define   RTL8365MB_PHY_BASE					0x2000
#define RTL8365MB_INDIRECT_ACCESS_WRITE_DATA_REG		0x1F03
#define RTL8365MB_INDIRECT_ACCESS_READ_DATA_REG			0x1F04

/* PHY OCP address prefix register */
#define RTL8365MB_GPHY_OCP_MSB_0_REG			0x1D15
#define   RTL8365MB_GPHY_OCP_MSB_0_CFG_CPU_OCPADR_MASK	0x0FC0
#define RTL8365MB_PHY_OCP_ADDR_PREFIX_MASK		0xFC00

/* The PHY OCP addresses of PHY registers 0~31 start here */
#define RTL8365MB_PHY_OCP_ADDR_PHYREG_BASE		0xA400

/* External interface port mode values - used in DIGITAL_INTERFACE_SELECT */
#define RTL8365MB_EXT_PORT_MODE_DISABLE		0
#define RTL8365MB_EXT_PORT_MODE_RGMII		1
#define RTL8365MB_EXT_PORT_MODE_MII_MAC		2
#define RTL8365MB_EXT_PORT_MODE_MII_PHY		3
#define RTL8365MB_EXT_PORT_MODE_TMII_MAC	4
#define RTL8365MB_EXT_PORT_MODE_TMII_PHY	5
#define RTL8365MB_EXT_PORT_MODE_GMII		6
#define RTL8365MB_EXT_PORT_MODE_RMII_MAC	7
#define RTL8365MB_EXT_PORT_MODE_RMII_PHY	8
#define RTL8365MB_EXT_PORT_MODE_SGMII		9
#define RTL8365MB_EXT_PORT_MODE_HSGMII		10
#define RTL8365MB_EXT_PORT_MODE_1000X_100FX	11
#define RTL8365MB_EXT_PORT_MODE_1000X		12
#define RTL8365MB_EXT_PORT_MODE_100FX		13

/* External interface mode configuration registers 0~1 */
#define RTL8365MB_DIGITAL_INTERFACE_SELECT_REG0		0x1305 /* EXT0,EXT1 */
#define RTL8365MB_DIGITAL_INTERFACE_SELECT_REG1		0x13C3 /* EXT2 */
#define RTL8365MB_DIGITAL_INTERFACE_SELECT_REG(_extint) \
		((_extint) <= 1 ? RTL8365MB_DIGITAL_INTERFACE_SELECT_REG0 : \
		 (_extint) == 2 ? RTL8365MB_DIGITAL_INTERFACE_SELECT_REG1 : \
		 0x0)
#define   RTL8365MB_DIGITAL_INTERFACE_SELECT_MODE_MASK(_extint) \
		(0xF << (((_extint) % 2)))
#define   RTL8365MB_DIGITAL_INTERFACE_SELECT_MODE_OFFSET(_extint) \
		(((_extint) % 2) * 4)

/* External interface RGMII TX/RX delay configuration registers 0~2 */
#define RTL8365MB_EXT_RGMXF_REG0		0x1306 /* EXT0 */
#define RTL8365MB_EXT_RGMXF_REG1		0x1307 /* EXT1 */
#define RTL8365MB_EXT_RGMXF_REG2		0x13C5 /* EXT2 */
#define RTL8365MB_EXT_RGMXF_REG(_extint) \
		((_extint) == 0 ? RTL8365MB_EXT_RGMXF_REG0 : \
		 (_extint) == 1 ? RTL8365MB_EXT_RGMXF_REG1 : \
		 (_extint) == 2 ? RTL8365MB_EXT_RGMXF_REG2 : \
		 0x0)
#define   RTL8365MB_EXT_RGMXF_RXDELAY_MASK	0x0007
#define   RTL8365MB_EXT_RGMXF_TXDELAY_MASK	0x0008

/* External interface port speed values - used in DIGITAL_INTERFACE_FORCE */
#define RTL8365MB_PORT_SPEED_10M	0
#define RTL8365MB_PORT_SPEED_100M	1
#define RTL8365MB_PORT_SPEED_1000M	2

/* External interface force configuration registers 0~2 */
#define RTL8365MB_DIGITAL_INTERFACE_FORCE_REG0		0x1310 /* EXT0 */
#define RTL8365MB_DIGITAL_INTERFACE_FORCE_REG1		0x1311 /* EXT1 */
#define RTL8365MB_DIGITAL_INTERFACE_FORCE_REG2		0x13C4 /* EXT2 */
#define RTL8365MB_DIGITAL_INTERFACE_FORCE_REG(_extint) \
		((_extint) == 0 ? RTL8365MB_DIGITAL_INTERFACE_FORCE_REG0 : \
		 (_extint) == 1 ? RTL8365MB_DIGITAL_INTERFACE_FORCE_REG1 : \
		 (_extint) == 2 ? RTL8365MB_DIGITAL_INTERFACE_FORCE_REG2 : \
		 0x0)
#define   RTL8365MB_DIGITAL_INTERFACE_FORCE_EN_MASK		0x1000
#define   RTL8365MB_DIGITAL_INTERFACE_FORCE_NWAY_MASK		0x0080
#define   RTL8365MB_DIGITAL_INTERFACE_FORCE_TXPAUSE_MASK	0x0040
#define   RTL8365MB_DIGITAL_INTERFACE_FORCE_RXPAUSE_MASK	0x0020
#define   RTL8365MB_DIGITAL_INTERFACE_FORCE_LINK_MASK		0x0010
#define   RTL8365MB_DIGITAL_INTERFACE_FORCE_DUPLEX_MASK		0x0004
#define   RTL8365MB_DIGITAL_INTERFACE_FORCE_SPEED_MASK		0x0003

/* CPU port mask register - controls which ports are treated as CPU ports */
#define RTL8365MB_CPU_PORT_MASK_REG	0x1219
#define   RTL8365MB_CPU_PORT_MASK_MASK	0x07FF

/* CPU control register */
#define RTL8365MB_CPU_CTRL_REG			0x121A
#define   RTL8365MB_CPU_CTRL_TRAP_PORT_EXT_MASK	0x0400
#define   RTL8365MB_CPU_CTRL_TAG_FORMAT_MASK	0x0200
#define   RTL8365MB_CPU_CTRL_RXBYTECOUNT_MASK	0x0080
#define   RTL8365MB_CPU_CTRL_TAG_POSITION_MASK	0x0040
#define   RTL8365MB_CPU_CTRL_TRAP_PORT_MASK	0x0038
#define   RTL8365MB_CPU_CTRL_INSERTMODE_MASK	0x0006
#define   RTL8365MB_CPU_CTRL_EN_MASK		0x0001

/* Maximum packet length register */
#define RTL8365MB_CFG0_MAX_LEN_REG	0x088C
#define   RTL8365MB_CFG0_MAX_LEN_MASK	0x3FFF
#define RTL8365MB_CFG0_MAX_LEN_MAX	0x3FFF

/* Port learning limit registers */
#define RTL8365MB_LUT_PORT_LEARN_LIMIT_BASE		0x0A20
#define RTL8365MB_LUT_PORT_LEARN_LIMIT_REG(_physport) \
		(RTL8365MB_LUT_PORT_LEARN_LIMIT_BASE + (_physport))

/* Port isolation (forwarding mask) registers */
#define RTL8365MB_PORT_ISOLATION_REG_BASE		0x08A2
#define RTL8365MB_PORT_ISOLATION_REG(_physport) \
		(RTL8365MB_PORT_ISOLATION_REG_BASE + (_physport))
#define   RTL8365MB_PORT_ISOLATION_MASK			0x07FF

/* MSTP port state registers - indexed by tree instance */
#define RTL8365MB_MSTI_CTRL_BASE			0x0A00
#define RTL8365MB_MSTI_CTRL_REG(_msti, _physport) \
		(RTL8365MB_MSTI_CTRL_BASE + ((_msti) << 1) + ((_physport) >> 3))
#define   RTL8365MB_MSTI_CTRL_PORT_STATE_OFFSET(_physport) ((_physport) << 1)
#define   RTL8365MB_MSTI_CTRL_PORT_STATE_MASK(_physport) \
		(0x3 << RTL8365MB_MSTI_CTRL_PORT_STATE_OFFSET((_physport)))

/* MIB counter value registers */
#define RTL8365MB_MIB_COUNTER_BASE	0x1000
#define RTL8365MB_MIB_COUNTER_REG(_x)	(RTL8365MB_MIB_COUNTER_BASE + (_x))

/* MIB counter address register */
#define RTL8365MB_MIB_ADDRESS_REG		0x1004
#define   RTL8365MB_MIB_ADDRESS_PORT_OFFSET	0x007C
#define   RTL8365MB_MIB_ADDRESS(_p, _x) \
		(((RTL8365MB_MIB_ADDRESS_PORT_OFFSET) * (_p) + (_x)) >> 2)

#define RTL8365MB_MIB_CTRL0_REG			0x1005
#define   RTL8365MB_MIB_CTRL0_RESET_MASK	0x0002
#define   RTL8365MB_MIB_CTRL0_BUSY_MASK		0x0001

/* The DSA callback .get_stats64 runs in atomic context, so we are not allowed
 * to block. On the other hand, accessing MIB counters absolutely requires us to
 * block. The solution is thus to schedule work which polls the MIB counters
 * asynchronously and updates some private data, which the callback can then
 * fetch atomically. Three seconds should be a good enough polling interval.
 */
#define RTL8365MB_STATS_INTERVAL_JIFFIES	(3 * HZ)

/* Table access registers */
#define RTL8365MB_TABLE_CONTROL_REG		0x0500
#define   RTL8365MB_TABLE_CONTROL_TABLE_MASK	GENMASK(2, 0)
#define   RTL8365MB_TABLE_CONTROL_COMMAND_MASK	GENMASK(3, 3)
#define   RTL8365MB_TABLE_CONTROL_METHOD_MASK	GENMASK(7, 4)
#define   RTL8365MB_TABLE_CONTROL_SPA_MASK	GENMASK(11, 8)
#define RTL8365MB_TABLE_ACCESS_ADDR_REG		0x0501
#define  RTL8365MB_TABLE_ACCESS_ADDR_REG_MASK	GENMASK(13, 0)
#define RTL8365MB_TABLE_LUT_REG			0x0502
#define   RTL8365MB_TABLE_LUT_ADDR_MASK		GENMASK(10, 0)
#define   RTL8365MB_TABLE_LUT_TYPE_MASK		GENMASK(11, 11)
#define   RTL8365MB_TABLE_LUT_HIT_STATUS_MASK	GENMASK(12, 12)
#define   RTL8365MB_TABLE_LUT_BUSY_FLAG_MASK	GENMASK(13, 13)
#define   RTL8365MB_TABLE_LUT_ADDR2_MASK	GENMASK(14, 14)
#define RTL8365MB_TABLE_WRITE_DATA_REG_BASE	0x0510 /* up to 0x0519 */
#define RTL8365MB_TABLE_READ_DATA_REG_BASE	0x0520 /* up to 0x0529 */
/* both last read/write register (10th) uses only the less 4 significant bits */
#define   RTL8365MB_TABLE_10TH_DATA_REG_MASK	GENMASK(3, 0)

/* VLAN enable registers */
#define RTL8365MB_VLAN_CTRL_REG			0x07A8
#define   RTL8365MB_VLAN_CTRL_EN_VLAN_MASK	GENMASK(0, 0)

/* VLAN filtering registers */
#define RTL8365MB_VLAN_INGRESS_REG    0x07A9
#define RTL8365MB_VLAN_INGRESS_MASK   GENMASK(10, 0)

/* RTL8367S supports 4k vlans (vid<=4095) and 32 enhanced vlans
 * for VIDs up to 8191
 */
#define RTL8365MB_MAX_4K_VID		0x0FFF /* 4095 */
#define RTL8365MB_MAX_MC_VID		0x1FFF /* 8191 */

/* Frame type filtering registers */
#define RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_BASE	0x07aa
#define RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_REG(port) \
		(RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_BASE + (port >> 3))
/* required as FIELD_PREP cannot use non-constant masks */
#define RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_MASK(port) \
		(0x3 << RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_OFFSET(port))
#define RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_OFFSET(port) \
		((port & 0x7) << 1)

/* PVID registers */
#define RTL8365MB_VLAN_PVID_CTRL_BASE			0x0700
#define RTL8365MB_VLAN_PVID_CTRL_REG(port) \
	(RTL8365MB_VLAN_PVID_CTRL_BASE + ((port) >> 1))
/* required as FIELD_PREP cannot use non-constant masks */
#define RTL8365MB_VLAN_PVID_CTRL_MASK(port) \
	(0xFF << RTL8365MB_VLAN_PVID_CTRL_OFFSET(port))
#define RTL8365MB_VLAN_PVID_CTRL_OFFSET(port) \
	(((port) & 1) << 3)

/* VLAN 4k table entry */
#define RTL8365MB_VLAN_4K_ENTRY_SIZE			3 /* 48-bits */
#define RTL8365MB_VLAN_4K_CONF0_MEMBERS_LS_MASK		GENMASK(7, 0)
#define RTL8365MB_VLAN_4K_CONF2_MEMBERS_MS_MASK		GENMASK(2, 0)
#define RTL8365MB_VLAN_4K_CONF0_UNTAG_LS_MASK		GENMASK(15, 8)
#define RTL8365MB_VLAN_4K_CONF2_UNTAG_MS_MASK		GENMASK(5, 3)
#define RTL8365MB_VLAN_4K_CONF1_FID_MSI_MASK		GENMASK(3, 0)
#define RTL8365MB_VLAN_4K_CONF1_VBPEN_MASK		GENMASK(4, 4)
#define RTL8365MB_VLAN_4K_CONF1_VBPRI_MASK		GENMASK(7, 5)
#define RTL8365MB_VLAN_4K_CONF1_ENVLANPOL_MASK		GENMASK(8, 8)
#define RTL8365MB_VLAN_4K_CONF1_METER_IDX_LS_MASK	GENMASK(13, 9)
#define RTL8365MB_VLAN_4K_CONF2_METER_IDX_MS_MASK	GENMASK(6, 6)

/* VLAN MC registers */
#define RTL8365MB_VLAN_MC_CONF_BASE			0x0728
#define RTL8365MB_VLAN_MC_CONF_ENTRY_SIZE 		4 /* 64-bit */
#define RTL8365MB_VLAN_MC_CONF_REG(index) \
		(RTL8365MB_VLAN_MC_CONF_BASE + \
		 RTL8365MB_VLAN_MC_CONF_ENTRY_SIZE * (index))
#define RTL8365MB_VLAN_MC_CONF_SIZE 			32
#define  RTL8365MB_VLAN_MC_CONF0_MEMBERS_MSK		GENMASK(10, 0)
#define  RTL8365MB_VLAN_MC_CONF1_FID_MSI_MSK		GENMASK(3, 0)
#define  RTL8365MB_VLAN_MC_CONF2_VBPEN_MSK		GENMASK(0, 0)
#define  RTL8365MB_VLAN_MC_CONF2_VBPRI_MSK		GENMASK(3, 1)
#define  RTL8365MB_VLAN_MC_CONF2_ENVLANPOL_MSK		GENMASK(4, 4)
#define  RTL8365MB_VLAN_MC_CONF2_METER_IDX_MSK		GENMASK(10, 5)
#define  RTL8365MB_VLAN_MC_CONF3_EVID_MSK		GENMASK(12, 0)

enum rtl8365mb_table {
	RTL8365MB_TABLE_ACL_RULE = 1,
	RTL8365MB_TABLE_ACL_ACT,
	RTL8365MB_TABLE_CVLAN, /* 4k vlan table */
	RTL8365MB_TABLE_L2,
	RTL8365MB_TABLE_IGMP_GROUP,

	RTL8365MB_NUM_TABLES
};

enum rtl8365mb_table_op {
	RTL8365MB_TABLE_READ = 0,
	RTL8365MB_TABLE_WRITE
};

static const int rtl8365mb_table_entry_size[] = {
	[RTL8365MB_TABLE_ACL_RULE] = 0,
	[RTL8365MB_TABLE_ACL_ACT] = 0,
	[RTL8365MB_TABLE_CVLAN] = 3,
	[RTL8365MB_TABLE_L2] = 0,
	[RTL8365MB_TABLE_IGMP_GROUP] = 0
};

enum rtl8365mb_frame_type {
	RTL8365MB_FRAME_TYPE_ANY_FRAME = 0,
	RTL8365MB_FRAME_TYPE_TAGGED_ONLY,
	RTL8365MB_FRAME_TYPE_UNTAGGED_ONLY,
};

enum rtl8365mb_mib_counter_index {
	RTL8365MB_MIB_ifInOctets,
	RTL8365MB_MIB_dot3StatsFCSErrors,
	RTL8365MB_MIB_dot3StatsSymbolErrors,
	RTL8365MB_MIB_dot3InPauseFrames,
	RTL8365MB_MIB_dot3ControlInUnknownOpcodes,
	RTL8365MB_MIB_etherStatsFragments,
	RTL8365MB_MIB_etherStatsJabbers,
	RTL8365MB_MIB_ifInUcastPkts,
	RTL8365MB_MIB_etherStatsDropEvents,
	RTL8365MB_MIB_ifInMulticastPkts,
	RTL8365MB_MIB_ifInBroadcastPkts,
	RTL8365MB_MIB_inMldChecksumError,
	RTL8365MB_MIB_inIgmpChecksumError,
	RTL8365MB_MIB_inMldSpecificQuery,
	RTL8365MB_MIB_inMldGeneralQuery,
	RTL8365MB_MIB_inIgmpSpecificQuery,
	RTL8365MB_MIB_inIgmpGeneralQuery,
	RTL8365MB_MIB_inMldLeaves,
	RTL8365MB_MIB_inIgmpLeaves,
	RTL8365MB_MIB_etherStatsOctets,
	RTL8365MB_MIB_etherStatsUnderSizePkts,
	RTL8365MB_MIB_etherOversizeStats,
	RTL8365MB_MIB_etherStatsPkts64Octets,
	RTL8365MB_MIB_etherStatsPkts65to127Octets,
	RTL8365MB_MIB_etherStatsPkts128to255Octets,
	RTL8365MB_MIB_etherStatsPkts256to511Octets,
	RTL8365MB_MIB_etherStatsPkts512to1023Octets,
	RTL8365MB_MIB_etherStatsPkts1024to1518Octets,
	RTL8365MB_MIB_ifOutOctets,
	RTL8365MB_MIB_dot3StatsSingleCollisionFrames,
	RTL8365MB_MIB_dot3StatsMultipleCollisionFrames,
	RTL8365MB_MIB_dot3StatsDeferredTransmissions,
	RTL8365MB_MIB_dot3StatsLateCollisions,
	RTL8365MB_MIB_etherStatsCollisions,
	RTL8365MB_MIB_dot3StatsExcessiveCollisions,
	RTL8365MB_MIB_dot3OutPauseFrames,
	RTL8365MB_MIB_ifOutDiscards,
	RTL8365MB_MIB_dot1dTpPortInDiscards,
	RTL8365MB_MIB_ifOutUcastPkts,
	RTL8365MB_MIB_ifOutMulticastPkts,
	RTL8365MB_MIB_ifOutBroadcastPkts,
	RTL8365MB_MIB_outOampduPkts,
	RTL8365MB_MIB_inOampduPkts,
	RTL8365MB_MIB_inIgmpJoinsSuccess,
	RTL8365MB_MIB_inIgmpJoinsFail,
	RTL8365MB_MIB_inMldJoinsSuccess,
	RTL8365MB_MIB_inMldJoinsFail,
	RTL8365MB_MIB_inReportSuppressionDrop,
	RTL8365MB_MIB_inLeaveSuppressionDrop,
	RTL8365MB_MIB_outIgmpReports,
	RTL8365MB_MIB_outIgmpLeaves,
	RTL8365MB_MIB_outIgmpGeneralQuery,
	RTL8365MB_MIB_outIgmpSpecificQuery,
	RTL8365MB_MIB_outMldReports,
	RTL8365MB_MIB_outMldLeaves,
	RTL8365MB_MIB_outMldGeneralQuery,
	RTL8365MB_MIB_outMldSpecificQuery,
	RTL8365MB_MIB_inKnownMulticastPkts,
	RTL8365MB_MIB_END,
};

struct rtl8365mb_mib_counter {
	u32 offset;
	u32 length;
	const char *name;
};

#define RTL8365MB_MAKE_MIB_COUNTER(_offset, _length, _name) \
		[RTL8365MB_MIB_ ## _name] = { _offset, _length, #_name }

static struct rtl8365mb_mib_counter rtl8365mb_mib_counters[] = {
	RTL8365MB_MAKE_MIB_COUNTER(0, 4, ifInOctets),
	RTL8365MB_MAKE_MIB_COUNTER(4, 2, dot3StatsFCSErrors),
	RTL8365MB_MAKE_MIB_COUNTER(6, 2, dot3StatsSymbolErrors),
	RTL8365MB_MAKE_MIB_COUNTER(8, 2, dot3InPauseFrames),
	RTL8365MB_MAKE_MIB_COUNTER(10, 2, dot3ControlInUnknownOpcodes),
	RTL8365MB_MAKE_MIB_COUNTER(12, 2, etherStatsFragments),
	RTL8365MB_MAKE_MIB_COUNTER(14, 2, etherStatsJabbers),
	RTL8365MB_MAKE_MIB_COUNTER(16, 2, ifInUcastPkts),
	RTL8365MB_MAKE_MIB_COUNTER(18, 2, etherStatsDropEvents),
	RTL8365MB_MAKE_MIB_COUNTER(20, 2, ifInMulticastPkts),
	RTL8365MB_MAKE_MIB_COUNTER(22, 2, ifInBroadcastPkts),
	RTL8365MB_MAKE_MIB_COUNTER(24, 2, inMldChecksumError),
	RTL8365MB_MAKE_MIB_COUNTER(26, 2, inIgmpChecksumError),
	RTL8365MB_MAKE_MIB_COUNTER(28, 2, inMldSpecificQuery),
	RTL8365MB_MAKE_MIB_COUNTER(30, 2, inMldGeneralQuery),
	RTL8365MB_MAKE_MIB_COUNTER(32, 2, inIgmpSpecificQuery),
	RTL8365MB_MAKE_MIB_COUNTER(34, 2, inIgmpGeneralQuery),
	RTL8365MB_MAKE_MIB_COUNTER(36, 2, inMldLeaves),
	RTL8365MB_MAKE_MIB_COUNTER(38, 2, inIgmpLeaves),
	RTL8365MB_MAKE_MIB_COUNTER(40, 4, etherStatsOctets),
	RTL8365MB_MAKE_MIB_COUNTER(44, 2, etherStatsUnderSizePkts),
	RTL8365MB_MAKE_MIB_COUNTER(46, 2, etherOversizeStats),
	RTL8365MB_MAKE_MIB_COUNTER(48, 2, etherStatsPkts64Octets),
	RTL8365MB_MAKE_MIB_COUNTER(50, 2, etherStatsPkts65to127Octets),
	RTL8365MB_MAKE_MIB_COUNTER(52, 2, etherStatsPkts128to255Octets),
	RTL8365MB_MAKE_MIB_COUNTER(54, 2, etherStatsPkts256to511Octets),
	RTL8365MB_MAKE_MIB_COUNTER(56, 2, etherStatsPkts512to1023Octets),
	RTL8365MB_MAKE_MIB_COUNTER(58, 2, etherStatsPkts1024to1518Octets),
	RTL8365MB_MAKE_MIB_COUNTER(60, 4, ifOutOctets),
	RTL8365MB_MAKE_MIB_COUNTER(64, 2, dot3StatsSingleCollisionFrames),
	RTL8365MB_MAKE_MIB_COUNTER(66, 2, dot3StatsMultipleCollisionFrames),
	RTL8365MB_MAKE_MIB_COUNTER(68, 2, dot3StatsDeferredTransmissions),
	RTL8365MB_MAKE_MIB_COUNTER(70, 2, dot3StatsLateCollisions),
	RTL8365MB_MAKE_MIB_COUNTER(72, 2, etherStatsCollisions),
	RTL8365MB_MAKE_MIB_COUNTER(74, 2, dot3StatsExcessiveCollisions),
	RTL8365MB_MAKE_MIB_COUNTER(76, 2, dot3OutPauseFrames),
	RTL8365MB_MAKE_MIB_COUNTER(78, 2, ifOutDiscards),
	RTL8365MB_MAKE_MIB_COUNTER(80, 2, dot1dTpPortInDiscards),
	RTL8365MB_MAKE_MIB_COUNTER(82, 2, ifOutUcastPkts),
	RTL8365MB_MAKE_MIB_COUNTER(84, 2, ifOutMulticastPkts),
	RTL8365MB_MAKE_MIB_COUNTER(86, 2, ifOutBroadcastPkts),
	RTL8365MB_MAKE_MIB_COUNTER(88, 2, outOampduPkts),
	RTL8365MB_MAKE_MIB_COUNTER(90, 2, inOampduPkts),
	RTL8365MB_MAKE_MIB_COUNTER(92, 4, inIgmpJoinsSuccess),
	RTL8365MB_MAKE_MIB_COUNTER(96, 2, inIgmpJoinsFail),
	RTL8365MB_MAKE_MIB_COUNTER(98, 2, inMldJoinsSuccess),
	RTL8365MB_MAKE_MIB_COUNTER(100, 2, inMldJoinsFail),
	RTL8365MB_MAKE_MIB_COUNTER(102, 2, inReportSuppressionDrop),
	RTL8365MB_MAKE_MIB_COUNTER(104, 2, inLeaveSuppressionDrop),
	RTL8365MB_MAKE_MIB_COUNTER(106, 2, outIgmpReports),
	RTL8365MB_MAKE_MIB_COUNTER(108, 2, outIgmpLeaves),
	RTL8365MB_MAKE_MIB_COUNTER(110, 2, outIgmpGeneralQuery),
	RTL8365MB_MAKE_MIB_COUNTER(112, 2, outIgmpSpecificQuery),
	RTL8365MB_MAKE_MIB_COUNTER(114, 2, outMldReports),
	RTL8365MB_MAKE_MIB_COUNTER(116, 2, outMldLeaves),
	RTL8365MB_MAKE_MIB_COUNTER(118, 2, outMldGeneralQuery),
	RTL8365MB_MAKE_MIB_COUNTER(120, 2, outMldSpecificQuery),
	RTL8365MB_MAKE_MIB_COUNTER(122, 2, inKnownMulticastPkts),
};

static_assert(ARRAY_SIZE(rtl8365mb_mib_counters) == RTL8365MB_MIB_END);

struct rtl8365mb_jam_tbl_entry {
	u16 reg;
	u16 val;
};

/* Lifted from the vendor driver sources */
static const struct rtl8365mb_jam_tbl_entry rtl8365mb_init_jam_8365mb_vc[] = {
	{ 0x13EB, 0x15BB }, { 0x1303, 0x06D6 }, { 0x1304, 0x0700 },
	{ 0x13E2, 0x003F }, { 0x13F9, 0x0090 }, { 0x121E, 0x03CA },
	{ 0x1233, 0x0352 }, { 0x1237, 0x00A0 }, { 0x123A, 0x0030 },
	{ 0x1239, 0x0084 }, { 0x0301, 0x1000 }, { 0x1349, 0x001F },
	{ 0x18E0, 0x4004 }, { 0x122B, 0x241C }, { 0x1305, 0xC000 },
	{ 0x13F0, 0x0000 },
};

static const struct rtl8365mb_jam_tbl_entry rtl8365mb_init_jam_common[] = {
	{ 0x1200, 0x7FCB }, { 0x0884, 0x0003 }, { 0x06EB, 0x0001 },
	{ 0x03Fa, 0x0007 }, { 0x08C8, 0x00C0 }, { 0x0A30, 0x020E },
	{ 0x0800, 0x0000 }, { 0x0802, 0x0000 }, { 0x09DA, 0x0013 },
	{ 0x1D32, 0x0002 },
};

enum rtl8365mb_phy_interface_mode {
	RTL8365MB_PHY_INTERFACE_MODE_INVAL = 0,
	RTL8365MB_PHY_INTERFACE_MODE_INTERNAL = BIT(0),
	RTL8365MB_PHY_INTERFACE_MODE_MII = BIT(1),
	RTL8365MB_PHY_INTERFACE_MODE_TMII = BIT(2),
	RTL8365MB_PHY_INTERFACE_MODE_RMII = BIT(3),
	RTL8365MB_PHY_INTERFACE_MODE_RGMII = BIT(4),
	RTL8365MB_PHY_INTERFACE_MODE_SGMII = BIT(5),
	RTL8365MB_PHY_INTERFACE_MODE_HSGMII = BIT(6),
};

/**
 * struct rtl8365mb_extint - external interface info
 * @port: the port with an external interface
 * @id: the external interface ID, which is either 0, 1, or 2
 * @supported_interfaces: a bitmask of supported PHY interface modes
 *
 * Represents a mapping: port -> { id, supported_interfaces }. To be embedded
 * in &struct rtl8365mb_chip_info for every port with an external interface.
 */
struct rtl8365mb_extint {
	int port;
	int id;
	unsigned int supported_interfaces;
};

/**
 * struct rtl8365mb_chip_info - static chip-specific info
 * @name: human-readable chip name
 * @chip_id: chip identifier
 * @chip_ver: chip silicon revision
 * @extints: available external interfaces
 * @jam_table: chip-specific initialization jam table
 * @jam_size: size of the chip's jam table
 *
 * These data are specific to a given chip in the family of switches supported
 * by this driver. When adding support for another chip in the family, a new
 * chip info should be added to the rtl8365mb_chip_infos array.
 */
struct rtl8365mb_chip_info {
	const char *name;
	u32 chip_id;
	u32 chip_ver;
	const struct rtl8365mb_extint extints[RTL8365MB_MAX_NUM_EXTINTS];
	const struct rtl8365mb_jam_tbl_entry *jam_table;
	size_t jam_size;
};

/* Chip info for each supported switch in the family */
#define PHY_INTF(_mode) (RTL8365MB_PHY_INTERFACE_MODE_ ## _mode)
static const struct rtl8365mb_chip_info rtl8365mb_chip_infos[] = {
	{
		.name = "RTL8365MB-VC",
		.chip_id = 0x6367,
		.chip_ver = 0x0040,
		.extints = {
			{ 6, 1, PHY_INTF(MII) | PHY_INTF(TMII) |
				PHY_INTF(RMII) | PHY_INTF(RGMII) },
		},
		.jam_table = rtl8365mb_init_jam_8365mb_vc,
		.jam_size = ARRAY_SIZE(rtl8365mb_init_jam_8365mb_vc),
	},
	{
		.name = "RTL8367S",
		.chip_id = 0x6367,
		.chip_ver = 0x00A0,
		.extints = {
			{ 6, 1, PHY_INTF(SGMII) | PHY_INTF(HSGMII) },
			{ 7, 2, PHY_INTF(MII) | PHY_INTF(TMII) |
				PHY_INTF(RMII) | PHY_INTF(RGMII) },
		},
		.jam_table = rtl8365mb_init_jam_8365mb_vc,
		.jam_size = ARRAY_SIZE(rtl8365mb_init_jam_8365mb_vc),
	},
	{
		.name = "RTL8367RB-VB",
		.chip_id = 0x6367,
		.chip_ver = 0x0020,
		.extints = {
			{ 6, 1, PHY_INTF(MII) | PHY_INTF(TMII) |
				PHY_INTF(RMII) | PHY_INTF(RGMII) },
			{ 7, 2, PHY_INTF(MII) | PHY_INTF(TMII) |
				PHY_INTF(RMII) | PHY_INTF(RGMII) },
		},
		.jam_table = rtl8365mb_init_jam_8365mb_vc,
		.jam_size = ARRAY_SIZE(rtl8365mb_init_jam_8365mb_vc),
	},
};

enum rtl8365mb_stp_state {
	RTL8365MB_STP_STATE_DISABLED = 0,
	RTL8365MB_STP_STATE_BLOCKING = 1,
	RTL8365MB_STP_STATE_LEARNING = 2,
	RTL8365MB_STP_STATE_FORWARDING = 3,
};

enum rtl8365mb_cpu_insert {
	RTL8365MB_CPU_INSERT_TO_ALL = 0,
	RTL8365MB_CPU_INSERT_TO_TRAPPING = 1,
	RTL8365MB_CPU_INSERT_TO_NONE = 2,
};

enum rtl8365mb_cpu_position {
	RTL8365MB_CPU_POS_AFTER_SA = 0,
	RTL8365MB_CPU_POS_BEFORE_CRC = 1,
};

enum rtl8365mb_cpu_format {
	RTL8365MB_CPU_FORMAT_8BYTES = 0,
	RTL8365MB_CPU_FORMAT_4BYTES = 1,
};

enum rtl8365mb_cpu_rxlen {
	RTL8365MB_CPU_RXLEN_72BYTES = 0,
	RTL8365MB_CPU_RXLEN_64BYTES = 1,
};

/**
 * struct rtl8365mb_cpu - CPU port configuration
 * @enable: enable/disable hardware insertion of CPU tag in switch->CPU frames
 * @mask: port mask of ports that parse should parse CPU tags
 * @trap_port: forward trapped frames to this port
 * @insert: CPU tag insertion mode in switch->CPU frames
 * @position: position of CPU tag in frame
 * @rx_length: minimum CPU RX length
 * @format: CPU tag format
 *
 * Represents the CPU tagging and CPU port configuration of the switch. These
 * settings are configurable at runtime.
 */
struct rtl8365mb_cpu {
	bool enable;
	u32 mask;
	u32 trap_port;
	enum rtl8365mb_cpu_insert insert;
	enum rtl8365mb_cpu_position position;
	enum rtl8365mb_cpu_rxlen rx_length;
	enum rtl8365mb_cpu_format format;
};

/**
 * struct rtl8365mb_port - private per-port data
 * @priv: pointer to parent realtek_priv data
 * @index: DSA port index, same as dsa_port::index
 * @stats: link statistics populated by rtl8365mb_stats_poll, ready for atomic
 *         access via rtl8365mb_get_stats64
 * @stats_lock: protect the stats structure during read/update
 * @mib_work: delayed work for polling MIB counters
 */
struct rtl8365mb_port {
	struct realtek_priv *priv;
	unsigned int index;
	struct rtnl_link_stats64 stats;
	spinlock_t stats_lock;
	struct delayed_work mib_work;
};

/**
 * struct rtl8365mb - driver private data
 * @priv: pointer to parent realtek_priv data
 * @irq: registered IRQ or zero
 * @chip_info: chip-specific info about the attached switch
 * @cpu: CPU tagging and CPU port configuration for this chip
 * @mib_lock: prevent concurrent reads of MIB counters
 * @table_lock: prevent concurrent reads of tables
 * @ports: per-port data
 *
 * Private data for this driver.
 */
struct rtl8365mb {
	struct realtek_priv *priv;
	int irq;
	const struct rtl8365mb_chip_info *chip_info;
	struct rtl8365mb_cpu cpu;
	struct mutex mib_lock;
	struct mutex table_lock;
	struct rtl8365mb_port ports[RTL8365MB_MAX_NUM_PORTS];
};

static int rtl8365mb_phy_poll_busy(struct realtek_priv *priv)
{
	u32 val;

	return regmap_read_poll_timeout(priv->map_nolock,
					RTL8365MB_INDIRECT_ACCESS_STATUS_REG,
					val, !val, 10, 100);
}

static int rtl8365mb_phy_ocp_prepare(struct realtek_priv *priv, int phy,
				     u32 ocp_addr)
{
	u32 val;
	int ret;

	/* Set OCP prefix */
	val = FIELD_GET(RTL8365MB_PHY_OCP_ADDR_PREFIX_MASK, ocp_addr);
	ret = regmap_update_bits(
		priv->map_nolock, RTL8365MB_GPHY_OCP_MSB_0_REG,
		RTL8365MB_GPHY_OCP_MSB_0_CFG_CPU_OCPADR_MASK,
		FIELD_PREP(RTL8365MB_GPHY_OCP_MSB_0_CFG_CPU_OCPADR_MASK, val));
	if (ret)
		return ret;

	/* Set PHY register address */
	val = RTL8365MB_PHY_BASE;
	val |= FIELD_PREP(RTL8365MB_INDIRECT_ACCESS_ADDRESS_PHYNUM_MASK, phy);
	val |= FIELD_PREP(RTL8365MB_INDIRECT_ACCESS_ADDRESS_OCPADR_5_1_MASK,
			  ocp_addr >> 1);
	val |= FIELD_PREP(RTL8365MB_INDIRECT_ACCESS_ADDRESS_OCPADR_9_6_MASK,
			  ocp_addr >> 6);
	ret = regmap_write(priv->map_nolock,
			   RTL8365MB_INDIRECT_ACCESS_ADDRESS_REG, val);
	if (ret)
		return ret;

	return 0;
}

static int rtl8365mb_phy_ocp_read(struct realtek_priv *priv, int phy,
				  u32 ocp_addr, u16 *data)
{
	u32 val;
	int ret;

	rtl83xx_lock(priv);

	ret = rtl8365mb_phy_poll_busy(priv);
	if (ret)
		goto out;

	ret = rtl8365mb_phy_ocp_prepare(priv, phy, ocp_addr);
	if (ret)
		goto out;

	/* Execute read operation */
	val = FIELD_PREP(RTL8365MB_INDIRECT_ACCESS_CTRL_CMD_MASK,
			 RTL8365MB_INDIRECT_ACCESS_CTRL_CMD_VALUE) |
	      FIELD_PREP(RTL8365MB_INDIRECT_ACCESS_CTRL_RW_MASK,
			 RTL8365MB_INDIRECT_ACCESS_CTRL_RW_READ);
	ret = regmap_write(priv->map_nolock, RTL8365MB_INDIRECT_ACCESS_CTRL_REG,
			   val);
	if (ret)
		goto out;

	ret = rtl8365mb_phy_poll_busy(priv);
	if (ret)
		goto out;

	/* Get PHY register data */
	ret = regmap_read(priv->map_nolock,
			  RTL8365MB_INDIRECT_ACCESS_READ_DATA_REG, &val);
	if (ret)
		goto out;

	*data = val & 0xFFFF;

out:
	rtl83xx_unlock(priv);

	return ret;
}

static int rtl8365mb_phy_ocp_write(struct realtek_priv *priv, int phy,
				   u32 ocp_addr, u16 data)
{
	u32 val;
	int ret;

	rtl83xx_lock(priv);

	ret = rtl8365mb_phy_poll_busy(priv);
	if (ret)
		goto out;

	ret = rtl8365mb_phy_ocp_prepare(priv, phy, ocp_addr);
	if (ret)
		goto out;

	/* Set PHY register data */
	ret = regmap_write(priv->map_nolock,
			   RTL8365MB_INDIRECT_ACCESS_WRITE_DATA_REG, data);
	if (ret)
		goto out;

	/* Execute write operation */
	val = FIELD_PREP(RTL8365MB_INDIRECT_ACCESS_CTRL_CMD_MASK,
			 RTL8365MB_INDIRECT_ACCESS_CTRL_CMD_VALUE) |
	      FIELD_PREP(RTL8365MB_INDIRECT_ACCESS_CTRL_RW_MASK,
			 RTL8365MB_INDIRECT_ACCESS_CTRL_RW_WRITE);
	ret = regmap_write(priv->map_nolock, RTL8365MB_INDIRECT_ACCESS_CTRL_REG,
			   val);
	if (ret)
		goto out;

	ret = rtl8365mb_phy_poll_busy(priv);
	if (ret)
		goto out;

out:
	rtl83xx_unlock(priv);

	return 0;
}

static int rtl8365mb_phy_read(struct realtek_priv *priv, int phy, int regnum)
{
	u32 ocp_addr;
	u16 val;
	int ret;

	if (phy > RTL8365MB_PHYADDRMAX)
		return -EINVAL;

	if (regnum > RTL8365MB_PHYREGMAX)
		return -EINVAL;

	ocp_addr = RTL8365MB_PHY_OCP_ADDR_PHYREG_BASE + regnum * 2;

	ret = rtl8365mb_phy_ocp_read(priv, phy, ocp_addr, &val);
	if (ret) {
		dev_err(priv->dev,
			"failed to read PHY%d reg %02x @ %04x, ret %d\n", phy,
			regnum, ocp_addr, ret);
		return ret;
	}

	dev_dbg(priv->dev, "read PHY%d register 0x%02x @ %04x, val <- %04x\n",
		phy, regnum, ocp_addr, val);

	return val;
}

static int rtl8365mb_phy_write(struct realtek_priv *priv, int phy, int regnum,
			       u16 val)
{
	u32 ocp_addr;
	int ret;

	if (phy > RTL8365MB_PHYADDRMAX)
		return -EINVAL;

	if (regnum > RTL8365MB_PHYREGMAX)
		return -EINVAL;

	ocp_addr = RTL8365MB_PHY_OCP_ADDR_PHYREG_BASE + regnum * 2;

	ret = rtl8365mb_phy_ocp_write(priv, phy, ocp_addr, val);
	if (ret) {
		dev_err(priv->dev,
			"failed to write PHY%d reg %02x @ %04x, ret %d\n", phy,
			regnum, ocp_addr, ret);
		return ret;
	}

	dev_dbg(priv->dev, "write PHY%d register 0x%02x @ %04x, val -> %04x\n",
		phy, regnum, ocp_addr, val);

	return 0;
}

static int rtl8365mb_table_access(struct realtek_priv *priv,
				  enum rtl8365mb_table table,
				  enum rtl8365mb_table_op op,
				  u16 index, u16 *val)
{
	struct rtl8365mb *mb = priv->chip_data;
	size_t val_size;
	u32 lut;
	int ret;

	if (table >= RTL8365MB_NUM_TABLES)
		return -EINVAL;

	if (!FIELD_FIT(RTL8365MB_TABLE_ACCESS_ADDR_REG_MASK, index))
		return -EINVAL;

	val_size = rtl8365mb_table_entry_size[table];

	mutex_lock(&mb->table_lock);
	if (op == RTL8365MB_TABLE_WRITE) {
		ret = regmap_bulk_write(priv->map,
					RTL8365MB_TABLE_WRITE_DATA_REG_BASE,
					val, val_size == 10 ? 9 : val_size);

		if (ret)
			goto out;

		/* 10th register uses only 4 less significant bits (TODO: not tested) */
		if (val_size == 10)
			ret = regmap_update_bits(priv->map,
					RTL8365MB_TABLE_WRITE_DATA_REG_BASE + 9,
					RTL8365MB_TABLE_10TH_DATA_REG_MASK,
					FIELD_PREP(RTL8365MB_TABLE_10TH_DATA_REG_MASK, val[9]));
		if (ret)
			goto out;

	} else {
		/* vendor driver checks busy flag only on read */
		ret = regmap_read_poll_timeout(priv->map,
				RTL8365MB_TABLE_LUT_REG, lut,
				!FIELD_PREP(RTL8365MB_TABLE_LUT_BUSY_FLAG_MASK, lut),
				10, 100);
		if (ret)
			goto out;
	}

	ret = regmap_write(priv->map,
			   RTL8365MB_TABLE_ACCESS_ADDR_REG,
			   FIELD_PREP(RTL8365MB_TABLE_ACCESS_ADDR_REG_MASK, index));
	if (ret)
		goto out;

	ret = regmap_update_bits(priv->map,
			RTL8365MB_TABLE_CONTROL_REG,
			RTL8365MB_TABLE_CONTROL_COMMAND_MASK |
			RTL8365MB_TABLE_CONTROL_TABLE_MASK,
			FIELD_PREP(RTL8365MB_TABLE_CONTROL_COMMAND_MASK, op) |
			FIELD_PREP(RTL8365MB_TABLE_CONTROL_TABLE_MASK, table));
	if (ret)
		goto out;

	if (op == RTL8365MB_TABLE_READ) {
		ret = regmap_read(priv->map,
				  RTL8365MB_TABLE_LUT_REG,
				  &lut);

		ret = regmap_read_poll_timeout(priv->map,
				RTL8365MB_TABLE_LUT_REG, lut,
				!FIELD_PREP(RTL8365MB_TABLE_LUT_BUSY_FLAG_MASK,
					    lut),
				10, 100);
		if (ret)
			goto out;

		ret = regmap_bulk_read(priv->map,
				       RTL8365MB_TABLE_READ_DATA_REG_BASE,
				       val, val_size);
		if (ret)
			goto out;

		/* 10th register uses only 4 less significant bits (TODO: not tested) */
		if (val_size == 10)
			val[9] &= RTL8365MB_TABLE_10TH_DATA_REG_MASK;
	}

out:
	mutex_unlock(&mb->table_lock);
	return ret;
}

static int rtl8365mb_vlan_filtering(struct dsa_switch *ds, int port,
				    bool vlan_filtering,
				    struct netlink_ext_ack *extack)
{
	struct realtek_priv *priv = ds->priv;

	dev_dbg(priv->dev, "port %d: %s VLAN filtering\n", port,
		vlan_filtering ? "enable" : "disable");

	/* If the port is not in the member set, the frame will be dropped */
	return regmap_update_bits(priv->map, RTL8365MB_VLAN_INGRESS_REG,
				 BIT(port), vlan_filtering ? BIT(port) : 0);
}

static void rtl8365mb_buf_vlan4k(u16 *buf, struct rtl8366_vlan_4k *vlan4k)
{
	/* vlan4k.vid = vlan->vid; */
	vlan4k->member = FIELD_GET(RTL8365MB_VLAN_4K_CONF0_MEMBERS_LS_MASK, buf[0]) |
		  (FIELD_GET(RTL8365MB_VLAN_4K_CONF2_MEMBERS_MS_MASK, buf[2]) <<
		   FIELD_WIDTH(RTL8365MB_VLAN_4K_CONF0_MEMBERS_LS_MASK));
	vlan4k->untag = FIELD_GET(RTL8365MB_VLAN_4K_CONF0_UNTAG_LS_MASK, buf[0]) |
		   (FIELD_GET(RTL8365MB_VLAN_4K_CONF2_UNTAG_MS_MASK, buf[2]) <<
		    FIELD_WIDTH(RTL8365MB_VLAN_4K_CONF0_UNTAG_LS_MASK));

	vlan4k->fid = FIELD_GET(RTL8365MB_VLAN_4K_CONF1_FID_MSI_MASK, buf[1]);
	/* vlan4k->vlan_based_pri_enabled = FIELD_GET(RTL8365MB_VLAN_4K_CONF1_VBPEN_MASK, buf[1]); */
	/* vlan4k->priority = FIELD_GET(RTL8365MB_VLAN_4K_CONF1_VBPRI_MASK, buf[1]); */
	/* vlan4k->vlan_policy_enabled = FIELD_GET(RTL8365MB_VLAN_4K_CONF1_ENVLANPOL_MASK, buf[1]); */
	/* vlan4k->meter_idx = FIELD_GET(RTL8365MB_VLAN_4K_CONF1_METER_IDX_LS_MASK, buf[1]) |
		    (FIELD_GET(RTL8365MB_VLAN_4K_CONF2_METER_IDX_MS_MASK, buf[2]) <<
		     FIELD_WIDTH(RTL8365MB_VLAN_4K_CONF1_METER_IDX_LS_MASK));
	*/
}

static void rtl8365mb_vlan4k_buf(struct rtl8366_vlan_4k *vlan4k, u16 *buf)
{
	buf[0] &= ~RTL8365MB_VLAN_4K_CONF0_MEMBERS_LS_MASK;
	buf[0] |= FIELD_PREP(RTL8365MB_VLAN_4K_CONF0_MEMBERS_LS_MASK,
				    vlan4k->member & FIELD_MAX(RTL8365MB_VLAN_4K_CONF0_MEMBERS_LS_MASK));
	buf[2] &= ~RTL8365MB_VLAN_4K_CONF2_MEMBERS_MS_MASK;
	buf[2] |= FIELD_PREP(RTL8365MB_VLAN_4K_CONF2_MEMBERS_MS_MASK,
				    vlan4k->member >> FIELD_WIDTH(RTL8365MB_VLAN_4K_CONF0_MEMBERS_LS_MASK));

	buf[1] &= ~RTL8365MB_VLAN_4K_CONF1_FID_MSI_MASK;
	buf[1] |= FIELD_PREP(RTL8365MB_VLAN_4K_CONF1_FID_MSI_MASK,
				    vlan4k->fid);

	/*buf[1] &= ~RTL8365MB_VLAN_4K_CONF1_VBPRI_MASK;
	buf[1] |= FIELD_PREP(RTL8365MB_VLAN_4K_CONF1_VBPRI_MASK,
				    vlan4k->priority);*/

	buf[0] &= ~RTL8365MB_VLAN_4K_CONF0_UNTAG_LS_MASK;
	buf[0] |= FIELD_PREP(RTL8365MB_VLAN_4K_CONF0_UNTAG_LS_MASK,
				    vlan4k->untag & FIELD_MAX(RTL8365MB_VLAN_4K_CONF0_UNTAG_LS_MASK));
	buf[2] &= ~RTL8365MB_VLAN_4K_CONF2_UNTAG_MS_MASK;
	buf[2] |= FIELD_PREP(RTL8365MB_VLAN_4K_CONF2_UNTAG_MS_MASK,
				    vlan4k->untag >> FIELD_WIDTH(RTL8365MB_VLAN_4K_CONF0_UNTAG_LS_MASK));
}

static int rtl8365mb_vlan4k_set(struct dsa_switch *ds, int port,
			      const struct switchdev_obj_port_vlan *vlan,
			      struct netlink_ext_ack *extack, bool include)
{
	u16 vlan_entry[RTL8365MB_VLAN_4K_ENTRY_SIZE] = {0, 0, 0};
	struct realtek_priv *priv = ds->priv;
	struct rtl8366_vlan_4k vlan4k = {0};
	int ret;

	dev_dbg(priv->dev, "%s VLAN %u 4K on port %d\n",
		include?"add":"del",
		vlan->vid, port);

	if (vlan->vid > RTL8365MB_MAX_4K_VID) {
		if (extack)
			NL_SET_ERR_MSG_FMT_MOD(extack, \
				   "VLAN ID greater than %d", \
				    RTL8365MB_MAX_4K_VID);
		return -EINVAL;
	}

	ret = rtl8365mb_table_access(priv, RTL8365MB_TABLE_CVLAN,
				     RTL8365MB_TABLE_READ, vlan->vid,
				     vlan_entry);
	if (ret) {
		if (extack)
			NL_SET_ERR_MSG_MOD(extack, \
					   "Failed to read VLAN 4k table");
		return ret;
	}

	/* vlan4k.vid = vlan->vid; */
	rtl8365mb_buf_vlan4k(vlan_entry, &vlan4k);

	if (include)
		vlan4k.member |= BIT(port);
	else
		vlan4k.member &= ~BIT(port);

	if (include && (vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED)) {
		vlan4k.untag |= BIT(port);
	} else {
		vlan4k.untag &= ~BIT(port);
	}

	rtl8365mb_vlan4k_buf(&vlan4k, vlan_entry);

	ret = rtl8365mb_table_access(priv, RTL8365MB_TABLE_CVLAN,
				     RTL8365MB_TABLE_WRITE, vlan->vid,
				     vlan_entry);

	return ret;
}

static void rtl8365mb_buf_vlanmc(u16 *buf, struct rtl8366_vlan_mc *vlanmc)
{
	vlanmc->member = FIELD_GET(RTL8365MB_VLAN_MC_CONF0_MEMBERS_MSK, buf[0]);
	/* vlan_mc does not have untag info in this device */
	/* vlanmc->untag = ?? */
	vlanmc->fid = FIELD_GET(RTL8365MB_VLAN_MC_CONF1_FID_MSI_MSK, buf[1]);
	vlanmc->priority = FIELD_GET(RTL8365MB_VLAN_MC_CONF2_VBPRI_MSK, buf[2]);
	vlanmc->vid = FIELD_GET(RTL8365MB_VLAN_MC_CONF3_EVID_MSK, buf[3]);
}

static void rtl8365mb_vlanmc_buf(struct rtl8366_vlan_mc *vlanmc, u16 *buf)
{
	buf[0] &= ~RTL8365MB_VLAN_MC_CONF0_MEMBERS_MSK;
	buf[0] |= FIELD_PREP(RTL8365MB_VLAN_MC_CONF0_MEMBERS_MSK, vlanmc->member);

	buf[1] &= ~RTL8365MB_VLAN_MC_CONF1_FID_MSI_MSK;
	buf[1] |= FIELD_PREP(RTL8365MB_VLAN_MC_CONF1_FID_MSI_MSK, vlanmc->fid);

	buf[2] &= ~RTL8365MB_VLAN_MC_CONF2_VBPRI_MSK;
	buf[2] |= FIELD_PREP(RTL8365MB_VLAN_MC_CONF2_VBPRI_MSK, vlanmc->priority);

	buf[3] &= ~RTL8365MB_VLAN_MC_CONF3_EVID_MSK;
	buf[3] |= FIELD_PREP(RTL8365MB_VLAN_MC_CONF3_EVID_MSK, vlanmc->vid);
}

static int rtl8365mb_vlanmc_set(struct dsa_switch *ds, int port,
			      const struct switchdev_obj_port_vlan *vlan,
			      struct netlink_ext_ack *extack, bool include)
{
	u16 vlan_entry[RTL8365MB_VLAN_MC_CONF_ENTRY_SIZE] = {0};
	enum rtl8365mb_frame_type accepted_frame;
	struct realtek_priv *priv = ds->priv;
	struct rtl8366_vlan_4k vlan4k = {0};
	struct rtl8366_vlan_mc vlanmc = {0};
	u32 data;
	bool accepted_frame_changed = false;
	int first_unused = -1;
	int pvid_vlanmc_idx, vlanmc_idx;
	u16 evid;
	int ret;

	dev_dbg(priv->dev, "%s VLAN %u MC on port %d\n",
		include?"add":"del",
		vlan->vid, port);

	if (vlan->vid > RTL8365MB_MAX_MC_VID) {
		if (extack)
			NL_SET_ERR_MSG_FMT_MOD(extack, "VLAN ID greater than %d",
					RTL8365MB_MAX_MC_VID);

		return -EINVAL;
	}

	/* look for existing entry or an empty one */
	/* reserve vlanmc_idx=0 to the non-member (see rtl8365mb_vlan_init)  */
	for (vlanmc_idx = 1; vlanmc_idx < RTL8365MB_VLAN_MC_CONF_SIZE; vlanmc_idx++) {
		ret = regmap_bulk_read(priv->map,
				       RTL8365MB_VLAN_MC_CONF_REG(vlanmc_idx),
				       vlan_entry,
				       RTL8365MB_VLAN_MC_CONF_ENTRY_SIZE);
		if (ret) {
			if (extack)
				NL_SET_ERR_MSG_MOD(extack,
						   "Failed to read vlan MC entry");
			return ret;
		}

		evid = FIELD_GET(RTL8365MB_VLAN_MC_CONF3_EVID_MSK, vlan_entry[3]);

		if (evid == vlan->vid)
			break;

		if (evid == 0x0 && first_unused < 0)
			first_unused = vlanmc_idx;
	}

	if (vlanmc_idx == RTL8365MB_VLAN_MC_CONF_SIZE) {
		/* clear last read vlan_entry */
		memset(vlan_entry, 0, sizeof(vlan_entry));

		/* for now, vlan_mc is only required for PVID */
		if (!(vlan->flags & BRIDGE_VLAN_INFO_PVID)) {
			dev_dbg(priv->dev, "Not creating VlanMC for vlan %d until a port uses PVID (%d does not)\n",
			vlan->vid, port);
			return 0;
		}

		if (first_unused < 0) {
			if (extack)
				NL_SET_ERR_MSG_FMT_MOD(extack,
					   "All VLAN MC entries (%d) are in use.", \
					   RTL8365MB_VLAN_MC_CONF_SIZE);
			return -EINVAL;
		}

		/* we might have missed members without PVID before
		 * get them now from vlan4k and add to vlanmc */
		if (vlan->vid <= RTL8365MB_MAX_4K_VID) {
			ret = rtl8365mb_table_access(priv,
						     RTL8365MB_TABLE_CVLAN,
						     RTL8365MB_TABLE_READ,
						     vlan->vid, vlan_entry);
			if (ret) {
				if (extack)
					NL_SET_ERR_MSG_MOD(extack,
						"Failed to read VLAN 4k table");
				return ret;
			}

			rtl8365mb_buf_vlan4k(vlan_entry, &vlan4k);
		}

		vlanmc_idx = first_unused;
	}

	ret = regmap_read(priv->map,
			  RTL8365MB_VLAN_PVID_CTRL_REG(port),
			  &data);
	if (ret) {
		if (extack)
			NL_SET_ERR_MSG_MOD(extack,
					   "Failed to read port PVID");
		return ret;
	}

	pvid_vlanmc_idx = (data & RTL8365MB_VLAN_PVID_CTRL_MASK(port))
			  >> RTL8365MB_VLAN_PVID_CTRL_OFFSET(port);

	ret = regmap_read(priv->map,
		RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_REG(port),
		&data);
	if (ret) {
		if (extack)
			NL_SET_ERR_MSG_MOD(extack,
				"Failed to read port accepted frames");
		return ret;
	}

	accepted_frame = (data & RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_MASK(port))
			  >> RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_OFFSET(port);

	dev_dbg(priv->dev, "Current port PVID VLANMC index %d, acpt frame %d\n",
		pvid_vlanmc_idx, accepted_frame);

	rtl8365mb_buf_vlanmc(vlan_entry, &vlanmc);

	/* for new vlans, add current vlan4k members */
	vlanmc.member |= vlan4k.member;

	if (include)
		vlanmc.member |= BIT(port);
	else
		vlanmc.member &= ~BIT(port);
	vlanmc.vid = vlan->vid;

	/* DSA adds CPU port to the vlan but do not remove it when there is
	 * no more ports (user or dsa). Ignore the CPU port while checking
	 * if a vlan is empty
	 *
	 * TODO: There is a second situation that we could clear the vlanmc
	 * entry when no more ports are using PVID. We would need to keep a
	 * record about which pvid each port are usings or iterate over PVID
	 * registers where accepted_frame == RTL8365MB_FRAME_TYPE_ANY_FRAME
	 */
	if (!include && !(vlanmc.member & ~dsa_cpu_ports(ds))) {
		dev_dbg(priv->dev, "Clearing Vlan4K index %d previously used by VID %d\n",
			vlanmc_idx, vlan->vid);
		memset(vlan_entry, 0, sizeof(vlan_entry));
	} else {
		rtl8365mb_vlanmc_buf(&vlanmc, vlan_entry);
	}

	ret = regmap_bulk_write(priv->map,
		       RTL8365MB_VLAN_MC_CONF_REG(vlanmc_idx),
		       vlan_entry,
		       RTL8365MB_VLAN_MC_CONF_ENTRY_SIZE);

	if (ret) {
		if (extack)
			NL_SET_ERR_MSG_MOD(extack,
				   "Failed to write vlan MC entry");
		return ret;
	}

	/* Adjust accepted frame types only when adding a PVID vlan and untagged
	 * frames are ignored or when removing a vlan used as PVID */
	if (!include) {
		if ((accepted_frame == RTL8365MB_FRAME_TYPE_ANY_FRAME) &&
				(pvid_vlanmc_idx == vlanmc_idx)) {
			accepted_frame = RTL8365MB_FRAME_TYPE_TAGGED_ONLY;
			accepted_frame_changed = true;
		}
	} else if (vlan->flags & BRIDGE_VLAN_INFO_PVID) {
		if (accepted_frame == RTL8365MB_FRAME_TYPE_TAGGED_ONLY) {
			accepted_frame = RTL8365MB_FRAME_TYPE_ANY_FRAME;
			accepted_frame_changed = true;
		}
		/* Only update PVID if it is setting a different VLAN. PVID is not
		 * enough to let a frame in without being a member of vlan PVID */
		if (vlanmc_idx != pvid_vlanmc_idx) {
			dev_dbg(priv->dev, "Set port %d PVID to %d (@ %d idx)\n",
				port, vlan->vid, vlanmc_idx);

			ret = regmap_update_bits(priv->map,
				 RTL8365MB_VLAN_PVID_CTRL_REG(port),
				 RTL8365MB_VLAN_PVID_CTRL_MASK(port),
				 vlanmc_idx << RTL8365MB_VLAN_PVID_CTRL_OFFSET(port));
			if (ret) {
				if (extack)
					NL_SET_ERR_MSG_MOD(extack,
						   "Vlan member was updated but"
						   " setting port PVID failed");
				return ret;
			}
		}
	}
	if (accepted_frame_changed) {
		dev_dbg(priv->dev, "Set port %d acpt frame to %d\n",
			port, accepted_frame);

		/* Even if ACCEPT_FRAME_TYPE_ANY, the switch will still check if the port
		 * is a member of vlan PVID
		 */
		ret = regmap_update_bits(priv->map,
			RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_REG(port),
			RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_MASK(port),
			accepted_frame << RTL8365MB_VLAN_ACCEPT_FRAME_TYPE_OFFSET(port));
		if (ret) {
			if (extack)
				NL_SET_ERR_MSG_MOD(extack,
					  "Vlan member and PVID were updated but "
					  "setting port accepted frame types failed");
			return ret;
		}
	}

	return ret;
}

static int rtl8365mb_vlan_add(struct dsa_switch *ds, int port,
			      const struct switchdev_obj_port_vlan *vlan,
			      struct netlink_ext_ack *extack)
{
	bool untagged = !!(vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED);
	bool pvid = !!(vlan->flags & BRIDGE_VLAN_INFO_PVID);
	struct realtek_priv *priv = ds->priv;
	int ret;

	dev_dbg(priv->dev, "add VLAN %d on port %d, %s, %s\n",
		vlan->vid, port, untagged ? "untagged" : "tagged",
		pvid ? "PVID" : "no PVID");

	/* Vlan mc knowns nothing about untagged but it is required for pvid */
	ret = rtl8365mb_vlanmc_set(ds, port, vlan, extack, 1);
	if (ret)
		return ret;

	/* vlan4k knowns nothing about PVID */
	ret = rtl8365mb_vlan4k_set(ds, port, vlan, extack, 1);
	if (ret) {
		rtl8365mb_vlanmc_set(ds, port, vlan, extack, 0);
		return ret;
	}

	/* TODO: fid? */
	//ret_t rtl8367c_getAsicPortBasedFid(rtk_uint32 port, rtk_uint32* pFid)

	return 0;
}

static int rtl8365mb_vlan_del(struct dsa_switch *ds, int port,
			      const struct switchdev_obj_port_vlan *vlan)
{
	struct realtek_priv *priv = ds->priv;
	int ret, ret2;

	dev_dbg(priv->dev, "del VLAN %d on port %d\n", vlan->vid, port);

	ret = rtl8365mb_vlan4k_set(ds, port, vlan, NULL, 0);
	/* clean vlan mc if present */
	ret2 = rtl8365mb_vlanmc_set(ds, port, vlan, NULL, 0);

	return ret || ret2;
}

static const struct rtl8365mb_extint *
rtl8365mb_get_port_extint(struct realtek_priv *priv, int port)
{
	struct rtl8365mb *mb = priv->chip_data;
	int i;

	for (i = 0; i < RTL8365MB_MAX_NUM_EXTINTS; i++) {
		const struct rtl8365mb_extint *extint =
			&mb->chip_info->extints[i];

		if (!extint->supported_interfaces)
			continue;

		if (extint->port == port)
			return extint;
	}

	return NULL;
}

static enum dsa_tag_protocol
rtl8365mb_get_tag_protocol(struct dsa_switch *ds, int port,
			   enum dsa_tag_protocol mp)
{
	struct realtek_priv *priv = ds->priv;
	struct rtl8365mb_cpu *cpu;
	struct rtl8365mb *mb;

	mb = priv->chip_data;
	cpu = &mb->cpu;

	if (cpu->position == RTL8365MB_CPU_POS_BEFORE_CRC)
		return DSA_TAG_PROTO_RTL8_4T;

	return DSA_TAG_PROTO_RTL8_4;
}

static int rtl8365mb_ext_config_rgmii(struct realtek_priv *priv, int port,
				      phy_interface_t interface)
{
	const struct rtl8365mb_extint *extint =
		rtl8365mb_get_port_extint(priv, port);
	struct dsa_switch *ds = &priv->ds;
	struct device_node *dn;
	struct dsa_port *dp;
	int tx_delay = 0;
	int rx_delay = 0;
	u32 val;
	int ret;

	if (!extint)
		return -ENODEV;

	dp = dsa_to_port(ds, port);
	dn = dp->dn;

	/* Set the RGMII TX/RX delay
	 *
	 * The Realtek vendor driver indicates the following possible
	 * configuration settings:
	 *
	 *   TX delay:
	 *     0 = no delay, 1 = 2 ns delay
	 *   RX delay:
	 *     0 = no delay, 7 = maximum delay
	 *     Each step is approximately 0.3 ns, so the maximum delay is about
	 *     2.1 ns.
	 *
	 * The vendor driver also states that this must be configured *before*
	 * forcing the external interface into a particular mode, which is done
	 * in the rtl8365mb_phylink_mac_link_{up,down} functions.
	 *
	 * Only configure an RGMII TX (resp. RX) delay if the
	 * tx-internal-delay-ps (resp. rx-internal-delay-ps) OF property is
	 * specified. We ignore the detail of the RGMII interface mode
	 * (RGMII_{RXID, TXID, etc.}), as this is considered to be a PHY-only
	 * property.
	 */
	if (!of_property_read_u32(dn, "tx-internal-delay-ps", &val)) {
		val = val / 1000; /* convert to ns */

		if (val == 0 || val == 2)
			tx_delay = val / 2;
		else
			dev_warn(priv->dev,
				 "RGMII TX delay must be 0 or 2 ns\n");
	}

	if (!of_property_read_u32(dn, "rx-internal-delay-ps", &val)) {
		val = DIV_ROUND_CLOSEST(val, 300); /* convert to 0.3 ns step */

		if (val <= 7)
			rx_delay = val;
		else
			dev_warn(priv->dev,
				 "RGMII RX delay must be 0 to 2.1 ns\n");
	}

	ret = regmap_update_bits(
		priv->map, RTL8365MB_EXT_RGMXF_REG(extint->id),
		RTL8365MB_EXT_RGMXF_TXDELAY_MASK |
			RTL8365MB_EXT_RGMXF_RXDELAY_MASK,
		FIELD_PREP(RTL8365MB_EXT_RGMXF_TXDELAY_MASK, tx_delay) |
			FIELD_PREP(RTL8365MB_EXT_RGMXF_RXDELAY_MASK, rx_delay));
	if (ret)
		return ret;

	ret = regmap_update_bits(
		priv->map, RTL8365MB_DIGITAL_INTERFACE_SELECT_REG(extint->id),
		RTL8365MB_DIGITAL_INTERFACE_SELECT_MODE_MASK(extint->id),
		RTL8365MB_EXT_PORT_MODE_RGMII
			<< RTL8365MB_DIGITAL_INTERFACE_SELECT_MODE_OFFSET(
				   extint->id));
	if (ret)
		return ret;

	return 0;
}

static int rtl8365mb_ext_config_forcemode(struct realtek_priv *priv, int port,
					  bool link, int speed, int duplex,
					  bool tx_pause, bool rx_pause)
{
	const struct rtl8365mb_extint *extint =
		rtl8365mb_get_port_extint(priv, port);
	u32 r_tx_pause;
	u32 r_rx_pause;
	u32 r_duplex;
	u32 r_speed;
	u32 r_link;
	int val;
	int ret;

	if (!extint)
		return -ENODEV;

	if (link) {
		/* Force the link up with the desired configuration */
		r_link = 1;
		r_rx_pause = rx_pause ? 1 : 0;
		r_tx_pause = tx_pause ? 1 : 0;

		if (speed == SPEED_1000) {
			r_speed = RTL8365MB_PORT_SPEED_1000M;
		} else if (speed == SPEED_100) {
			r_speed = RTL8365MB_PORT_SPEED_100M;
		} else if (speed == SPEED_10) {
			r_speed = RTL8365MB_PORT_SPEED_10M;
		} else {
			dev_err(priv->dev, "unsupported port speed %s\n",
				phy_speed_to_str(speed));
			return -EINVAL;
		}

		if (duplex == DUPLEX_FULL) {
			r_duplex = 1;
		} else if (duplex == DUPLEX_HALF) {
			r_duplex = 0;
		} else {
			dev_err(priv->dev, "unsupported duplex %s\n",
				phy_duplex_to_str(duplex));
			return -EINVAL;
		}
	} else {
		/* Force the link down and reset any programmed configuration */
		r_link = 0;
		r_tx_pause = 0;
		r_rx_pause = 0;
		r_speed = 0;
		r_duplex = 0;
	}

	val = FIELD_PREP(RTL8365MB_DIGITAL_INTERFACE_FORCE_EN_MASK, 1) |
	      FIELD_PREP(RTL8365MB_DIGITAL_INTERFACE_FORCE_TXPAUSE_MASK,
			 r_tx_pause) |
	      FIELD_PREP(RTL8365MB_DIGITAL_INTERFACE_FORCE_RXPAUSE_MASK,
			 r_rx_pause) |
	      FIELD_PREP(RTL8365MB_DIGITAL_INTERFACE_FORCE_LINK_MASK, r_link) |
	      FIELD_PREP(RTL8365MB_DIGITAL_INTERFACE_FORCE_DUPLEX_MASK,
			 r_duplex) |
	      FIELD_PREP(RTL8365MB_DIGITAL_INTERFACE_FORCE_SPEED_MASK, r_speed);
	ret = regmap_write(priv->map,
			   RTL8365MB_DIGITAL_INTERFACE_FORCE_REG(extint->id),
			   val);
	if (ret)
		return ret;

	return 0;
}

static void rtl8365mb_phylink_get_caps(struct dsa_switch *ds, int port,
				       struct phylink_config *config)
{
	const struct rtl8365mb_extint *extint =
		rtl8365mb_get_port_extint(ds->priv, port);

	config->mac_capabilities = MAC_SYM_PAUSE | MAC_ASYM_PAUSE |
				   MAC_10 | MAC_100 | MAC_1000FD;

	if (!extint) {
		__set_bit(PHY_INTERFACE_MODE_INTERNAL,
			  config->supported_interfaces);

		/* GMII is the default interface mode for phylib, so
		 * we have to support it for ports with integrated PHY.
		 */
		__set_bit(PHY_INTERFACE_MODE_GMII,
			  config->supported_interfaces);
		return;
	}

	/* Populate according to the modes supported by _this driver_,
	 * not necessarily the modes supported by the hardware, some of
	 * which remain unimplemented.
	 */

	if (extint->supported_interfaces & RTL8365MB_PHY_INTERFACE_MODE_RGMII)
		phy_interface_set_rgmii(config->supported_interfaces);
}

static void rtl8365mb_phylink_mac_config(struct phylink_config *config,
					 unsigned int mode,
					 const struct phylink_link_state *state)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct realtek_priv *priv = dp->ds->priv;
	u8 port = dp->index;
	int ret;

	if (mode != MLO_AN_PHY && mode != MLO_AN_FIXED) {
		dev_err(priv->dev,
			"port %d supports only conventional PHY or fixed-link\n",
			port);
		return;
	}

	if (phy_interface_mode_is_rgmii(state->interface)) {
		ret = rtl8365mb_ext_config_rgmii(priv, port, state->interface);
		if (ret)
			dev_err(priv->dev,
				"failed to configure RGMII mode on port %d: %d\n",
				port, ret);
		return;
	}

	/* TODO: Implement MII and RMII modes, which the RTL8365MB-VC also
	 * supports
	 */
}

static void rtl8365mb_phylink_mac_link_down(struct phylink_config *config,
					    unsigned int mode,
					    phy_interface_t interface)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct realtek_priv *priv = dp->ds->priv;
	struct rtl8365mb_port *p;
	struct rtl8365mb *mb;
	u8 port = dp->index;
	int ret;

	mb = priv->chip_data;
	p = &mb->ports[port];
	cancel_delayed_work_sync(&p->mib_work);

	if (phy_interface_mode_is_rgmii(interface)) {
		ret = rtl8365mb_ext_config_forcemode(priv, port, false, 0, 0,
						     false, false);
		if (ret)
			dev_err(priv->dev,
				"failed to reset forced mode on port %d: %d\n",
				port, ret);

		return;
	}
}

static void rtl8365mb_phylink_mac_link_up(struct phylink_config *config,
					  struct phy_device *phydev,
					  unsigned int mode,
					  phy_interface_t interface,
					  int speed, int duplex, bool tx_pause,
					  bool rx_pause)
{
	struct dsa_port *dp = dsa_phylink_to_port(config);
	struct realtek_priv *priv = dp->ds->priv;
	struct rtl8365mb_port *p;
	struct rtl8365mb *mb;
	u8 port = dp->index;
	int ret;

	mb = priv->chip_data;
	p = &mb->ports[port];
	schedule_delayed_work(&p->mib_work, 0);

	if (phy_interface_mode_is_rgmii(interface)) {
		ret = rtl8365mb_ext_config_forcemode(priv, port, true, speed,
						     duplex, tx_pause,
						     rx_pause);
		if (ret)
			dev_err(priv->dev,
				"failed to force mode on port %d: %d\n", port,
				ret);

		return;
	}
}

static int rtl8365mb_port_change_mtu(struct dsa_switch *ds, int port,
				     int new_mtu)
{
	struct realtek_priv *priv = ds->priv;
	int frame_size;

	/* When a new MTU is set, DSA always sets the CPU port's MTU to the
	 * largest MTU of the user ports. Because the switch only has a global
	 * RX length register, only allowing CPU port here is enough.
	 */
	if (!dsa_is_cpu_port(ds, port))
		return 0;

	frame_size = new_mtu + VLAN_ETH_HLEN + ETH_FCS_LEN;

	dev_dbg(priv->dev, "changing mtu to %d (frame size: %d)\n",
		new_mtu, frame_size);

	return regmap_update_bits(priv->map, RTL8365MB_CFG0_MAX_LEN_REG,
				  RTL8365MB_CFG0_MAX_LEN_MASK,
				  FIELD_PREP(RTL8365MB_CFG0_MAX_LEN_MASK,
					     frame_size));
}

static int rtl8365mb_port_max_mtu(struct dsa_switch *ds, int port)
{
	return RTL8365MB_CFG0_MAX_LEN_MAX - VLAN_ETH_HLEN - ETH_FCS_LEN;
}

static int
rtl8365mb_port_bridge_join(struct dsa_switch *ds, int port,
			   struct dsa_bridge bridge,
			   bool *tx_fwd_offload,
			   struct netlink_ext_ack *extack)
{
	struct realtek_priv *priv = ds->priv;
	unsigned int port_bitmap = 0;
	struct dsa_port *dp;
	int ret;

	dsa_switch_for_each_available_port(dp, ds) {
		/* Current port handled last */
		if (port == dp->index)
			continue;

		/* Not on this bridge */
		if (!dsa_port_offloads_bridge(dp, &bridge))
			continue;

		/* Join this port to each other port on the bridge */
		ret = regmap_update_bits(priv->map,
					 RTL8365MB_PORT_ISOLATION_REG(dp->index),
					 BIT(port), BIT(port));
		if (ret)
			dev_err(priv->dev, "failed to join port %d\n", port);

		port_bitmap |= BIT(dp->index);
	}

	/* Set the bits for the ports we can access */
	return port_bitmap ? regmap_update_bits(priv->map,
				  RTL8365MB_PORT_ISOLATION_REG(port),
				  port_bitmap, port_bitmap) : 0;
}

static void
rtl8365mb_port_bridge_leave(struct dsa_switch *ds, int port,
			    struct dsa_bridge bridge)
{
	struct realtek_priv *priv = ds->priv;
	unsigned int port_bitmap = 0;
	struct dsa_port *dp;
	int ret;

	dsa_switch_for_each_available_port(dp, ds) {
		/* Current port handled last */
		if (port == dp->index)
			continue;

		/* Not on this bridge */
		if (!dsa_port_offloads_bridge(dp, &bridge))
			continue;

		/* Remove this port from any other port on the bridge */
		ret = regmap_update_bits(priv->map,
					 RTL8365MB_PORT_ISOLATION_REG(dp->index),
					 BIT(port), 0);
		if (ret)
			dev_err(priv->dev, "failed to leave port %d\n", port);

		port_bitmap |= BIT(dp->index);
	}

	/* Clear the bits for the ports we can not access, leave ourselves */
	regmap_update_bits(priv->map,
			   RTL8365MB_PORT_ISOLATION_REG(port),
			   port_bitmap, 0);
}

static void rtl8365mb_port_stp_state_set(struct dsa_switch *ds, int port,
					 u8 state)
{
	struct realtek_priv *priv = ds->priv;
	enum rtl8365mb_stp_state val;
	int msti = 0;

	switch (state) {
	case BR_STATE_DISABLED:
		val = RTL8365MB_STP_STATE_DISABLED;
		break;
	case BR_STATE_BLOCKING:
	case BR_STATE_LISTENING:
		val = RTL8365MB_STP_STATE_BLOCKING;
		break;
	case BR_STATE_LEARNING:
		val = RTL8365MB_STP_STATE_LEARNING;
		break;
	case BR_STATE_FORWARDING:
		val = RTL8365MB_STP_STATE_FORWARDING;
		break;
	default:
		dev_err(priv->dev, "invalid STP state: %u\n", state);
		return;
	}

	regmap_update_bits(priv->map, RTL8365MB_MSTI_CTRL_REG(msti, port),
			   RTL8365MB_MSTI_CTRL_PORT_STATE_MASK(port),
			   val << RTL8365MB_MSTI_CTRL_PORT_STATE_OFFSET(port));
}

static int rtl8365mb_port_set_learning(struct realtek_priv *priv, int port,
				       bool enable)
{
	/* Enable/disable learning by limiting the number of L2 addresses the
	 * port can learn. Realtek documentation states that a limit of zero
	 * disables learning. When enabling learning, set it to the chip's
	 * maximum.
	 */
	return regmap_write(priv->map, RTL8365MB_LUT_PORT_LEARN_LIMIT_REG(port),
			    enable ? RTL8365MB_LEARN_LIMIT_MAX : 0);
}

static int
rtl8365mb_port_pre_bridge_flags(struct dsa_switch *ds, int port,
				struct switchdev_brport_flags flags,
				struct netlink_ext_ack *extack)
{
	/* We support enabling/disabling learning */
	if (flags.mask & ~(BR_LEARNING))
		return -EINVAL;

	return 0;
}

static int
rtl8365mb_port_bridge_flags(struct dsa_switch *ds, int port,
			    struct switchdev_brport_flags flags,
			    struct netlink_ext_ack *extack)
{
	if (flags.mask & BR_LEARNING)
		return rtl8365mb_port_set_learning(ds->priv, port,
						   !!(flags.val & BR_LEARNING));

	return 0;
}

static int rtl8365mb_port_set_isolation(struct realtek_priv *priv, int port,
					u32 mask)
{
	return regmap_write(priv->map, RTL8365MB_PORT_ISOLATION_REG(port), mask);
}

static int rtl8365mb_mib_counter_read(struct realtek_priv *priv, int port,
				      u32 offset, u32 length, u64 *mibvalue)
{
	u64 tmpvalue = 0;
	u32 val;
	int ret;
	int i;

	/* The MIB address is an SRAM address. We request a particular address
	 * and then poll the control register before reading the value from some
	 * counter registers.
	 */
	ret = regmap_write(priv->map, RTL8365MB_MIB_ADDRESS_REG,
			   RTL8365MB_MIB_ADDRESS(port, offset));
	if (ret)
		return ret;

	/* Poll for completion */
	ret = regmap_read_poll_timeout(priv->map, RTL8365MB_MIB_CTRL0_REG, val,
				       !(val & RTL8365MB_MIB_CTRL0_BUSY_MASK),
				       10, 100);
	if (ret)
		return ret;

	/* Presumably this indicates a MIB counter read failure */
	if (val & RTL8365MB_MIB_CTRL0_RESET_MASK)
		return -EIO;

	/* There are four MIB counter registers each holding a 16 bit word of a
	 * MIB counter. Depending on the offset, we should read from the upper
	 * two or lower two registers. In case the MIB counter is 4 words, we
	 * read from all four registers.
	 */
	if (length == 4)
		offset = 3;
	else
		offset = (offset + 1) % 4;

	/* Read the MIB counter 16 bits at a time */
	for (i = 0; i < length; i++) {
		ret = regmap_read(priv->map,
				  RTL8365MB_MIB_COUNTER_REG(offset - i), &val);
		if (ret)
			return ret;

		tmpvalue = ((tmpvalue) << 16) | (val & 0xFFFF);
	}

	/* Only commit the result if no error occurred */
	*mibvalue = tmpvalue;

	return 0;
}

static void rtl8365mb_get_ethtool_stats(struct dsa_switch *ds, int port, u64 *data)
{
	struct realtek_priv *priv = ds->priv;
	struct rtl8365mb *mb;
	int ret;
	int i;

	mb = priv->chip_data;

	mutex_lock(&mb->mib_lock);
	for (i = 0; i < RTL8365MB_MIB_END; i++) {
		struct rtl8365mb_mib_counter *mib = &rtl8365mb_mib_counters[i];

		ret = rtl8365mb_mib_counter_read(priv, port, mib->offset,
						 mib->length, &data[i]);
		if (ret) {
			dev_err(priv->dev,
				"failed to read port %d counters: %d\n", port,
				ret);
			break;
		}
	}
	mutex_unlock(&mb->mib_lock);
}

static void rtl8365mb_get_strings(struct dsa_switch *ds, int port, u32 stringset, u8 *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < RTL8365MB_MIB_END; i++) {
		struct rtl8365mb_mib_counter *mib = &rtl8365mb_mib_counters[i];
		ethtool_puts(&data, mib->name);
	}
}

static int rtl8365mb_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	return RTL8365MB_MIB_END;
}

static void rtl8365mb_get_phy_stats(struct dsa_switch *ds, int port,
				    struct ethtool_eth_phy_stats *phy_stats)
{
	struct realtek_priv *priv = ds->priv;
	struct rtl8365mb_mib_counter *mib;
	struct rtl8365mb *mb;

	mb = priv->chip_data;
	mib = &rtl8365mb_mib_counters[RTL8365MB_MIB_dot3StatsSymbolErrors];

	mutex_lock(&mb->mib_lock);
	rtl8365mb_mib_counter_read(priv, port, mib->offset, mib->length,
				   &phy_stats->SymbolErrorDuringCarrier);
	mutex_unlock(&mb->mib_lock);
}

static void rtl8365mb_get_mac_stats(struct dsa_switch *ds, int port,
				    struct ethtool_eth_mac_stats *mac_stats)
{
	u64 cnt[RTL8365MB_MIB_END] = {
		[RTL8365MB_MIB_ifOutOctets] = 1,
		[RTL8365MB_MIB_ifOutUcastPkts] = 1,
		[RTL8365MB_MIB_ifOutMulticastPkts] = 1,
		[RTL8365MB_MIB_ifOutBroadcastPkts] = 1,
		[RTL8365MB_MIB_dot3OutPauseFrames] = 1,
		[RTL8365MB_MIB_ifOutDiscards] = 1,
		[RTL8365MB_MIB_ifInOctets] = 1,
		[RTL8365MB_MIB_ifInUcastPkts] = 1,
		[RTL8365MB_MIB_ifInMulticastPkts] = 1,
		[RTL8365MB_MIB_ifInBroadcastPkts] = 1,
		[RTL8365MB_MIB_dot3InPauseFrames] = 1,
		[RTL8365MB_MIB_dot3StatsSingleCollisionFrames] = 1,
		[RTL8365MB_MIB_dot3StatsMultipleCollisionFrames] = 1,
		[RTL8365MB_MIB_dot3StatsFCSErrors] = 1,
		[RTL8365MB_MIB_dot3StatsDeferredTransmissions] = 1,
		[RTL8365MB_MIB_dot3StatsLateCollisions] = 1,
		[RTL8365MB_MIB_dot3StatsExcessiveCollisions] = 1,

	};
	struct realtek_priv *priv = ds->priv;
	struct rtl8365mb *mb;
	int ret;
	int i;

	mb = priv->chip_data;

	mutex_lock(&mb->mib_lock);
	for (i = 0; i < RTL8365MB_MIB_END; i++) {
		struct rtl8365mb_mib_counter *mib = &rtl8365mb_mib_counters[i];

		/* Only fetch required MIB counters (marked = 1 above) */
		if (!cnt[i])
			continue;

		ret = rtl8365mb_mib_counter_read(priv, port, mib->offset,
						 mib->length, &cnt[i]);
		if (ret)
			break;
	}
	mutex_unlock(&mb->mib_lock);

	/* The RTL8365MB-VC exposes MIB objects, which we have to translate into
	 * IEEE 802.3 Managed Objects. This is not always completely faithful,
	 * but we try out best. See RFC 3635 for a detailed treatment of the
	 * subject.
	 */

	mac_stats->FramesTransmittedOK = cnt[RTL8365MB_MIB_ifOutUcastPkts] +
					 cnt[RTL8365MB_MIB_ifOutMulticastPkts] +
					 cnt[RTL8365MB_MIB_ifOutBroadcastPkts] +
					 cnt[RTL8365MB_MIB_dot3OutPauseFrames] -
					 cnt[RTL8365MB_MIB_ifOutDiscards];
	mac_stats->SingleCollisionFrames =
		cnt[RTL8365MB_MIB_dot3StatsSingleCollisionFrames];
	mac_stats->MultipleCollisionFrames =
		cnt[RTL8365MB_MIB_dot3StatsMultipleCollisionFrames];
	mac_stats->FramesReceivedOK = cnt[RTL8365MB_MIB_ifInUcastPkts] +
				      cnt[RTL8365MB_MIB_ifInMulticastPkts] +
				      cnt[RTL8365MB_MIB_ifInBroadcastPkts] +
				      cnt[RTL8365MB_MIB_dot3InPauseFrames];
	mac_stats->FrameCheckSequenceErrors =
		cnt[RTL8365MB_MIB_dot3StatsFCSErrors];
	mac_stats->OctetsTransmittedOK = cnt[RTL8365MB_MIB_ifOutOctets] -
					 18 * mac_stats->FramesTransmittedOK;
	mac_stats->FramesWithDeferredXmissions =
		cnt[RTL8365MB_MIB_dot3StatsDeferredTransmissions];
	mac_stats->LateCollisions = cnt[RTL8365MB_MIB_dot3StatsLateCollisions];
	mac_stats->FramesAbortedDueToXSColls =
		cnt[RTL8365MB_MIB_dot3StatsExcessiveCollisions];
	mac_stats->OctetsReceivedOK = cnt[RTL8365MB_MIB_ifInOctets] -
				      18 * mac_stats->FramesReceivedOK;
	mac_stats->MulticastFramesXmittedOK =
		cnt[RTL8365MB_MIB_ifOutMulticastPkts];
	mac_stats->BroadcastFramesXmittedOK =
		cnt[RTL8365MB_MIB_ifOutBroadcastPkts];
	mac_stats->MulticastFramesReceivedOK =
		cnt[RTL8365MB_MIB_ifInMulticastPkts];
	mac_stats->BroadcastFramesReceivedOK =
		cnt[RTL8365MB_MIB_ifInBroadcastPkts];
}

static void rtl8365mb_get_ctrl_stats(struct dsa_switch *ds, int port,
				     struct ethtool_eth_ctrl_stats *ctrl_stats)
{
	struct realtek_priv *priv = ds->priv;
	struct rtl8365mb_mib_counter *mib;
	struct rtl8365mb *mb;

	mb = priv->chip_data;
	mib = &rtl8365mb_mib_counters[RTL8365MB_MIB_dot3ControlInUnknownOpcodes];

	mutex_lock(&mb->mib_lock);
	rtl8365mb_mib_counter_read(priv, port, mib->offset, mib->length,
				   &ctrl_stats->UnsupportedOpcodesReceived);
	mutex_unlock(&mb->mib_lock);
}

static void rtl8365mb_stats_update(struct realtek_priv *priv, int port)
{
	u64 cnt[RTL8365MB_MIB_END] = {
		[RTL8365MB_MIB_ifOutOctets] = 1,
		[RTL8365MB_MIB_ifOutUcastPkts] = 1,
		[RTL8365MB_MIB_ifOutMulticastPkts] = 1,
		[RTL8365MB_MIB_ifOutBroadcastPkts] = 1,
		[RTL8365MB_MIB_ifOutDiscards] = 1,
		[RTL8365MB_MIB_ifInOctets] = 1,
		[RTL8365MB_MIB_ifInUcastPkts] = 1,
		[RTL8365MB_MIB_ifInMulticastPkts] = 1,
		[RTL8365MB_MIB_ifInBroadcastPkts] = 1,
		[RTL8365MB_MIB_etherStatsDropEvents] = 1,
		[RTL8365MB_MIB_etherStatsCollisions] = 1,
		[RTL8365MB_MIB_etherStatsFragments] = 1,
		[RTL8365MB_MIB_etherStatsJabbers] = 1,
		[RTL8365MB_MIB_dot3StatsFCSErrors] = 1,
		[RTL8365MB_MIB_dot3StatsLateCollisions] = 1,
	};
	struct rtl8365mb *mb = priv->chip_data;
	struct rtnl_link_stats64 *stats;
	int ret;
	int i;

	stats = &mb->ports[port].stats;

	mutex_lock(&mb->mib_lock);
	for (i = 0; i < RTL8365MB_MIB_END; i++) {
		struct rtl8365mb_mib_counter *c = &rtl8365mb_mib_counters[i];

		/* Only fetch required MIB counters (marked = 1 above) */
		if (!cnt[i])
			continue;

		ret = rtl8365mb_mib_counter_read(priv, port, c->offset,
						 c->length, &cnt[i]);
		if (ret)
			break;
	}
	mutex_unlock(&mb->mib_lock);

	/* Don't update statistics if there was an error reading the counters */
	if (ret)
		return;

	spin_lock(&mb->ports[port].stats_lock);

	stats->rx_packets = cnt[RTL8365MB_MIB_ifInUcastPkts] +
			    cnt[RTL8365MB_MIB_ifInMulticastPkts] +
			    cnt[RTL8365MB_MIB_ifInBroadcastPkts] -
			    cnt[RTL8365MB_MIB_ifOutDiscards];

	stats->tx_packets = cnt[RTL8365MB_MIB_ifOutUcastPkts] +
			    cnt[RTL8365MB_MIB_ifOutMulticastPkts] +
			    cnt[RTL8365MB_MIB_ifOutBroadcastPkts];

	/* if{In,Out}Octets includes FCS - remove it */
	stats->rx_bytes = cnt[RTL8365MB_MIB_ifInOctets] - 4 * stats->rx_packets;
	stats->tx_bytes =
		cnt[RTL8365MB_MIB_ifOutOctets] - 4 * stats->tx_packets;

	stats->rx_dropped = cnt[RTL8365MB_MIB_etherStatsDropEvents];
	stats->tx_dropped = cnt[RTL8365MB_MIB_ifOutDiscards];

	stats->multicast = cnt[RTL8365MB_MIB_ifInMulticastPkts];
	stats->collisions = cnt[RTL8365MB_MIB_etherStatsCollisions];

	stats->rx_length_errors = cnt[RTL8365MB_MIB_etherStatsFragments] +
				  cnt[RTL8365MB_MIB_etherStatsJabbers];
	stats->rx_crc_errors = cnt[RTL8365MB_MIB_dot3StatsFCSErrors];
	stats->rx_errors = stats->rx_length_errors + stats->rx_crc_errors;

	stats->tx_aborted_errors = cnt[RTL8365MB_MIB_ifOutDiscards];
	stats->tx_window_errors = cnt[RTL8365MB_MIB_dot3StatsLateCollisions];
	stats->tx_errors = stats->tx_aborted_errors + stats->tx_window_errors;

	spin_unlock(&mb->ports[port].stats_lock);
}

static void rtl8365mb_stats_poll(struct work_struct *work)
{
	struct rtl8365mb_port *p = container_of(to_delayed_work(work),
						struct rtl8365mb_port,
						mib_work);
	struct realtek_priv *priv = p->priv;

	rtl8365mb_stats_update(priv, p->index);

	schedule_delayed_work(&p->mib_work, RTL8365MB_STATS_INTERVAL_JIFFIES);
}

static void rtl8365mb_get_stats64(struct dsa_switch *ds, int port,
				  struct rtnl_link_stats64 *s)
{
	struct realtek_priv *priv = ds->priv;
	struct rtl8365mb_port *p;
	struct rtl8365mb *mb;

	mb = priv->chip_data;
	p = &mb->ports[port];

	spin_lock(&p->stats_lock);
	memcpy(s, &p->stats, sizeof(*s));
	spin_unlock(&p->stats_lock);
}

static void rtl8365mb_stats_setup(struct realtek_priv *priv)
{
	struct rtl8365mb *mb = priv->chip_data;
	struct dsa_switch *ds = &priv->ds;
	int i;

	/* Per-chip global mutex to protect MIB counter access, since doing
	 * so requires accessing a series of registers in a particular order.
	 */
	mutex_init(&mb->mib_lock);

	for (i = 0; i < priv->num_ports; i++) {
		struct rtl8365mb_port *p = &mb->ports[i];

		if (dsa_is_unused_port(ds, i))
			continue;

		/* Per-port spinlock to protect the stats64 data */
		spin_lock_init(&p->stats_lock);

		/* This work polls the MIB counters and keeps the stats64 data
		 * up-to-date.
		 */
		INIT_DELAYED_WORK(&p->mib_work, rtl8365mb_stats_poll);
	}
}

static void rtl8365mb_stats_teardown(struct realtek_priv *priv)
{
	struct rtl8365mb *mb = priv->chip_data;
	struct dsa_switch *ds = &priv->ds;
	int i;

	for (i = 0; i < priv->num_ports; i++) {
		struct rtl8365mb_port *p = &mb->ports[i];

		if (dsa_is_unused_port(ds, i))
			continue;

		cancel_delayed_work_sync(&p->mib_work);
	}
}

static int rtl8365mb_get_and_clear_status_reg(struct realtek_priv *priv, u32 reg,
					      u32 *val)
{
	int ret;

	ret = regmap_read(priv->map, reg, val);
	if (ret)
		return ret;

	return regmap_write(priv->map, reg, *val);
}

static irqreturn_t rtl8365mb_irq(int irq, void *data)
{
	struct realtek_priv *priv = data;
	unsigned long line_changes = 0;
	u32 stat;
	int line;
	int ret;

	ret = rtl8365mb_get_and_clear_status_reg(priv, RTL8365MB_INTR_STATUS_REG,
						 &stat);
	if (ret)
		goto out_error;

	if (stat & RTL8365MB_INTR_LINK_CHANGE_MASK) {
		u32 linkdown_ind;
		u32 linkup_ind;
		u32 val;

		ret = rtl8365mb_get_and_clear_status_reg(
			priv, RTL8365MB_PORT_LINKUP_IND_REG, &val);
		if (ret)
			goto out_error;

		linkup_ind = FIELD_GET(RTL8365MB_PORT_LINKUP_IND_MASK, val);

		ret = rtl8365mb_get_and_clear_status_reg(
			priv, RTL8365MB_PORT_LINKDOWN_IND_REG, &val);
		if (ret)
			goto out_error;

		linkdown_ind = FIELD_GET(RTL8365MB_PORT_LINKDOWN_IND_MASK, val);

		line_changes = linkup_ind | linkdown_ind;
	}

	if (!line_changes)
		goto out_none;

	for_each_set_bit(line, &line_changes, priv->num_ports) {
		int child_irq = irq_find_mapping(priv->irqdomain, line);

		handle_nested_irq(child_irq);
	}

	return IRQ_HANDLED;

out_error:
	dev_err(priv->dev, "failed to read interrupt status: %d\n", ret);

out_none:
	return IRQ_NONE;
}

static struct irq_chip rtl8365mb_irq_chip = {
	.name = "rtl8365mb",
	/* The hardware doesn't support masking IRQs on a per-port basis */
};

static int rtl8365mb_irq_map(struct irq_domain *domain, unsigned int irq,
			     irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, domain->host_data);
	irq_set_chip_and_handler(irq, &rtl8365mb_irq_chip, handle_simple_irq);
	irq_set_nested_thread(irq, 1);
	irq_set_noprobe(irq);

	return 0;
}

static void rtl8365mb_irq_unmap(struct irq_domain *d, unsigned int irq)
{
	irq_set_nested_thread(irq, 0);
	irq_set_chip_and_handler(irq, NULL, NULL);
	irq_set_chip_data(irq, NULL);
}

static const struct irq_domain_ops rtl8365mb_irqdomain_ops = {
	.map = rtl8365mb_irq_map,
	.unmap = rtl8365mb_irq_unmap,
	.xlate = irq_domain_xlate_onecell,
};

static int rtl8365mb_set_irq_enable(struct realtek_priv *priv, bool enable)
{
	return regmap_update_bits(priv->map, RTL8365MB_INTR_CTRL_REG,
				  RTL8365MB_INTR_LINK_CHANGE_MASK,
				  FIELD_PREP(RTL8365MB_INTR_LINK_CHANGE_MASK,
					     enable ? 1 : 0));
}

static int rtl8365mb_irq_enable(struct realtek_priv *priv)
{
	return rtl8365mb_set_irq_enable(priv, true);
}

static int rtl8365mb_irq_disable(struct realtek_priv *priv)
{
	return rtl8365mb_set_irq_enable(priv, false);
}

static int rtl8365mb_irq_setup(struct realtek_priv *priv)
{
	struct rtl8365mb *mb = priv->chip_data;
	struct device_node *intc;
	u32 irq_trig;
	int virq;
	int irq;
	u32 val;
	int ret;
	int i;

	intc = of_get_child_by_name(priv->dev->of_node, "interrupt-controller");
	if (!intc) {
		dev_err(priv->dev, "missing child interrupt-controller node\n");
		return -EINVAL;
	}

	/* rtl8365mb IRQs cascade off this one */
	irq = of_irq_get(intc, 0);
	if (irq <= 0) {
		if (irq != -EPROBE_DEFER)
			dev_err(priv->dev, "failed to get parent irq: %d\n",
				irq);
		ret = irq ? irq : -EINVAL;
		goto out_put_node;
	}

	priv->irqdomain = irq_domain_add_linear(intc, priv->num_ports,
						&rtl8365mb_irqdomain_ops, priv);
	if (!priv->irqdomain) {
		dev_err(priv->dev, "failed to add irq domain\n");
		ret = -ENOMEM;
		goto out_put_node;
	}

	for (i = 0; i < priv->num_ports; i++) {
		virq = irq_create_mapping(priv->irqdomain, i);
		if (!virq) {
			dev_err(priv->dev,
				"failed to create irq domain mapping\n");
			ret = -EINVAL;
			goto out_remove_irqdomain;
		}

		irq_set_parent(virq, irq);
	}

	/* Configure chip interrupt signal polarity */
	irq_trig = irq_get_trigger_type(irq);
	switch (irq_trig) {
	case IRQF_TRIGGER_RISING:
	case IRQF_TRIGGER_HIGH:
		val = RTL8365MB_INTR_POLARITY_HIGH;
		break;
	case IRQF_TRIGGER_FALLING:
	case IRQF_TRIGGER_LOW:
		val = RTL8365MB_INTR_POLARITY_LOW;
		break;
	default:
		dev_err(priv->dev, "unsupported irq trigger type %u\n",
			irq_trig);
		ret = -EINVAL;
		goto out_remove_irqdomain;
	}

	ret = regmap_update_bits(priv->map, RTL8365MB_INTR_POLARITY_REG,
				 RTL8365MB_INTR_POLARITY_MASK,
				 FIELD_PREP(RTL8365MB_INTR_POLARITY_MASK, val));
	if (ret)
		goto out_remove_irqdomain;

	/* Disable the interrupt in case the chip has it enabled on reset */
	ret = rtl8365mb_irq_disable(priv);
	if (ret)
		goto out_remove_irqdomain;

	/* Clear the interrupt status register */
	ret = regmap_write(priv->map, RTL8365MB_INTR_STATUS_REG,
			   RTL8365MB_INTR_ALL_MASK);
	if (ret)
		goto out_remove_irqdomain;

	ret = request_threaded_irq(irq, NULL, rtl8365mb_irq, IRQF_ONESHOT,
				   "rtl8365mb", priv);
	if (ret) {
		dev_err(priv->dev, "failed to request irq: %d\n", ret);
		goto out_remove_irqdomain;
	}

	/* Store the irq so that we know to free it during teardown */
	mb->irq = irq;

	ret = rtl8365mb_irq_enable(priv);
	if (ret)
		goto out_free_irq;

	of_node_put(intc);

	return 0;

out_free_irq:
	free_irq(mb->irq, priv);
	mb->irq = 0;

out_remove_irqdomain:
	for (i = 0; i < priv->num_ports; i++) {
		virq = irq_find_mapping(priv->irqdomain, i);
		irq_dispose_mapping(virq);
	}

	irq_domain_remove(priv->irqdomain);
	priv->irqdomain = NULL;

out_put_node:
	of_node_put(intc);

	return ret;
}

static void rtl8365mb_irq_teardown(struct realtek_priv *priv)
{
	struct rtl8365mb *mb = priv->chip_data;
	int virq;
	int i;

	if (mb->irq) {
		free_irq(mb->irq, priv);
		mb->irq = 0;
	}

	if (priv->irqdomain) {
		for (i = 0; i < priv->num_ports; i++) {
			virq = irq_find_mapping(priv->irqdomain, i);
			irq_dispose_mapping(virq);
		}

		irq_domain_remove(priv->irqdomain);
		priv->irqdomain = NULL;
	}
}

static int rtl8365mb_cpu_config(struct realtek_priv *priv)
{
	struct rtl8365mb *mb = priv->chip_data;
	struct rtl8365mb_cpu *cpu = &mb->cpu;
	u32 val;
	int ret;

	ret = regmap_update_bits(priv->map, RTL8365MB_CPU_PORT_MASK_REG,
				 RTL8365MB_CPU_PORT_MASK_MASK,
				 FIELD_PREP(RTL8365MB_CPU_PORT_MASK_MASK,
					    cpu->mask));
	if (ret)
		return ret;

	val = FIELD_PREP(RTL8365MB_CPU_CTRL_EN_MASK, cpu->enable ? 1 : 0) |
	      FIELD_PREP(RTL8365MB_CPU_CTRL_INSERTMODE_MASK, cpu->insert) |
	      FIELD_PREP(RTL8365MB_CPU_CTRL_TAG_POSITION_MASK, cpu->position) |
	      FIELD_PREP(RTL8365MB_CPU_CTRL_RXBYTECOUNT_MASK, cpu->rx_length) |
	      FIELD_PREP(RTL8365MB_CPU_CTRL_TAG_FORMAT_MASK, cpu->format) |
	      FIELD_PREP(RTL8365MB_CPU_CTRL_TRAP_PORT_MASK, cpu->trap_port & 0x7) |
	      FIELD_PREP(RTL8365MB_CPU_CTRL_TRAP_PORT_EXT_MASK,
			 cpu->trap_port >> 3 & 0x1);
	ret = regmap_write(priv->map, RTL8365MB_CPU_CTRL_REG, val);
	if (ret)
		return ret;

	return 0;
}

static int rtl8365mb_change_tag_protocol(struct dsa_switch *ds,
					 enum dsa_tag_protocol proto)
{
	struct realtek_priv *priv = ds->priv;
	struct rtl8365mb_cpu *cpu;
	struct rtl8365mb *mb;

	mb = priv->chip_data;
	cpu = &mb->cpu;

	switch (proto) {
	case DSA_TAG_PROTO_RTL8_4:
		cpu->format = RTL8365MB_CPU_FORMAT_8BYTES;
		cpu->position = RTL8365MB_CPU_POS_AFTER_SA;
		break;
	case DSA_TAG_PROTO_RTL8_4T:
		cpu->format = RTL8365MB_CPU_FORMAT_8BYTES;
		cpu->position = RTL8365MB_CPU_POS_BEFORE_CRC;
		break;
	/* The switch also supports a 4-byte format, similar to rtl4a but with
	 * the same 0x04 8-bit version and probably 8-bit port source/dest.
	 * There is no public doc about it. Not supported yet and it will probably
	 * never be.
	 */
	default:
		return -EPROTONOSUPPORT;
	}

	return rtl8365mb_cpu_config(priv);
}

static int rtl8365mb_switch_init(struct realtek_priv *priv)
{
	struct rtl8365mb *mb = priv->chip_data;
	const struct rtl8365mb_chip_info *ci;
	int ret;
	int i;

	ci = mb->chip_info;

	/* Do any chip-specific init jam before getting to the common stuff */
	if (ci->jam_table) {
		for (i = 0; i < ci->jam_size; i++) {
			ret = regmap_write(priv->map, ci->jam_table[i].reg,
					   ci->jam_table[i].val);
			if (ret)
				return ret;
		}
	}

	/* Common init jam */
	for (i = 0; i < ARRAY_SIZE(rtl8365mb_init_jam_common); i++) {
		ret = regmap_write(priv->map, rtl8365mb_init_jam_common[i].reg,
				   rtl8365mb_init_jam_common[i].val);
		if (ret)
			return ret;
	}

	return 0;
}

static int rtl8365mb_reset_chip(struct realtek_priv *priv)
{
	u32 val;

	priv->write_reg_noack(priv, RTL8365MB_CHIP_RESET_REG,
			      FIELD_PREP(RTL8365MB_CHIP_RESET_HW_MASK, 1));

	/* Realtek documentation says the chip needs 1 second to reset. Sleep
	 * for 100 ms before accessing any registers to prevent ACK timeouts.
	 */
	msleep(100);
	return regmap_read_poll_timeout(priv->map, RTL8365MB_CHIP_RESET_REG, val,
					!(val & RTL8365MB_CHIP_RESET_HW_MASK),
					20000, 1e6);
}

/* VLAN support is always enabled in the switch.
 *
 * When a port is not a member of any VLANs (i.e. using a user port directly
 * and not in a bridge), the PVID property still matters.  With the default
 * PVID value of 0 (it is VlanMC index), forwarding to CPU will only work if
 * the VLAN mentioned in the VID at VlanMC index 0 includes the CPU port as an
 * untagged member. And the membership in the VlanMC does not matter as the
 * switch only considers the Vlan4k membership.
 *
 * Vlan4k starts at index 0, which is equivalent to VID 0. Let's include the
 * CPU port to that entry and create a static VlanMC entry at 0.
 */
static int rtl8365mb_vlan_init(struct dsa_switch *ds)
{
	u16 vlan_entry[RTL8365MB_VLAN_MC_CONF_ENTRY_SIZE] = {0};
	struct realtek_priv *priv = ds->priv;
	struct switchdev_obj_port_vlan vlan;
	struct rtl8366_vlan_mc vlanmc = {0};
	struct dsa_port *cpu_dp;
	int vlanmc_idx;
	int ret;

	/* fake VID 0 for user ports that are not member of any VLAN */
	/* vlanMC at idx 0 will be reserved for that */
	vlan.vid = 0;
	vlan.flags = BRIDGE_VLAN_INFO_UNTAGGED;

	/* just to be clear we do want it to be zero */
	vlanmc.vid = 0;
	/* there is not need to set the vlanmc.member a vlan4k is enough */

	/* and we do need it to be VlanMC at index 0 */
	vlanmc_idx = 0;

	dsa_switch_for_each_cpu_port(cpu_dp, ds) {
		ret = rtl8365mb_vlan4k_set(ds, cpu_dp->index, &vlan, NULL,
					   true);
		if (ret) {
			dev_err(priv->dev,
				"Failed to init VLAN 0 (for non members)\n");
			return ret;
		}
	}

	rtl8365mb_vlanmc_buf(&vlanmc, vlan_entry);
	ret = regmap_bulk_write(priv->map,
		       RTL8365MB_VLAN_MC_CONF_REG(vlanmc_idx),
		       vlan_entry,
		       RTL8365MB_VLAN_MC_CONF_ENTRY_SIZE);
	if (ret) {
		dev_err(priv->dev, "Failed to write vlan MC entry (vlan 0)\n");
		return ret;
	}

	/* VLAN is always enabled. */
	ret = regmap_update_bits(priv->map,
			 RTL8365MB_VLAN_CTRL_REG,
			 RTL8365MB_VLAN_CTRL_EN_VLAN_MASK,
			 FIELD_PREP(RTL8365MB_VLAN_CTRL_EN_VLAN_MASK, 1));
	return ret;
}

static int rtl8365mb_setup(struct dsa_switch *ds)
{
	struct realtek_priv *priv = ds->priv;
	struct rtl8365mb_cpu *cpu;
	struct dsa_port *cpu_dp;
	struct rtl8365mb *mb;
	u32 user_ports;
	int ret;
	int i;

	mb = priv->chip_data;
	cpu = &mb->cpu;

	ret = rtl8365mb_reset_chip(priv);
	if (ret) {
		dev_err(priv->dev, "failed to reset chip: %d\n", ret);
		goto out_error;
	}

	/* Configure switch to vendor-defined initial state */
	ret = rtl8365mb_switch_init(priv);
	if (ret) {
		dev_err(priv->dev, "failed to initialize switch: %d\n", ret);
		goto out_error;
	}

	/* Set up cascading IRQs */
	ret = rtl8365mb_irq_setup(priv);
	if (ret == -EPROBE_DEFER)
		return ret;
	else if (ret)
		dev_info(priv->dev, "no interrupt support\n");

	user_ports = dsa_user_ports(ds);

	/* Configure CPU tagging */
	dsa_switch_for_each_cpu_port(cpu_dp, ds) {
		cpu->mask |= BIT(cpu_dp->index);

		/* Forward to all user ports */
		ret = rtl8365mb_port_set_isolation(priv, cpu_dp->index,
						   user_ports);

		if (cpu->trap_port == RTL8365MB_MAX_NUM_PORTS)
			cpu->trap_port = cpu_dp->index;
	}
	cpu->enable = cpu->mask > 0;
	ret = rtl8365mb_cpu_config(priv);
	if (ret)
		goto out_teardown_irq;

	/* Configure ports */
	for (i = 0; i < priv->num_ports; i++) {
		struct rtl8365mb_port *p = &mb->ports[i];

		if (dsa_is_unused_port(ds, i))
			continue;

		if ((cpu->mask & (1 << i)) == 0 ) {
			/* Forward only to the CPU */
			ret = rtl8365mb_port_set_isolation(priv, i, cpu->mask);
			if (ret)
				goto out_teardown_irq;
		}

		/* Disable learning */
		ret = rtl8365mb_port_set_learning(priv, i, false);
		if (ret)
			goto out_teardown_irq;

		/* Set the initial STP state of all ports to DISABLED, otherwise
		 * ports will still forward frames to the CPU despite being
		 * administratively down by default.
		 */
		rtl8365mb_port_stp_state_set(ds, i, BR_STATE_DISABLED);

		/* Set up per-port private data */
		p->priv = priv;
		p->index = i;
	}

	ret = rtl8365mb_port_change_mtu(ds, cpu->trap_port, ETH_DATA_LEN);
	if (ret)
		goto out_teardown_irq;

	ret = rtl8365mb_vlan_init(ds);
	if (ret)
		goto out_teardown_irq;

	/* vlan config will only be effective for ports with vlan filtering */
	ds->configure_vlan_while_not_filtering = 1;

	ret = rtl83xx_setup_user_mdio(ds);
	if (ret) {
		dev_err(priv->dev, "could not set up MDIO bus\n");
		goto out_teardown_irq;
	}

	/* Start statistics counter polling */
	rtl8365mb_stats_setup(priv);

	/* Table access mutex */
	mutex_init(&mb->table_lock);

	return 0;

out_teardown_irq:
	rtl8365mb_irq_teardown(priv);

out_error:
	return ret;
}

static void rtl8365mb_teardown(struct dsa_switch *ds)
{
	struct realtek_priv *priv = ds->priv;

	rtl8365mb_stats_teardown(priv);
	rtl8365mb_irq_teardown(priv);
}

static int rtl8365mb_get_chip_id_and_ver(struct regmap *map, u32 *id, u32 *ver)
{
	int ret;

	/* For some reason we have to write a magic value to an arbitrary
	 * register whenever accessing the chip ID/version registers.
	 */
	ret = regmap_write(map, RTL8365MB_MAGIC_REG, RTL8365MB_MAGIC_VALUE);
	if (ret)
		return ret;

	ret = regmap_read(map, RTL8365MB_CHIP_ID_REG, id);
	if (ret)
		return ret;

	ret = regmap_read(map, RTL8365MB_CHIP_VER_REG, ver);
	if (ret)
		return ret;

	/* Reset magic register */
	ret = regmap_write(map, RTL8365MB_MAGIC_REG, 0);
	if (ret)
		return ret;

	return 0;
}

static int rtl8365mb_detect(struct realtek_priv *priv)
{
	struct rtl8365mb *mb = priv->chip_data;
	u32 chip_id;
	u32 chip_ver;
	int ret;
	int i;

	ret = rtl8365mb_get_chip_id_and_ver(priv->map, &chip_id, &chip_ver);
	if (ret) {
		dev_err(priv->dev, "failed to read chip id and version: %d\n",
			ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(rtl8365mb_chip_infos); i++) {
		const struct rtl8365mb_chip_info *ci = &rtl8365mb_chip_infos[i];

		if (ci->chip_id == chip_id && ci->chip_ver == chip_ver) {
			mb->chip_info = ci;
			break;
		}
	}

	if (!mb->chip_info) {
		dev_err(priv->dev,
			"unrecognized switch (id=0x%04x, ver=0x%04x)", chip_id,
			chip_ver);
		return -ENODEV;
	}

	dev_info(priv->dev, "found an %s switch\n", mb->chip_info->name);

	priv->num_ports = RTL8365MB_MAX_NUM_PORTS;
	mb->priv = priv;
	mb->cpu.trap_port = RTL8365MB_MAX_NUM_PORTS;
	mb->cpu.insert = RTL8365MB_CPU_INSERT_TO_ALL;
	mb->cpu.position = RTL8365MB_CPU_POS_AFTER_SA;
	mb->cpu.rx_length = RTL8365MB_CPU_RXLEN_64BYTES;
	mb->cpu.format = RTL8365MB_CPU_FORMAT_8BYTES;

	return 0;
}

static const struct phylink_mac_ops rtl8365mb_phylink_mac_ops = {
	.mac_config = rtl8365mb_phylink_mac_config,
	.mac_link_down = rtl8365mb_phylink_mac_link_down,
	.mac_link_up = rtl8365mb_phylink_mac_link_up,
};

static const struct dsa_switch_ops rtl8365mb_switch_ops = {
	.get_tag_protocol = rtl8365mb_get_tag_protocol,
	.change_tag_protocol = rtl8365mb_change_tag_protocol,
	.setup = rtl8365mb_setup,
	.teardown = rtl8365mb_teardown,
	.phylink_get_caps = rtl8365mb_phylink_get_caps,
	.port_stp_state_set = rtl8365mb_port_stp_state_set,
	.get_strings = rtl8365mb_get_strings,
	.get_ethtool_stats = rtl8365mb_get_ethtool_stats,
	.get_sset_count = rtl8365mb_get_sset_count,
	.get_eth_phy_stats = rtl8365mb_get_phy_stats,
	.get_eth_mac_stats = rtl8365mb_get_mac_stats,
	.get_eth_ctrl_stats = rtl8365mb_get_ctrl_stats,
	.get_stats64 = rtl8365mb_get_stats64,
	.port_change_mtu = rtl8365mb_port_change_mtu,
	.port_max_mtu = rtl8365mb_port_max_mtu,
	.port_vlan_add = rtl8365mb_vlan_add,
	.port_vlan_del = rtl8365mb_vlan_del,
	.port_vlan_filtering = rtl8365mb_vlan_filtering,
	.port_bridge_join = rtl8365mb_port_bridge_join,
	.port_bridge_leave = rtl8365mb_port_bridge_leave,
	.port_bridge_flags = rtl8365mb_port_bridge_flags,
	.port_pre_bridge_flags = rtl8365mb_port_pre_bridge_flags,
};

static const struct realtek_ops rtl8365mb_ops = {
	.detect = rtl8365mb_detect,
	.phy_read = rtl8365mb_phy_read,
	.phy_write = rtl8365mb_phy_write,
};

const struct realtek_variant rtl8365mb_variant = {
	.ds_ops = &rtl8365mb_switch_ops,
	.ops = &rtl8365mb_ops,
	.phylink_mac_ops = &rtl8365mb_phylink_mac_ops,
	.clk_delay = 10,
	.cmd_read = 0xb9,
	.cmd_write = 0xb8,
	.chip_data_sz = sizeof(struct rtl8365mb),
};

static const struct of_device_id rtl8365mb_of_match[] = {
	{ .compatible = "realtek,rtl8365mb", .data = &rtl8365mb_variant, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, rtl8365mb_of_match);

static struct platform_driver rtl8365mb_smi_driver = {
	.driver = {
		.name = "rtl8365mb-smi",
		.of_match_table = rtl8365mb_of_match,
	},
	.probe  = realtek_smi_probe,
	.remove_new = realtek_smi_remove,
	.shutdown = realtek_smi_shutdown,
};

static struct mdio_driver rtl8365mb_mdio_driver = {
	.mdiodrv.driver = {
		.name = "rtl8365mb-mdio",
		.of_match_table = rtl8365mb_of_match,
	},
	.probe  = realtek_mdio_probe,
	.remove = realtek_mdio_remove,
	.shutdown = realtek_mdio_shutdown,
};

static int rtl8365mb_init(void)
{
	int ret;

	ret = realtek_mdio_driver_register(&rtl8365mb_mdio_driver);
	if (ret)
		return ret;

	ret = realtek_smi_driver_register(&rtl8365mb_smi_driver);
	if (ret) {
		realtek_mdio_driver_unregister(&rtl8365mb_mdio_driver);
		return ret;
	}

	return 0;
}
module_init(rtl8365mb_init);

static void __exit rtl8365mb_exit(void)
{
	realtek_smi_driver_unregister(&rtl8365mb_smi_driver);
	realtek_mdio_driver_unregister(&rtl8365mb_mdio_driver);
}
module_exit(rtl8365mb_exit);

MODULE_AUTHOR("Alvin Šipraga <alsi@bang-olufsen.dk>");
MODULE_DESCRIPTION("Driver for RTL8365MB-VC ethernet switch");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(REALTEK_DSA);
