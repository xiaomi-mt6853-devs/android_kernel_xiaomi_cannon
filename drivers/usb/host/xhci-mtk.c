/*
 * MediaTek xHCI Host Controller Driver
 *
 * Copyright (c) 2015 MediaTek Inc.
 * Author:
 *  Chunfeng Yun <chunfeng.yun@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include "xhci.h"
#include "xhci-mtk.h"

#if IS_ENABLED(CONFIG_MACH_MT6853)
#include "mtu3_hal.h"
#endif

/* ip_pw_ctrl0 register */
#define CTRL0_IP_SW_RST	BIT(0)

/* ip_pw_ctrl1 register */
#define CTRL1_IP_HOST_PDN	BIT(0)

/* ip_pw_ctrl2 register */
#define CTRL2_IP_DEV_PDN	BIT(0)

/* ip_pw_sts1 register */
#define STS1_IP_SLEEP_STS	BIT(30)
#define STS1_XHCI_RST		BIT(11)
#define STS1_SYS125_RST	BIT(10)
#define STS1_REF_RST		BIT(8)
#define STS1_SYSPLL_STABLE	BIT(0)

/* ip_xhci_cap register */
#define CAP_U3_PORT_NUM(p)	((p) & 0xff)
#define CAP_U2_PORT_NUM(p)	(((p) >> 8) & 0xff)

/* u3_ctrl_p register */
#define CTRL_U3_PORT_HOST_SEL	BIT(2)
#define CTRL_U3_PORT_PDN	BIT(1)
#define CTRL_U3_PORT_DIS	BIT(0)

/* u2_ctrl_p register */
#define CTRL_U2_PORT_HOST_SEL	BIT(2)
#define CTRL_U2_PORT_PDN	BIT(1)
#define CTRL_U2_PORT_DIS	BIT(0)

/* u2_phy_pll register */
#define CTRL_U2_FORCE_PLL_STB	BIT(28)

#define PERI_WK_CTRL0		0x400
#define UWK_CTR0_0P_LS_PE	BIT(8)  /* posedge */
#define UWK_CTR0_0P_LS_NE	BIT(7)  /* negedge for 0p linestate*/
#define UWK_CTL1_1P_LS_C(x)	(((x) & 0xf) << 1)
#define UWK_CTL1_1P_LS_E	BIT(0)

#define PERI_WK_CTRL1		0x404
#define UWK_CTL1_IS_C(x)	(((x) & 0xf) << 26)
#define UWK_CTL1_IS_E		BIT(25)
#define UWK_CTL1_0P_LS_C(x)	(((x) & 0xf) << 21)
#define UWK_CTL1_0P_LS_E	BIT(20)
#define UWK_CTL1_IDDIG_C(x)	(((x) & 0xf) << 11)  /* cycle debounce */
#define UWK_CTL1_IDDIG_E	BIT(10) /* enable debounce */
#define UWK_CTL1_IDDIG_P	BIT(9)  /* polarity */
#define UWK_CTL1_0P_LS_P	BIT(7)
#define UWK_CTL1_IS_P		BIT(6)  /* polarity for ip sleep */

/* test mode */
#define PROC_DIR_MTK_USB "mtk_usb"
#define PROC_FILE_TESTMODE "mtk_usb/testmode"
#define HOST_CMD_TEST_J             0x1
#define HOST_CMD_TEST_K             0x2
#define HOST_CMD_TEST_SE0_NAK       0x3
#define HOST_CMD_TEST_PACKET        0x4
#define PMSC_PORT_TEST_CTRL_OFFSET  28

static struct proc_dir_entry *file_testmode;

/* frmcnt */
#define INIT_FRMCNT_LEV1_FULL_RANGE 0x944
#define INIT_FRMCNT_FRMCNT_C(x)	(((x) & 0xfff) << 8)

#if IS_ENABLED(CONFIG_MACH_MT6853)
static void xhci_mtk_frmcnt_range(struct usb_hcd *hcd, int speed)
{
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	u32 value;

	value = readl(hcd->regs + INIT_FRMCNT_LEV1_FULL_RANGE);
	value &= ~(INIT_FRMCNT_FRMCNT_C(0xfff));

	if (speed == DEV_SPEED_FULL)
		value |= INIT_FRMCNT_FRMCNT_C(0x144);
	else
		value |= INIT_FRMCNT_FRMCNT_C(0x12b);

	writel(value, hcd->regs + INIT_FRMCNT_LEV1_FULL_RANGE);
}
#endif

void xhci_mtk_set_port_mode(struct usb_hcd *hcd,
		__le32 __iomem **port_array, int port_id)
{
#if IS_ENABLED(CONFIG_MACH_MT6853)
	struct xhci_hcd_mtk *mtk = hcd_to_mtk(hcd);
	struct xhci_hcd *xhci = hcd_to_xhci(mtk->hcd);
	unsigned long flags;
	u32 temp;
	int old_speed;
	int new_speed;

	temp = readl(port_array[port_id]);
	old_speed = mtk->last_speed;
	new_speed = DEV_PORT_SPEED(temp);
	mtk->last_speed = new_speed;
	xhci_info(xhci, "old_speed=%d, new_speed=%d\n",
			  old_speed, new_speed);

	if (old_speed == new_speed || (old_speed != DEV_SPEED_FULL &&
			new_speed != DEV_SPEED_FULL))
		return;

	xhci_set_link_state(xhci, port_array, port_id, XDEV_U3);

	spin_unlock_irqrestore(&xhci->lock, flags);
	if (readl_poll_timeout_atomic(port_array[port_id], temp,
			(temp & PORT_PLS_MASK) == XDEV_U3, 100, 100000))
		xhci_warn(xhci, "port suspend timeout\n");
	spin_lock_irqsave(&xhci->lock, flags);

	spin_unlock_irqrestore(&xhci->lock, flags);
	ssusb_set_phy_mode(new_speed);
	spin_lock_irqsave(&xhci->lock, flags);

	xhci_mtk_frmcnt_range(hcd, new_speed);

	xhci_set_link_state(xhci, port_array, port_id, XDEV_RESUME);
	spin_unlock_irqrestore(&xhci->lock, flags);
	mdelay(USB_RESUME_TIMEOUT);
	spin_lock_irqsave(&xhci->lock, flags);

	xhci_set_link_state(xhci, port_array, port_id, XDEV_U0);
	spin_unlock_irqrestore(&xhci->lock, flags);
	mdelay(50);
	spin_lock_irqsave(&xhci->lock, flags);

	temp = readl(port_array[port_id]);
	if ((temp & PORT_PLS_MASK) != XDEV_U0)
		xhci_warn(xhci, "error resume 0x%08x!!!\n", temp);
#endif
}


static ssize_t xhci_mtk_test_mode_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)

{
	struct seq_file *s = file->private_data;
	struct xhci_hcd_mtk *mtk = s->private;
	struct xhci_hcd *xhci = hcd_to_xhci(mtk->hcd);
	int ports = HCS_MAX_PORTS(xhci->hcs_params1);
	char buf[20];
	u8 test = 0;
	u32 temp;
	u32 __iomem *addr;
	int i;

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(buf, ubuf,
			min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "test packet", 10))
		test = HOST_CMD_TEST_PACKET;
	else if (!strncmp(buf, "test K", 6))
		test = HOST_CMD_TEST_K;
	else if (!strncmp(buf, "test J", 6))
		test = HOST_CMD_TEST_J;
	else if (!strncmp(buf, "test SE0 NAK", 12))
		test = HOST_CMD_TEST_SE0_NAK;

	if (test) {
		xhci_info(xhci, "set test mode %d\n", test);

		/* set the Run/Stop in USBCMD to 0 */
		addr = &xhci->op_regs->command;
		temp = readl(addr);
		temp &= ~CMD_RUN;
		writel(temp, addr);

		/*  wait for HCHalted */
		xhci_halt(xhci);

		/* test mode */
		for (i = 0; i < ports; i++) {
			addr = &xhci->op_regs->port_power_base +
				NUM_PORT_REGS * (i & 0xff);
			temp = readl(addr);
			temp &= ~(0xf << PMSC_PORT_TEST_CTRL_OFFSET);
			temp |= (test << PMSC_PORT_TEST_CTRL_OFFSET);
			writel(temp, addr);
		}
	} else {
		xhci_info(xhci, "test mode command error\n");
	}

	return count;
}

static int xhci_mtk_test_mode_show(struct seq_file *s, void *unused)
{
	seq_puts(s, "xhci_mtk test mode\n");
	return 0;
}


static int xhci_mtk_test_mode_open(struct inode *inode,
					struct file *file)
{
	return single_open(file, xhci_mtk_test_mode_show,
					   PDE_DATA(inode));
}

static const struct file_operations xhci_mtk_test_mode_fops = {
	.open = xhci_mtk_test_mode_open,
	.write = xhci_mtk_test_mode_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int xhci_mtk_dbg_init(struct xhci_hcd_mtk *mtk)
{
	int ret = 0;

	file_testmode = NULL;
	proc_mkdir(PROC_DIR_MTK_USB, NULL);

	file_testmode = proc_create_data(PROC_FILE_TESTMODE, 0644, NULL,
						&xhci_mtk_test_mode_fops, mtk);
	if (file_testmode) {
		ret = -ENOMEM;
		goto err0;
	}

	return 0;
err0:
	return ret;
}

static int xhci_mtk_dbg_exit(struct xhci_hcd_mtk *mtk)
{
	if (file_testmode)
		proc_remove(file_testmode);
	return 0;
}

#if !IS_ENABLED(CONFIG_USB_XHCI_MTK_SUSPEND)
static int mtk_xhci_wakelock_lock(struct xhci_hcd_mtk *mtk)
{
	struct device_node *of_node = mtk->dev->of_node;
	struct xhci_hcd *xhci = hcd_to_xhci(mtk->hcd);

	if (of_device_is_compatible(of_node, "mediatek,mt67xx-xhci")) {
		pm_stay_awake(mtk->dev);
		xhci_info(xhci, "wakelock_lock\n");
	}
	return 0;
}

static int mtk_xhci_wakelock_unlock(struct xhci_hcd_mtk *mtk)
{
	struct device_node *of_node = mtk->dev->of_node;
	struct xhci_hcd *xhci = hcd_to_xhci(mtk->hcd);

	if (of_device_is_compatible(of_node, "mediatek,mt67xx-xhci")) {
		pm_relax(mtk->dev);
		xhci_info(xhci, "wakelock_unlock\n");
	}
	return 0;
}
#endif

enum ssusb_wakeup_src {
	SSUSB_WK_IP_SLEEP = 1,
	SSUSB_WK_LINE_STATE = 2,
};

static int xhci_mtk_host_enable(struct xhci_hcd_mtk *mtk)
{
	struct mu3c_ippc_regs __iomem *ippc = mtk->ippc_regs;
	u32 value, check_val;
	int ret;
	int i;

	if (!mtk->has_ippc)
		return 0;

	/* power on host ip */
	value = readl(&ippc->ip_pw_ctr1);
	value &= ~CTRL1_IP_HOST_PDN;
	writel(value, &ippc->ip_pw_ctr1);

	/* power on and enable all u3 ports */
	for (i = 0; i < mtk->num_u3_ports; i++) {
		value = readl(&ippc->u3_ctrl_p[i]);
		value &= ~(CTRL_U3_PORT_PDN | CTRL_U3_PORT_DIS);
		value |= CTRL_U3_PORT_HOST_SEL;
		writel(value, &ippc->u3_ctrl_p[i]);
	}

	/* power on and enable all u2 ports */
	for (i = 0; i < mtk->num_u2_ports; i++) {
		value = readl(&ippc->u2_ctrl_p[i]);
		value &= ~(CTRL_U2_PORT_PDN | CTRL_U2_PORT_DIS);
		value |= CTRL_U2_PORT_HOST_SEL;
		writel(value, &ippc->u2_ctrl_p[i]);
	}

	/*
	 * wait for clocks to be stable, and clock domains reset to
	 * be inactive after power on and enable ports
	 */
	check_val = STS1_SYSPLL_STABLE | STS1_REF_RST |
			STS1_SYS125_RST | STS1_XHCI_RST;

	ret = readl_poll_timeout(&ippc->ip_pw_sts1, value,
			  (check_val == (value & check_val)), 100, 20000);
	if (ret) {
		dev_err(mtk->dev, "clocks are not stable (0x%x)\n", value);
		return ret;
	}

	return 0;
}

static int xhci_mtk_host_disable(struct xhci_hcd_mtk *mtk)
{
	struct mu3c_ippc_regs __iomem *ippc = mtk->ippc_regs;
	u32 value;
	int ret;
	int i;

	if (!mtk->has_ippc)
		return 0;

	/* power down all u3 ports */
	for (i = 0; i < mtk->num_u3_ports; i++) {
		value = readl(&ippc->u3_ctrl_p[i]);
		value |= CTRL_U3_PORT_PDN;
		writel(value, &ippc->u3_ctrl_p[i]);
	}

	/* power down all u2 ports */
	for (i = 0; i < mtk->num_u2_ports; i++) {
		value = readl(&ippc->u2_ctrl_p[i]);
		value |= CTRL_U2_PORT_PDN;
		writel(value, &ippc->u2_ctrl_p[i]);
	}

	/* power down host ip */
	value = readl(&ippc->ip_pw_ctr1);
	value |= CTRL1_IP_HOST_PDN;
	writel(value, &ippc->ip_pw_ctr1);

	/* wait for host ip to sleep */
	ret = readl_poll_timeout(&ippc->ip_pw_sts1, value,
			  (value & STS1_IP_SLEEP_STS), 100, 100000);
	if (ret) {
		dev_err(mtk->dev, "ip sleep failed!!!\n");
		return ret;
	}
	return 0;
}

static int xhci_mtk_ssusb_config(struct xhci_hcd_mtk *mtk)
{
	struct mu3c_ippc_regs __iomem *ippc = mtk->ippc_regs;
	u32 value;

	if (!mtk->has_ippc)
		return 0;

	/* reset whole ip */
	value = readl(&ippc->ip_pw_ctr0);
	value |= CTRL0_IP_SW_RST;
	writel(value, &ippc->ip_pw_ctr0);
	udelay(1);
	value = readl(&ippc->ip_pw_ctr0);
	value &= ~CTRL0_IP_SW_RST;
	writel(value, &ippc->ip_pw_ctr0);

	/*
	 * device ip is default power-on in fact
	 * power down device ip, otherwise ip-sleep will fail
	 */
	value = readl(&ippc->ip_pw_ctr2);
	value |= CTRL2_IP_DEV_PDN;
	writel(value, &ippc->ip_pw_ctr2);

	value = readl(&ippc->ip_xhci_cap);
	mtk->num_u3_ports = CAP_U3_PORT_NUM(value);
	mtk->num_u2_ports = CAP_U2_PORT_NUM(value);
	dev_dbg(mtk->dev, "%s u2p:%d, u3p:%d\n", __func__,
			mtk->num_u2_ports, mtk->num_u3_ports);

	return xhci_mtk_host_enable(mtk);
}

static int xhci_mtk_clks_enable(struct xhci_hcd_mtk *mtk)
{
	int ret;

	ret = clk_prepare_enable(mtk->ref_clk);
	if (ret) {
		dev_err(mtk->dev, "failed to enable ref_clk\n");
		goto ref_clk_err;
	}

	ret = clk_prepare_enable(mtk->sys_clk);
	if (ret) {
		dev_err(mtk->dev, "failed to enable sys_clk\n");
		goto sys_clk_err;
	}

	if (mtk->wakeup_src) {
		ret = clk_prepare_enable(mtk->wk_deb_p0);
		if (ret) {
			dev_err(mtk->dev, "failed to enable wk_deb_p0\n");
			goto usb_p0_err;
		}

		ret = clk_prepare_enable(mtk->wk_deb_p1);
		if (ret) {
			dev_err(mtk->dev, "failed to enable wk_deb_p1\n");
			goto usb_p1_err;
		}
	}
	return 0;

usb_p1_err:
	clk_disable_unprepare(mtk->wk_deb_p0);
usb_p0_err:
	clk_disable_unprepare(mtk->sys_clk);
sys_clk_err:
	clk_disable_unprepare(mtk->ref_clk);
ref_clk_err:
	return -EINVAL;
}

static void xhci_mtk_clks_disable(struct xhci_hcd_mtk *mtk)
{
	if (mtk->wakeup_src) {
		clk_disable_unprepare(mtk->wk_deb_p1);
		clk_disable_unprepare(mtk->wk_deb_p0);
	}
	clk_disable_unprepare(mtk->sys_clk);
	clk_disable_unprepare(mtk->ref_clk);
}

/* only clocks can be turn off for ip-sleep wakeup mode */
static void usb_wakeup_ip_sleep_en(struct xhci_hcd_mtk *mtk)
{
	u32 tmp;
	struct regmap *pericfg = mtk->pericfg;

	regmap_read(pericfg, PERI_WK_CTRL1, &tmp);
	tmp &= ~UWK_CTL1_IS_P;
	tmp &= ~(UWK_CTL1_IS_C(0xf));
	tmp |= UWK_CTL1_IS_C(0x8);
	regmap_write(pericfg, PERI_WK_CTRL1, tmp);
	regmap_write(pericfg, PERI_WK_CTRL1, tmp | UWK_CTL1_IS_E);

	regmap_read(pericfg, PERI_WK_CTRL1, &tmp);
	dev_dbg(mtk->dev, "%s(): WK_CTRL1[P6,E25,C26:29]=%#x\n",
		__func__, tmp);
}

static void usb_wakeup_ip_sleep_dis(struct xhci_hcd_mtk *mtk)
{
	u32 tmp;

	regmap_read(mtk->pericfg, PERI_WK_CTRL1, &tmp);
	tmp &= ~UWK_CTL1_IS_E;
	regmap_write(mtk->pericfg, PERI_WK_CTRL1, tmp);
}

/*
* for line-state wakeup mode, phy's power should not power-down
* and only support cable plug in/out
*/
static void usb_wakeup_line_state_en(struct xhci_hcd_mtk *mtk)
{
	u32 tmp;
	struct regmap *pericfg = mtk->pericfg;

	/* line-state of u2-port0 */
	regmap_read(pericfg, PERI_WK_CTRL1, &tmp);
	tmp &= ~UWK_CTL1_0P_LS_P;
	tmp &= ~(UWK_CTL1_0P_LS_C(0xf));
	tmp |= UWK_CTL1_0P_LS_C(0x8);
	regmap_write(pericfg, PERI_WK_CTRL1, tmp);
	regmap_read(pericfg, PERI_WK_CTRL1, &tmp);
	regmap_write(pericfg, PERI_WK_CTRL1, tmp | UWK_CTL1_0P_LS_E);

	/* line-state of u2-port1 */
	regmap_read(pericfg, PERI_WK_CTRL0, &tmp);
	tmp &= ~(UWK_CTL1_1P_LS_C(0xf));
	tmp |= UWK_CTL1_1P_LS_C(0x8);
	regmap_write(pericfg, PERI_WK_CTRL0, tmp);
	regmap_write(pericfg, PERI_WK_CTRL0, tmp | UWK_CTL1_1P_LS_E);
}

static void usb_wakeup_line_state_dis(struct xhci_hcd_mtk *mtk)
{
	u32 tmp;
	struct regmap *pericfg = mtk->pericfg;

	/* line-state of u2-port0 */
	regmap_read(pericfg, PERI_WK_CTRL1, &tmp);
	tmp &= ~UWK_CTL1_0P_LS_E;
	regmap_write(pericfg, PERI_WK_CTRL1, tmp);

	/* line-state of u2-port1 */
	regmap_read(pericfg, PERI_WK_CTRL0, &tmp);
	tmp &= ~UWK_CTL1_1P_LS_E;
	regmap_write(pericfg, PERI_WK_CTRL0, tmp);
}

static void usb_wakeup_enable(struct xhci_hcd_mtk *mtk)
{
	if (mtk->wakeup_src == SSUSB_WK_IP_SLEEP)
		usb_wakeup_ip_sleep_en(mtk);
	else if (mtk->wakeup_src == SSUSB_WK_LINE_STATE)
		usb_wakeup_line_state_en(mtk);
}

static void usb_wakeup_disable(struct xhci_hcd_mtk *mtk)
{
	if (mtk->wakeup_src == SSUSB_WK_IP_SLEEP)
		usb_wakeup_ip_sleep_dis(mtk);
	else if (mtk->wakeup_src == SSUSB_WK_LINE_STATE)
		usb_wakeup_line_state_dis(mtk);
}

static int usb_wakeup_of_property_parse(struct xhci_hcd_mtk *mtk,
				struct device_node *dn)
{
	struct device *dev = mtk->dev;

	/*
	* wakeup function is optional, so it is not an error if this property
	* does not exist, and in such case, no need to get relative
	* properties anymore.
	*/
	of_property_read_u32(dn, "mediatek,wakeup-src", &mtk->wakeup_src);
	if (!mtk->wakeup_src)
		return 0;

	mtk->wk_deb_p0 = devm_clk_get(dev, "wakeup_deb_p0");
	if (IS_ERR(mtk->wk_deb_p0)) {
		dev_err(dev, "fail to get wakeup_deb_p0\n");
		return PTR_ERR(mtk->wk_deb_p0);
	}

	mtk->wk_deb_p1 = devm_clk_get(dev, "wakeup_deb_p1");
	if (IS_ERR(mtk->wk_deb_p1)) {
		dev_err(dev, "fail to get wakeup_deb_p1\n");
		return PTR_ERR(mtk->wk_deb_p1);
	}

	mtk->pericfg = syscon_regmap_lookup_by_phandle(dn,
						"mediatek,syscon-wakeup");
	if (IS_ERR(mtk->pericfg)) {
		dev_err(dev, "fail to get pericfg regs\n");
		return PTR_ERR(mtk->pericfg);
	}

	return 0;
}

static int xhci_mtk_setup(struct usb_hcd *hcd);
static const struct xhci_driver_overrides xhci_mtk_overrides __initconst = {
	.reset = xhci_mtk_setup,
};

static struct hc_driver __read_mostly xhci_mtk_hc_driver;

static int xhci_mtk_phy_init(struct xhci_hcd_mtk *mtk)
{
	int i;
	int ret;

	for (i = 0; i < mtk->num_phys; i++) {
		ret = phy_init(mtk->phys[i]);
		if (ret)
			goto exit_phy;
	}
	return 0;

exit_phy:
	for (; i > 0; i--)
		phy_exit(mtk->phys[i - 1]);

	return ret;
}

static int xhci_mtk_phy_exit(struct xhci_hcd_mtk *mtk)
{
	int i;

	for (i = 0; i < mtk->num_phys; i++)
		phy_exit(mtk->phys[i]);

	return 0;
}

static int xhci_mtk_phy_power_on(struct xhci_hcd_mtk *mtk)
{
	int i;
	int ret;

	for (i = 0; i < mtk->num_phys; i++) {
		ret = phy_power_on(mtk->phys[i]);
		if (ret)
			goto power_off_phy;
	}
	return 0;

power_off_phy:
	for (; i > 0; i--)
		phy_power_off(mtk->phys[i - 1]);

	return ret;
}

static void xhci_mtk_phy_power_off(struct xhci_hcd_mtk *mtk)
{
	unsigned int i;

	for (i = 0; i < mtk->num_phys; i++)
		phy_power_off(mtk->phys[i]);
}

static int xhci_mtk_ldos_enable(struct xhci_hcd_mtk *mtk)
{
	int ret;

	ret = regulator_enable(mtk->vbus);
	if (ret) {
		dev_err(mtk->dev, "failed to enable vbus\n");
		return ret;
	}

	ret = regulator_enable(mtk->vusb33);
	if (ret) {
		dev_err(mtk->dev, "failed to enable vusb33\n");
		regulator_disable(mtk->vbus);
		return ret;
	}
	return 0;
}

static void xhci_mtk_ldos_disable(struct xhci_hcd_mtk *mtk)
{
	regulator_disable(mtk->vbus);
	regulator_disable(mtk->vusb33);
}

static void xhci_mtk_quirks(struct device *dev, struct xhci_hcd *xhci)
{
	struct usb_hcd *hcd = xhci_to_hcd(xhci);
	struct xhci_hcd_mtk *mtk = hcd_to_mtk(hcd);

	/*
	 * As of now platform drivers don't provide MSI support so we ensure
	 * here that the generic code does not try to make a pci_dev from our
	 * dev struct in order to setup MSI
	 */
	xhci->quirks |= XHCI_PLAT;
	xhci->quirks |= XHCI_MTK_HOST;
	/*
	 * MTK host controller gives a spurious successful event after a
	 * short transfer. Ignore it.
	 */
	xhci->quirks |= XHCI_SPURIOUS_SUCCESS;
	if (mtk->lpm_support)
		xhci->quirks |= XHCI_LPM_SUPPORT;

	/*
	 * MTK xHCI 0.96: PSA is 1 by default even if doesn't support stream,
	 * and it's 3 when support it.
	 */
	if (xhci->hci_version < 0x100 && HCC_MAX_PSA(xhci->hcc_params) == 4)
		xhci->quirks |= XHCI_BROKEN_STREAMS;
}

/* called during probe() after chip reset completes */
static int xhci_mtk_setup(struct usb_hcd *hcd)
{
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	struct xhci_hcd_mtk *mtk = hcd_to_mtk(hcd);
	int ret;

	if (usb_hcd_is_primary_hcd(hcd)) {
		ret = xhci_mtk_ssusb_config(mtk);
		if (ret)
			return ret;
	}

	ret = xhci_gen_setup(hcd, xhci_mtk_quirks);
	if (ret)
		return ret;

	if (usb_hcd_is_primary_hcd(hcd)) {
		mtk->num_u3_ports = xhci->num_usb3_ports;
		mtk->num_u2_ports = xhci->num_usb2_ports;
		ret = xhci_mtk_sch_init(mtk);
		if (ret)
			return ret;
	}

	/* set runtime pm available */
	pm_runtime_put_noidle(mtk->dev);
	return ret;
}

static int xhci_mtk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct xhci_hcd_mtk *mtk;
	const struct hc_driver *driver;
	struct xhci_hcd *xhci;
	struct resource *res;
	struct usb_hcd *hcd;
	struct phy *phy;
	int phy_num;
	int ret = -ENODEV;
	int irq;

	if (usb_disabled())
		return -ENODEV;

	if (of_device_is_compatible(node, "mediatek,mt67xx-xhci")) {
		ret = device_rename(dev, node->name);
		if (ret)
			dev_info(&pdev->dev, "failed to rename\n");
		else {
			/* fix uaf(use after free) issue: backup pdev->name,
			 * device_rename will free pdev->name
			 */
			pdev->name = pdev->dev.kobj.name;
		}
	}

	driver = &xhci_mtk_hc_driver;
	mtk = devm_kzalloc(dev, sizeof(*mtk), GFP_KERNEL);
	if (!mtk)
		return -ENOMEM;

	mtk->dev = dev;
	mtk->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(mtk->vbus)) {
		dev_err(dev, "fail to get vbus\n");
		return PTR_ERR(mtk->vbus);
	}

	mtk->vusb33 = devm_regulator_get(dev, "vusb33");
	if (IS_ERR(mtk->vusb33)) {
		dev_err(dev, "fail to get vusb33\n");
		return PTR_ERR(mtk->vusb33);
	}

	mtk->sys_clk = devm_clk_get(dev, "sys_ck");
	if (IS_ERR(mtk->sys_clk)) {
		if (PTR_ERR(mtk->sys_clk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		mtk->sys_clk = NULL;
	}

	/*
	 * reference clock is usually a "fixed-clock", make it optional
	 * for backward compatibility and ignore the error if it does
	 * not exist.
	 */
	mtk->ref_clk = devm_clk_get(dev, "ref_ck");
	if (IS_ERR(mtk->ref_clk)) {
		if (PTR_ERR(mtk->ref_clk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		mtk->ref_clk = NULL;
	}

	mtk->lpm_support = of_property_read_bool(node, "usb3-lpm-capable");

	ret = usb_wakeup_of_property_parse(mtk, node);
	if (ret)
		return ret;

	mtk->num_phys = of_count_phandle_with_args(node,
			"phys", "#phy-cells");
	if (mtk->num_phys > 0) {
		mtk->phys = devm_kcalloc(dev, mtk->num_phys,
					sizeof(*mtk->phys), GFP_KERNEL);
		if (!mtk->phys)
			return -ENOMEM;
	} else {
		mtk->num_phys = 0;
	}
	pm_runtime_enable(dev);
	pm_runtime_get_noresume(dev);
	device_enable_async_suspend(dev);

	ret = xhci_mtk_ldos_enable(mtk);
	if (ret)
		goto disable_pm;

	ret = xhci_mtk_clks_enable(mtk);
	if (ret)
		goto disable_ldos;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto disable_clk;
	}

	/* Initialize dma_mask and coherent_dma_mask to 32-bits */
	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret)
		goto disable_clk;

	if (!dev->dma_mask)
		dev->dma_mask = &dev->coherent_dma_mask;
	else
		dma_set_mask(dev, DMA_BIT_MASK(32));

	hcd = usb_create_hcd(driver, dev, dev_name(dev));
	if (!hcd) {
		ret = -ENOMEM;
		goto disable_clk;
	}

	/*
	 * USB 2.0 roothub is stored in the platform_device.
	 * Swap it with mtk HCD.
	 */
	mtk->hcd = platform_get_drvdata(pdev);
	platform_set_drvdata(pdev, mtk);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mac");
	hcd->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto put_usb2_hcd;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ippc");
	if (res) {	/* ippc register is optional */
		mtk->ippc_regs = devm_ioremap_resource(dev, res);
		if (IS_ERR(mtk->ippc_regs)) {
			ret = PTR_ERR(mtk->ippc_regs);
			goto put_usb2_hcd;
		}
		mtk->has_ippc = true;
	} else {
		mtk->has_ippc = false;
	}

	for (phy_num = 0; phy_num < mtk->num_phys; phy_num++) {
		phy = devm_of_phy_get_by_index(dev, node, phy_num);
		if (IS_ERR(phy)) {
			ret = PTR_ERR(phy);
			goto put_usb2_hcd;
		}
		mtk->phys[phy_num] = phy;
	}

	ret = xhci_mtk_phy_init(mtk);
	if (ret)
		goto put_usb2_hcd;

	ret = xhci_mtk_phy_power_on(mtk);
	if (ret)
		goto exit_phys;

	device_init_wakeup(dev, true);
	dma_set_max_seg_size(dev, UINT_MAX);

	xhci = hcd_to_xhci(hcd);
	xhci->main_hcd = hcd;
	xhci->shared_hcd = usb_create_shared_hcd(driver, dev,
			dev_name(dev), hcd);
	if (!xhci->shared_hcd) {
		ret = -ENOMEM;
		goto power_off_phys;
	}

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (ret)
		goto put_usb3_hcd;

	if (HCC_MAX_PSA(xhci->hcc_params) >= 4 &&
	    !(xhci->quirks & XHCI_BROKEN_STREAMS))
		xhci->shared_hcd->can_do_streams = 1;

	ret = usb_add_hcd(xhci->shared_hcd, irq, IRQF_SHARED);
	if (ret)
		goto dealloc_usb2_hcd;

	xhci_mtk_dbg_init(mtk);

#if !IS_ENABLED(CONFIG_USB_XHCI_MTK_SUSPEND)
	mtk_xhci_wakelock_lock(mtk);
#else
	device_set_wakeup_enable(&hcd->self.root_hub->dev, 1);
	device_set_wakeup_enable(&xhci->shared_hcd->self.root_hub->dev, 1);
#endif
	return 0;

dealloc_usb2_hcd:
	usb_remove_hcd(hcd);

put_usb3_hcd:
	xhci_mtk_sch_exit(mtk);
	usb_put_hcd(xhci->shared_hcd);

power_off_phys:
	xhci_mtk_phy_power_off(mtk);
	device_init_wakeup(dev, false);

exit_phys:
	xhci_mtk_phy_exit(mtk);

put_usb2_hcd:
	usb_put_hcd(hcd);

disable_clk:
	xhci_mtk_clks_disable(mtk);

disable_ldos:
	xhci_mtk_ldos_disable(mtk);

disable_pm:
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	return ret;
}

static int xhci_mtk_remove(struct platform_device *dev)
{
	struct xhci_hcd_mtk *mtk = platform_get_drvdata(dev);
	struct usb_hcd	*hcd = mtk->hcd;
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	struct usb_hcd  *shared_hcd = xhci->shared_hcd;

	pm_runtime_put_noidle(&dev->dev);
	pm_runtime_disable(&dev->dev);

	xhci->xhc_state |= XHCI_STATE_REMOVING;

#if IS_ENABLED(CONFIG_MACH_MT6853)
	if (mtk->last_speed == DEV_SPEED_FULL) {
		ssusb_set_phy_mode(DEV_SPEED_INACTIVE);
		xhci_mtk_frmcnt_range(hcd, DEV_SPEED_INACTIVE);
	}
#endif

	usb_remove_hcd(shared_hcd);
	xhci->shared_hcd = NULL;
	xhci_mtk_phy_power_off(mtk);
	xhci_mtk_phy_exit(mtk);
	device_init_wakeup(&dev->dev, false);

#if !IS_ENABLED(CONFIG_USB_XHCI_MTK_SUSPEND)
	mtk_xhci_wakelock_unlock(mtk);
#endif
	xhci_mtk_dbg_exit(mtk);
	usb_remove_hcd(hcd);
	usb_put_hcd(shared_hcd);
	usb_put_hcd(hcd);
	xhci_mtk_sch_exit(mtk);
	xhci_mtk_clks_disable(mtk);
	xhci_mtk_ldos_disable(mtk);

	return 0;
}

static int __maybe_unused xhci_mtk_runtime_suspend(struct device *dev)
{
	struct xhci_hcd_mtk *mtk = dev_get_drvdata(dev);
	struct xhci_hcd *xhci;

	if (!mtk->hcd)
		return -ESHUTDOWN;

	xhci = hcd_to_xhci(mtk->hcd);

	xhci_info(xhci, "%s\n", __func__);
	xhci_mtk_host_disable(mtk);
	return 0;
}

static int __maybe_unused xhci_mtk_runtime_resume(struct device *dev)
{
	struct xhci_hcd_mtk *mtk = dev_get_drvdata(dev);
	struct xhci_hcd *xhci = hcd_to_xhci(mtk->hcd);

	xhci_info(xhci, "%s\n", __func__);
	xhci_mtk_host_enable(mtk);
	return 0;
}

/*
 * if ip sleep fails, and all clocks are disabled, access register will hang
 * AHB bus, so stop polling roothubs to avoid regs access on bus suspend.
 * and no need to check whether ip sleep failed or not; this will cause SPM
 * to wake up system immediately after system suspend complete if ip sleep
 * fails, it is what we wanted.
 */
static int __maybe_unused xhci_mtk_suspend(struct device *dev)
{
	struct xhci_hcd_mtk *mtk = dev_get_drvdata(dev);
	struct usb_hcd *hcd = mtk->hcd;
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);

	xhci_info(xhci, "%s\n", __func__);
	xhci_dbg(xhci, "%s: stop port polling\n", __func__);
	clear_bit(HCD_FLAG_POLL_RH, &hcd->flags);
	del_timer_sync(&hcd->rh_timer);
	clear_bit(HCD_FLAG_POLL_RH, &xhci->shared_hcd->flags);
	del_timer_sync(&xhci->shared_hcd->rh_timer);

	xhci_mtk_host_disable(mtk);
	xhci_mtk_phy_power_off(mtk);
	xhci_mtk_clks_disable(mtk);
	usb_wakeup_enable(mtk);
	return 0;
}

static int __maybe_unused xhci_mtk_resume(struct device *dev)
{
	struct xhci_hcd_mtk *mtk = dev_get_drvdata(dev);
	struct usb_hcd *hcd = mtk->hcd;
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);

	xhci_info(xhci, "%s\n", __func__);
	usb_wakeup_disable(mtk);
	xhci_mtk_clks_enable(mtk);
	xhci_mtk_phy_power_on(mtk);
	xhci_mtk_host_enable(mtk);

	xhci_dbg(xhci, "%s: restart port polling\n", __func__);
	set_bit(HCD_FLAG_POLL_RH, &xhci->shared_hcd->flags);
	usb_hcd_poll_rh_status(xhci->shared_hcd);
	set_bit(HCD_FLAG_POLL_RH, &hcd->flags);
	usb_hcd_poll_rh_status(hcd);
	return 0;
}

static const struct dev_pm_ops xhci_mtk_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(xhci_mtk_suspend, xhci_mtk_resume)
};
#define DEV_PM_OPS IS_ENABLED(CONFIG_PM) ? &xhci_mtk_pm_ops : NULL

static const struct dev_pm_ops xhci_mtk_phone_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(xhci_mtk_suspend, xhci_mtk_resume)
	SET_RUNTIME_PM_OPS(xhci_mtk_runtime_suspend,
			xhci_mtk_runtime_resume, NULL)
};
#define DEV_PHONE_PM_OPS (IS_ENABLED(CONFIG_PM) ? &xhci_mtk_phone_pm_ops : NULL)


#ifdef CONFIG_OF
static const struct of_device_id mtk_xhci_of_match[] = {
	{ .compatible = "mediatek,mt8173-xhci"},
	{ .compatible = "mediatek,mtk-xhci"},
	{ },
};
MODULE_DEVICE_TABLE(of, mtk_xhci_of_match);

static const struct of_device_id mtk_xhci_phone_of_match[] = {
	{ .compatible = "mediatek,mt67xx-xhci"},
	{ },
};
MODULE_DEVICE_TABLE(of, mtk_xhci_phone_of_match);

#endif

static struct platform_driver mtk_xhci_driver = {
	.probe	= xhci_mtk_probe,
	.remove	= xhci_mtk_remove,
	.driver	= {
		.name = "xhci-mtk",
		.pm = DEV_PM_OPS,
		.of_match_table = of_match_ptr(mtk_xhci_of_match),
	},
};

static struct platform_driver mtk_xhci_phone_driver = {
	.probe	= xhci_mtk_probe,
	.remove	= xhci_mtk_remove,
	.driver	= {
		.name = "xhci-mtk-phone",
		.pm = DEV_PHONE_PM_OPS,
		.of_match_table = of_match_ptr(mtk_xhci_phone_of_match),
	},
};

MODULE_ALIAS("platform:xhci-mtk");

int xhci_mtk_register_plat(void)
{
	return platform_driver_register(&mtk_xhci_phone_driver);
}

void xhci_mtk_unregister_plat(void)
{
	platform_driver_unregister(&mtk_xhci_phone_driver);
}

static int __init xhci_mtk_init(void)
{
	xhci_init_driver(&xhci_mtk_hc_driver, &xhci_mtk_overrides);
	return platform_driver_register(&mtk_xhci_driver);
}
module_init(xhci_mtk_init);

static void __exit xhci_mtk_exit(void)
{
	platform_driver_unregister(&mtk_xhci_driver);
}
module_exit(xhci_mtk_exit);

MODULE_AUTHOR("Chunfeng Yun <chunfeng.yun@mediatek.com>");
MODULE_DESCRIPTION("MediaTek xHCI Host Controller Driver");
MODULE_LICENSE("GPL v2");
