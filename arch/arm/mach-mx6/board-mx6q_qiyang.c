/*
 * Copyright (C) 2012-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/mxc-hdmi-core.h>
#include <linux/spi/ads7846.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/imx_rfkill.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include <linux/misc/buzzer.h>
#include "board-mx6q_qiyang.h"
#include "board-mx6dl_qiyang.h"

#include <linux/misc/watchdog.h>

static struct clk *sata_clk;
static struct clk *clko;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;


#define BUZZER_PIN      IMX_GPIO_NR(6, 31)

#define ENABLE_PIN      IMX_GPIO_NR(3, 23)  
#define FEEDDOG_PIN    IMX_GPIO_NR(3, 22)   

#define QY_SD3_CD          IMX_GPIO_NR(7 , 0)
#define QY_SD3_WP          IMX_GPIO_NR(7 , 1)
#define QY_CAM_PWDN        IMX_GPIO_NR(5 , 23)
#define QY_CAM_RST         IMX_GPIO_NR(5 , 24)
#define QY_OTG_PWR         IMX_GPIO_NR(6 , 18)
#define QY_OTG_OC          IMX_GPIO_NR(6 , 17)
#define QY_OTG_ID          IMX_GPIO_NR(7 , 8)
#define QY_PCIE_DIS        IMX_GPIO_NR(1 , 7)
#define QY_PCIE_RST        IMX_GPIO_NR(1 , 8)
#define QY_HUB_RST         IMX_GPIO_NR(1 , 10)
#define QY_FEC_RST         IMX_GPIO_NR(1 , 11)
#define QY_SPI1_CS0        IMX_GPIO_NR(2 , 30)
#define QY_SPI1_CS1        IMX_GPIO_NR(3 , 19)
#define QY_SPI5_CS0        IMX_GPIO_NR(1 , 17)
#define QY_SPI5_CS1        IMX_GPIO_NR(1 , 19)
#define QY_SPI5_CS2        IMX_GPIO_NR(1 , 21)
#define QY_TS_BUSY         IMX_GPIO_NR(3 , 20)
#define QY_TS_INT          IMX_GPIO_NR(3 , 21)
#define QY_SATA_PWR        IMX_GPIO_NR(7 , 11)

static const struct esdhc_platform_data mx6q_sd3_data __initconst = {
        .cd_gpio = QY_SD3_CD,
        .wp_gpio = QY_SD3_WP,
        .keep_power_at_suspend = 1,
        .support_8bit = 0,
        .delay_line = 0,
        .cd_type = ESDHC_CD_CONTROLLER,
};

static const struct esdhc_platform_data mx6q_sd4_data __initconst = {
        .always_present = 1,
        .keep_power_at_suspend = 1,
        .support_8bit = 1,
        .delay_line = 0,
        .cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data mx6q_anatop_thermal_data __initconst = 
{
	.name = "anatop_thermal",
};
static inline void mx6q_init_uart(void)
{
	imx6q_add_imx_uart(4, NULL);
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(0, NULL);
}

static int mx6q_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;
        phy_write(phydev, 0x0, 0x80); //reset phy soft
        msleep(5);
        phy_write(phydev, 0xd, 0x3);
        phy_write(phydev, 0xe, 0x805d);
        phy_write(phydev, 0xd, 0x4003);
        val = phy_read(phydev, 0xe);
        val &= ~(0x1 << 8);
        phy_write(phydev, 0xe, val);

        /* To enable AR8031 ouput a 125MHz clk from CLK_25M */
        phy_write(phydev, 0xd, 0x7);
        phy_write(phydev, 0xe, 0x8016);
        phy_write(phydev, 0xd, 0x4007);
        val = phy_read(phydev, 0xe);

        val &= 0xffe3;
        val |= 0x18;
        phy_write(phydev, 0xe, val);

        /* Introduce tx clock delay */
        phy_write(phydev, 0x1d, 0x5);
        val = phy_read(phydev, 0x1e);
        val |= 0x0100;
        phy_write(phydev, 0x1e, val);

	/*check phy power*/
        val = phy_read(phydev, 0x0);

        if (val & BMCR_PDOWN)
                phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

        return 0;
}

static struct fec_platform_data fec_data_rgmii __initdata = {
        .init = mx6q_fec_phy_init,
        .phy = PHY_INTERFACE_MODE_RGMII,
};

static int mx6q_spi1_cs[] = {
	QY_SPI1_CS0,
	QY_SPI1_CS1,
};

static const struct spi_imx_master mx6q_spi1_data __initconst = {
	.chipselect     = mx6q_spi1_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_spi1_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition imx6_spi_nor_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data imx6_spi_flash_data = {
	.name = "m25p80",
	.parts = imx6_spi_nor_partitions,
	.nr_parts = ARRAY_SIZE(imx6_spi_nor_partitions),
	//.type = "sst25vf016b",
	.type = "n25q032",
};
#endif

static int get_pendown_state(void)
{
        return !gpio_get_value(QY_TS_INT);
}

static struct ads7846_platform_data ads_info = {
	    .swap_xy             = true,
        .x_max               = 800,
        .y_max               = 480,
        .x_plate_ohms        = 180,
        .pressure_max        = 255,
        .debounce_max        = 10,
        .debounce_tol        = 3,
        .debounce_rep        = 1,
        .get_pendown_state    = get_pendown_state,
        .keep_vref_on        = 1,
        .settle_delay_usecs    = 150,
        .wakeup            = true,
};

static struct spi_board_info imx6_spi1_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
        {
                .modalias = "m25p80",
                .max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
                .bus_num = 0,
                .chip_select = 0,
                .platform_data = &imx6_spi_flash_data,
        },
#endif
	   {
                .modalias       = "ads7846",
                .max_speed_hz   = 1500000,
                .bus_num        = 0,
                .chip_select    = 1,
                .irq            = gpio_to_irq(QY_TS_INT),
                .platform_data  = &ads_info,
        },
};
static int mx6q_spi5_cs[] = {
        QY_SPI5_CS0,
        QY_SPI5_CS1,
	      QY_SPI5_CS2,
};

static const struct spi_imx_master mx6q_spi5_data __initconst = {
        .chipselect     = mx6q_spi5_cs,
        .num_chipselect = ARRAY_SIZE(mx6q_spi5_cs),
};
static struct spi_board_info imx6_spi5_device[] __initdata = {
		    {
                .modalias       = "spidev",
                .max_speed_hz   = 125000 * 16,
                .bus_num        = 4,
                .chip_select    = 0,
             	.mode 			= SPI_MODE_0,
        },
               {
                .modalias       = "spidev",
                .max_speed_hz   = 125000 * 16,
                .bus_num        = 4,
                .chip_select    = 1,
             	.mode 			= SPI_MODE_0,
        },
               {
                .modalias       = "spidev",
                .max_speed_hz   = 125000 * 16,
                .bus_num        = 4,
                .chip_select    = 2,
             	.mode 			= SPI_MODE_0,
        },//add slave device
};
static void spi_device_init(void)
{
	spi_register_board_info(imx6_spi1_device,
				ARRAY_SIZE(imx6_spi1_device));

	if(cpu_is_mx6q())
		spi_register_board_info(imx6_spi5_device,
                                ARRAY_SIZE(imx6_spi5_device));
}

static void mx6q_csi0_io_init(void)
{

	/* Camera reset */
	gpio_request(QY_CAM_RST, "cam-reset");
	gpio_direction_output(QY_CAM_RST, 1);

	/* Camera power down */
	gpio_request(QY_CAM_PWDN, "cam-pwdn");
	gpio_direction_output(QY_CAM_PWDN, 1);
	msleep(5);
	gpio_set_value(QY_CAM_PWDN, 0);
	msleep(5);
	gpio_set_value(QY_CAM_RST, 0);
	msleep(1);
	gpio_set_value(QY_CAM_RST, 1);
	msleep(5);
	gpio_set_value(QY_CAM_PWDN, 1);

	/* For MX6Q:
	 * GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 *
	 * For MX6DL:
	 * GPR13 bit 0-2 IPU_CSI0_MUX
	 *   000 MIPI_CSI0
	 *   100 IPU CSI0
	 */
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 4);
}
static void mx6q_csi0_cam_powerdown(int powerdown)
{

        if (powerdown)
                gpio_set_value(QY_CAM_PWDN, 1);
        else
                gpio_set_value(QY_CAM_PWDN, 0);

        msleep(2);

}
static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};

static struct imxi2c_platform_data mx6q_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
                I2C_BOARD_INFO("ov564x", 0x3c),
                .platform_data = (void *)&camera_data,
        },

};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
     {
		I2C_BOARD_INFO("ds1338", 0x68),
	},
 	{
    I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
    },
 
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
			I2C_BOARD_INFO("ft5x0x_ts", 0x38),
	},
};


static void imx6q_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(QY_OTG_PWR, 1);
	else
		gpio_set_value(QY_OTG_PWR, 0);
}
static void __init imx6q_init_usb(void)
{
	int ret = 0;
	ret = gpio_request(QY_HUB_RST, "hub-rst");
	gpio_direction_output(QY_HUB_RST, 0);
	msleep(50);
	gpio_direction_output(QY_HUB_RST, 1);

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	ret = gpio_request(QY_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO QY_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(QY_OTG_PWR, 0);
#if 0
	ret = gpio_request(QY_OTG_OC, "otg-oc");
        if (ret) {
                printk(KERN_ERR"failed to get GPIO QY_OTG_OC:"
                        " %d\n", ret);
                return;
        }
        gpio_direction_input(QY_OTG_OC);
#endif
#if 0
	mxc_iomux_set_gpr_register(1, 13, 1, 1);
#else
	//use extenal otg id gpio 
#endif
	mx6_set_otghost_vbus_func(imx6q_usbotg_vbus);
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_sata_data = {
	.init = mx6q_sata_init,
	.exit = mx6q_sata_exit,
};
#endif

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data fb_data[] = {
	{
        .disp_dev = "lcd",
        .interface_pix_fmt = IPU_PIX_FMT_RGB24,
        //.mode_str = "1024x768M@60",
        .mode_str = "QY-LCD-800X480",
        //.default_bpp = 32,
        .default_bpp = 16,
        .int_clk = false,
        .late_init = false,
        },

	{
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = false,
    .late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

static void hdmi_enable_ddc_pin(void)
{
}

static void hdmi_disable_ddc_pin(void)
{
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB24,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 0,
	.sec_disp_id = 1,
};
static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	},
};


static void suspend_enter(void)
{
}

static void suspend_exit(void)
{
}
static const struct pm_platform_data mx6q_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = suspend_enter,
	.suspend_exit = suspend_exit,
};

static int mx6q_sd_bt_power_change(int status)
{
	return 0;
}
static struct platform_device mxc_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct imx_bt_rfkill_platform_data mxc_bt_rfkill_data = {
	.power_change = mx6q_sd_bt_power_change,
};
struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = 0,
};

static struct platform_device mx6_audio_tlv320_device = {
        .name = "imx-tlv320",
};

static int tlv320_clk_enable(int enable)
{
        if (enable)
                clk_enable(clko);
        else
                clk_disable(clko);

        return 0;
}
static struct mxc_audio_platform_data tlv320_data;
static int mxc_tlv320_init(void)
{
        int rate;

        clko = clk_get(NULL, "clko_clk");
        if (IS_ERR(clko)) {
                pr_err("can't get CLKO clock.\n");
                return PTR_ERR(clko);
        }
        /* both audio codec and comera use CLKO clk*/
        rate = clk_round_rate(clko, 24000000);
        clk_set_rate(clko, rate);

        tlv320_data.sysclk = rate;

        return 0;
}
static struct mxc_audio_platform_data tlv320_data = {
        .ssi_num = 1,
        .src_port = 2,
        .ext_port = 4,
        .init = mxc_tlv320_init,
        .clock_enable = tlv320_clk_enable,
};

static struct imx_ssi_platform_data mx6_ssi_pdata = {
        .flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static int __init imx6q_init_audio(void)
{
	mxc_register_device(&mx6_audio_tlv320_device,
                                    &tlv320_data);
        imx6q_add_imx_ssi(1, &mx6_ssi_pdata);
        mxc_tlv320_init();
	return 0;
}
#ifndef CONFIG_IMX_PCIE
static void pcie_3v3_power(void)
{
}

static void pcie_3v3_reset(void)
{
	/* reset miniPCIe */
	gpio_request(QY_PCIE_RST, "pcie_reset_rebB");
	gpio_direction_output(QY_PCIE_RST, 0);
	/* The PCI Express Mini CEM specification states that PREST# is
	deasserted minimum 1ms after 3.3vVaux has been applied and stable*/
	mdelay(1);
	gpio_set_value(QY_PCIE_RST, 1);
	gpio_free(QY_PCIE_RST);
}
#endif

#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)

#define GPIO_LED(gpio_led, name_led, act_low, state_suspend, trigger)	\
{									\
	.gpio			= gpio_led,				\
	.name			= name_led,				\
	.active_low		= act_low,				\
	.retain_state_suspended = state_suspend,			\
	.default_state		= 0,					\
	.default_trigger	= trigger,		\
}

/* use to show a external power source is connected
 * GPIO_LED(SABRESD_CHARGE_DONE, "chg_detect", 0, 1, "ac-online"),
 */
static struct gpio_led imx6q_gpio_leds[] = {
	GPIO_LED(QY_SATA_PWR , "sata pwr" , 1 , 0, "sata-pwr"),
};

static struct gpio_led_platform_data imx6q_gpio_leds_data = {
	.leds		= imx6q_gpio_leds,
	.num_leds	= ARRAY_SIZE(imx6q_gpio_leds),
};

static struct platform_device imx6q_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_gpio_leds_data,
	}
};

static void __init imx6q_add_device_gpio_leds(void)
{
	platform_device_register(&imx6q_gpio_led_device);
}
#else
static void __init imx6q_add_device_gpio_leds(void) {}
#endif

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button imx6q_buttons[] = {
};

static struct gpio_keys_platform_data imx6q_button_data = {
	.buttons	= imx6q_buttons,
	.nbuttons	= ARRAY_SIZE(imx6q_buttons),
};

static struct platform_device imx6q_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_button_data,
	}
};

static void __init imx6q_add_device_buttons(void)
{
	platform_device_register(&imx6q_button_device);
}
#else
static void __init imx6q_add_device_buttons(void) {}
#endif

static struct mxc_dvfs_platform_data dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = fb_data;
    	printk("============fire qiyang evb ==============ssss\n");
	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			/* Primary framebuffer base address */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fb0base=");
			if (str != NULL) {
				str += 8;
				pdata_fb[0].res_base[0] =
						simple_strtol(str, &str, 16);
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data mx6_pcie_data __initconst = {
	.pcie_dis	= QY_PCIE_DIS,
	.pcie_rst	= QY_PCIE_RST,
};

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
	.name = "android ram console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console = {
	.name = "ram_console",
	.num_resources = 1,
	.resource = &ram_console_resource,
};

static int __init imx6x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#else
#define imx6x_add_ram_console() do {} while (0)
#endif
void qiyang_setup_pads(void)
{
	if(cpu_is_mx6q())
	{
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_i2c_pads,ARRAY_SIZE(mx6q_qiyang_i2c_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_spi_pads,ARRAY_SIZE(mx6q_qiyang_spi_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_misc_pads,ARRAY_SIZE(mx6q_qiyang_misc_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_csi0_pads,ARRAY_SIZE(mx6q_qiyang_csi0_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_disp0_pads,ARRAY_SIZE(mx6q_qiyang_disp0_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_sd_pads,ARRAY_SIZE(mx6q_qiyang_sd_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_uart_pads,ARRAY_SIZE(mx6q_qiyang_uart_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_rgmii_pads,ARRAY_SIZE(mx6q_qiyang_rgmii_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_audio_pads,ARRAY_SIZE(mx6q_qiyang_audio_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_can_pads,ARRAY_SIZE(mx6q_qiyang_can_pads));
		mxc_iomux_v3_setup_multiple_pads(mx6q_qiyang_matrix_keys_pads,ARRAY_SIZE(mx6q_qiyang_matrix_keys_pads));
		
	}
	else if(cpu_is_mx6dl())
	{
		mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_i2c_pads,ARRAY_SIZE(mx6dl_qiyang_i2c_pads));
                mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_spi_pads,ARRAY_SIZE(mx6dl_qiyang_spi_pads));
                mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_misc_pads,ARRAY_SIZE(mx6dl_qiyang_misc_pads));
                mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_csi0_pads,ARRAY_SIZE(mx6dl_qiyang_csi0_pads));
                mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_disp0_pads,ARRAY_SIZE(mx6dl_qiyang_disp0_pads));
                mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_sd_pads,ARRAY_SIZE(mx6dl_qiyang_sd_pads));
                mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_uart_pads,ARRAY_SIZE(mx6dl_qiyang_uart_pads));
                mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_rgmii_pads,ARRAY_SIZE(mx6dl_qiyang_rgmii_pads));
                mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_audio_pads,ARRAY_SIZE(mx6dl_qiyang_audio_pads));
                mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_can_pads,ARRAY_SIZE(mx6dl_qiyang_can_pads));
		   mxc_iomux_v3_setup_multiple_pads(mx6dl_qiyang_matrix_keys_pads,ARRAY_SIZE(mx6dl_qiyang_matrix_keys_pads));
	}
}



static struct platform_device imx6_gpio_device = {
	.name = "qiyang_imx6_gpio",
};
static void imx6_imx6_gpio_device_init(void) {

	platform_device_register(&imx6_gpio_device);
}

static struct watchdog_platform_data watchdog_data = {
		.enable_pin  = ENABLE_PIN,
		.feeddog_pin = FEEDDOG_PIN,
};

static struct platform_device watchdog_device ={
	.name = "qy_watchdog",
	.dev  = {
		.platform_data = &watchdog_data,
	},
};

static void imx6_watchdog_init(void)
{

	printk("Qy imx6 watchdog init!\n");
	platform_device_register(&watchdog_device);
}

/*Add by wujiajie @ 2014,5,24*/
static struct buzzer_platform_data imx6_buzzer_data =
{
	.pin_number = BUZZER_PIN,
};

static struct platform_device imx6_buzzer_device = 
{
	.name = "qiyang_buzzer",
	.dev = 
	{
		.platform_data = &imx6_buzzer_data,
	},
};

static void imx6_buzzer_init(void)
{
	printk("Qy imx6 buzzer init!\n");
	platform_device_register(&imx6_buzzer_device);
}



/* Add by wujiajie @ 2014-06-30*/
/* matrix keypad  */
static const uint32_t imx_sabresd_matrix_keys[] = {
#if 0
	KEY(0,0, KEY_7), KEY(1,0, KEY_8), KEY(2,0, KEY_9), KEY(3,0, KEY_DOWN), 
	KEY(0,1, KEY_BACKSPACE), KEY(1,1, KEY_0), KEY(2,1, KEY_DOT), KEY(3,1, KEY_ENTER), 
    KEY(0,2, KEY_4), KEY(1,2, KEY_5), KEY(2,2, KEY_6), KEY(3,2, KEY_UP), 
    KEY(0,3, KEY_1), KEY(1,3, KEY_2), KEY(2,3,KEY_3),KEY(3,3,KEY_EXIT), 
   #endif

	 KEY(0,1, KEY_2), KEY(1,1, KEY_5), KEY(2,1, KEY_8), KEY(3,1, KEY_0), 
	 KEY(0,0, KEY_1), KEY(1,0, KEY_4), KEY(2,0, KEY_7), KEY(3,0, KEY_BACKSPACE), 
    //KEY(0,1, KEY_2), KEY(1,1, KEY_5), KEY(2,1, KEY_8), KEY(3,1, KEY_0), 

   KEY(0,3, KEY_EXIT), KEY(1,3, KEY_UP), KEY(2,3,KEY_DOWN),KEY(3,3,KEY_ENTER), 
    KEY(0,2, KEY_3), KEY(1,2, KEY_6), KEY(2,2, KEY_9), KEY(3,2, KEY_DOT), 
    //KEY(0,3, KEY_EXIT), KEY(1,3, KEY_UP), KEY(2,3,KEY_DOWN),KEY(3,3,KEY_ENTER), 

    //KEY(0,0, KEY_1),KEY(0,1, KEY_2),KEY(0,2, KEY_3),KEY(0,3, KEY_EXIT),
    //KEY(1,0, KEY_4),KEY(1,1, KEY_5),KEY(1,2, KEY_6),KEY(1,3, KEY_UP),
    //KEY(2,0, KEY_7),KEY(2,1, KEY_8),KEY(2,2, KEY_9),KEY(2,3,KEY_DOWN),
    //KEY(3,0, KEY_BACKSPACE), KEY(3,1, KEY_0),KEY(3,2, KEY_DOT), KEY(3,3,KEY_ENTER),
};

const struct matrix_keymap_data imx_sabresd_keymap_data = {
	.keymap      = imx_sabresd_matrix_keys,
	.keymap_size = ARRAY_SIZE(imx_sabresd_matrix_keys),
};

static const unsigned int imx_sabresd_keypad_row_gpios[] = {
	
	IMX_GPIO_NR(2, 0), IMX_GPIO_NR(2, 2), IMX_GPIO_NR(2, 4),IMX_GPIO_NR(2, 6)
};

static const unsigned int imx_sabresd_keypad_col_gpios[] = {
	
	IMX_GPIO_NR(2, 1), IMX_GPIO_NR(2, 3),IMX_GPIO_NR(2, 5), IMX_GPIO_NR(2, 7)
};

static struct matrix_keypad_platform_data imx_sabresd_keypad_platform_data = {
	.keymap_data       = &imx_sabresd_keymap_data,
	.row_gpios         = imx_sabresd_keypad_row_gpios,
	.num_row_gpios     = ARRAY_SIZE(imx_sabresd_keypad_row_gpios),
	.col_gpios         = imx_sabresd_keypad_col_gpios,
	.num_col_gpios     = ARRAY_SIZE(imx_sabresd_keypad_col_gpios),
	.active_low        = true,
	.debounce_ms       = 5,
	.col_scan_delay_us = 2,
};
static struct platform_device imx_sabresd_keyboard = {
	.name  = "matrix-keypad",
	.id    = -1,
	.dev   = {
		.platform_data = &imx_sabresd_keypad_platform_data,
	},
};

static void imx6_matrix_keypad_init(void)
{
	int err;
	err = platform_device_register(&imx_sabresd_keyboard);
	if (err) {
		pr_err("failed to register matrix keypad (2x3) device\n");
	}
}


static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};
/*!
 * Board specific initialization.
 */
static void __init mx6_board_init(void)
{	
	
	int i;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	qiyang_setup_pads();
	imx6_imx6_gpio_device_init();
	gp_reg_id = dvfscore_data.reg_id;
	soc_reg_id = dvfscore_data.soc_id;
	mx6q_init_uart();
	imx6x_add_ram_console();
  mxc_register_device(&mxc_bt_rfkill, &mxc_bt_rfkill_data);
	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		hdmi_core_data.ipu_id = 0;
		ldb_data.ipu_id = 0;
		ldb_data.sec_ipu_id = 0;
		strcpy(fb_data[1].mode_str , "1920x1080M@60");
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(fb_data); i++)
			imx6q_add_ipuv3fb(i, &fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(fb_data); i++)
			imx6q_add_ipuv3fb(i, &fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}

	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata); 

	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	imx6q_add_ecspi(0, &mx6q_spi1_data);
	spi_device_init();
	
	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_anatop_thermal_data);

	gpio_request(QY_FEC_RST, "fec-rst");
    gpio_direction_output(QY_FEC_RST, 0);
	msleep(10);
	gpio_set_value(QY_FEC_RST,1);

	imx6_init_fec(fec_data_rgmii);

	imx6q_add_pm_imx(0, &mx6q_pm_data);

	imx6q_add_sdhci_usdhc_imx(3, &mx6q_sd4_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_sd3_data);

	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_sata_data);
#else
		mx6q_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&dvfscore_data);

	imx6q_add_device_buttons();


	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}
	
	imx6q_add_flexcan0(NULL);
	imx6q_add_flexcan1(NULL);

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);
	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);
#ifndef CONFIG_IMX_PCIE
	/* enable pcie 3v3 power without pcie driver */
	pcie_3v3_power();
	mdelay(10);
	pcie_3v3_reset();
#endif

	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_pcie_data);
	if (cpu_is_mx6dl()) {
	}

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

	imx6_watchdog_init();
	imx6_buzzer_init();		
	imx6_matrix_keypad_init();
}

extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_timer = {
	.init   = mx6_timer_init,
};

static void __init mx6q_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(fb_data);
	if (fb_array_size > 0 && fb_data[0].res_base[0] &&
	    fb_data[0].res_size[0]) {
		if (fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			    fb_data[0].res_base[0]);
		memblock_reserve(fb_data[0].res_base[0],
				 fb_data[0].res_size[0]);
		memblock_remove(fb_data[0].res_base[0],
				fb_data[0].res_size[0]);
		fb_data[0].late_init = true;
		ipu_data[ldb_data.ipu_id].bypass_reset = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(fb_data[i].res_size[0],
						SZ_4K, SZ_2G);
			memblock_remove(phys, fb_data[i].res_size[0]);
			fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_1M, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_1M);
	memblock_free(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif

	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_8MILE_EVB data structure.
 */
MACHINE_START(MX6Q_QIYANG, "Freescale i.MX 6Quad/DualLite/Solo QiYang Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_board_init,
	.timer = &mx6_timer,
	.reserve = mx6q_reserve,
MACHINE_END
