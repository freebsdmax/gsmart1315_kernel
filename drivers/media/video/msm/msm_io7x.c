/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include <mach/clk.h>

#define CAMIF_CFG_RMSK 0x1fffff
#define CAM_SEL_BMSK 0x2
#define CAM_PCLK_SRC_SEL_BMSK 0x60000
#define CAM_PCLK_INVERT_BMSK 0x80000
#define CAM_PAD_REG_SW_RESET_BMSK 0x100000

#define EXT_CAM_HSYNC_POL_SEL_BMSK 0x10000
#define EXT_CAM_VSYNC_POL_SEL_BMSK 0x8000
#define MDDI_CLK_CHICKEN_BIT_BMSK  0x80

#define CAM_SEL_SHFT 0x1
#define CAM_PCLK_SRC_SEL_SHFT 0x11
#define CAM_PCLK_INVERT_SHFT 0x13
#define CAM_PAD_REG_SW_RESET_SHFT 0x14

#define EXT_CAM_HSYNC_POL_SEL_SHFT 0x10
#define EXT_CAM_VSYNC_POL_SEL_SHFT 0xF
#define MDDI_CLK_CHICKEN_BIT_SHFT  0x7
/* MIPI	CSI	controller registers */
#define	MIPI_PHY_CONTROL			0x00000000
#define	MIPI_PROTOCOL_CONTROL		0x00000004
#define	MIPI_INTERRUPT_STATUS		0x00000008
#define	MIPI_INTERRUPT_MASK			0x0000000C
#define	MIPI_CAMERA_CNTL			0x00000024
#define	MIPI_CALIBRATION_CONTROL	0x00000018
#define	MIPI_PHY_D0_CONTROL2		0x00000038
#define	MIPI_PHY_D1_CONTROL2		0x0000003C
#define	MIPI_PHY_D2_CONTROL2		0x00000040
#define	MIPI_PHY_D3_CONTROL2		0x00000044
#define	MIPI_PHY_CL_CONTROL			0x00000048
#define	MIPI_PHY_D0_CONTROL			0x00000034
#define	MIPI_PHY_D1_CONTROL			0x00000020
#define	MIPI_PHY_D2_CONTROL			0x0000002C
#define	MIPI_PHY_D3_CONTROL			0x00000030
#define	MIPI_PROTOCOL_CONTROL_SW_RST_BMSK			0x8000000
#define	MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK	0x200000
#define	MIPI_PROTOCOL_CONTROL_DATA_FORMAT_BMSK			0x180000
#define	MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK			0x40000
#define	MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK			0x20000
#define	MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT		0x16
#define	MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT	0x15
#define	MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT		0x14
#define	MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT	0x7
#define	MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT			0x13
#define	MIPI_PROTOCOL_CONTROL_DPCM_SCHEME_SHFT			0x1e
#define	MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_D1_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D1_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D1_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D1_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_D2_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D2_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D2_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D2_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_D3_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D3_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D3_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D3_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT			0x18
#define	MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT				0x2
#define	MIPI_PHY_D0_CONTROL_HS_REC_EQ_SHFT				0x1c
#define	MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT		0x9
#define	MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT	0x8
#define APPS_RESET_OFFSET 0x00000210

static struct clk *camio_vfe_mdc_clk;
static struct clk *camio_mdc_clk;
static struct clk *camio_vfe_clk;
static struct clk *camio_vfe_camif_clk;
static struct clk *camio_vfe_pbdg_clk;
static struct clk *camio_cam_m_clk;
static struct clk *camio_camif_pad_pbdg_clk;
static struct clk *camio_csi_clk;
static struct clk *camio_csi_pclk;
static struct clk *camio_csi_vfe_clk;
static struct clk *camio_jpeg_clk;
static struct clk *camio_jpeg_pclk;
static struct clk *camio_vpe_clk;
static struct vreg *vreg_gp2;
static struct vreg *vreg_lvsw1;
static struct vreg *vreg_gp6;
static struct vreg *vreg_gp16;
#if defined(CONFIG_MT9P111) || defined(CONFIG_MT9V114)
static struct vreg *vreg_gp13;
#define GPIO_CAM_POWER		    (19)
//#define GPIO_CAM_RESET	        (2)
#endif
static struct msm_camera_io_ext camio_ext;
static struct resource *appio, *mdcio;
void __iomem *appbase, *mdcbase;

static struct msm_camera_io_ext camio_clk;
static struct resource *appio, *mdcio;
void __iomem *appbase, *mdcbase;

void msm_io_w(u32 data, void __iomem *addr)
{
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	writel((data), (addr));
}

void msm_io_w_mb(u32 data, void __iomem *addr)
{
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	wmb();
	writel((data), (addr));
	wmb();
}

u32 msm_io_r(void __iomem *addr)
{
	uint32_t data = readl(addr);
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	return data;
}

u32 msm_io_r_mb(void __iomem *addr)
{
	uint32_t data;
	rmb();
	data = readl(addr);
	rmb();
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	return data;
}

void msm_io_memcpy_toio(void __iomem *dest_addr,
	void __iomem *src_addr, u32 len)
{
	int i;
	u32 *d = (u32 *) dest_addr;
	u32 *s = (u32 *) src_addr;
	/* memcpy_toio does not work. Use writel for now */
	for (i = 0; i < len; i++)
		writel(*s++, d++);
}

void msm_io_dump(void __iomem *addr, int size)
{
	char line_str[128], *p_str;
	int i;
	u32 *p = (u32 *) addr;
	u32 data;
	CDBG("%s: %p %d\n", __func__, addr, size);
	line_str[0] = '\0';
	p_str = line_str;
	for (i = 0; i < size/4; i++) {
		if (i % 4 == 0) {
			sprintf(p_str, "%08x: ", (u32) p);
			p_str += 10;
		}
		data = readl(p++);
		sprintf(p_str, "%08x ", data);
		p_str += 9;
		if ((i + 1) % 4 == 0) {
			CDBG("%s\n", line_str);
			line_str[0] = '\0';
			p_str = line_str;
		}
	}
	if (line_str[0] != '\0')
		CDBG("%s\n", line_str);
}

void msm_io_memcpy(void __iomem *dest_addr, void __iomem *src_addr, u32 len)
{
	CDBG("%s: %p %p %d\n", __func__, dest_addr, src_addr, len);
	msm_io_memcpy_toio(dest_addr, src_addr, len / 4);
	msm_io_dump(dest_addr, len);
}

static void msm_camera_vreg_enable(struct platform_device *pdev)
{
#if defined(CONFIG_MT9P111) || defined(CONFIG_MT9V114)
    if(strcmp(pdev->name, "msm_camera_mt9p111") == 0)
    { 
        
        CDBG("%s: %s \n", __func__, pdev->name);
        //vdd power on
        gpio_set_value(GPIO_CAM_POWER, 0);  
        // msleep(200);
        //vdd io
	    vreg_lvsw1 = vreg_get(NULL, "lvsw1");
	    if (IS_ERR(vreg_lvsw1)) {
		    pr_err("%s: VREG LVSW1 get failed %ld\n", __func__,
			    PTR_ERR(vreg_lvsw1));
		    vreg_lvsw1 = NULL;
		    return;
	    }
	    if (vreg_set_level(vreg_lvsw1, 1800)) {
		    pr_err("%s: VREG LVSW1 set failed\n", __func__);
		    goto lvsw1_put;
	    }
	    if (vreg_enable(vreg_lvsw1)) {
		    pr_err("%s: VREG LVSW1 enable failed\n", __func__);
		    goto lvsw1_put;
	    }      
        //mdelay(200);
    
        vreg_gp2 = vreg_get(NULL, "gp2");
	    if (IS_ERR(vreg_gp2)) {
		    pr_err("%s: VREG GP2 get failed %ld\n", __func__,
			    PTR_ERR(vreg_gp2));
		    vreg_gp2 = NULL;
            goto lvsw1_disable;
	    }
	    if (vreg_set_level(vreg_gp2, 2800)) {
		    pr_err("%s: VREG GP2 set failed\n", __func__);
		    goto gp2_put;
	    }

	    if (vreg_enable(vreg_gp2)) {
		    pr_err("%s: VREG GP2 enable failed\n", __func__);
		    goto gp2_put;
	    }   

        vreg_gp13 = vreg_get(NULL, "gp13");
	    if (IS_ERR(vreg_gp13)) {
		    pr_err("%s: VREG vreg_gp13 get failed %ld\n", __func__,
			    PTR_ERR(vreg_gp13));
		    vreg_gp13 = NULL;
		    goto gp2_disable;
		}
	    if (vreg_set_level(vreg_gp13, 2800)) {
		    pr_err("%s: VREG gp13 set failed\n", __func__);
		    goto gp13_put;
	    }
	    if (vreg_enable(vreg_gp13)) {
		    pr_err("%s: VREG gp13 enable failed\n", __func__);
		    goto gp13_put;
	    }
    }
    else{
        CDBG("%s: %s \n", __func__, pdev->name);
#endif
    
	vreg_gp2 = vreg_get(NULL, "gp2");
	if (IS_ERR(vreg_gp2)) {
		pr_err("%s: VREG GP2 get failed %ld\n", __func__,
			PTR_ERR(vreg_gp2));
		vreg_gp2 = NULL;
		return;
	}

#if defined(CONFIG_MT9P111) || defined(CONFIG_MT9V114)
	if (vreg_set_level(vreg_gp2, 2800)) {
#else
    if (vreg_set_level(vreg_gp2, 2600)) {
#endif
		pr_err("%s: VREG GP2 set failed\n", __func__);
		goto gp2_put;
	}

	if (vreg_enable(vreg_gp2)) {
		pr_err("%s: VREG GP2 enable failed\n", __func__);
		goto gp2_put;
	}

	vreg_lvsw1 = vreg_get(NULL, "lvsw1");
	if (IS_ERR(vreg_lvsw1)) {
		pr_err("%s: VREG LVSW1 get failed %ld\n", __func__,
			PTR_ERR(vreg_lvsw1));
		vreg_lvsw1 = NULL;
		goto gp2_disable;
	}
	if (vreg_set_level(vreg_lvsw1, 1800)) {
		pr_err("%s: VREG LVSW1 set failed\n", __func__);
		goto lvsw1_put;
	}
	if (vreg_enable(vreg_lvsw1)) {
		pr_err("%s: VREG LVSW1 enable failed\n", __func__);
		goto lvsw1_put;
	}
#if defined(CONFIG_MT9P111) || defined(CONFIG_MT9V114)        
    }
    //msleep(20);
#endif
	if (!strcmp(pdev->name, "msm_camera_sn12m0pz")) {
		vreg_gp6 = vreg_get(NULL, "gp6");
		if (IS_ERR(vreg_gp6)) {
			pr_err("%s: VREG GP6 get failed %ld\n", __func__,
				PTR_ERR(vreg_gp6));
			vreg_gp6 = NULL;
			goto lvsw1_disable;
		}

		if (vreg_set_level(vreg_gp6, 3050)) {
			pr_err("%s: VREG GP6 set failed\n", __func__);
			goto gp6_put;
		}

		if (vreg_enable(vreg_gp6)) {
			pr_err("%s: VREG GP6 enable failed\n", __func__);
			goto gp6_put;
		}
		vreg_gp16 = vreg_get(NULL, "gp16");
		if (IS_ERR(vreg_gp16)) {
			pr_err("%s: VREG GP16 get failed %ld\n", __func__,
				PTR_ERR(vreg_gp16));
			vreg_gp16 = NULL;
			goto gp6_disable;
		}

		if (vreg_set_level(vreg_gp16, 1200)) {
			pr_err("%s: VREG GP16 set failed\n", __func__);
			goto gp16_put;
		}

		if (vreg_enable(vreg_gp16)) {
			pr_err("%s: VREG GP16 enable failed\n", __func__);
			goto gp16_put;
		}
	}
	return;
    
#if defined(CONFIG_MT9P111) || defined(CONFIG_MT9V114)
gp13_put:
	vreg_put(vreg_gp13);
    vreg_gp13 = NULL;
#endif
gp16_put:
	vreg_put(vreg_gp16);
	vreg_gp16 = NULL;
gp6_disable:
	 vreg_disable(vreg_gp6);
gp6_put:
	vreg_put(vreg_gp6);
	vreg_gp6 = NULL;
lvsw1_disable:
	vreg_disable(vreg_lvsw1);
lvsw1_put:
	vreg_put(vreg_lvsw1);
	vreg_lvsw1 = NULL;
gp2_disable:
	vreg_disable(vreg_gp2);
gp2_put:
	vreg_put(vreg_gp2);
	vreg_gp2 = NULL;
}

    
#if defined(CONFIG_MT9P111) || defined(CONFIG_MT9V114)
static void msm_camera_vreg_disable(struct platform_device *pdev)
#else
static void msm_camera_vreg_disable(void)
#endif
{
#if defined(CONFIG_MT9P111) || defined(CONFIG_MT9V114)
    if (vreg_gp13) {
		vreg_disable(vreg_gp13);
		vreg_put(vreg_gp13);
        vreg_gp13 = NULL;
	} 
#endif
	if (vreg_gp2) {
		vreg_disable(vreg_gp2);
		vreg_put(vreg_gp2);
		vreg_gp2 = NULL;
	}
	if (vreg_lvsw1) {
		vreg_disable(vreg_lvsw1);
		vreg_put(vreg_lvsw1);
		vreg_lvsw1 = NULL;
	}
	if (vreg_gp6) {
		vreg_disable(vreg_gp6);
		vreg_put(vreg_gp6);
		vreg_gp6 = NULL;
	}
	if (vreg_gp16) {
		vreg_disable(vreg_gp16);
		vreg_put(vreg_gp16);
		vreg_gp16 = NULL;
	}
#if defined(CONFIG_MT9P111) || defined(CONFIG_MT9V114)
    if(strcmp(pdev->name, "msm_camera_mt9p111") == 0)
    { 
        gpio_set_value(GPIO_CAM_POWER, 1);  
        //msleep(100);
    }
#endif
}

int msm_camio_clk_enable(enum msm_camio_clk_type clktype)
{
	int rc = -1;
	struct clk *clk = NULL;
    int i;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		clk = camio_vfe_mdc_clk = clk_get(NULL, "vfe_mdc_clk");
		break;

	case CAMIO_MDC_CLK:
		clk = camio_mdc_clk = clk_get(NULL, "mdc_clk");
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk = clk_get(NULL, "vfe_clk");
		break;

	default:
		break;
	}

	if (!IS_ERR(clk)) {
		clk_enable(clk);

		for (i = 0; ((i < 10) && (!clk_is_enabled(clk))); i++)
			udelay(1);

		if(i < 10)
			rc = 0;
		else
		{
			pr_err("msm_camio_clk_enable failed!\n");
			rc = -EIO;
		}
	}

	return rc;
}

int msm_camio_clk_disable(enum msm_camio_clk_type clktype)
{
	int rc = -1;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		clk = camio_vfe_mdc_clk;
		break;

	case CAMIO_MDC_CLK:
		clk = camio_mdc_clk;
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk;
		break;

	default:
		break;
	}

	if (!IS_ERR(clk)) {
		clk_disable(clk);
		clk_put(clk);
		rc = 0;
	}

	return rc;
}

void msm_camio_clk_rate_set(int rate)
{
	struct clk *clk = camio_vfe_clk;

	if (clk != ERR_PTR(-ENOENT))
		clk_set_rate(clk, rate);
}

int msm_camio_enable(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;

	camio_ext = camdev->ioext;

	appio = request_mem_region(camio_ext.appphy,
		camio_ext.appsz, pdev->name);
	if (!appio) {
		rc = -EBUSY;
		goto enable_fail;
	}

	appbase = ioremap(camio_ext.appphy,
		camio_ext.appsz);
	if (!appbase) {
		rc = -ENOMEM;
		goto apps_no_mem;
	}

	mdcio = request_mem_region(camio_ext.mdcphy,
		camio_ext.mdcsz, pdev->name);
	if (!mdcio) {
		rc = -EBUSY;
		goto mdc_busy;
	}

	mdcbase = ioremap(camio_ext.mdcphy,
		camio_ext.mdcsz);
	if (!mdcbase) {
		rc = -ENOMEM;
		goto mdc_no_mem;
	}

	camdev->camera_gpio_on();

	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_enable(CAMIO_MDC_CLK);
	msm_camio_clk_enable(CAMIO_VFE_MDC_CLK);
	return 0;

mdc_no_mem:
	release_mem_region(camio_ext.mdcphy, camio_ext.mdcsz);
mdc_busy:
	iounmap(appbase);
apps_no_mem:
	release_mem_region(camio_ext.appphy, camio_ext.appsz);
enable_fail:
	return rc;
}

void msm_camio_disable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;

	iounmap(mdcbase);
	release_mem_region(camio_ext.mdcphy, camio_ext.mdcsz);
	iounmap(appbase);
	release_mem_region(camio_ext.appphy, camio_ext.appsz);

	camdev->camera_gpio_off();

	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_MDC_CLK);
	msm_camio_clk_disable(CAMIO_VFE_MDC_CLK);
}

void msm_disable_io_gpio_clk(struct platform_device *pdev)
{
	return;
}

void msm_camio_camif_pad_reg_reset(void)
{
	uint32_t reg;
	uint32_t mask, value;

	/* select CLKRGM_VFE_SRC_CAM_VFE_SRC:  internal source */
	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_INTERNAL);

	reg = (readl(mdcbase)) & CAMIF_CFG_RMSK;

	mask = CAM_SEL_BMSK |
		CAM_PCLK_SRC_SEL_BMSK |
		CAM_PCLK_INVERT_BMSK;

	value = 1 << CAM_SEL_SHFT |
		3 << CAM_PCLK_SRC_SEL_SHFT |
		0 << CAM_PCLK_INVERT_SHFT;

	writel((reg & (~mask)) | (value & mask), mdcbase);
	msleep(10);

	reg = (readl(mdcbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 1 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), mdcbase);
	msleep(10);

	reg = (readl(mdcbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 0 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), mdcbase);
	msleep(10);

	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_EXTERNAL);
	msleep(10);
}

void msm_camio_vfe_blk_reset(void)
{
	uint32_t val;

	/* do apps reset */
	val = readl(appbase + 0x00000210);
	val |= 0x1;
	writel(val, appbase + 0x00000210);
	mdelay(10);

	val = readl(appbase + 0x00000210);
	val &= ~0x1;
	writel(val, appbase + 0x00000210);
	mdelay(10);

	/* do axi reset */
	val = readl(appbase + 0x00000208);
	val |= 0x1;
	writel(val, appbase + 0x00000208);
	mdelay(10);

	val = readl(appbase + 0x00000208);
	val &= ~0x1;
	writel(val, appbase + 0x00000208);
	mdelay(10);
}

void msm_camio_camif_pad_reg_reset_2(void)
{
	uint32_t reg;
	uint32_t mask, value;

	reg = (readl(mdcbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 1 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), mdcbase);
	mdelay(10);

	reg = (readl(mdcbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 0 << CAM_PAD_REG_SW_RESET_SHFT;
	writel((reg & (~mask)) | (value & mask), mdcbase);
	mdelay(10);
}

void msm_camio_clk_sel(enum msm_camio_clk_src_type srctype)
{
	struct clk *clk = NULL;

	clk = camio_vfe_clk;

	if (clk != NULL && clk != ERR_PTR(-ENOENT)) {
		switch (srctype) {
		case MSM_CAMIO_CLK_SRC_INTERNAL:
			clk_set_flags(clk, 0x00000100 << 1);
			break;

		case MSM_CAMIO_CLK_SRC_EXTERNAL:
			clk_set_flags(clk, 0x00000100);
			break;

		default:
			break;
		}
	}
}

int msm_camio_csi_config(struct msm_camera_csi_params *csi_params)
{
	int rc = 0;
	uint32_t val = 0;

	CDBG("msm_camio_csi_config \n");

	/* SOT_ECC_EN enable error correction for SYNC (data-lane) */
	msm_io_w(0x4, mdcbase + MIPI_PHY_CONTROL);

	/* SW_RST to the CSI core */
	msm_io_w(MIPI_PROTOCOL_CONTROL_SW_RST_BMSK,
		mdcbase + MIPI_PROTOCOL_CONTROL);

	/* PROTOCOL CONTROL */
	val = MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK |
		MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK |
		MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK;
	val |= (uint32_t)(csi_params->data_format) <<
		MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT;
	val |= csi_params->dpcm_scheme <<
		MIPI_PROTOCOL_CONTROL_DPCM_SCHEME_SHFT;
	CDBG("%s MIPI_PROTOCOL_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, mdcbase + MIPI_PROTOCOL_CONTROL);

	/* SW CAL EN */
	val = (0x1 << MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT) |
		(0x1 <<
		MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT) |
		(0x1 << MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT) |
		(0x1 << MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT);
	CDBG("%s MIPI_CALIBRATION_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, mdcbase + MIPI_CALIBRATION_CONTROL);

	/* settle_cnt is very sensitive to speed!
	increase this value to run at higher speeds */
	val = (csi_params->settle_cnt <<
			MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
		(0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
		(0x1 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
		(0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
	CDBG("%s MIPI_PHY_D0_CONTROL2 val=0x%x\n", __func__, val);
	msm_io_w(val, mdcbase + MIPI_PHY_D0_CONTROL2);
	msm_io_w(val, mdcbase + MIPI_PHY_D1_CONTROL2);
	msm_io_w(val, mdcbase + MIPI_PHY_D2_CONTROL2);
	msm_io_w(val, mdcbase + MIPI_PHY_D3_CONTROL2);


	val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
		(0x1 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
	CDBG("%s MIPI_PHY_CL_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, mdcbase + MIPI_PHY_CL_CONTROL);

	val = 0 << MIPI_PHY_D0_CONTROL_HS_REC_EQ_SHFT;
	msm_io_w(val, mdcbase + MIPI_PHY_D0_CONTROL);

	val = (0x1 << MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT) |
		(0x1 << MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT);
	CDBG("%s MIPI_PHY_D1_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, mdcbase + MIPI_PHY_D1_CONTROL);

	msm_io_w(0x00000000, mdcbase + MIPI_PHY_D2_CONTROL);
	msm_io_w(0x00000000, mdcbase + MIPI_PHY_D3_CONTROL);

	/* halcyon only supports 1 or 2 lane */
	switch (csi_params->lane_cnt) {
	case 1:
		msm_io_w(csi_params->lane_assign << 8 | 0x4,
			mdcbase + MIPI_CAMERA_CNTL);
		break;
	case 2:
		msm_io_w(csi_params->lane_assign << 8 | 0x5,
			mdcbase + MIPI_CAMERA_CNTL);
		break;
	case 3:
		msm_io_w(csi_params->lane_assign << 8 | 0x6,
			mdcbase + MIPI_CAMERA_CNTL);
		break;
	case 4:
		msm_io_w(csi_params->lane_assign << 8 | 0x7,
			mdcbase + MIPI_CAMERA_CNTL);
		break;
	}

	/* mask out ID_ERROR[19], DATA_CMM_ERR[11]
	and CLK_CMM_ERR[10] - de-featured */
	msm_io_w(0xFFF7F3FF, mdcbase + MIPI_INTERRUPT_MASK);
	/*clear IRQ bits*/
	msm_io_w(0xFFF7F3FF, mdcbase + MIPI_INTERRUPT_STATUS);

	return rc;
}

int msm_camio_probe_on(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camdev->camera_gpio_on();
	return msm_camio_clk_enable(CAMIO_VFE_CLK);
}

int msm_camio_probe_off(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
#if defined(CONFIG_MT9P111) || defined(CONFIG_MT9V114)
    msm_camera_vreg_disable(pdev);
#else
	msm_camera_vreg_disable();
#endif
	camdev->camera_gpio_off();
	return msm_camio_clk_disable(CAMIO_VFE_CLK);
}
