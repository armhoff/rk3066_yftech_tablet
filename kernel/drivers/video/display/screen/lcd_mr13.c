#include <linux/fb.h>
#include <linux/delay.h>
#include "../../rk29_fb.h"
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include "screen.h"
#include <mach/yfmach.h>

/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_D888_P666
#define OUT_CLK		 66666666
#define LCDC_ACLK           500000000
/* Timing */
#define H_PW			32
#define H_BP			80
#define H_VD			1280
#define H_FP			48

#define V_PW			6
#define V_BP			14
#define V_VD			800
#define V_FP			3

#define LCD_WIDTH   320// 142  // 202
#define LCD_HEIGHT  180 //106//  152
/* Other */
#define DCLK_POL		0 // 
#define SWAP_RB			0

void set_lcd_info(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info )
{
    /* screen type & face */
    screen->type = env_get_u32("lcd_out_type", OUT_TYPE);
    screen->face = env_get_u32("lcd_out_face", OUT_FACE);

    /* Screen size */
    screen->x_res = env_get_u32("lcd_h_vd", H_VD);
    screen->y_res = env_get_u32("lcd_v_vd", V_VD);

    screen->width = env_get_u32("lcd_width", LCD_WIDTH);
    screen->height = env_get_u32("lcd_height", LCD_HEIGHT);

    /* Timing */
    screen->lcdc_aclk = env_get_u32("lcd_aclk", LCDC_ACLK);
    screen->pixclock = env_get_u32("lcd_clk", OUT_CLK);
	screen->left_margin = env_get_u32("lcd_h_bp", H_BP);
	screen->right_margin = env_get_u32("lcd_h_fp", H_FP);
	screen->hsync_len = env_get_u32("lcd_h_pw", H_PW);
	screen->upper_margin = env_get_u32("lcd_v_bp", V_BP);
	screen->lower_margin = env_get_u32("lcd_v_fp", V_FP);
	screen->vsync_len = env_get_u32("lcd_v_pw", V_PW);

	/* Pin polarity */
	screen->pin_hsync = env_get_u32("lcd_h_pol", 0);
	screen->pin_vsync = env_get_u32("lcd_v_pol", 0);
	screen->pin_den = env_get_u32("lcd_den_pol", 0);
	screen->pin_dclk = env_get_u32("lcd_dclk_pol", DCLK_POL);

	/* Swap rule */
    screen->swap_rb = env_get_u32("lcd_swap_rb", SWAP_RB);
    screen->swap_rg = env_get_u32("lcd_swap_rg", 0);
    screen->swap_gb = env_get_u32("lcd_swap_gb", 0);
    screen->swap_delta = 0;
    screen->swap_dumy = 0;

    /* Operation function*/
    screen->init = NULL;
    screen->standby = NULL;
}
