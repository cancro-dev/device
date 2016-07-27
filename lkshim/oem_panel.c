/* Copyright (C) 2016 sndnvaps<sndnvaps@gmail.com> All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <debug.h>
#include <err.h>
#include <smem.h>
#include <msm_panel.h>
#include <board.h>
//#include <platform/gpio.h>
#include <pm8x41.h>
#include <mipi_dsi.h>
#include <string.h>

#include <target/display.h>
#include "include/panel.h"
#include "panel_display.h"

#include "include/panel_sharp_fhd_cmd_95.h"


#define DISPLAY_MAX_PANEL_DETECTION 3
extern void mdelay(unsigned msecs);

static uint32_t panel_id;

/*---------------------------------------------------------------------------*/
/* static panel selection variable                                           */
/*---------------------------------------------------------------------------*/
enum {
	JDI_1080P_VIDEO_PANEL = 0,
	SHARP_FHD_CMD_95_PANEL = 1,
	UNKNOWN_PANEL
};

/*
 * The list of panels that are supported on this target.
 * Any panel in this list can be selected using fastboot oem command.
 */
static struct panel_list supp_panels[] = {
	{"sharp_fhd_cmd_95", SHARP_FHD_CMD_95_PANEL},

};

	

int init_panel_data(struct panel_struct *panelstruct,
			struct msm_panel_info *pinfo,
			struct mdss_dsi_phy_ctrl *phy_db)
{
		int pan_type = PANEL_TYPE_DSI;

	//	panelstruct->paneldata->panel_destination = (char *)"DISPLAY_1";
		panelstruct->paneldata    = &sharp_fhd_cmd_95_panel_data;
		panelstruct->panelres     = &sharp_fhd_cmd_95_panel_res;
		panelstruct->color        = &sharp_fhd_cmd_95_color;
		panelstruct->videopanel   = &sharp_fhd_cmd_95_video_panel;
		panelstruct->commandpanel = &sharp_fhd_cmd_95_command_panel;
		panelstruct->state        = &sharp_fhd_cmd_95_state;
		panelstruct->laneconfig   = &sharp_fhd_cmd_95_lane_config;
		panelstruct->paneltiminginfo = &sharp_fhd_cmd_95_timing_info;
		panelstruct->panelresetseq = &sharp_fhd_cmd_95_reset_seq;
		panelstruct->backlightinfo = &sharp_fhd_cmd_95_backlight;
		pinfo->mipi.panel_on_cmds = sharp_fhd_cmd_95_on_command;
		pinfo->mipi.num_of_panel_on_cmds = SHARP_FHD_CMD_95_ON_COMMAND;
		pinfo->mipi.panel_off_cmds = sharp_fhd_cmd_95_off_command;
		pinfo->mipi.num_of_panel_off_cmds = SHARP_FHD_CMD_95_OFF_COMMAND;
		memcpy(phy_db->timing, sharp_fhd_cmd_95_timings, TIMING_SIZE);

		return pan_type;
}
/*
static int is_sharp_fhd_panel(void)
{

	uint8_t status = 0;
	struct pm8x41_gpio gpio;

	gpio.direction = PM_GPIO_DIR_IN;
	gpio.function  = 0;
	gpio.pull      = PM_GPIO_PULL_UP_30;
	gpio.vin_sel   = 2;
	pm8x41_gpio_config(58, &gpio);
	
	mdelay(3);
	pm8x41_gpio_get(58, &status);
	
	return !status;
}

*/
int oem_panel_select(const char *panel_name, struct panel_struct *panelstruct,
			struct msm_panel_info *pinfo,
			struct mdss_dsi_phy_ctrl *phy_db)
{	
	/*
	uint32_t hw_id = board_hardware_id();
	int32_t panel_override_id;

	if (panel_name) {
		panel_override_id = panel_name_to_id(supp_panels,
				ARRAY_SIZE(supp_panels), panel_name);

		if (panel_override_id < 0) {
			dprintf(CRITICAL, "Not able to search the panel:%s\n",
					 panel_name);
		} else if (panel_override_id < UNKNOWN_PANEL) {
		
			panel_id = panel_override_id;

			dprintf(INFO, "OEM panel override:%s\n",
					panel_name);
			goto panel_init;
		}
	}

	switch (hw_id) {
		case HW_PLATFORM_MTP:
			if (is_sharp_fhd_panel())
				panel_id = SHARP_FHD_CMD_95_PANEL;//SHARP_FHD_VIDEO_PANEL;
			break;
		default:
			dprintf(CRITICAL, "Display not enabled for %d HW type\n"
					, hw_id);
			return PANEL_TYPE_UNKNOWN;
	}

panel_init:
	return init_panel_data(panelstruct, pinfo, phy_db);
	*/
	panel_id = SHARP_FHD_CMD_95_PANEL;//SHARP_FHD_VIDEO_PANEL;
	return init_panel_data(panelstruct, pinfo, phy_db);
}
