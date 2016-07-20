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
#include <platform/gpio.h>
#include <mipi_dsi.h>
#include <string.h>

#include <target/display.h>
#include "include/panel.h"
#include "panel_display.h"

#include "include/panel_jdi_1080p_video_hammerhead.h"


#define DISPLAY_MAX_PANEL_DETECTION 3
extern void mdelay(unsigned msecs);

static uint32_t panel_id;

/*---------------------------------------------------------------------------*/
/* static panel selection variable                                           */
/*---------------------------------------------------------------------------*/
enum {
	JDI_1080P_VIDEO_PANEL = 0,
	JDI_1080P_VIDIO_PANEL_HAMMERHEAD = 1,
	UNKNOWN_PANEL
};

/*
 * The list of panels that are supported on this target.
 * Any panel in this list can be selected using fastboot oem command.
 */
static struct panel_list supp_panels[] = {
	{"jdi_1080p_video_hammerhead", JDI_1080P_VIDIO_PANEL_HAMMERHEAD}
};

	

int init_panel_data(struct panel_struct *panelstruct,
			struct msm_panel_info *pinfo,
			struct mdss_dsi_phy_ctrl *phy_db)
{
	int pan_type = PANEL_TYPE_DSI;
		panelstruct->paneldata    = &jdi_1080p_video_hammerhead_panel_data;
		panelstruct->panelres     = &jdi_1080p_video_hammerhead_panel_res;
		panelstruct->color        = &jdi_1080p_video_hammerhead_color;
		panelstruct->videopanel   = &jdi_1080p_video_hammerhead_video_panel;
		panelstruct->commandpanel = &jdi_1080p_video_hammerhead_command_panel;
		panelstruct->state        = &jdi_1080p_video_hammerhead_state;
		panelstruct->laneconfig   = &jdi_1080p_video_hammerhead_lane_config;
		panelstruct->paneltiminginfo
			= &jdi_1080p_video_hammerhead_timing_info;
		panelstruct->panelresetseq
					 = &jdi_1080p_video_hammerhead_panel_reset_seq;
		panelstruct->backlightinfo = &jdi_1080p_video_hammerhead_backlight;
		panelstruct->paneldata->panel_destination = (char *)"DISPLAY_1";
		pinfo->mipi.panel_on_cmds
			= jdi_1080p_video_hammerhead_on_command;
		pinfo->mipi.num_of_panel_on_cmds
			= JDI_1080P_VIDEO_ON_COMMAND;
		pinfo->mipi.panel_off_cmds
			= jdi_1080p_video_hammerhead_off_command;
		pinfo->mipi.num_of_panel_off_cmds
			= JDI_1080P_VIDEO_OFF_COMMAND;
		memcpy(phy_db->timing,
			jdi_1080p_video_hammerhead_timings, TIMING_SIZE);
		pinfo->mipi.signature = JDI_1080P_VIDEO_SIGNATURE;

		return pan_type;
}


int oem_panel_select(const char *panel_name, struct panel_struct *panelstruct,
			struct msm_panel_info *pinfo,
			struct mdss_dsi_phy_ctrl *phy_db)
{	
	panel_id = JDI_1080P_VIDIO_PANEL_HAMMERHEAD;
	return init_panel_data(panelstruct, pinfo, phy_db);
}
