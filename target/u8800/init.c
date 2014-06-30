/*
 * Copyright (c) 2009, Google Inc.
 * All rights reserved.
 * Copyright (c) 2009-2011, The Linux Foundation. All rights reserved.
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
 *  * Neither the name of Google, Inc. nor the names of its contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
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
#include <dev/keys.h>
#include <dev/gpio.h>
#include <dev/ssbi.h>
#include <lib/ptable.h>
#include <smem.h>
#include <reg.h>
#include <mmc.h>
#include <platform/iomap.h>
#include <platform/machtype.h>
#if TARGET_USES_RSPIN_LOCK
#include <platform/remote_spinlock.h>
#endif
#include <platform.h>
#include <crypto_hash.h>

static unsigned mmc_sdc_base[] =
	{ MSM_SDC1_BASE, MSM_SDC2_BASE, MSM_SDC3_BASE, MSM_SDC4_BASE };

void target_init(void)
{
	unsigned base_addr;
	unsigned char slot;

	dprintf(INFO, "target_init()\n");

	display_init();
	dprintf(INFO, "Diplay initialized\n");

#if TARGET_USES_RSPIN_LOCK
	if(remote_spinlock_init(&rlock))
		dprintf(SPEW,"Failed to Initialize remote spin locks\n");
#endif

#if (!ENABLE_NANDWRITE)
	keys_init();
	keypad_init();
#endif

	/* Display splash screen if enabled */
#if DISPLAY_SPLASH_SCREEN
	display_image_on_screen();
#endif

	dprintf(INFO, "Waiting for modem-up...\n");
	/* Must wait for modem-up before we can intialize MMC.
	 */
	while (readl(MSM_SHARED_BASE + 0x14) != 1) ;

	/* Trying Slot 2 first */
	slot = 2;
	base_addr = mmc_sdc_base[slot - 1];
	if (mmc_boot_main(slot, base_addr)) {
		/* Trying Slot 4 next */
		slot = 4;
		base_addr = mmc_sdc_base[slot - 1];
		if (mmc_boot_main(slot, base_addr)) {
			dprintf(CRITICAL, "mmc init failed!");
			ASSERT(0);
		}
	}
}

unsigned board_machtype(void)
{
	return LINUX_MACHTYPE_7x30_U8800;
}

void reboot_device(unsigned reboot_reason)
{
	reboot(reboot_reason);
}

unsigned check_reboot_mode(void)
{
	unsigned mode[2] = { 0, 0 };
	unsigned int mode_len = sizeof(mode);
	unsigned smem_status;

	smem_status = smem_read_alloc_entry(SMEM_APPS_BOOT_MODE,
					    &mode, mode_len);
	if (smem_status) {
		dprintf(CRITICAL,
			"ERROR: unable to read shared memory for reboot mode\n");
		return 0;
	}
	return mode[0];
}

static unsigned target_check_power_on_reason(void)
{
	unsigned power_on_status = 0;
	unsigned int status_len = sizeof(power_on_status);
	unsigned smem_status;

	smem_status = smem_read_alloc_entry(SMEM_POWER_ON_STATUS_INFO,
					    &power_on_status, status_len);

	if (smem_status) {
		dprintf(CRITICAL,
			"ERROR: unable to read shared memory for power on reason\n");
	}

	return power_on_status;
}

unsigned target_pause_for_battery_charge(void)
{
	if (target_check_power_on_reason() &
		(PWR_ON_EVENT_USB_CHG | PWR_ON_EVENT_WALL_CHG))
		return 1;
	return 0;
}

#if _EMMC_BOOT
void target_serialno(unsigned char *buf)
{
	unsigned int serialno;
	serialno = mmc_get_psn();
	snprintf(buf, 13, "%x", serialno);
}

int emmc_recovery_init(void)
{
	int rc;
	rc = _emmc_recovery_init();
	return rc;
}
#endif

/* Setting this variable to different values defines the
 * behavior of CE engine:
 * platform_ce_type = CRYPTO_ENGINE_TYPE_NONE : No CE engine
 * platform_ce_type = CRYPTO_ENGINE_TYPE_SW : Software CE engine
 * platform_ce_type = CRYPTO_ENGINE_TYPE_HW : Hardware CE engine
 * Behavior is determined in the target code.
 */
crypto_engine_type board_ce_type(void)
{
	return CRYPTO_ENGINE_TYPE_HW;
}
