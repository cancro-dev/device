EMMC_BOOT := 1
#DISPLAY_2NDSTAGE := 1
DEBUG_ENABLE_UEFI_FBCON :=1
#ENABLE_FBCON_DISPLAY_MSG :=1

MEMSIZE := 0x00400000 # 4MB

MODULES += \
	$(EFIDROID_DEVICE_DIR)/lkshim
