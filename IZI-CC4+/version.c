/*
 * version.c
 *
 *  Created on: 4 aug. 2020
 *      Author: Milo
 */

#include "version.h"
#include "iziplus_module_def.h"

const firmware_t __attribute__ ((section (".firmwareinfosection"))) firmare_info =
{
	.version.v.major = 1, .version.v.minor = 2, .version.v.build = 24,
	.image.start = 0x4000, .image.marker = IMAGE_MARKER,
	.image.device_type = IZIPLUS_DEVTYPE_BASE, .image.device_type_range = 64,		// HP and MP
	.image.hw_version_start = 1, .image.hw_version_end = 3
};

