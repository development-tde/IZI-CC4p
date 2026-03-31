/*
 * version.c
 *
 *  Created on: 4 aug. 2020
 *      Author: Milo
 */

#include "version.h"

const version_u __attribute__ ((section (".firmwareinfosection"))) version = { .v.major = 1, .v.minor = 0, .v.build = 4  };
	