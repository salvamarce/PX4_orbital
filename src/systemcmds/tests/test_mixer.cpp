/****************************************************************************
 *
 *  Copyright (C) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file test_mixer.cpp
 * Mixer load test
 */

#include <dirent.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <lib/mixer/mixer_load.h>
#include <math.h>
#include <px4_platform_common/px4_config.h>
#include <px4iofirmware/protocol.h>
#include <stdlib.h>
#include <string.h>
#include <uORB/topics/actuator_controls.h>
#include <unistd.h>
#include <unit_test.h>

#include <lib/mixer/MixerGroup.hpp>

#include "tests_main.h"

static int mixer_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &control);

static const unsigned output_max = 8;
static float actuator_controls[output_max];
static bool should_prearm = false;

#ifdef __PX4_DARWIN
#define MIXER_DIFFERENCE_THRESHOLD 30
#else
#define MIXER_DIFFERENCE_THRESHOLD 2
#endif

#ifndef PATH_MAX
#ifdef __PX4_NUTTX
#define PATH_MAX 512
#else
#define PATH_MAX 4096
#endif
#endif

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
#define MIXER_PATH(_file) "etc/mixers/" #_file
#define MIXER_ONBOARD_PATH "etc/mixers"
#else
#define MIXER_ONBOARD_PATH "/etc/mixers"
#define MIXER_PATH(_file) MIXER_ONBOARD_PATH "/" #_file
#endif

#define MIXER_VERBOSE

class MixerTest : public UnitTest {
public:
	virtual bool run_tests();
	MixerTest() = default;

private:
	bool loadIOPass();
	bool loadVTOL1Test();
	bool loadVTOL2Test();
	bool loadQuadTest();
	bool loadComplexTest();
	bool loadAllTest();
	bool load_mixer(const char *filename, unsigned expected_count, bool verbose = false);
	bool load_mixer(const char *filename, const char *buf, unsigned loaded, unsigned expected_count,
			const unsigned chunk_size, bool verbose);

	MixerGroup mixer_group;
};

bool MixerTest::run_tests() {
	ut_run_test(loadIOPass);
	ut_run_test(loadQuadTest);
	ut_run_test(loadVTOL1Test);
	ut_run_test(loadVTOL2Test);
	ut_run_test(loadComplexTest);
	ut_run_test(loadAllTest);

	return (_tests_failed == 0);
}

ut_declare_test_c(test_mixer, MixerTest)

	bool MixerTest::loadIOPass() {
	return load_mixer(MIXER_PATH(IO_pass.mix), 8);
}

bool MixerTest::loadQuadTest() { return load_mixer(MIXER_PATH(quad_test.mix), 5); }

bool MixerTest::loadVTOL1Test() { return load_mixer(MIXER_PATH(vtol1_test.mix), 4); }

bool MixerTest::loadVTOL2Test() { return load_mixer(MIXER_PATH(vtol2_test.mix), 6); }

bool MixerTest::loadComplexTest() { return load_mixer(MIXER_PATH(complex_test.mix), 8); }

bool MixerTest::loadAllTest() {
	PX4_INFO("Testing all mixers in %s", MIXER_ONBOARD_PATH);

	DIR *dp = opendir(MIXER_ONBOARD_PATH);

	if (dp == nullptr) {
		PX4_ERR("File open failed");
		return false;
	}

	struct dirent *result = nullptr;

	for (;;) {
		errno = 0;
		result = readdir(dp);

		// read the directory entry
		if (result == nullptr) {
			if (errno) {
				PX4_ERR("readdir failed");
				closedir(dp);

				return false;
			}

			// We are just at the last directory entry
			break;
		}

		// Determine the directory entry type
		switch (result->d_type) {
#ifdef __PX4_NUTTX

			case DTYPE_FILE:
#else
			case DT_REG:
#endif
				if (strncmp(result->d_name, ".", 1) != 0) {
					char buf[PATH_MAX];

					if (snprintf(buf, PATH_MAX, "%s/%s", MIXER_ONBOARD_PATH, result->d_name) >=
					    PATH_MAX) {
						PX4_ERR("mixer path too long %s", result->d_name);
						closedir(dp);
						return false;
					}

					bool ret = load_mixer(buf, 0);

					if (!ret) {
						PX4_ERR("Error testing mixer %s", buf);
						closedir(dp);
						return false;
					}
				}

				break;

			default:
				break;
		}
	}

	closedir(dp);

	return true;
}

bool MixerTest::load_mixer(const char *filename, unsigned expected_count, bool verbose) {
	char buf[2048];

	load_mixer_file(filename, &buf[0], sizeof(buf));
	unsigned loaded = strlen(buf);

	if (verbose) {
		PX4_INFO("loaded: \n\"%s\"\n (file: %s, %d chars)", &buf[0], filename, loaded);
	}

	// Test a number of chunk sizes
	for (unsigned chunk_size = 6; chunk_size < PX4IO_MAX_TRANSFER_LEN + 1; chunk_size++) {
		bool ret = load_mixer(filename, buf, loaded, expected_count, chunk_size, verbose);

		if (!ret) {
			PX4_ERR("Mixer load failed with chunk size %u", chunk_size);
			return ret;
		}
	}

	return true;
}

bool MixerTest::load_mixer(const char *filename, const char *buf, unsigned loaded, unsigned expected_count,
			   const unsigned chunk_size, bool verbose) {
	/* load the mixer in chunks, like
	 * in the case of a remote load,
	 * e.g. on PX4IO.
	 */

	/* load at once test */
	unsigned xx = loaded;
	mixer_group.reset();
	mixer_group.load_from_buf(mixer_callback, 0, &buf[0], xx);

	if (expected_count > 0) {
		ut_compare("check number of mixers loaded", mixer_group.count(), expected_count);
	}

	unsigned empty_load = 2;
	char empty_buf[2];
	empty_buf[0] = ' ';
	empty_buf[1] = '\0';
	mixer_group.reset();
	mixer_group.load_from_buf(mixer_callback, 0, &empty_buf[0], empty_load);

	if (verbose) {
		PX4_INFO("empty buffer load: loaded %u mixers, used: %u", mixer_group.count(), empty_load);
	}

	ut_compare("empty buffer load", empty_load, 0);

	/* reset, load in chunks */
	mixer_group.reset();
	char mixer_text[330]; /* large enough for one mixer */

	unsigned mixer_text_length = 0;
	unsigned transmitted = 0;
	unsigned resid = 0;

	while (transmitted < loaded) {
		unsigned text_length = (loaded - transmitted > chunk_size) ? chunk_size : loaded - transmitted;

		/* check for overflow - this would be really fatal */
		if ((mixer_text_length + text_length + 1) > sizeof(mixer_text)) {
			PX4_ERR("Mixer text length overflow for file: %s. Is PX4IO_MAX_MIXER_LENGTH too small? (curr "
				"len: %d)",
				filename, 330);
			return false;
		}

		/* append mixer text and nul-terminate */
		memcpy(&mixer_text[mixer_text_length], &buf[transmitted], text_length);
		mixer_text_length += text_length;
		mixer_text[mixer_text_length] = '\0';
		// fprintf(stderr, "buflen %u, text:\n\"%s\"\n", mixer_text_length, &mixer_text[0]);

		/* process the text buffer, adding new mixers as their descriptions can be parsed */
		resid = mixer_text_length;
		mixer_group.load_from_buf(mixer_callback, 0, &mixer_text[0], resid);

		/* if anything was parsed */
		if (resid != mixer_text_length) {
			// PX4_INFO("loaded %d mixers, used %u\n", mixer_group.count(), mixer_text_length - resid);

			/* copy any leftover text to the base of the buffer for re-use */
			if (resid > 0) {
				memmove(&mixer_text[0], &mixer_text[mixer_text_length - resid], resid);
				/* enforce null termination */
				mixer_text[resid] = '\0';
			}

			mixer_text_length = resid;
		}

		transmitted += text_length;

		if (verbose) {
			PX4_INFO("transmitted: %d, loaded: %d", transmitted, loaded);
		}
	}

	if (verbose) {
		PX4_INFO("chunked load: loaded %u mixers", mixer_group.count());
	}

	if (expected_count > 0 && mixer_group.count() != expected_count) {
		PX4_ERR("Load of mixer failed, last chunk: %s, transmitted: %u, text length: %u, resid: %u", mixer_text,
			transmitted, mixer_text_length, resid);
		ut_compare("check number of mixers loaded (chunk)", mixer_group.count(), expected_count);
	}

	return true;
}

static int mixer_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &control) {
	control = 0.0f;

	if (control_group != 0) {
		return -1;
	}

	if (control_index >= (sizeof(actuator_controls) / sizeof(actuator_controls[0]))) {
		return -1;
	}

	control = actuator_controls[control_index];

	if (should_prearm && control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
	    control_index == actuator_controls_s::INDEX_THROTTLE) {
		control = NAN;
	}

	return 0;
}
