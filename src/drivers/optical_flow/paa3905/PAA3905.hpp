/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file PAA3905.hpp
 *
 * Driver for the PAA3905E1-Q: Optical Motion Tracking Chip
 */

#pragma once

#include <conversion/rotation.h>
#include <drivers/device/spi.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/sensor_optical_flow.h>

#include <uORB/PublicationMulti.hpp>

#include "PixArt_PAA3905_Registers.hpp"

using namespace time_literals;
using namespace PixArt_PAA3905;

#define DIR_WRITE(a) ((a) | Bit7)
#define DIR_READ(a) ((a)&0x7F)

class PAA3905 : public device::SPI, public I2CSPIDriver<PAA3905> {
public:
	PAA3905(const I2CSPIDriverConfig &config);
	virtual ~PAA3905();

	static void print_usage();

	int init() override;

	void print_status() override;

	void RunImpl();

private:
	void exit_and_cleanup() override;

	int probe() override;

	void Reset();

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	uint8_t RegisterRead(uint8_t reg);
	void RegisterWrite(uint8_t reg, uint8_t data);

	void Configure();

	void ConfigureAutomaticModeSwitching();

	void ConfigureModeBright();
	void ConfigureModeLowLight();
	void ConfigureModeSuperLowLight();

	void ConfigureStandardDetectionSetting();
	void ConfigureEnhancedDetectionMode();

	void EnableLed();

	bool UpdateMode(const uint8_t observation);

	uORB::PublicationMulti<sensor_optical_flow_s> _sensor_optical_flow_pub{ORB_ID(sensor_optical_flow)};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};
	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME ": reset")};
	perf_counter_t _false_motion_perf{perf_alloc(PC_COUNT, MODULE_NAME ": false motion report")};
	perf_counter_t _mode_change_bright_perf{perf_alloc(PC_COUNT, MODULE_NAME ": mode change bright (0)")};
	perf_counter_t _mode_change_low_light_perf{perf_alloc(PC_COUNT, MODULE_NAME ": mode change low light (1)")};
	perf_counter_t _mode_change_super_low_light_perf{
		perf_alloc(PC_COUNT, MODULE_NAME ": mode change super low light (2)")};
	perf_counter_t _no_motion_interrupt_perf{nullptr};

	const spi_drdy_gpio_t _drdy_gpio;

	matrix::Dcmf _rotation;

	int _discard_reading{3};

	Mode _mode{Mode::LowLight};

	uint32_t _scheduled_interval_us{SAMPLE_INTERVAL_MODE_0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	hrt_abstime _last_write_time{0};
	hrt_abstime _last_read_time{0};

	// force reset if there hasn't been valid data for an extended period (sensor could be in a bad state)
	static constexpr hrt_abstime RESET_TIMEOUT_US = 3_s;

	hrt_abstime _last_good_data{0};
	hrt_abstime _last_reset{0};
};
