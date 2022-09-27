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
 * @file ev_height_control.cpp
 * Control functions for ekf external vision height fusion
 */

#include "ekf.h"

void Ekf::controlEvHeightFusion(const extVisionSample &ev_sample)
{
	_ev_hgt_b_est.predict(_dt_ekf_avg);

	if (!(_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VPOS))) {
		stopEvHgtFusion();
		return;
	}

	const bool ev_intermittent = !isNewestSampleRecent(_time_last_ext_vision_buffer_push, 2 * EV_MAX_INTERVAL);

	if (_ev_data_ready) {

		// determine if we should use the horizontal position observations
		const bool continuing_conditions_passing = !ev_intermittent && PX4_ISFINITE(ev_sample.pos(2));
		const bool starting_conditions_passing = continuing_conditions_passing;

		if (_control_status.flags.ev_hgt) {
			if (continuing_conditions_passing) {
				/* fuseEvHgt(); */ // Done in fuseEvPos

				const bool ev_reset = (ev_sample.reset_counter != _ev_sample_delayed_prev.reset_counter)
							  && (fabsf(ev_sample.pos(2) - _ev_sample_delayed_prev.pos(2)) > 0.01f);

				const bool is_fusion_failing = isTimedOut(_aid_src_ev_pos.time_last_fuse[2], _params.hgt_fusion_timeout_max);

				if (isHeightResetRequired()) {
					// All height sources are failing
					resetHeightToEv(ev_sample);

					// If the sample has a valid vertical velocity estimate, use it
					if (PX4_ISFINITE(ev_sample.vel(2))) {
						resetVerticalVelocityTo(ev_sample.vel(2));

						// the state variance is the same as the observation
						P.uncorrelateCovarianceSetVariance<1>(6, ev_sample.velVar(2));
					}

				} else if (ev_reset && isOnlyActiveSourceOfVerticalPositionAiding(_control_status.flags.ev_hgt)) {
					resetHeightToEv(ev_sample);

				} else if (is_fusion_failing) {
					// Some other height source is still working
					stopEvHgtFusion();
				}

			} else {
				stopEvHgtFusion();
			}

		} else {
			if (starting_conditions_passing) {
				startEvHgtFusion(ev_sample);
			}
		}

	} else if (_control_status.flags.ev_hgt && ev_intermittent) {
		stopEvHgtFusion();
	}
}

void Ekf::startEvHgtFusion(const extVisionSample &ev_sample)
{
	if (!_control_status.flags.ev_hgt) {

		if (_params.height_sensor_ref == HeightSensor::EV) {
			_rng_hgt_b_est.reset();
			_height_sensor_ref = HeightSensor::EV;
			resetHeightToEv(ev_sample);

		} else {
			_ev_hgt_b_est.setBias(-_state.pos(2) + ev_sample.pos(2));

			// Reset the timeout value here because the fusion isn't done at the same place and would immediately trigger a timeout
			_aid_src_ev_pos.time_last_fuse[2] = _imu_sample_delayed.time_us;
		}

		_control_status.flags.ev_hgt = true;
		_ev_hgt_b_est.setFusionActive();
		ECL_INFO("starting EV height fusion");
	}
}

void Ekf::resetHeightToEv(const extVisionSample &ev_sample)
{
	ECL_INFO("reset height to EV");
	_information_events.flags.reset_hgt_to_ev = true;

	resetVerticalPositionTo(ev_sample.pos(2) - _ev_hgt_b_est.getBias());

	// the state variance is the same as the observation
	P.uncorrelateCovarianceSetVariance<1>(9, fmaxf(ev_sample.posVar(2), sq(0.01f)));

	_baro_b_est.setBias(_baro_b_est.getBias() + _state_reset_status.posD_change);
	_gps_hgt_b_est.setBias(_gps_hgt_b_est.getBias() + _state_reset_status.posD_change);
	_rng_hgt_b_est.setBias(_rng_hgt_b_est.getBias() + _state_reset_status.posD_change);

	_aid_src_ev_pos.time_last_fuse[2] = _imu_sample_delayed.time_us;
}

void Ekf::stopEvHgtFusion()
{
	if (_control_status.flags.ev_hgt) {

		if (_height_sensor_ref == HeightSensor::EV) {
			_height_sensor_ref = HeightSensor::UNKNOWN;
		}

		_control_status.flags.ev_hgt = false;
		_ev_hgt_b_est.setFusionInactive();
		ECL_INFO("stopping EV height fusion");
	}
}
