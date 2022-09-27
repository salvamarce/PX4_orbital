/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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
 * @file gps_control.cpp
 * Control functions for ekf GNSS fusion
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::controlGpsFusion()
{
	// Check for new GPS data that has fallen behind the fusion time horizon
	if (_gps_data_ready) {

		const gpsSample &gps_sample{_gps_sample_delayed};
		const bool gps_checks_passing = isTimedOut(_last_gps_fail_us, (uint64_t)5e6);
		const bool gps_checks_failing = isTimedOut(_last_gps_pass_us, (uint64_t)5e6);

		// GNSS velocity
		const Vector3f velocity{gps_sample.vel};
		const float vel_var = sq(gps_sample.sacc);
		const Vector3f vel_obs_var{vel_var, vel_var, vel_var * sq(1.5f)};
		updateVelocityAidSrcStatus(gps_sample.time_us, velocity, vel_obs_var, fmaxf(_params.gps_vel_innov_gate, 1.f), _aid_src_gnss_vel);

		// GNSS position, vertical position GNSS measurement has opposite sign to earth z axis
		const float height_measurement = gps_sample.hgt - getEkfGlobalOriginAltitude();
		const Vector3f position{gps_sample.pos(0), gps_sample.pos(1), -(height_measurement - _gps_hgt_b_est.getBias())};
		const float pos_var_lower_limit = fmaxf(_params.gps_pos_noise, 0.01f);
		float pos_var = sq(fmaxf(gps_sample.hacc, pos_var_lower_limit));

		if (!isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {
			// if we are not using another source of aiding, then we are reliant on the GPS
			// observations to constrain attitude errors and must limit the observation noise value.
			float upper_limit = fmaxf(_params.pos_noaid_noise, pos_var_lower_limit);
			pos_var = fminf(pos_var, upper_limit);
		}

		const float height_measurement_var = getGpsHeightVariance(gps_sample);

		const Vector3f position_obs_var{pos_var, pos_var, height_measurement_var + _gps_hgt_b_est.getBiasVar()};
		updatePositionAidSrcStatus(gps_sample.time_us, position, position_obs_var, fmaxf(_params.gps_pos_innov_gate, 1.f), _aid_src_gnss_pos);

		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		_gps_hgt_b_est.setMaxStateNoise(height_measurement_var);
		_gps_hgt_b_est.setProcessNoiseSpectralDensity(_params.gps_hgt_bias_nsd);
		_gps_hgt_b_est.fuseBias(height_measurement - (-_state.pos(2)), height_measurement_var + P(9, 9));


		updateGpsYaw(gps_sample);

		controlGpsYawFusion(gps_sample, gps_checks_passing, gps_checks_failing);

		// update GSF yaw estimator velocity (basic sanity check on GNSS velocity data)
		if (gps_checks_passing && !gps_checks_failing) {
			_yawEstimator.setVelocity(velocity.xy(), gps_sample.sacc);
		}

		// Determine if we should use GPS aiding for velocity and horizontal position
		// To start using GPS we need angular alignment completed, the local NED origin set and GPS data that has not failed checks recently
		const bool mandatory_conditions_passing = ((_params.gnss_ctrl & GnssCtrl::HPOS) || (_params.gnss_ctrl & GnssCtrl::VEL))
				&& _control_status.flags.tilt_align
				&& _control_status.flags.yaw_align
				&& _NED_origin_initialised;

		const bool continuing_conditions_passing = mandatory_conditions_passing && !gps_checks_failing;
		const bool starting_conditions_passing = continuing_conditions_passing && gps_checks_passing;

		if (_control_status.flags.gps) {
			if (mandatory_conditions_passing) {
				if (continuing_conditions_passing
				    || !isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

					_aid_src_gnss_vel.fusion_enabled[0] = (_params.gnss_ctrl & GnssCtrl::VEL) && _control_status.flags.gps;
					_aid_src_gnss_vel.fusion_enabled[1] = (_params.gnss_ctrl & GnssCtrl::VEL) && _control_status.flags.gps;
					_aid_src_gnss_vel.fusion_enabled[2] = (_params.gnss_ctrl & GnssCtrl::VEL) && _control_status.flags.gps;
					fuseVelocity(_aid_src_gnss_vel);

					_aid_src_gnss_pos.fusion_enabled[0] = (_params.gnss_ctrl & GnssCtrl::HPOS) && _control_status.flags.gps;
					_aid_src_gnss_pos.fusion_enabled[1] = (_params.gnss_ctrl & GnssCtrl::HPOS) && _control_status.flags.gps;
					_aid_src_gnss_pos.fusion_enabled[2] = (_params.gnss_ctrl & GnssCtrl::VPOS) && _control_status.flags.gps_hgt;
					fusePosition(_aid_src_gnss_pos);

					if (shouldResetGpsFusion()) {
						const bool was_gps_signal_lost = isTimedOut(_time_prev_gps_us, 1'000'000);

						/* A reset is not performed when getting GPS back after a significant period of no data
						 * because the timeout could have been caused by bad GPS.
						 * The total number of resets allowed per boot cycle is limited.
						 */
						if (isYawFailure()
						    && _control_status.flags.in_air
						    && !was_gps_signal_lost
						    && _ekfgsf_yaw_reset_count < _params.EKFGSF_reset_count_limit
						    && isTimedOut(_ekfgsf_yaw_reset_time, 5'000'000)) {
							// The minimum time interval between resets to the EKF-GSF estimate is limited to allow the EKF-GSF time
							// to improve its estimate if the previous reset was not successful.
							if (resetYawToEKFGSF()) {
								ECL_WARN("GPS emergency yaw reset");
							}

						} else {
							// use GPS velocity data to check and correct yaw angle if a FW vehicle
							if (_control_status.flags.fixed_wing && _control_status.flags.in_air) {
								// if flying a fixed wing aircraft, do a complete reset that includes yaw
								_mag_yaw_reset_req = true;
							}

							_warning_events.flags.gps_fusion_timout = true;
							ECL_WARN("GPS fusion timeout - resetting");
						}

						resetVelocityToGps(gps_sample);
						resetHorizontalPositionToGps(gps_sample);
					}

				} else {
					stopGpsFusion();
					_warning_events.flags.gps_quality_poor = true;
					ECL_WARN("GPS quality poor - stopping use");
				}

			} else { // mandatory conditions are not passing
				stopGpsFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Do not use external vision for yaw if using GPS because yaw needs to be
				// defined relative to an NED reference frame
				if (_control_status.flags.ev_yaw
				    || _mag_inhibit_yaw_reset_req
				    || _mag_yaw_reset_req) {

					_mag_yaw_reset_req = true;

					// Stop the vision for yaw fusion and do not allow it to start again
					stopEvYawFusion();
					_inhibit_ev_yaw_use = true;

				} else {
					startGpsFusion(gps_sample);
				}

			} else if ((_params.gnss_ctrl & GnssCtrl::HPOS) && gps_checks_passing && _NED_origin_initialised && !_control_status.flags.yaw_align) {

				// align using the yaw estimator (if available)
				if (resetYawToEKFGSF()) {

					stopEvYawFusion();

					_information_events.flags.yaw_aligned_to_imu_gps = true;
					ECL_INFO("Yaw aligned using IMU and GPS");

					startGpsFusion(gps_sample);
				}
			}
		}

	} else if (_control_status.flags.gps && !isNewestSampleRecent(_time_last_gps_buffer_push, (uint64_t)10e6)) {
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped = true;
		ECL_WARN("GPS data stopped");

	}  else if (_control_status.flags.gps && !isNewestSampleRecent(_time_last_gps_buffer_push, (uint64_t)1e6)
		    && isOtherSourceOfHorizontalAidingThan(_control_status.flags.gps)) {

		// Handle the case where we are fusing another position source along GPS,
		// stop waiting for GPS after 1 s of lost signal
		stopGpsFusion();
		_warning_events.flags.gps_data_stopped_using_alternate = true;
		ECL_WARN("GPS data stopped, using only EV, OF or air data");
	}
}

bool Ekf::shouldResetGpsFusion() const
{
	/* We are relying on aiding to constrain drift so after a specified time
	 * with no aiding we need to do something
	 */
	const bool is_reset_required = hasHorizontalAidingTimedOut()
				       || isTimedOut(_time_last_hor_pos_fuse, 2 * _params.reset_timeout_max);

	/* Logic controlling the reset of navigation filter yaw to the EKF-GSF estimate to recover from loss of
	 * navigation casued by a bad yaw estimate.

	 * A rapid reset to the EKF-GSF estimate is performed after a recent takeoff if horizontal velocity
	 * innovation checks fail. This enables recovery from a bad yaw estimate. After 30 seconds from takeoff,
	 * different test criteria are used that take longer to trigger and reduce false positives. A reset is
	 * not performed if the fault condition was present before flight to prevent triggering due to GPS glitches
	 * or other sensor errors.
	 */
	const bool is_recent_takeoff_nav_failure = _control_status.flags.in_air
			&& isRecent(_time_last_on_ground_us, 30000000)
			&& isTimedOut(_time_last_hor_vel_fuse, _params.EKFGSF_reset_delay)
			&& (_time_last_hor_vel_fuse > _time_last_on_ground_us);

	const bool is_inflight_nav_failure = _control_status.flags.in_air
					     && isTimedOut(_time_last_hor_vel_fuse, _params.reset_timeout_max)
					     && isTimedOut(_time_last_hor_pos_fuse, _params.reset_timeout_max)
					     && (_time_last_hor_vel_fuse > _time_last_on_ground_us)
					     && (_time_last_hor_pos_fuse > _time_last_on_ground_us);

	return (is_reset_required || is_recent_takeoff_nav_failure || is_inflight_nav_failure);
}

bool Ekf::isYawFailure() const
{
	if (!isYawEmergencyEstimateAvailable()) {
		return false;
	}

	const float euler_yaw = getEulerYaw(_R_to_earth);
	const float yaw_error = wrap_pi(euler_yaw - _yawEstimator.getYaw());

	return fabsf(yaw_error) > math::radians(25.f);
}
