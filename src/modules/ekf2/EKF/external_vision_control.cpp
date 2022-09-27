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
 * @file external_vision_control.cpp
 * Control functions for ekf external vision control
 */

#include "ekf.h"

void Ekf::controlExternalVisionFusion()
{
	_ev_pos_b_est.predict(_dt_ekf_avg);

	// Check for new external vision data
	if (_ev_data_ready) {

		const extVisionSample &ev_sample{_ev_sample_delayed};

		controlEvYawFusion(ev_sample);
		controlEvPosFusion(ev_sample);
		controlEvVelFusion(ev_sample);

		// record observation and estimate for use next time
		_ev_sample_delayed_prev = ev_sample;

	} else if ((_control_status.flags.ev_pos || _control_status.flags.ev_vel ||  _control_status.flags.ev_yaw)
		   && !isNewestSampleRecent(_time_last_ext_vision_buffer_push, (uint64_t)_params.reset_timeout_max)) {

		// Turn off EV fusion mode if no data has been received
		stopEvPosFusion();
		stopEvVelFusion();
		stopEvYawFusion();

		_warning_events.flags.vision_data_stopped = true;
		ECL_WARN("vision data stopped");
	}
}

void Ekf::controlEvPosFusion(const extVisionSample &ev_sample)
{
	if (_ev_data_ready) {

		// correct position and height for offset relative to IMU
		const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
		const Vector3f pos_offset_earth = _R_to_earth * pos_offset_body;

		Vector3f pos{NAN, NAN, NAN};
		Matrix3f pos_cov{};
		_ev_pos_b_est.setFusionInactive();

		switch (ev_sample.pos_frame) {
		case PositionFrame::LOCAL_FRAME_NED:
			if (_control_status.flags.yaw_align) {
				pos = ev_sample.pos - pos_offset_earth;
				pos_cov = matrix::diag(ev_sample.posVar);
			}

			if (_control_status.flags.gps) {
				_ev_pos_b_est.setFusionActive();

			} else {
				_ev_pos_b_est.setFusionInactive();
			}

			break;

		case PositionFrame::LOCAL_FRAME_FRD:
			if (_control_status.flags.ev_yaw) {
				// using EV frame
				pos = ev_sample.pos - pos_offset_earth;
				pos_cov = matrix::diag(ev_sample.posVar);

				_ev_pos_b_est.setFusionInactive();
				_ev_pos_b_est.reset();

			} else {
				// rotate EV to the EKF reference frame
				const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());
				const Dcmf R_ev_to_ekf = Dcmf(q_error);

				pos = R_ev_to_ekf * ev_sample.pos - pos_offset_earth;
				pos_cov = R_ev_to_ekf * matrix::diag(ev_sample.posVar) * R_ev_to_ekf.transpose();

				if (_control_status.flags.gps) {
					_ev_pos_b_est.setFusionActive();

				} else {
					_ev_pos_b_est.setFusionInactive();
				}
			}

			break;
		}

		const Vector3f measurement_var{
			fmaxf(pos_cov(0, 0), sq(0.01f)),
			fmaxf(pos_cov(1, 1), sq(0.01f)),
			fmaxf(pos_cov(2, 2), sq(0.01f))
		};

		const Vector3f obs_var{
			measurement_var(0) + _ev_pos_b_est.getBiasVar(0),
			measurement_var(1) + _ev_pos_b_est.getBiasVar(1),
			measurement_var(2) + _ev_hgt_b_est.getBiasVar()
		};

		const Vector3f ev_pos{
			pos(0) - _ev_pos_b_est.getBias(0),
			pos(1) - _ev_pos_b_est.getBias(1),
			pos(2) - _ev_hgt_b_est.getBias()
		};

		updatePositionAidSrcStatus(ev_sample.time_us, ev_pos, obs_var, fmaxf(_params.ev_pos_innov_gate, 1.f), _aid_src_ev_pos);


		// update the bias estimator before updating the main filter but after
		// using its current state to compute the vertical position innovation
		_ev_pos_b_est.setMaxStateNoise(Vector2f(measurement_var));
		_ev_pos_b_est.setProcessNoiseSpectralDensity(_params.ev_hgt_bias_nsd); // TODO
		_ev_pos_b_est.fuseBias(Vector2f(pos.xy()) - Vector2f(_state.pos.xy()), Vector2f(measurement_var.xy()) + Vector2f(P(7, 7), P(8, 8)));

		_ev_hgt_b_est.setMaxStateNoise(measurement_var(2));
		_ev_hgt_b_est.setProcessNoiseSpectralDensity(_params.ev_hgt_bias_nsd);
		_ev_hgt_b_est.fuseBias(pos(2) - _state.pos(2), measurement_var(2) + P(9, 9));



		// determine if we should use the horizontal position observations
		const bool quality_sufficient = ((_params.ev_quality_minimum <= 0) || ((_params.ev_quality_minimum > 0)
						 && (ev_sample.quality >= _params.ev_quality_minimum)));

		const bool continuing_conditions_passing = isNewestSampleRecent(_time_last_ext_vision_buffer_push, 2 * EV_MAX_INTERVAL)
				&& _control_status.flags.tilt_align
				&& ((_params.ev_ctrl & static_cast<int32_t>(EvCtrl::HPOS)) || (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VPOS)));

		const bool starting_conditions_passing = continuing_conditions_passing
				&& quality_sufficient;

		if (_control_status.flags.ev_pos || _control_status.flags.ev_hgt) {
			if (continuing_conditions_passing) {

				const bool ev_pos_reset = (ev_sample.reset_counter != _ev_sample_delayed_prev.reset_counter)
					  && (ev_sample.pos - _ev_sample_delayed_prev.pos).longerThan(0.01f);

				if (ev_pos_reset && !_control_status.flags.gps) {
					// reset count changed in EV sample, reset vision position unless GPS is active
					_information_events.flags.reset_pos_to_vision = true;
					ECL_INFO("reset horizontal position to EV");
					resetHorizontalPositionTo(Vector2f(ev_pos));
					P.uncorrelateCovarianceSetVariance<2>(7, obs_var.xy());

				} else {
					_aid_src_ev_pos.fusion_enabled[0] = quality_sufficient && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::HPOS)) && _control_status.flags.ev_pos;
					_aid_src_ev_pos.fusion_enabled[1] = quality_sufficient && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::HPOS)) && _control_status.flags.ev_pos;
					_aid_src_ev_pos.fusion_enabled[2] = quality_sufficient && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VPOS)) && _control_status.flags.ev_hgt;
					fusePosition(_aid_src_ev_pos);

					bool is_fusion_failing = _control_status.flags.ev_pos
							&& (isTimedOut(_aid_src_ev_pos.time_last_fuse[0], _params.reset_timeout_max)
								 || isTimedOut(_aid_src_ev_pos.time_last_fuse[1], _params.reset_timeout_max));

					if (is_fusion_failing) {
						if (_nb_ev_pos_reset_available > 0) {
							// Data seems good, attempt a reset
							_information_events.flags.reset_pos_to_vision = true;
							ECL_INFO("reset horiztonal position to EV");
							resetHorizontalPositionTo(Vector2f(pos));
							P.uncorrelateCovarianceSetVariance<2>(7, obs_var.xy());

							if (_control_status.flags.in_air) {
								_nb_ev_pos_reset_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							//_control_status.flags.ev_pos_fault = true;
							stopEvPosFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvPosFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvPosFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV position fusion
				if (!_control_status.flags.ev_pos) {
					_control_status.flags.ev_pos = true;

					if (!_control_status.flags.gps) {
						_information_events.flags.reset_pos_to_vision = true;
						ECL_INFO("reset horiztonal position to EV");
						resetHorizontalPositionTo(Vector2f(ev_pos));
						P.uncorrelateCovarianceSetVariance<2>(7, obs_var.xy());
					}

					_information_events.flags.starting_vision_pos_fusion = true;
					ECL_INFO("starting vision pos fusion");
				}

				if (_control_status.flags.ev_pos) {
					_nb_ev_pos_reset_available = 3;
				}
			}
		}

	} else if (_control_status.flags.ev_pos && !isNewestSampleRecent(_time_last_ext_vision_buffer_push, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvPosFusion();
	}
}

void Ekf::controlEvVelFusion(const extVisionSample &ev_sample)
{
	if (_ev_data_ready) {

		bool continuing_conditions_passing = isNewestSampleRecent(_time_last_ext_vision_buffer_push, 2 * EV_MAX_INTERVAL)
						     && _control_status.flags.tilt_align
						     && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL));

		// correct velocity for offset relative to IMU
		const Vector3f pos_offset_body = _params.ev_pos_body - _params.imu_pos_body;
		const Vector3f vel_offset_body = _ang_rate_delayed_raw % pos_offset_body;
		const Vector3f vel_offset_earth = _R_to_earth * vel_offset_body;

		// rotate measurement into correct earth frame if required
		Vector3f vel{NAN, NAN, NAN};
		Matrix3f vel_cov{};

		switch (ev_sample.vel_frame) {
		case VelocityFrame::LOCAL_FRAME_NED:
			if (_control_status.flags.yaw_align) {
				vel = ev_sample.vel - vel_offset_earth;
				vel_cov = matrix::diag(ev_sample.velVar);

			} else {
				continuing_conditions_passing = false;
			}

			break;

		case VelocityFrame::LOCAL_FRAME_FRD:
			if (_control_status.flags.ev_yaw) {
				// using EV frame
				vel = ev_sample.vel - vel_offset_earth;
				vel_cov = matrix::diag(ev_sample.velVar);

			} else {
				// rotate EV to the EKF reference frame
				const Quatf q_error((_state.quat_nominal * ev_sample.quat.inversed()).normalized());
				const Dcmf R_ev_to_ekf = Dcmf(q_error);

				vel = R_ev_to_ekf * ev_sample.vel - vel_offset_earth;
				vel_cov = R_ev_to_ekf * matrix::diag(ev_sample.velVar) * R_ev_to_ekf.transpose();
			}

			break;

		case VelocityFrame::BODY_FRAME_FRD:
			vel = _R_to_earth * (ev_sample.vel - vel_offset_body);
			vel_cov = _R_to_earth * matrix::diag(ev_sample.velVar) * _R_to_earth.transpose();
			break;
		}

		const Vector3f obs_var {
			fmaxf(vel_cov(0, 0), sq(0.01f)),
			fmaxf(vel_cov(1, 1), sq(0.01f)),
			fmaxf(vel_cov(2, 2), sq(0.01f))
		};

		updateVelocityAidSrcStatus(ev_sample.time_us, vel, obs_var, fmaxf(_params.ev_vel_innov_gate, 1.f), _aid_src_ev_vel);

		const bool quality_sufficient = ((_params.ev_quality_minimum <= 0) || ((_params.ev_quality_minimum > 0)
						 && (ev_sample.quality >= _params.ev_quality_minimum)));

		const bool starting_conditions_passing = continuing_conditions_passing
				&& quality_sufficient;

		if (_control_status.flags.ev_vel) {

			if (continuing_conditions_passing) {

				// reset count changed in EV sample, only reset if necessary
				const bool ev_vel_reset = (ev_sample.reset_counter != _ev_sample_delayed_prev.reset_counter)
							  && (ev_sample.vel - _ev_sample_delayed_prev.vel).longerThan(0.01f);

				if (ev_vel_reset && isOnlyActiveSourceOfHorizontalAiding(_control_status.flags.ev_vel)) {
					_information_events.flags.reset_vel_to_vision = true;
					ECL_INFO("reset to vision velocity");
					resetVelocityTo(vel);
					P.uncorrelateCovarianceSetVariance<3>(4, vel_cov.diag());

				} else {
					_aid_src_ev_vel.fusion_enabled[0] = PX4_ISFINITE(vel(0)) && quality_sufficient && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL));
					_aid_src_ev_vel.fusion_enabled[1] = PX4_ISFINITE(vel(1)) && quality_sufficient && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL));
					_aid_src_ev_vel.fusion_enabled[2] = PX4_ISFINITE(vel(2)) && quality_sufficient && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::VEL));
					fuseVelocity(_aid_src_ev_vel);

					const bool is_fusion_failing = isTimedOut(_aid_src_ev_vel.time_last_fuse[0], _params.reset_timeout_max)
								       || isTimedOut(_aid_src_ev_vel.time_last_fuse[1], _params.reset_timeout_max)
								       || isTimedOut(_aid_src_ev_vel.time_last_fuse[2], _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_nb_ev_vel_reset_available > 0) {
							// Data seems good, attempt a reset
							_information_events.flags.reset_vel_to_vision = true;
							ECL_INFO("reset to vision velocity");
							resetVelocityTo(vel);
							P.uncorrelateCovarianceSetVariance<3>(4, vel_cov.diag());

							if (_control_status.flags.in_air) {
								_nb_ev_vel_reset_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							//_control_status.flags.ev_vel_fault = true;
							stopEvVelFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvVelFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvVelFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV velocity fusion
				if (!_control_status.flags.ev_vel) {

					if (!isHorizontalAidingActive()) {
						// reset if necessary
						_information_events.flags.reset_vel_to_vision = true;
						ECL_INFO("reset to vision velocity");
						resetVelocityTo(vel);
						P.uncorrelateCovarianceSetVariance<3>(4, vel_cov.diag());
					}

					_control_status.flags.ev_vel = true;

					_information_events.flags.starting_vision_vel_fusion = true;
					ECL_INFO("starting vision velocity fusion");
				}

				if (_control_status.flags.ev_vel) {
					_nb_ev_vel_reset_available = 3;
				}

			}
		}

	} else if (_control_status.flags.ev_vel && !isNewestSampleRecent(_time_last_ext_vision_buffer_push, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvVelFusion();
	}
}

void Ekf::controlEvYawFusion(const extVisionSample &ev_sample)
{
	if (_ev_data_ready) {
		// determine if we should use the yaw observation
		resetEstimatorAidStatusFlags(_aid_src_ev_yaw);
		const float measured_hdg = getEulerYaw(ev_sample.quat);
		const float ev_yaw_obs_var = fmaxf(ev_sample.angVar, 1.e-4f);

		if (PX4_ISFINITE(measured_hdg)) {
			_aid_src_ev_yaw.timestamp_sample = ev_sample.time_us;
			_aid_src_ev_yaw.observation = measured_hdg;
			_aid_src_ev_yaw.observation_variance = ev_yaw_obs_var;

			_aid_src_ev_yaw.innovation = wrap_pi(getEulerYaw(_R_to_earth) - measured_hdg);
		}

		// if GPS enabled only allow EV yaw if EV is NED
		if (_control_status.flags.gps && _control_status.flags.yaw_align) {
			if (ev_sample.pos_frame != PositionFrame::LOCAL_FRAME_NED) {
				stopEvYawFusion();
				return;
			}
		}

		const bool quality_sufficient = ((_params.ev_quality_minimum <= 0) || ((_params.ev_quality_minimum > 0)
						 && (ev_sample.quality >= _params.ev_quality_minimum)));

		bool continuing_conditions_passing = isNewestSampleRecent(_time_last_ext_vision_buffer_push, 2 * EV_MAX_INTERVAL)
						     && (_params.ev_ctrl & static_cast<int32_t>(EvCtrl::YAW))
						     && PX4_ISFINITE(measured_hdg)
						     && !_inhibit_ev_yaw_use;

		bool starting_conditions_passing = continuing_conditions_passing
						   && _control_status.flags.tilt_align
						   && quality_sufficient;

		if (_control_status.flags.ev_yaw) {

			if (continuing_conditions_passing) {
				// reset count changed in EV sample, only reset if necessary
				const bool ev_yaw_reset = (ev_sample.reset_counter != _ev_sample_delayed_prev.reset_counter)
							  && (fabsf(wrap_pi(getEulerYaw(ev_sample.quat) - getEulerYaw(_ev_sample_delayed_prev.quat))) > math::radians(10.f));

				if (ev_yaw_reset) {
					// reset count changed in EV sample
					resetQuatStateYaw(measured_hdg, ev_yaw_obs_var);

				} else {
					_aid_src_ev_yaw.fusion_enabled = true;

					if (quality_sufficient) {
						fuseYaw(_aid_src_ev_yaw.innovation, _aid_src_ev_yaw.observation_variance, _aid_src_ev_yaw);
					}

					const bool is_fusion_failing = isTimedOut(_aid_src_ev_yaw.time_last_fuse, _params.reset_timeout_max);

					if (is_fusion_failing) {
						if (_nb_ev_yaw_reset_available > 0) {
							// Data seems good, attempt a reset
							const float yaw_new = getEulerYaw(ev_sample.quat);
							const float yaw_new_variance = fmaxf(ev_sample.angVar, sq(1.e-2f));
							resetQuatStateYaw(yaw_new, yaw_new_variance);

							if (_control_status.flags.in_air) {
								_nb_ev_yaw_reset_available--;
							}

						} else if (starting_conditions_passing) {
							// Data seems good, but previous reset did not fix the issue
							// something else must be wrong, declare the sensor faulty and stop the fusion
							//_control_status.flags.ev_yaw_fault = true;
							stopEvYawFusion();

						} else {
							// A reset did not fix the issue but all the starting checks are not passing
							// This could be a temporary issue, stop the fusion without declaring the sensor faulty
							stopEvYawFusion();
						}
					}
				}

			} else {
				// Stop fusion but do not declare it faulty
				stopEvYawFusion();
			}

		} else {
			if (starting_conditions_passing) {
				// Try to activate EV yaw fusion
				if (!_control_status.flags.ev_yaw) {

					if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_NED) {

						if (!_control_status.flags.yaw_align) {
							// reset yaw to EV and set yaw_align
							const float yaw_new = getEulerYaw(ev_sample.quat);
							const float yaw_new_variance = fmaxf(ev_sample.angVar, sq(1.e-2f));
							resetQuatStateYaw(yaw_new, yaw_new_variance);

							_control_status.flags.yaw_align = true;
						}

						_control_status.flags.ev_yaw = true;
						_information_events.flags.starting_vision_yaw_fusion = true;
						ECL_INFO("starting vision yaw fusion (aligned north)");

					} else if (ev_sample.pos_frame == PositionFrame::LOCAL_FRAME_FRD) {
						if (!_control_status.flags.gps) {
							// turn on fusion of external vision yaw measurements and disable all other heading fusion
							stopMagFusion();
							stopGpsYawFusion();

							_control_status.flags.yaw_align = false;

							// reset yaw to EV
							ECL_INFO("reset yaw to EV");
							const float yaw_new = getEulerYaw(ev_sample.quat);
							const float yaw_new_variance = fmaxf(ev_sample.angVar, sq(1.e-2f));
							resetQuatStateYaw(yaw_new, yaw_new_variance);

							_control_status.flags.ev_yaw = true;
							_information_events.flags.starting_vision_yaw_fusion = true;
							ECL_INFO("starting vision yaw fusion");
						}
					}
				}

				if (_control_status.flags.ev_yaw) {
					_nb_ev_yaw_reset_available = 2;
				}
			}
		}

	} else if (_control_status.flags.ev_yaw && !isNewestSampleRecent(_time_last_ext_vision_buffer_push, _params.reset_timeout_max)) {
		// No data anymore. Stop until it comes back.
		stopEvYawFusion();
	}
}

void Ekf::stopEvPosFusion()
{
	if (_control_status.flags.ev_pos) {
		ECL_INFO("stopping EV pos fusion");
		_control_status.flags.ev_pos = false;
	}
}

void Ekf::stopEvVelFusion()
{
	if (_control_status.flags.ev_vel) {
		ECL_INFO("stopping EV vel fusion");
		_control_status.flags.ev_vel = false;
	}
}

void Ekf::stopEvYawFusion()
{
	if (_control_status.flags.ev_yaw) {
		ECL_INFO("stopping EV yaw fusion");
		_control_status.flags.ev_yaw = false;
	}
}
