/****************************************************************************
 *
 *   Copyright (C) 2016-2018 PX4 Development Team. All rights reserved.
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
 * @file server_io.cpp
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Beat Küng <beat-kueng@gmx.net>
 * @author Mara Bos <m-ou.se@m-ou.se>
 */

#include <assert.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <px4_daemon/server_io.h>
#include <px4_platform_common/log.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <string>

#include "server.h"
#include "sock_protocol.h"

using namespace px4_daemon;

FILE *get_stdout(bool *isatty_) {
	Server::CmdThreadSpecificData *thread_data_ptr;

	// If we are not in a thread that has been started by a client, we don't
	// have any thread specific data set and we won't have a pipe to write
	// stdout to.
	if (!Server::is_running() || (thread_data_ptr = (Server::CmdThreadSpecificData *)pthread_getspecific(
					      Server::get_pthread_key())) == nullptr) {
		if (isatty_) {
			*isatty_ = isatty(1);
		}

		return stdout;
	}

	if (thread_data_ptr->thread_stdout == nullptr) {
		if (isatty_) {
			*isatty_ = isatty(1);
		}

		return stdout;
	}

	if (isatty_) {
		*isatty_ = thread_data_ptr->is_atty;
	}

	return thread_data_ptr->thread_stdout;
}
