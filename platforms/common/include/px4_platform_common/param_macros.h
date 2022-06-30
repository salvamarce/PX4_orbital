/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file param_macros.h
 *
 * Helper macros used by px4_param.h
 */

#pragma once

#define APPLY0(t)
#define APPLY1(t, aaa) t(aaa)
#define APPLY2(t, aaa, aab) t(aaa) t(aab)
#define APPLY3(t, aaa, aab, aac) t(aaa) t(aab) t(aac)
#define APPLY4(t, aaa, aab, aac, aad) t(aaa) t(aab) t(aac) t(aad)
#define APPLY5(t, aaa, aab, aac, aad, aae) t(aaa) t(aab) t(aac) t(aad) t(aae)
#define APPLY6(t, aaa, aab, aac, aad, aae, aaf) t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf)
#define APPLY7(t, aaa, aab, aac, aad, aae, aaf, aag) t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag)
#define APPLY8(t, aaa, aab, aac, aad, aae, aaf, aag, aah) t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah)
#define APPLY9(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai) \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai)
#define APPLY10(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj) \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj)
#define APPLY11(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak) \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak)
#define APPLY12(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal) \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal)
#define APPLY13(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam) \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam)
#define APPLY14(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan) \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan)
#define APPLY15(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao) \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)
#define APPLY16(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap) \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(aap)
#define APPLY17(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq)          \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) \
		t(aap) t(aaq)
#define APPLY18(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar)     \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) \
		t(aap) t(aaq) t(aar)
#define APPLY19(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas) \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)  \
		t(aap) t(aaq) t(aar) t(aas)
#define APPLY20(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat) \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat)
#define APPLY21(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau)                                                                                                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau)
#define APPLY22(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav)                                                                                              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav)
#define APPLY23(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw)                                                                                         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw)
#define APPLY24(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax)                                                                                    \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax)
#define APPLY25(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay)                                                                               \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay)
#define APPLY26(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz)                                                                          \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz)
#define APPLY27(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba)                                                                     \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba)
#define APPLY28(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb)                                                                \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb)
#define APPLY29(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc)                                                           \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)
#define APPLY30(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd)                                                      \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd)
#define APPLY31(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe)                                                 \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe)
#define APPLY32(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf)                                            \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf)
#define APPLY33(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg)                                       \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg)
#define APPLY34(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh)                                  \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh)
#define APPLY35(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi)                             \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi)
#define APPLY36(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj)                        \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj)
#define APPLY37(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk)                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk)
#define APPLY38(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl)              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl)
#define APPLY39(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm)         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm)
#define APPLY40(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn)    \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn)
#define APPLY41(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo)                                                                                                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo)
#define APPLY42(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp)                                                                                              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)
#define APPLY43(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq)                                                                                         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq)
#define APPLY44(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr)                                                                                    \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr)
#define APPLY45(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs)                                                                               \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs)
#define APPLY46(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt)                                                                          \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt)
#define APPLY47(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu)                                                                     \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu)
#define APPLY48(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv)                                                                \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv)
#define APPLY49(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw)                                                           \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw)
#define APPLY50(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx)                                                      \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx)
#define APPLY51(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby)                                                 \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby)
#define APPLY52(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz)                                            \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz)
#define APPLY53(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca)                                       \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca)
#define APPLY54(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb)                                  \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)
#define APPLY55(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc)                             \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc)
#define APPLY56(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd)                        \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd)
#define APPLY57(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace)                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace)
#define APPLY58(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf)              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf)
#define APPLY59(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg)         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg)
#define APPLY60(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach)    \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach)
#define APPLY61(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci)                                                                                                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci)
#define APPLY62(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj)                                                                                              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj)
#define APPLY63(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack)                                                                                         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack)
#define APPLY64(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl)                                                                                    \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl)
#define APPLY65(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm)                                                                               \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)
#define APPLY66(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn)                                                                          \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn)
#define APPLY67(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco)                                                                     \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco)
#define APPLY68(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp)                                                                \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp)
#define APPLY69(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq)                                                           \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq)
#define APPLY70(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr)                                                      \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr)
#define APPLY71(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs)                                                 \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs)
#define APPLY72(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act)                                            \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act)
#define APPLY73(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu)                                       \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu)
#define APPLY74(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv)                                  \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv)
#define APPLY75(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw)                             \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)
#define APPLY76(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx)                        \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx)
#define APPLY77(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy)                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy)
#define APPLY78(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz)              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz)
#define APPLY79(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada)         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada)
#define APPLY80(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb)    \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb)
#define APPLY81(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc)                                                                                                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc)
#define APPLY82(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add)                                                                                              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)
#define APPLY83(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade)                                                                                         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade)
#define APPLY84(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf)                                                                                    \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf)
#define APPLY85(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg)                                                                               \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg)
#define APPLY86(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh)                                                                          \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh)
#define APPLY87(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi)                                                                     \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi)
#define APPLY88(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj)                                                                \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj)
#define APPLY89(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk)                                                           \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk)
#define APPLY90(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk, adl)                                                      \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)
#define APPLY91(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm)                                                 \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)
#define APPLY92(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn)                                            \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn)
#define APPLY93(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado)                                       \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado)
#define APPLY94(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp)                                  \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp)
#define APPLY95(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq)                             \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq)
#define APPLY96(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr)                        \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr)
#define APPLY97(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads)                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)
#define APPLY98(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt)              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt)
#define APPLY99(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas, aat, \
		aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm, abn,    \
		abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg, ach,    \
		aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada, adb,    \
		adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu)         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu)
#define APPLY100(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv)                                                                                                  \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu) t(adv)
#define APPLY101(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw)                                                                                             \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu) t(adv) t(adw)
#define APPLY102(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx)                                                                                        \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu) t(adv) t(adw) t(adx)
#define APPLY103(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady)                                                                                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu) t(adv) t(adw) t(adx)     \
											t(ady)
#define APPLY104(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz)                                                                              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu) t(adv) t(adw) t(adx)     \
											t(ady) t(adz)
#define APPLY105(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea)                                                                         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu) t(adv) t(adw) t(adx)     \
											t(ady) t(adz) t(aea)
#define APPLY106(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb)                                                                    \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu) t(adv) t(adw) t(adx)     \
											t(ady) t(adz) t(aea) t(aeb)
#define APPLY107(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec)                                                               \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu) t(adv) t(adw) t(adx)     \
											t(ady) t(adz) t(aea) t(aeb)    \
												t(aec)
#define APPLY108(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed)                                                          \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu) t(adv) t(adw) t(adx)     \
											t(ady) t(adz) t(aea) t(aeb)    \
												t(aec) t(aed)
#define APPLY109(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee)                                                     \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao)       \
		t(aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc)      \
			t(abd) t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp)     \
				t(abq) t(abr) t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb)    \
					t(acc) t(acd) t(ace) t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm)   \
						t(acn) t(aco) t(acp) t(acq) t(acr) t(acs) t(act) t(acu) t(acv) t(acw)  \
							t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add) t(ade) t(adf) \
								t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm)       \
									t(adn) t(ado) t(adp) t(adq) t(adr) t(ads)      \
										t(adt) t(adu) t(adv) t(adw) t(adx)     \
											t(ady) t(adz) t(aea) t(aeb)    \
												t(aec) t(aed) t(aee)
#define APPLY110(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef)                                                \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef)
#define APPLY111(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg)                                           \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg)
#define APPLY112(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh)                                      \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)
#define APPLY113(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei)                                 \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei)
#define APPLY114(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej)                            \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej)
#define APPLY115(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek)                       \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek)
#define APPLY116(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael)                  \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek) t(ael)
#define APPLY117(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem)             \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek) t(ael) t(aem)
#define APPLY118(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen)        \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek) t(ael) t(aem)     \
											t(aen)
#define APPLY119(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo)   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek) t(ael) t(aem)     \
											t(aen) t(aeo)
#define APPLY120(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep)                                                                                                  \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek) t(ael) t(aem)     \
											t(aen) t(aeo) t(aep)
#define APPLY121(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq)                                                                                             \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek) t(ael) t(aem)     \
											t(aen) t(aeo) t(aep) t(aeq)
#define APPLY122(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer)                                                                                        \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek) t(ael) t(aem)     \
											t(aen) t(aeo) t(aep) t(aeq)    \
												t(aer)
#define APPLY123(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes)                                                                                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek) t(ael) t(aem)     \
											t(aen) t(aeo) t(aep) t(aeq)    \
												t(aer) t(aes)
#define APPLY124(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet)                                                                              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek) t(ael) t(aem)     \
											t(aen) t(aeo) t(aep) t(aeq)    \
												t(aer) t(aes) t(aet)
#define APPLY125(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu)                                                                         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)
#define APPLY126(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev)                                                                    \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr)      \
			t(abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)     \
				t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq)    \
					t(acr) t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb)   \
						t(adc) t(add) t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl)  \
							t(adm) t(adn) t(ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) \
								t(adv) t(adw) t(adx) t(ady) t(adz) t(aea) t(aeb)       \
									t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)      \
										t(aei) t(aej) t(aek) t(ael) t(aem)     \
											t(aen) t(aeo) t(aep) t(aeq)    \
												t(aer) t(aes) t(aet)   \
													t(aeu) t(aev)
#define APPLY127(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew)                                                               \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew)
#define APPLY128(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex)                                                          \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex)
#define APPLY129(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey)                                                     \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey)
#define APPLY130(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez)                                                \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)
#define APPLY131(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa)                                           \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)     \
											t(afa)
#define APPLY132(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb)                                      \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)     \
											t(afa) t(afb)
#define APPLY133(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc)                                 \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)     \
											t(afa) t(afb) t(afc)
#define APPLY134(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd)                            \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)     \
											t(afa) t(afb) t(afc) t(afd)
#define APPLY135(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe)                       \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)     \
											t(afa) t(afb) t(afc) t(afd)    \
												t(afe)
#define APPLY136(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff)                  \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)     \
											t(afa) t(afb) t(afc) t(afd)    \
												t(afe) t(aff)
#define APPLY137(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg)             \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)     \
											t(afa) t(afb) t(afc) t(afd)    \
												t(afe) t(aff) t(afg)
#define APPLY138(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh)        \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)     \
											t(afa) t(afb) t(afc) t(afd)    \
												t(afe) t(aff) t(afg)   \
													t(afh)
#define APPLY139(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi)   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)     \
											t(afa) t(afb) t(afc) t(afd)    \
												t(afe) t(aff) t(afg)   \
													t(afh) t(afi)
#define APPLY140(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi,   \
		 afj)                                                                                                  \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(ado)   \
						t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx) t(ady)  \
							t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh) \
								t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo)       \
									t(aep) t(aeq) t(aer) t(aes) t(aet) t(aeu)      \
										t(aev) t(aew) t(aex) t(aey) t(aez)     \
											t(afa) t(afb) t(afc) t(afd)    \
												t(afe) t(aff) t(afg)   \
													t(afh) t(afi)  \
														t(afj)
#define APPLY141(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi,   \
		 afj, afk)                                                                                             \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(       \
						ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx)    \
						t(ady) t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)  \
							t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo) t(aep) t(aeq) \
								t(aer) t(aes) t(aet) t(aeu) t(aev) t(aew) t(aex)       \
									t(aey) t(aez) t(afa) t(afb) t(afc) t(afd)      \
										t(afe) t(aff) t(afg) t(afh) t(afi)     \
											t(afj) t(afk)
#define APPLY142(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi,   \
		 afj, afk, afl)                                                                                        \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(       \
						ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx)    \
						t(ady) t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)  \
							t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo) t(aep) t(aeq) \
								t(aer) t(aes) t(aet) t(aeu) t(aev) t(aew) t(aex)       \
									t(aey) t(aez) t(afa) t(afb) t(afc) t(afd)      \
										t(afe) t(aff) t(afg) t(afh) t(afi)     \
											t(afj) t(afk) t(afl)
#define APPLY143(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi,   \
		 afj, afk, afl, afm)                                                                                   \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(       \
						ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx)    \
						t(ady) t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)  \
							t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo) t(aep) t(aeq) \
								t(aer) t(aes) t(aet) t(aeu) t(aev) t(aew) t(aex)       \
									t(aey) t(aez) t(afa) t(afb) t(afc) t(afd)      \
										t(afe) t(aff) t(afg) t(afh) t(afi)     \
											t(afj) t(afk) t(afl) t(afm)
#define APPLY144(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi,   \
		 afj, afk, afl, afm, afn)                                                                              \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(       \
						ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx)    \
						t(ady) t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)  \
							t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo) t(aep) t(aeq) \
								t(aer) t(aes) t(aet) t(aeu) t(aev) t(aew) t(aex)       \
									t(aey) t(aez) t(afa) t(afb) t(afc) t(afd)      \
										t(afe) t(aff) t(afg) t(afh) t(afi)     \
											t(afj) t(afk) t(afl) t(afm)    \
												t(afn)
#define APPLY145(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi,   \
		 afj, afk, afl, afm, afn, afo)                                                                         \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(       \
						ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx)    \
						t(ady) t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)  \
							t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo) t(aep) t(aeq) \
								t(aer) t(aes) t(aet) t(aeu) t(aev) t(aew) t(aex)       \
									t(aey) t(aez) t(afa) t(afb) t(afc) t(afd)      \
										t(afe) t(aff) t(afg) t(afh) t(afi)     \
											t(afj) t(afk) t(afl) t(afm)    \
												t(afn) t(afo)
#define APPLY146(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi,   \
		 afj, afk, afl, afm, afn, afo, afp)                                                                    \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(       \
						ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx)    \
						t(ady) t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)  \
							t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo) t(aep) t(aeq) \
								t(aer) t(aes) t(aet) t(aeu) t(aev) t(aew) t(aex)       \
									t(aey) t(aez) t(afa) t(afb) t(afc) t(afd)      \
										t(afe) t(aff) t(afg) t(afh) t(afi)     \
											t(afj) t(afk) t(afl) t(afm)    \
												t(afn) t(afo) t(afp)
#define APPLY147(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi,   \
		 afj, afk, afl, afm, afn, afo, afp, afq)                                                               \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(       \
						ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx)    \
						t(ady) t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)  \
							t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo) t(aep) t(aeq) \
								t(aer) t(aes) t(aet) t(aeu) t(aev) t(aew) t(aex)       \
									t(aey) t(aez) t(afa) t(afb) t(afc) t(afd)      \
										t(afe) t(aff) t(afg) t(afh) t(afi)     \
											t(afj) t(afk) t(afl) t(afm)    \
												t(afn) t(afo) t(afp)   \
													t(afq)
#define APPLY148(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi,   \
		 afj, afk, afl, afm, afn, afo, afp, afq, afr)                                                          \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(       \
						ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx)    \
						t(ady) t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)  \
							t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo) t(aep) t(aeq) \
								t(aer) t(aes) t(aet) t(aeu) t(aev) t(aew) t(aex)       \
									t(aey) t(aez) t(afa) t(afb) t(afc) t(afd)      \
										t(afe) t(aff) t(afg) t(afh) t(afi)     \
											t(afj) t(afk) t(afl) t(afm)    \
												t(afn) t(afo) t(afp)   \
													t(afq) t(afr)
#define APPLY149(t, aaa, aab, aac, aad, aae, aaf, aag, aah, aai, aaj, aak, aal, aam, aan, aao, aap, aaq, aar, aas,     \
		 aat, aau, aav, aaw, aax, aay, aaz, aba, abb, abc, abd, abe, abf, abg, abh, abi, abj, abk, abl, abm,   \
		 abn, abo, abp, abq, abr, abs, abt, abu, abv, abw, abx, aby, abz, aca, acb, acc, acd, ace, acf, acg,   \
		 ach, aci, acj, ack, acl, acm, acn, aco, acp, acq, acr, acs, act, acu, acv, acw, acx, acy, acz, ada,   \
		 adb, adc, add, ade, adf, adg, adh, adi, adj, adk, adl, adm, adn, ado, adp, adq, adr, ads, adt, adu,   \
		 adv, adw, adx, ady, adz, aea, aeb, aec, aed, aee, aef, aeg, aeh, aei, aej, aek, ael, aem, aen, aeo,   \
		 aep, aeq, aer, aes, aet, aeu, aev, aew, aex, aey, aez, afa, afb, afc, afd, afe, aff, afg, afh, afi,   \
		 afj, afk, afl, afm, afn, afo, afp, afq, afr, afs)                                                     \
	t(aaa) t(aab) t(aac) t(aad) t(aae) t(aaf) t(aag) t(aah) t(aai) t(aaj) t(aak) t(aal) t(aam) t(aan) t(aao) t(    \
		aap) t(aaq) t(aar) t(aas) t(aat) t(aau) t(aav) t(aaw) t(aax) t(aay) t(aaz) t(aba) t(abb) t(abc) t(abd) \
		t(abe) t(abf) t(abg) t(abh) t(abi) t(abj) t(abk) t(abl) t(abm) t(abn) t(abo) t(abp) t(abq) t(abr) t(   \
			abs) t(abt) t(abu) t(abv) t(abw) t(abx) t(aby) t(abz) t(aca) t(acb) t(acc) t(acd) t(ace)       \
			t(acf) t(acg) t(ach) t(aci) t(acj) t(ack) t(acl) t(acm) t(acn) t(aco) t(acp) t(acq) t(acr)     \
				t(acs) t(act) t(acu) t(acv) t(acw) t(acx) t(acy) t(acz) t(ada) t(adb) t(adc) t(add)    \
					t(ade) t(adf) t(adg) t(adh) t(adi) t(adj) t(adk) t(adl) t(adm) t(adn) t(       \
						ado) t(adp) t(adq) t(adr) t(ads) t(adt) t(adu) t(adv) t(adw) t(adx)    \
						t(ady) t(adz) t(aea) t(aeb) t(aec) t(aed) t(aee) t(aef) t(aeg) t(aeh)  \
							t(aei) t(aej) t(aek) t(ael) t(aem) t(aen) t(aeo) t(aep) t(aeq) \
								t(aer) t(aes) t(aet) t(aeu) t(aev) t(aew) t(aex)       \
									t(aey) t(aez) t(afa) t(afb) t(afc) t(afd)      \
										t(afe) t(aff) t(afg) t(afh) t(afi)     \
											t(afj) t(afk) t(afl) t(afm)    \
												t(afn) t(afo) t(afp)   \
													t(afq) t(afr)  \
														t(afs)

#define NUM_ARGS_H1(dummy, x149, x148, x147, x146, x145, x144, x143, x142, x141, x140, x139, x138, x137, x136, x135,  \
		    x134, x133, x132, x131, x130, x129, x128, x127, x126, x125, x124, x123, x122, x121, x120, x119,   \
		    x118, x117, x116, x115, x114, x113, x112, x111, x110, x109, x108, x107, x106, x105, x104, x103,   \
		    x102, x101, x100, x99, x98, x97, x96, x95, x94, x93, x92, x91, x90, x89, x88, x87, x86, x85, x84, \
		    x83, x82, x81, x80, x79, x78, x77, x76, x75, x74, x73, x72, x71, x70, x69, x68, x67, x66, x65,    \
		    x64, x63, x62, x61, x60, x59, x58, x57, x56, x55, x54, x53, x52, x51, x50, x49, x48, x47, x46,    \
		    x45, x44, x43, x42, x41, x40, x39, x38, x37, x36, x35, x34, x33, x32, x31, x30, x29, x28, x27,    \
		    x26, x25, x24, x23, x22, x21, x20, x19, x18, x17, x16, x15, x14, x13, x12, x11, x10, x9, x8, x7,  \
		    x6, x5, x4, x3, x2, x1, x0, ...)                                                                  \
	x0
#define NUM_ARGS(...)                                                                                                \
	NUM_ARGS_H1(dummy, ##__VA_ARGS__, 149, 148, 147, 146, 145, 144, 143, 142, 141, 140, 139, 138, 137, 136, 135, \
		    134, 133, 132, 131, 130, 129, 128, 127, 126, 125, 124, 123, 122, 121, 120, 119, 118, 117, 116,   \
		    115, 114, 113, 112, 111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, 99, 98, 97, 96,  \
		    95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72,  \
		    71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48,  \
		    47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24,  \
		    23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0)

#define APPLY_ALL_H3(t, n, ...) APPLY##n(t, __VA_ARGS__)
#define APPLY_ALL_H2(t, n, ...) APPLY_ALL_H3(t, n, __VA_ARGS__)
#define APPLY_ALL(t, ...) APPLY_ALL_H2(t, NUM_ARGS(__VA_ARGS__), __VA_ARGS__)

/*
 * The above macros are auto-generated using the following python script:
#! /bin/python
# generate a C/C++ macro APPLY_ALL up to 'count' arguments
import sys

count = 150

def output(s):
    sys.stdout.write(s)

def var_name(i):
    """ get a variable name in the form abc from a counter i """
    b = ord('a')
    return chr(b+((i/26/26) % 26)) + chr(b+((i/26) % 26)) + chr(b+(i % 26))

for i in range(count):
    output("#define APPLY{:}(t".format(i))
    output(''.join([", {:}".format(var_name(j)) for j in range(i)]))
    output(") ")
    output(''.join(["t({:}) ".format(var_name(j)) for j in range(i)]))
    output("\n")

output('''\n#define NUM_ARGS_H1(dummy{:}, ...) x0
'''.format(''.join([', x'+str(x) for x in reversed(range(count))])))
output('''#define NUM_ARGS(...) NUM_ARGS_H1(dummy, ##__VA_ARGS__{:})
'''.format(''.join([', '+str(x) for x in reversed(range(count))])))
output('''
#define APPLY_ALL_H3(t, n, ...) APPLY##n(t, __VA_ARGS__)
#define APPLY_ALL_H2(t, n, ...) APPLY_ALL_H3(t, n, __VA_ARGS__)
#define APPLY_ALL(t, ...) APPLY_ALL_H2(t, NUM_ARGS(__VA_ARGS__), __VA_ARGS__)
''')

 */

// helper macros to handle macro arguments in the form: (type) name

#define REM(...) __VA_ARGS__
#define EAT(...)

// Retrieve the type
#define TYPEOF(x) DETAIL_TYPEOF(DETAIL_TYPEOF_PROBE x, )
#define DETAIL_TYPEOF(...) DETAIL_TYPEOF_HEAD(__VA_ARGS__)
#define DETAIL_TYPEOF_HEAD(x, ...) REM x
#define DETAIL_TYPEOF_PROBE(...) (__VA_ARGS__),
// Strip off the type, get the name
#define STRIP(x) EAT x
// Show the type without parenthesis
#define PAIR(x) REM x
