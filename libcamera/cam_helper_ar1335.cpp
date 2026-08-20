/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2026, radcam1 project
 *
 * camera helper for the onsemi AR1335
 *
 * Install by copying into
 *   src/ipa/rpi/cam_helper/cam_helper_ar1335.cpp
 * and adding it to that directory's meson.build.
 */

#include "cam_helper.h"

using namespace RPiController;

class CamHelperAr1335 : public CamHelper
{
public:
	CamHelperAr1335();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	bool sensorEmbeddedDataPresent() const override;

private:
	/*
	 * Smallest difference between the frame length and integration time, in
	 * lines. The kernel driver caps COARSE_INTEGRATION_TIME (0x0202) at
	 * FRAME_LENGTH_LINES - 1, matching the vendor's sequences, so one line
	 * is all that is required here.
	 */
	static constexpr int frameIntegrationDiff = 1;
};

/*
 * The AR1335 mode sequences do not turn on embedded data output, so there is
 * no per-frame register block to parse. Passing an empty parser makes libcamera
 * fall back to counting frames to associate controls with results.
 */
CamHelperAr1335::CamHelperAr1335()
	: CamHelper({}, frameIntegrationDiff)
{
}

/*
 * The AR1335's GLOBAL_GAIN register (0x305E) uses a banded coarse/fine
 * encoding that is not monotonic in the raw register value, so the kernel
 * driver hides it: V4L2_CID_ANALOGUE_GAIN is exposed in linear units of
 * 1/1024, where 1024 == 1.0x, and the driver converts to the register form.
 * That keeps the mapping here trivial and, more importantly, monotonic, which
 * is what the AGC algorithm needs.
 */
uint32_t CamHelperAr1335::gainCode(double gain) const
{
	return static_cast<uint32_t>(gain * 1024.0);
}

double CamHelperAr1335::gain(uint32_t gainCode) const
{
	return static_cast<double>(gainCode) / 1024.0;
}

bool CamHelperAr1335::sensorEmbeddedDataPresent() const
{
	return false;
}

static CamHelper *create()
{
	return new CamHelperAr1335();
}

static RegisterCamHelper reg("ar1335", &create);
