// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 sensor driver for the onsemi AR1335 13 MP MIPI CSI-2 image sensor.
 *
 * Written for the radcam1 Raspberry Pi 5 camera system. Works on either the
 * CAM0 or CAM1 connector; see overlays/ar1335-overlay.dts.
 *
 * The mode register sequences live in ar1335_modes.h and are generated from
 * NXP's GPL-2.0 isp-vvcam AR1335 driver by tools/gen-regs.py. They assume a
 * 24 MHz EXTCLK, which the camera module must supply itself - the Pi's 22-pin
 * camera connector carries no clock signal.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#include "ar1335_modes.h"

/*
 * Bring-up aid: how long to wait after powering the sensor before touching it.
 * Raise this (e.g. to a few seconds) to hold the rails and EXTCLK on long
 * enough to probe the connector with a meter or scope.
 */
static unsigned int power_on_delay_ms = 10;
module_param(power_on_delay_ms, uint, 0644);
MODULE_PARM_DESC(power_on_delay_ms,
		 "delay after power-on before accessing the sensor (ms)");

/*
 * Some modules do not come up on the first attempt: the rails or the module's
 * own oscillator can take longer to settle than a single power-on delay
 * allows, and a half-powered sensor will not acknowledge its address. Rather
 * than fail the probe permanently - which needs a reboot to recover from -
 * fully power-cycle the module and try again.
 */
static unsigned int probe_retries = 8;
module_param(probe_retries, uint, 0644);
MODULE_PARM_DESC(probe_retries,
		 "power-cycle and retry this many times before giving up");

static unsigned int power_cycle_off_ms = 250;
module_param(power_cycle_off_ms, uint, 0644);
MODULE_PARM_DESC(power_cycle_off_ms,
		 "how long to hold the module powered down between retries (ms)");

/* Register map ----------------------------------------------------------- */

#define AR1335_REG_MODEL_ID		CCI_REG16(0x3000)
#define AR1335_MODEL_ID			0x0153

#define AR1335_REG_RESET		CCI_REG16(0x301A)
/*
 * RESET_REGISTER bit 0 is a soft reset and bit 2 is stream enable. The vendor
 * mode tables open with 0x0219 (reset asserted) and close with 0x021C, which
 * *clears* the reset and sets stream in one write. tools/gen-regs.py strips
 * that closing write because it carries the stream bit, so the driver must
 * clear the reset itself - otherwise the sensor stays held in reset, produces
 * no frames, and the CSI front end blocks forever waiting for one.
 */
#define AR1335_RESET_RESET		BIT(0)
#define AR1335_RESET_STREAM		BIT(2)

#define AR1335_REG_EXPOSURE		CCI_REG16(0x0202)	/* coarse integration */
#define AR1335_REG_GLOBAL_GAIN		CCI_REG16(0x305E)
#define AR1335_REG_FRAME_LENGTH_LINES	CCI_REG16(0x0340)
#define AR1335_REG_LINE_LENGTH_PCK	CCI_REG16(0x0342)
#define AR1335_REG_READ_MODE		CCI_REG16(0x3040)

/*
 * READ_MODE mirror/flip bits, per the onsemi AR-family register map. Both are
 * applied on top of the per-mode value of READ_MODE, whose low bits carry the
 * binning configuration.
 */
#define AR1335_READ_MODE_MIRROR		BIT(14)
#define AR1335_READ_MODE_FLIP		BIT(15)

/* Sensor limits ---------------------------------------------------------- */

#define AR1335_NATIVE_WIDTH		4208U
#define AR1335_NATIVE_HEIGHT		3120U

#define AR1335_VTS_MAX			0xFFFFU

/* COARSE_INTEGRATION_TIME is expressed in lines. */
#define AR1335_EXPOSURE_MIN		8U
#define AR1335_EXPOSURE_MARGIN		1U	/* max = frame length - this */
#define AR1335_EXPOSURE_DEFAULT		1000U

/*
 * Gain is carried in 1/1024 units, matching the fixed-point convention of the
 * vendor driver: 1024 == 1.0x. The sensor tops out at 24x.
 */
#define AR1335_GAIN_FRAC_BITS		10
#define AR1335_GAIN_MIN			(1U << AR1335_GAIN_FRAC_BITS)
#define AR1335_GAIN_MAX			(24U << AR1335_GAIN_FRAC_BITS)
#define AR1335_GAIN_DEFAULT		AR1335_GAIN_MIN

#define AR1335_XCLK_FREQ		24000000U
/*
 * A programmable divider rarely lands exactly on 24 MHz - the RP1 GPCLK gets
 * within a few parts per million - and the sensor does not care. Accept 1%.
 */
#define AR1335_XCLK_TOLERANCE		(AR1335_XCLK_FREQ / 100)

/* Link frequencies, indexed by ar1335_mode.link_freq_idx. */
static const s64 ar1335_link_freqs[] = {
	439200000,	/* binned modes:     24 MHz / 5 * 183      = 878.4 MHz VCO */
	468000000,	/* full resolution:  24 MHz / 1 *  39 / 2  = 468 MHz      */
};

/*
 * Bayer order is GRBG with no flips applied, and rotates as the readout is
 * mirrored or flipped. Indexed by (vflip << 1) | hflip.
 */
static const u32 ar1335_mbus_codes[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

static const char * const ar1335_supply_names[] = {
	"VANA",		/* analogue 2.8 V - also the module enable on this board */
	"VDIG",		/* digital core 1.5 V */
	"VDDL",		/* digital I/O 1.8 V */
};

#define AR1335_NUM_SUPPLIES ARRAY_SIZE(ar1335_supply_names)

/* Modes ------------------------------------------------------------------ */

struct ar1335_mode {
	unsigned int width;
	unsigned int height;

	/* Analogue crop rectangle on the native pixel array. */
	struct v4l2_rect crop;

	unsigned int line_length_pix;	/* LINE_LENGTH_PCK */
	unsigned int vts_def;		/* default FRAME_LENGTH_LINES */
	unsigned int vts_min;		/* fastest frame rate this mode allows */

	/*
	 * Effective pixel rate, chosen so that
	 *	line time = line_length_pix / pixel_rate
	 * reproduces the sensor's real line time.
	 */
	u64 pixel_rate;

	unsigned int link_freq_idx;

	const struct cci_reg_sequence *reg_list;
	unsigned int num_regs;
};

static const struct ar1335_mode ar1335_supported_modes[] = {
	{
		/* Full resolution, 25 fps. */
		.width = 4096,
		.height = 3072,
		.crop = {
			.left = 72,
			.top = 4,
			.width = 4096,
			.height = 3072,
		},
		.line_length_pix = 4656,
		.vts_def = 3216,
		.vts_min = 3216,
		.pixel_rate = 374400000,
		.link_freq_idx = 1,
		.reg_list = ar1335_mode_4096x3072_regs,
		.num_regs = ARRAY_SIZE(ar1335_mode_4096x3072_regs),
	},
	{
		/*
		 * 2x2 binned 1080p. The vendor's 30 fps and 60 fps sequences are
		 * identical apart from the trailing frame-length write, so this is
		 * a single mode whose frame rate is set through V4L2_CID_VBLANK:
		 * 3150 lines gives 30 fps, 1573 gives 60 fps.
		 */
		.width = 1920,
		.height = 1080,
		.crop = {
			.left = 200,
			.top = 496,
			.width = 3840,
			.height = 2158,
		},
		.line_length_pix = 4656,
		.vts_def = 3150,
		.vts_min = 1573,
		.pixel_rate = 439200000,
		.link_freq_idx = 0,
		.reg_list = ar1335_mode_1920x1080_30fps_regs,
		.num_regs = ARRAY_SIZE(ar1335_mode_1920x1080_30fps_regs),
	},
};

/* Driver state ----------------------------------------------------------- */

struct ar1335 {
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[AR1335_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	/* Cached READ_MODE for the current mode, before flip bits are applied. */
	u16 read_mode;
};

static inline struct ar1335 *to_ar1335(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar1335, sd);
}

/* Look up the mode matching a pad format, falling back to the first mode. */
static const struct ar1335_mode *
ar1335_mode_for_format(const struct v4l2_mbus_framefmt *fmt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ar1335_supported_modes); i++) {
		if (ar1335_supported_modes[i].width == fmt->width &&
		    ar1335_supported_modes[i].height == fmt->height)
			return &ar1335_supported_modes[i];
	}

	return &ar1335_supported_modes[0];
}

static u32 ar1335_get_format_code(struct ar1335 *ar1335)
{
	unsigned int i;

	lockdep_assert_held(ar1335->ctrl_handler.lock);

	i = (ar1335->vflip->val ? 2 : 0) | (ar1335->hflip->val ? 1 : 0);

	return ar1335_mbus_codes[i];
}

/*
 * Convert a linear gain in 1/1024 units into the AR1335's banded GLOBAL_GAIN
 * encoding. Ported from the vendor driver, whose breakpoints follow the
 * sensor's coarse/fine gain structure.
 */
static u16 ar1335_gain_to_reg(u32 gain)
{
	u16 reg;
	u32 div;

	if (gain < 0x400)
		return 0x2010;

	if (gain < (8 << AR1335_GAIN_FRAC_BITS)) {
		div = gain >> AR1335_GAIN_FRAC_BITS;
		if (div < 2) {
			reg = 0x2010;
			reg += 2 * ((gain - 0x400) / 125);
		} else if (div < 4) {
			reg = 0x2020;
			reg += (gain - 0x800) / 125;
		} else {
			reg = 0x2030;
			reg += (gain - 0x1000) / 250;
		}
		return reg;
	}

	if (gain < (24 << AR1335_GAIN_FRAC_BITS)) {
		reg = 0x21BF;
		reg += (0x100 * (gain - 0x2000) / 250) & 0xFF00;
		return reg;
	}

	return 0x633F;
}

/* Power ------------------------------------------------------------------ */

static int ar1335_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar1335 *ar1335 = to_ar1335(sd);
	int ret;

	ret = regulator_bulk_enable(AR1335_NUM_SUPPLIES, ar1335->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ar1335->xclk);
	if (ret) {
		dev_err(dev, "failed to enable EXTCLK: %d\n", ret);
		goto err_regulator;
	}

	gpiod_set_value_cansleep(ar1335->reset_gpio, 0);

	/*
	 * The AR1335 needs EXTCLK running and reset released before its serial
	 * interface responds; allow the internal power-up sequence to finish.
	 */
	msleep(power_on_delay_ms);

	return 0;

err_regulator:
	regulator_bulk_disable(AR1335_NUM_SUPPLIES, ar1335->supplies);
	return ret;
}

static int ar1335_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ar1335 *ar1335 = to_ar1335(sd);

	gpiod_set_value_cansleep(ar1335->reset_gpio, 1);
	clk_disable_unprepare(ar1335->xclk);
	regulator_bulk_disable(AR1335_NUM_SUPPLIES, ar1335->supplies);

	return 0;
}

/* Controls --------------------------------------------------------------- */

static int ar1335_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar1335 *ar1335 =
		container_of(ctrl->handler, struct ar1335, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&ar1335->sd);
	const struct ar1335_mode *mode;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	int ret = 0;
	u16 val;

	state = v4l2_subdev_get_locked_active_state(&ar1335->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);
	mode = ar1335_mode_for_format(fmt);

	/*
	 * The frame length bounds the exposure, so keep the exposure control's
	 * range in step with VBLANK even while powered down.
	 */
	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max =
			mode->height + ctrl->val - AR1335_EXPOSURE_MARGIN;

		__v4l2_ctrl_modify_range(ar1335->exposure,
					 ar1335->exposure->minimum,
					 exposure_max,
					 ar1335->exposure->step,
					 min(ar1335->exposure->val, exposure_max));
	}

	/* Nothing to write while the sensor is powered down. */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(ar1335->regmap, AR1335_REG_EXPOSURE, ctrl->val, &ret);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(ar1335->regmap, AR1335_REG_GLOBAL_GAIN,
			  ar1335_gain_to_reg(ctrl->val), &ret);
		break;

	case V4L2_CID_VBLANK:
		cci_write(ar1335->regmap, AR1335_REG_FRAME_LENGTH_LINES,
			  mode->height + ctrl->val, &ret);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		val = ar1335->read_mode;
		if (ar1335->hflip->val)
			val |= AR1335_READ_MODE_MIRROR;
		if (ar1335->vflip->val)
			val |= AR1335_READ_MODE_FLIP;
		cci_write(ar1335->regmap, AR1335_REG_READ_MODE, val, &ret);
		break;

	case V4L2_CID_CAMERA_ORIENTATION:
	case V4L2_CID_CAMERA_SENSOR_ROTATION:
		/* Read-only descriptions of how the sensor is mounted. */
		break;

	default:
		dev_dbg(&client->dev, "unhandled control 0x%x\n", ctrl->id);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ar1335_ctrl_ops = {
	.s_ctrl = ar1335_set_ctrl,
};

static int ar1335_init_controls(struct ar1335 *ar1335)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar1335->sd);
	const struct ar1335_mode *mode = &ar1335_supported_modes[0];
	struct v4l2_ctrl_handler *hdl = &ar1335->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	unsigned int exposure_max;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 12);
	if (ret)
		return ret;

	ar1335->pixel_rate = v4l2_ctrl_new_std(hdl, &ar1335_ctrl_ops,
					       V4L2_CID_PIXEL_RATE,
					       mode->pixel_rate, mode->pixel_rate,
					       1, mode->pixel_rate);

	ar1335->link_freq = v4l2_ctrl_new_int_menu(hdl, &ar1335_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(ar1335_link_freqs) - 1,
						   mode->link_freq_idx,
						   ar1335_link_freqs);

	ar1335->vblank = v4l2_ctrl_new_std(hdl, &ar1335_ctrl_ops,
					   V4L2_CID_VBLANK,
					   mode->vts_min - mode->height,
					   AR1335_VTS_MAX - mode->height, 1,
					   mode->vts_def - mode->height);

	ar1335->hblank = v4l2_ctrl_new_std(hdl, &ar1335_ctrl_ops,
					   V4L2_CID_HBLANK,
					   mode->line_length_pix - mode->width,
					   mode->line_length_pix - mode->width,
					   1,
					   mode->line_length_pix - mode->width);

	exposure_max = mode->vts_def - AR1335_EXPOSURE_MARGIN;
	ar1335->exposure = v4l2_ctrl_new_std(hdl, &ar1335_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     AR1335_EXPOSURE_MIN, exposure_max, 1,
					     min(AR1335_EXPOSURE_DEFAULT, exposure_max));

	v4l2_ctrl_new_std(hdl, &ar1335_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  AR1335_GAIN_MIN, AR1335_GAIN_MAX, 1,
			  AR1335_GAIN_DEFAULT);

	ar1335->hflip = v4l2_ctrl_new_std(hdl, &ar1335_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	ar1335->vflip = v4l2_ctrl_new_std(hdl, &ar1335_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);

	/*
	 * Expose CAMERA_ORIENTATION and CAMERA_SENSOR_ROTATION from the device
	 * tree's `orientation` and `rotation` properties. libcamera reads both
	 * to decide how the image relates to the world - without them it warns
	 * that the driver needs fixing and falls back to assuming nothing.
	 */
	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret) {
		dev_err(&client->dev, "cannot parse device properties: %d\n", ret);
		goto err_free;
	}

	ret = v4l2_ctrl_new_fwnode_properties(hdl, &ar1335_ctrl_ops, &props);
	if (ret) {
		dev_err(&client->dev, "cannot create property controls: %d\n", ret);
		goto err_free;
	}

	if (hdl->error) {
		ret = hdl->error;
		dev_err(&client->dev, "control init failed: %d\n", ret);
		goto err_free;
	}

	if (ar1335->pixel_rate)
		ar1335->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (ar1335->link_freq)
		ar1335->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (ar1335->hblank)
		ar1335->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/*
	 * Flipping changes the Bayer order, which changes the media bus code,
	 * so the two flips have to be applied together with a format update.
	 */
	if (ar1335->hflip)
		ar1335->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	if (ar1335->vflip)
		ar1335->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	ar1335->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

/* Subdev pad ops --------------------------------------------------------- */

static int ar1335_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar1335 *ar1335 = to_ar1335(sd);

	if (code->index)
		return -EINVAL;

	code->code = ar1335_get_format_code(ar1335);

	return 0;
}

static int ar1335_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar1335 *ar1335 = to_ar1335(sd);

	if (fse->index >= ARRAY_SIZE(ar1335_supported_modes))
		return -EINVAL;

	if (fse->code != ar1335_get_format_code(ar1335))
		return -EINVAL;

	fse->min_width = ar1335_supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ar1335_supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void ar1335_update_pad_format(struct ar1335 *ar1335,
				     const struct ar1335_mode *mode,
				     struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = ar1335_get_format_code(ar1335);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int ar1335_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_format *fmt)
{
	struct ar1335 *ar1335 = to_ar1335(sd);
	const struct ar1335_mode *mode;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;
	int exposure_max;

	mode = v4l2_find_nearest_size(ar1335_supported_modes,
				      ARRAY_SIZE(ar1335_supported_modes),
				      width, height,
				      fmt->format.width, fmt->format.height);

	format = v4l2_subdev_state_get_format(state, 0);
	ar1335_update_pad_format(ar1335, mode, format);
	fmt->format = *format;

	crop = v4l2_subdev_state_get_crop(state, 0);
	*crop = mode->crop;

	if (fmt->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	__v4l2_ctrl_modify_range(ar1335->pixel_rate, mode->pixel_rate,
				 mode->pixel_rate, 1, mode->pixel_rate);
	__v4l2_ctrl_s_ctrl(ar1335->link_freq, mode->link_freq_idx);

	__v4l2_ctrl_modify_range(ar1335->hblank,
				 mode->line_length_pix - mode->width,
				 mode->line_length_pix - mode->width, 1,
				 mode->line_length_pix - mode->width);

	__v4l2_ctrl_modify_range(ar1335->vblank,
				 mode->vts_min - mode->height,
				 AR1335_VTS_MAX - mode->height, 1,
				 mode->vts_def - mode->height);
	__v4l2_ctrl_s_ctrl(ar1335->vblank, mode->vts_def - mode->height);

	exposure_max = mode->vts_def - AR1335_EXPOSURE_MARGIN;
	__v4l2_ctrl_modify_range(ar1335->exposure, AR1335_EXPOSURE_MIN,
				 exposure_max, 1,
				 min(ar1335->exposure->val, exposure_max));

	return 0;
}

static int ar1335_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = AR1335_NATIVE_WIDTH;
		sel->r.height = AR1335_NATIVE_HEIGHT;
		return 0;
	}

	return -EINVAL;
}

static int ar1335_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = ar1335_supported_modes[0].width,
			.height = ar1335_supported_modes[0].height,
		},
	};

	return ar1335_set_pad_format(sd, state, &fmt);
}

/* Streaming -------------------------------------------------------------- */

static int ar1335_start_streaming(struct ar1335 *ar1335,
				  struct v4l2_subdev_state *state)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar1335->sd);
	const struct v4l2_mbus_framefmt *fmt;
	const struct ar1335_mode *mode;
	u64 read_mode;
	int ret;

	fmt = v4l2_subdev_state_get_format(state, 0);
	mode = ar1335_mode_for_format(fmt);

	/*
	 * The mode tables open with RESET_REGISTER (0x301A = 0x0219), which is a
	 * software reset. The sensor needs time to complete it before it will
	 * accept anything else - writing straight on gets the next registers
	 * NAKed, the mode ends up only partially programmed, and streaming then
	 * fails intermittently with CSI buffer timeouts.
	 *
	 * So write the reset on its own, wait, then write the remainder. This
	 * matches the vendor driver, which special-cases exactly this register
	 * at index 0 with a 100 ms delay.
	 */
	if (mode->num_regs && mode->reg_list[0].reg == AR1335_REG_RESET) {
		ret = cci_multi_reg_write(ar1335->regmap, mode->reg_list, 1, NULL);
		if (ret) {
			dev_err(&client->dev, "failed to write reset: %d\n", ret);
			return ret;
		}
		msleep(100);

		ret = cci_multi_reg_write(ar1335->regmap, mode->reg_list + 1,
					  mode->num_regs - 1, NULL);
	} else {
		ret = cci_multi_reg_write(ar1335->regmap, mode->reg_list,
					  mode->num_regs, NULL);
	}
	if (ret) {
		dev_err(&client->dev, "failed to write mode registers: %d\n", ret);
		return ret;
	}

	/* Let the internal sequencer settle before the control writes land. */
	msleep(20);

	/*
	 * The mode sequence programs READ_MODE with this mode's binning setup.
	 * Cache it, minus any flip bits, so the flip controls can be OR'd on
	 * without disturbing the binning configuration.
	 */
	ret = cci_read(ar1335->regmap, AR1335_REG_READ_MODE, &read_mode, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to read READ_MODE: %d\n", ret);
		return ret;
	}
	ar1335->read_mode = read_mode &
			    ~(AR1335_READ_MODE_MIRROR | AR1335_READ_MODE_FLIP);

	ret = __v4l2_ctrl_handler_setup(&ar1335->ctrl_handler);
	if (ret)
		return ret;

	/* Clear the soft reset and set stream, matching the vendor's 0x021C. */
	return cci_update_bits(ar1335->regmap, AR1335_REG_RESET,
			       AR1335_RESET_RESET | AR1335_RESET_STREAM,
			       AR1335_RESET_STREAM, NULL);
}

static int ar1335_stop_streaming(struct ar1335 *ar1335)
{
	return cci_update_bits(ar1335->regmap, AR1335_REG_RESET,
			       AR1335_RESET_STREAM, 0, NULL);
}

static int ar1335_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar1335 *ar1335 = to_ar1335(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret)
			goto out;

		ret = ar1335_start_streaming(ar1335, state);
		if (ret) {
			pm_runtime_mark_last_busy(&client->dev);
			pm_runtime_put_autosuspend(&client->dev);
		}
	} else {
		ar1335_stop_streaming(ar1335);
		pm_runtime_mark_last_busy(&client->dev);
		pm_runtime_put_autosuspend(&client->dev);
	}

out:
	v4l2_subdev_unlock_state(state);
	return ret;
}

/* Subdev plumbing -------------------------------------------------------- */

static const struct v4l2_subdev_core_ops ar1335_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops ar1335_video_ops = {
	.s_stream = ar1335_s_stream,
};

static const struct v4l2_subdev_pad_ops ar1335_pad_ops = {
	.enum_mbus_code = ar1335_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ar1335_set_pad_format,
	.get_selection = ar1335_get_selection,
	.enum_frame_size = ar1335_enum_frame_size,
};

static const struct v4l2_subdev_ops ar1335_subdev_ops = {
	.core = &ar1335_core_ops,
	.video = &ar1335_video_ops,
	.pad = &ar1335_pad_ops,
};

static const struct v4l2_subdev_internal_ops ar1335_internal_ops = {
	.init_state = ar1335_init_state,
};

/* Probe ------------------------------------------------------------------ */

static int ar1335_identify_module(struct ar1335 *ar1335, bool verbose)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ar1335->sd);
	u64 val;
	int ret;

	ret = cci_read(ar1335->regmap, AR1335_REG_MODEL_ID, &val, NULL);
	if (ret) {
		if (verbose)
			dev_err(&client->dev,
				"no response from sensor at 0x%02x on %s: %d\n",
				client->addr, client->adapter->name, ret);
		return ret;
	}

	if (val != AR1335_MODEL_ID) {
		dev_err(&client->dev,
			"wrong model ID 0x%04llx (expected 0x%04x)\n",
			val, AR1335_MODEL_ID);
		return -ENODEV;
	}

	dev_info(&client->dev, "AR1335 found, model ID 0x%04llx\n", val);

	return 0;
}

static int ar1335_check_hwcfg(struct device *dev, struct ar1335 *ar1335)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *endpoint, *fwnode = dev_fwnode(dev);
	unsigned int i, j;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &bus_cfg);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse endpoint\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != 4) {
		ret = dev_err_probe(dev, -EINVAL,
				    "only 4 data lanes are supported, got %u\n",
				    bus_cfg.bus.mipi_csi2.num_data_lanes);
		goto done;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(dev, -EINVAL,
				    "no link frequencies defined\n");
		goto done;
	}

	/* Every link frequency the modes need must be offered by the receiver. */
	for (i = 0; i < ARRAY_SIZE(ar1335_link_freqs); i++) {
		for (j = 0; j < bus_cfg.nr_of_link_frequencies; j++)
			if (bus_cfg.link_frequencies[j] == ar1335_link_freqs[i])
				break;

		if (j == bus_cfg.nr_of_link_frequencies) {
			ret = dev_err_probe(dev, -EINVAL,
					    "link frequency %lld not supported\n",
					    ar1335_link_freqs[i]);
			goto done;
		}
	}

done:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int ar1335_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ar1335 *ar1335;
	unsigned int i, attempt;
	u32 xclk_freq;
	int ret;

	ar1335 = devm_kzalloc(dev, sizeof(*ar1335), GFP_KERNEL);
	if (!ar1335)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&ar1335->sd, client, &ar1335_subdev_ops);
	ar1335->sd.internal_ops = &ar1335_internal_ops;

	ret = ar1335_check_hwcfg(dev, ar1335);
	if (ret)
		return ret;

	ar1335->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ar1335->regmap))
		return dev_err_probe(dev, PTR_ERR(ar1335->regmap),
				     "failed to init CCI regmap\n");

	ar1335->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(ar1335->xclk))
		return dev_err_probe(dev, PTR_ERR(ar1335->xclk),
				     "failed to get EXTCLK\n");

	/*
	 * The mode tables are calculated for a 24 MHz EXTCLK. A camera module
	 * with its own oscillator appears as a fixed-clock and must already be
	 * right; a programmable source (the RP1 GPCLK, when the Pi supplies the
	 * clock itself) can be retuned here.
	 */
	xclk_freq = clk_get_rate(ar1335->xclk);
	if (abs_diff(xclk_freq, AR1335_XCLK_FREQ) > AR1335_XCLK_TOLERANCE) {
		ret = clk_set_rate(ar1335->xclk, AR1335_XCLK_FREQ);
		if (ret)
			return dev_err_probe(dev, ret,
					     "EXTCLK is %u Hz and cannot be set to %u Hz\n",
					     xclk_freq, AR1335_XCLK_FREQ);

		xclk_freq = clk_get_rate(ar1335->xclk);
		if (abs_diff(xclk_freq, AR1335_XCLK_FREQ) > AR1335_XCLK_TOLERANCE)
			return dev_err_probe(dev, -EINVAL,
					     "EXTCLK settled at %u Hz, need %u Hz\n",
					     xclk_freq, AR1335_XCLK_FREQ);
	}

	dev_info(dev, "EXTCLK %u Hz\n", xclk_freq);

	for (i = 0; i < AR1335_NUM_SUPPLIES; i++)
		ar1335->supplies[i].supply = ar1335_supply_names[i];

	ret = devm_regulator_bulk_get(dev, AR1335_NUM_SUPPLIES, ar1335->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	/* Optional - this board gates the module with the VANA regulator. */
	ar1335->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(ar1335->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ar1335->reset_gpio),
				     "failed to get reset GPIO\n");

	/*
	 * Power-cycle and retry rather than failing permanently. A module whose
	 * rails or oscillator settle slowly would otherwise need a reboot to be
	 * picked up, which is exactly the failure a zero-intervention payload
	 * cannot afford.
	 */
	for (attempt = 1; ; attempt++) {
		bool last = attempt >= max(probe_retries, 1U);

		ret = ar1335_power_on(dev);
		if (ret)
			return ret;

		ret = ar1335_identify_module(ar1335, last);
		if (!ret)
			break;

		ar1335_power_off(dev);

		if (last) {
			dev_err(dev, "sensor did not respond after %u attempts\n",
				attempt);
			return ret;
		}

		dev_info(dev, "no response (attempt %u/%u), power-cycling\n",
			 attempt, max(probe_retries, 1U));
		msleep(power_cycle_off_ms);
	}

	if (attempt > 1)
		dev_info(dev, "sensor answered on attempt %u\n", attempt);

	ret = ar1335_init_controls(ar1335);
	if (ret)
		goto err_power_off;

	ar1335->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	ar1335->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ar1335->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&ar1335->sd.entity, 1, &ar1335->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init media entity\n");
		goto err_free_ctrls;
	}

	ar1335->sd.state_lock = ar1335->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&ar1335->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to finalize subdev\n");
		goto err_media_entity;
	}

	/*
	 * Enable runtime PM with autosuspend before registering, so the sensor
	 * powers down between captures - this system is power constrained.
	 */
	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = v4l2_async_register_subdev_sensor(&ar1335->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register subdev\n");
		goto err_pm;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	v4l2_subdev_cleanup(&ar1335->sd);
err_media_entity:
	media_entity_cleanup(&ar1335->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&ar1335->ctrl_handler);
err_power_off:
	ar1335_power_off(dev);
	return ret;
}

static void ar1335_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar1335 *ar1335 = to_ar1335(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&ar1335->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		ar1335_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct of_device_id ar1335_of_match[] = {
	{ .compatible = "onsemi,ar1335" },
	{ }
};
MODULE_DEVICE_TABLE(of, ar1335_of_match);

static const struct dev_pm_ops ar1335_pm_ops = {
	SET_RUNTIME_PM_OPS(ar1335_power_off, ar1335_power_on, NULL)
};

static struct i2c_driver ar1335_i2c_driver = {
	.driver = {
		.name = "ar1335",
		.of_match_table = ar1335_of_match,
		.pm = &ar1335_pm_ops,
	},
	.probe = ar1335_probe,
	.remove = ar1335_remove,
};

module_i2c_driver(ar1335_i2c_driver);

MODULE_AUTHOR("radcam1 project");
MODULE_DESCRIPTION("onsemi AR1335 sensor driver");
MODULE_LICENSE("GPL v2");
