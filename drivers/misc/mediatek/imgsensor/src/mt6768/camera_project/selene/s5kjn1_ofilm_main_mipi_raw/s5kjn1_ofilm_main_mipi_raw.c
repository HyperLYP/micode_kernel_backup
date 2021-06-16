/*****************************************************************************
 *
 *Filename:
 *---------
 *   S5Kjn1mipiraw_sensor.c
 *
 *Project:
 *--------
 *   ALPS MT6735
 *
 *Description:
 *------------
 *   Source code of Sensor driver
 *------------------------------------------------------------------------------
 *Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_typedef.h"
#include "s5kjn1_ofilm_main_mipi_raw.h"

#define MULTI_WRITE 1

#define PFX "S5KJN1_camera_sensor"

#define LOG_DBG(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)    pr_info(PFX "[%s] " format, __FUNCTION__, ##args)
#define LOG_ERR(format, args...)    pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
static struct imgsensor_info_struct imgsensor_info = {
	.sensor_id = S5KJN1_OFILM_MAIN_SENSOR_ID,
	.checksum_value = 0xdb9c643,
	.pre = {
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 40,
		.mipi_pixel_rate = 494400000,
		.max_framerate = 300,
	},
	.cap = {
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 40,
		.mipi_pixel_rate = 494400000,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 482000000,
		.linelength = 5024,
		.framelength = 3194,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4000,
		.grabwindow_height = 3000,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 460800000,
		.max_framerate = 300,
	},
	.normal_video = {
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 2296,
		.mipi_data_lp2hs_settle_dc = 40,
		.mipi_pixel_rate = 513600000,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 600000000,
		.linelength = 2096,
		.framelength = 2380,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 19,
		.mipi_pixel_rate = 640000000,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 492000000,
		.linelength = 5024,
		.framelength = 816,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 85,
		.mipi_pixel_rate = 145600000,
		.max_framerate = 1200,
	},
	.custom2 = {
		.pclk = 560000000,
		.linelength = 5910,
		.framelength = 3156,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 4080,
		.grabwindow_height = 3072,
		.mipi_data_lp2hs_settle_dc = 40,
		.mipi_pixel_rate = 494400000,
		.max_framerate = 300,
	},
	.margin = 5,
	.min_shutter = 5,
	.max_frame_length = 0xFFFF,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 0,
	.ae_ispGain_delay_frame = 2,
	.frame_time_delay_frame = 1,
	.ihdr_support = 0,
	.ihdr_le_firstline = 0,
	.sensor_mode_num = 7,
	.cap_delay_frame = 3,
	.pre_delay_frame = 3,
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
	.custom2_delay_frame = 1,
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2,
	.mipi_settle_delay_mode = 1,
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_4_LANE,
	.i2c_addr_table = {0x20, 0x5A, 0xff},
	.i2c_speed = 400,
};

static struct imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,
	.sensor_mode = IMGSENSOR_MODE_INIT,
	.shutter = 0x0200,
	.gain = 0x0100,
	.dummy_pixel = 0,
	.dummy_line = 0,
	.current_fps = 0,
	.autoflicker_en = KAL_FALSE,
	.test_pattern = KAL_FALSE,
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,
	.ihdr_en = KAL_FALSE,
	.i2c_write_id = 0x5A,
//cxc long exposure >
	.current_ae_effective_frame = 2,
//cxc long exposure <
};

static struct SENSOR_VC_INFO_STRUCT SENSOR_VC_INFO[4] = {
    /* preview mode setting */
	{
		0x02, 0x0A, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2B, 0x0ff0, 0x0c00, 0x01, 0x00, 0x0000, 0x0000,
		0x01, 0x30, 0x027C, 0x0BF0, 0x03, 0x00, 0x0000, 0x0000
	},
    /* capture mode setting */
	{
		0x02, 0x0A, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2B, 0x0ff0, 0x0c00, 0x01, 0x00, 0x0000, 0x0000,
		0x01, 0x30, 0x027C, 0x0BF0, 0x03, 0x00, 0x0000, 0x0000
	},
    /* normal_video mode setting */
	{
		0x02, 0x0A, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2B, 0x0ff0, 0x08f8, 0x01, 0x00, 0x0000, 0x0000,
		0x01, 0x30, 0x027C, 0x08f0, 0x03, 0x00, 0x0000, 0x0000
	},
	/* custom2 mode setting */
	{
		0x02, 0x0A, 0x00, 0x08, 0x40, 0x00,
		0x00, 0x2B, 0x0ff0, 0x0c00, 0x01, 0x00, 0x0000, 0x0000,
		0x01, 0x30, 0x027C, 0x0BF0, 0x03, 0x00, 0x0000, 0x0000
	}
};

static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[6] = {
	//preview
    { 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0, 0, 4080, 3072, 0,   0, 4080, 3072},
	//capture
    { 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0, 0, 4080, 3072, 0,   0, 4080, 3072},
	//normal_video
    { 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0, 0, 4080, 3072, 0, 388, 4080, 2296},
    //hs_video
    { 8160, 6144, 1520, 1632, 5120, 2880, 1280,  720,  0, 0, 1280,  720, 0,   0, 1280,  720},
	//slim_video
    { 4080, 3072,   80,  420, 3840, 2160, 1280,  720,  0, 0, 1280,  720, 0,   0, 1280,  720},
	//custom2
    { 8160, 6144,    0,    0, 8160, 6144, 4080, 3072,  0, 0, 4080, 3072, 0,   0, 4080, 3072},
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_info = {
	.i4OffsetX = 8,
	.i4OffsetY = 8,
	.i4PitchX = 8,
	.i4PitchY = 8,
	.i4PairNum = 4,
	.i4SubBlkW = 8,
	.i4SubBlkH = 2,
	.i4PosL = {{8, 8}, {10, 11}, {14, 12}, {12, 15}
	},
	.i4PosR = {{9, 8}, {11, 11}, {15, 12}, {13, 15}
	},
	.iMirrorFlip = 0,
	.i4BlockNumX = 508,
	.i4BlockNumY = 382,
};

static struct SET_PD_BLOCK_INFO_T imgsensor_pd_normal_video_info = {
 .i4OffsetX = 8,
 .i4OffsetY = 4,
 .i4PitchX = 8,
 .i4PitchY = 8,
 .i4PairNum = 4,
 .i4SubBlkW = 8,
 .i4SubBlkH = 2,
 .i4PosL = {{8, 4}, {10, 7}, {14, 8}, {12, 11}
 },
 .i4PosR = {{9, 4}, {11, 7}, {15, 8}, {13, 11}
 },
 .iMirrorFlip = 0,
 .i4BlockNumX = 508,
 .i4BlockNumY = 286,
};


#if MULTI_WRITE
#define I2C_BUFFER_LEN 225
#else
#define I2C_BUFFER_LEN 3
#endif

static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1,
				imgsensor.i2c_write_id);
	return get_byte;
}

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xFF) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1,
				imgsensor.i2c_write_id);
	return get_byte;
}

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {
	(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[4] = {
		(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8),
		(char)(para & 0xFF)
	};

	iWriteRegI2C(pusendcmd, 4, imgsensor.i2c_write_id);
}

static void set_dummy(void)
{
	LOG_DBG("dummyline = %d, dummypixels = %d\n", imgsensor.dummy_line,
			imgsensor.dummy_pixel);

	write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	write_cmos_sensor(0x0342, imgsensor.line_length & 0xFFFF);
}

static void set_max_framerate(UINT16 framerate, kal_bool min_framelength_en)
{
	kal_uint32 frame_length = imgsensor.frame_length;

	LOG_DBG("framerate = %d, min framelength should enable(%d)\n",
			framerate, min_framelength_en);
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length =
		(frame_length >
		 imgsensor.min_frame_length)
		  ? frame_length : imgsensor.min_frame_length;
	imgsensor.dummy_line =
		imgsensor.frame_length - imgsensor.min_frame_length;

	if (imgsensor.frame_length > imgsensor_info.max_frame_length) {
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line =
			imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}

#if 1
static void check_streamoff(void)
{
	unsigned int i = 0;
	int timeout = (10000 / imgsensor.current_fps) + 1;

	mdelay(3);
	for (i = 0; i < timeout; i++) {
		if (read_cmos_sensor_8(0x0005) != 0xFF)
			mdelay(1);
		else
			break;
	}
	LOG_INF(" check_streamoff exit!\n");
}

static kal_uint32 streaming_control(kal_bool enable)
{
	LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);
	if (enable) {
		write_cmos_sensor_byte(0x0100, 0X01);
	} else {
		write_cmos_sensor_byte(0x0100, 0x00);
		check_streamoff();
	}
	return ERROR_NONE;
}
#endif
#if 0
static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;

	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter =
		(shutter < imgsensor_info.min_shutter)
		  ? imgsensor_info.min_shutter : shutter;
	shutter =
		(shutter >
		 (imgsensor_info.max_frame_length -
		 imgsensor_info.margin)) ? (imgsensor_info.max_frame_length
				 - imgsensor_info.margin) : shutter;
	if (imgsensor.autoflicker_en) {
		realtime_fps =
			imgsensor.pclk / imgsensor.line_length * 10 /
			imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
	} else {

		write_cmos_sensor(0x0340, imgsensor.frame_length & 0xFFFF);
	}

	write_cmos_sensor(0x0202, (shutter) & 0xFFFF);
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,
			imgsensor.frame_length);

}
#endif

//cxc long exposure >
static bool bNeedSetNormalMode = KAL_FALSE;
#define SHUTTER_1		94750//30396
#define SHUTTER_2		189501//60792
#define SHUTTER_4		379003//56049
#define SHUTTER_8		758006//46563
#define SHUTTER_16		1516012//27591
#define SHUTTER_32		2842524

static void check_output_stream_off(void)
{
	kal_uint16 read_count = 0, read_register0005_value = 0;

	for (read_count = 0; read_count <= 4; read_count++) {
		read_register0005_value = read_cmos_sensor_8(0x0005);

		if (read_register0005_value == 0xff)
			break;
		mdelay(50);

		if (read_count == 4)
			LOG_INF("cxc stream off error\n");
	}

}
//cxc long exposure <

/*************************************************************************
*FUNCTION
*  set_shutter
*
*DESCRIPTION
*  This function set e-shutter of sensor to change exposure time.
*
*PARAMETERS
*  iShutter : exposured lines
*
*RETURNS
*  None
*
*GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint64 shutter)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_uint64 pre_shutter = 2877;

	LOG_DBG("enter  shutter = %d\n", shutter);

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	if (shutter < 94750) {
		if (bNeedSetNormalMode) {
			LOG_DBG("exit long shutter\n");
			write_cmos_sensor(0x6028, 0x4000);
			write_cmos_sensor_byte(0x0100, 0x00); //stream off
			write_cmos_sensor(0x0334, 0x0000);
			write_cmos_sensor(0x0E00, 0x0000);
			write_cmos_sensor(0x0E04, 0x0000);
			write_cmos_sensor(0x0E10, 0x0000);
			write_cmos_sensor(0x0E12, 0x0000);
			write_cmos_sensor(0x0E14, 0x0000);
			write_cmos_sensor(0x0E16, 0x0000);
			write_cmos_sensor(0x0704, 0x0000);
			write_cmos_sensor_byte(0x0100, 0x01); //stream on
			bNeedSetNormalMode = KAL_FALSE;
			imgsensor.current_ae_effective_frame = 2;
		}
		pre_shutter = shutter;

		spin_lock(&imgsensor_drv_lock);
		if (shutter > imgsensor.min_frame_length -
				imgsensor_info.margin)
			imgsensor.frame_length = shutter +
				imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length =
				imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		shutter =
			(shutter < imgsensor_info.min_shutter)
			 ? imgsensor_info.min_shutter : shutter;
		shutter =
			(shutter >
			 (imgsensor_info.max_frame_length -
			 imgsensor_info.margin)) ?
			 (imgsensor_info.max_frame_length -
			 imgsensor_info.margin) : shutter;
		if (imgsensor.autoflicker_en) {
			realtime_fps =
				imgsensor.pclk / imgsensor.line_length * 10 /
				imgsensor.frame_length;
			if (realtime_fps >= 297 && realtime_fps <= 305)
				set_max_framerate(296, 0);
			else if (realtime_fps >= 147 && realtime_fps <= 150)
				set_max_framerate(146, 0);
			else {
				write_cmos_sensor(0x0340,
					  imgsensor.frame_length & 0xFFFF);
			}
		} else {

			write_cmos_sensor(0x0340,
					imgsensor.frame_length & 0xFFFF);
		}

		write_cmos_sensor(0X0202, shutter & 0xFFFF);

	} else {
		LOG_INF("enter long shutter\n");
		bNeedSetNormalMode = KAL_TRUE;
		imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
		imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
		imgsensor.current_ae_effective_frame = 2;

		write_cmos_sensor(0x6028, 0x4000);
		write_cmos_sensor_byte(0x0100, 0x00); //stream off
		check_output_stream_off();
		write_cmos_sensor(0x0334, 0x0001);
		write_cmos_sensor(0x0E00, 0x0201);
		write_cmos_sensor(0x0E04, 0x0003);
		write_cmos_sensor(0x0E10, pre_shutter); //1st frame
		write_cmos_sensor(0x0E12, imgsensor.gain); //aGain 1st frame
		switch (shutter) {
		case SHUTTER_1:
			LOG_INF("enter SHUTTER_1\n");
			write_cmos_sensor(0x0E14, 0xB911); //2nd frame
			write_cmos_sensor(0x0E16, 0x0020); //aGain 2nd frame
			write_cmos_sensor(0x0704, 0x0100); //shifter for shutter
			break;
		case SHUTTER_2:
			LOG_INF("enter SHUTTER_2\n");
			write_cmos_sensor(0x0E14, 0xB911); //2nd frame
			write_cmos_sensor(0x0E16, 0x0020); //aGain 2nd frame
			write_cmos_sensor(0x0704, 0x0200); //shifter for shutter
			break;
		case SHUTTER_4:
			LOG_INF("enter SHUTTER_4\n");
			write_cmos_sensor(0x0E14, 0xB911); //2nd frame
			write_cmos_sensor(0x0E16, 0x0020); //aGain 2nd frame
			write_cmos_sensor(0x0704, 0x0300); //shifter for shutter
			break;
		case SHUTTER_8:
			LOG_INF("enter SHUTTER_8\n");
			write_cmos_sensor(0x0E14, 0xB911); //2nd frame
			write_cmos_sensor(0x0E16, 0x0020); //aGain 2nd frame
			write_cmos_sensor(0x0704, 0x0400); //shifter for shutter
			break;
		case SHUTTER_16:
			LOG_INF("enter SHUTTER_16\n");
			write_cmos_sensor(0x0E14, 0xB911); //2nd frame
			write_cmos_sensor(0x0E16, 0x0020); //aGain 2nd frame
			write_cmos_sensor(0x0704, 0x0500); //shifter for shutter
			break;
		case SHUTTER_32:
			LOG_INF("enter SHUTTER_32\n");
			write_cmos_sensor(0x0E14, 0xB911); //2nd frame
			write_cmos_sensor(0x0E16, 0x0020); //aGain 2nd frame
			write_cmos_sensor(0x0704, 0x0600); //shifter for shutter
			break;
		default:
			LOG_INF("enter default\n");
			write_cmos_sensor(0x0E14, ((shutter * 0xB911) /
				SHUTTER_1)); //2nd frame
			write_cmos_sensor(0x0E16, 0x0020); //aGain 2nd frame
			write_cmos_sensor(0x0704, 0x0100); //shifter for shutter
			break;
		}

		write_cmos_sensor_byte(0x0100, 0x01); //stream on
		pre_shutter = 0;
	}

	LOG_DBG("Exit! shutter =%d, framelength =%d\n", shutter,
			imgsensor.frame_length);
}

/*************************************************************************
*FUNCTION
*  set_shutter_frame_length
*
*DESCRIPTION
*  for frame &3A sync
*
*************************************************************************/

static void
set_shutter_frame_length(kal_uint16 shutter, kal_uint16 frame_length)
{
	unsigned long flags;
	kal_uint16 realtime_fps = 0;
	kal_int32 dummy_line = 0;
	//kal_bool autoflicker_closed = KAL_FALSE;

	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

	spin_lock(&imgsensor_drv_lock);

	if(frame_length > 1)
		dummy_line = frame_length - imgsensor.frame_length;
	imgsensor.frame_length = imgsensor.frame_length + dummy_line;
	imgsensor.min_frame_length = imgsensor.frame_length;

	if (shutter > imgsensor.frame_length - imgsensor_info.margin)
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter =
		(shutter < imgsensor_info.min_shutter)
		 ? imgsensor_info.min_shutter : shutter;
	shutter =
		(shutter >
		 (imgsensor_info.max_frame_length -
		  imgsensor_info.margin)) ? (imgsensor_info.max_frame_length -
			 imgsensor_info.margin) : shutter;

	//if (autoflicker_closed) {
	if(imgsensor.autoflicker_en) {
		realtime_fps =
			imgsensor.pclk / imgsensor.line_length * 10 /
			imgsensor.frame_length;
		if (realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296, 0);
		else if (realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146, 0);
		else
			write_cmos_sensor(0x0340, imgsensor.frame_length);
	} else
		write_cmos_sensor(0x0340, imgsensor.frame_length);


	write_cmos_sensor(0x0202, imgsensor.shutter);
	LOG_INF
	("Exit! shutter %d framelength %d/%d dummy_line=%d auto_extend=%d\n",
	 shutter, imgsensor.frame_length,
	  frame_length, dummy_line, read_cmos_sensor(0x0350));

}

static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;

	reg_gain = gain / 2;

	return (kal_uint16) reg_gain;
}

/*************************************************************************
*FUNCTION
*  set_gain
*
*DESCRIPTION
*  This function is to set global gain to sensor.
*
*PARAMETERS
*  iGain : sensor global gain(base: 0x40)
*
*RETURNS
*  the actually gain set to sensor.
*
*GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{

	kal_uint16 reg_gain;

	if (gain < BASEGAIN || gain > 64 * BASEGAIN) {
		LOG_ERR("Error gain setting");
		if (gain < BASEGAIN)
			gain = BASEGAIN;
		else if (gain > 64 * BASEGAIN)
			gain = 64 * BASEGAIN;
	}
	reg_gain = gain2reg(gain);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain;
	spin_unlock(&imgsensor_drv_lock);
	LOG_DBG("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);
	write_cmos_sensor(0x0204, (reg_gain & 0xFFFF));
	return gain;
}

#if 0
static void
ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n", le, se, gain);
	if (imgsensor.ihdr_en) {
		spin_lock(&imgsensor_drv_lock);
		if (le > imgsensor.min_frame_length - imgsensor_info.margin)
			imgsensor.frame_length = le + imgsensor_info.margin;
		else
			imgsensor.frame_length = imgsensor.min_frame_length;
		if (imgsensor.frame_length > imgsensor_info.max_frame_length)
			imgsensor.frame_length =
				imgsensor_info.max_frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (le < imgsensor_info.min_shutter)
			le = imgsensor_info.min_shutter;
		if (se < imgsensor_info.min_shutter)
			se = imgsensor_info.min_shutter;

		write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);
		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		write_cmos_sensor(0x3512, (se << 4) & 0xFF);
		write_cmos_sensor(0x3511, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3510, (se >> 12) & 0x0F);
		set_gain(gain);
	}
}
#endif
static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.mirror = image_mirror;
	spin_unlock(&imgsensor_drv_lock);
	switch (image_mirror) {
	case IMAGE_NORMAL:
        write_cmos_sensor_byte(0x0101, 0x00);
		break;
	case IMAGE_H_MIRROR:
		write_cmos_sensor_byte(0x0101, 0x01);
		break;
	case IMAGE_V_MIRROR:
		write_cmos_sensor_byte(0x0101, 0x02);
		break;
	case IMAGE_HV_MIRROR:
        write_cmos_sensor_byte(0x0101, 0x03);
		break;
	default:
		LOG_INF("Error image_mirror setting\n");
		break;
	}
}

static void sensor_init(void)
{
	LOG_INF("sensor_init start\n");
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0000, 0x0001);
	write_cmos_sensor(0x0000, 0x38e1);
	write_cmos_sensor(0x001e, 0x0005);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6010, 0x0001);
	mdelay(10);
	write_cmos_sensor(0x6226, 0x0001);
	mdelay(15);
	write_cmos_sensor(0x6028, 0x2400);
	write_cmos_sensor(0x602A, 0x7700);
	write_cmos_sensor(0x6F12, 0x1753);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0x6385);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9386);
	write_cmos_sensor(0x6F12, 0xC701);
	write_cmos_sensor(0x6F12, 0xB7B7);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x3777);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0xC77F);
	write_cmos_sensor(0x6F12, 0x1307);
	write_cmos_sensor(0x6F12, 0x07C7);
	write_cmos_sensor(0x6F12, 0x958F);
	write_cmos_sensor(0x6F12, 0x2328);
	write_cmos_sensor(0x6F12, 0xD774);
	write_cmos_sensor(0x6F12, 0x231A);
	write_cmos_sensor(0x6F12, 0xF774);
	write_cmos_sensor(0x6F12, 0x012F);
	write_cmos_sensor(0x6F12, 0xB777);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x1307);
	write_cmos_sensor(0x6F12, 0xD003);
	write_cmos_sensor(0x6F12, 0x23A0);
	write_cmos_sensor(0x6F12, 0xE774);
	write_cmos_sensor(0x6F12, 0x1753);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0x6700);
	write_cmos_sensor(0x6F12, 0x2384);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x8547);
	write_cmos_sensor(0x6F12, 0x6310);
	write_cmos_sensor(0x6F12, 0xF506);
	write_cmos_sensor(0x6F12, 0x3786);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9306);
	write_cmos_sensor(0x6F12, 0x4601);
	write_cmos_sensor(0x6F12, 0x83C7);
	write_cmos_sensor(0x6F12, 0x4600);
	write_cmos_sensor(0x6F12, 0xA1CB);
	write_cmos_sensor(0x6F12, 0xB737);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x83A7);
	write_cmos_sensor(0x6F12, 0x875C);
	write_cmos_sensor(0x6F12, 0x83D6);
	write_cmos_sensor(0x6F12, 0x2600);
	write_cmos_sensor(0x6F12, 0x83D7);
	write_cmos_sensor(0x6F12, 0x271E);
	write_cmos_sensor(0x6F12, 0x13D7);
	write_cmos_sensor(0x6F12, 0x2700);
	write_cmos_sensor(0x6F12, 0xB707);
	write_cmos_sensor(0x6F12, 0x0140);
	write_cmos_sensor(0x6F12, 0x83D5);
	write_cmos_sensor(0x6F12, 0x27F0);
	write_cmos_sensor(0x6F12, 0x8357);
	write_cmos_sensor(0x6F12, 0x4601);
	write_cmos_sensor(0x6F12, 0x1306);
	write_cmos_sensor(0x6F12, 0xE7FF);
	write_cmos_sensor(0x6F12, 0xB697);
	write_cmos_sensor(0x6F12, 0x8D8F);
	write_cmos_sensor(0x6F12, 0xC207);
	write_cmos_sensor(0x6F12, 0xC183);
	write_cmos_sensor(0x6F12, 0x9396);
	write_cmos_sensor(0x6F12, 0x0701);
	write_cmos_sensor(0x6F12, 0xC186);
	write_cmos_sensor(0x6F12, 0x635F);
	write_cmos_sensor(0x6F12, 0xD600);
	write_cmos_sensor(0x6F12, 0x8907);
	write_cmos_sensor(0x6F12, 0x998F);
	write_cmos_sensor(0x6F12, 0x9396);
	write_cmos_sensor(0x6F12, 0x0701);
	write_cmos_sensor(0x6F12, 0xC186);
	write_cmos_sensor(0x6F12, 0x9397);
	write_cmos_sensor(0x6F12, 0x0601);
	write_cmos_sensor(0x6F12, 0xC183);
	write_cmos_sensor(0x6F12, 0x37B7);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2311);
	write_cmos_sensor(0x6F12, 0xF7A0);
	write_cmos_sensor(0x6F12, 0x8280);
	write_cmos_sensor(0x6F12, 0xE3D8);
	write_cmos_sensor(0x6F12, 0x06FE);
	write_cmos_sensor(0x6F12, 0xBA97);
	write_cmos_sensor(0x6F12, 0xF917);
	write_cmos_sensor(0x6F12, 0xCDB7);
	write_cmos_sensor(0x6F12, 0xB717);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x07CA);
	write_cmos_sensor(0x6F12, 0xAA97);
	write_cmos_sensor(0x6F12, 0x3387);
	write_cmos_sensor(0x6F12, 0xB700);
	write_cmos_sensor(0x6F12, 0x8D8F);
	write_cmos_sensor(0x6F12, 0x83C5);
	write_cmos_sensor(0x6F12, 0xD705);
	write_cmos_sensor(0x6F12, 0xB747);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9386);
	write_cmos_sensor(0x6F12, 0x078F);
	write_cmos_sensor(0x6F12, 0x83D7);
	write_cmos_sensor(0x6F12, 0xE670);
	write_cmos_sensor(0x6F12, 0x0905);
	write_cmos_sensor(0x6F12, 0x0347);
	write_cmos_sensor(0x6F12, 0xC705);
	write_cmos_sensor(0x6F12, 0x630B);
	write_cmos_sensor(0x6F12, 0xF500);
	write_cmos_sensor(0x6F12, 0x8567);
	write_cmos_sensor(0x6F12, 0xB697);
	write_cmos_sensor(0x6F12, 0x03A6);
	write_cmos_sensor(0x6F12, 0x8794);
	write_cmos_sensor(0x6F12, 0xB306);
	write_cmos_sensor(0x6F12, 0xB700);
	write_cmos_sensor(0x6F12, 0xB296);
	write_cmos_sensor(0x6F12, 0x23A4);
	write_cmos_sensor(0x6F12, 0xD794);
	write_cmos_sensor(0x6F12, 0x2207);
	write_cmos_sensor(0x6F12, 0x3305);
	write_cmos_sensor(0x6F12, 0xB700);
	write_cmos_sensor(0x6F12, 0x4205);
	write_cmos_sensor(0x6F12, 0x4181);
	write_cmos_sensor(0x6F12, 0x8280);
	write_cmos_sensor(0x6F12, 0x5D71);
	write_cmos_sensor(0x6F12, 0xA2C6);
	write_cmos_sensor(0x6F12, 0xA6C4);
	write_cmos_sensor(0x6F12, 0x7324);
	write_cmos_sensor(0x6F12, 0x2034);
	write_cmos_sensor(0x6F12, 0xF324);
	write_cmos_sensor(0x6F12, 0x1034);
	write_cmos_sensor(0x6F12, 0x7360);
	write_cmos_sensor(0x6F12, 0x0430);
	write_cmos_sensor(0x6F12, 0x2AD8);
	write_cmos_sensor(0x6F12, 0x2ED6);
	write_cmos_sensor(0x6F12, 0x3545);
	write_cmos_sensor(0x6F12, 0x9305);
	write_cmos_sensor(0x6F12, 0x8008);
	write_cmos_sensor(0x6F12, 0x22DA);
	write_cmos_sensor(0x6F12, 0x3ECE);
	write_cmos_sensor(0x6F12, 0x86C2);
	write_cmos_sensor(0x6F12, 0x96C0);
	write_cmos_sensor(0x6F12, 0x1ADE);
	write_cmos_sensor(0x6F12, 0x1EDC);
	write_cmos_sensor(0x6F12, 0x32D4);
	write_cmos_sensor(0x6F12, 0x36D2);
	write_cmos_sensor(0x6F12, 0x3AD0);
	write_cmos_sensor(0x6F12, 0x42CC);
	write_cmos_sensor(0x6F12, 0x46CA);
	write_cmos_sensor(0x6F12, 0x72C8);
	write_cmos_sensor(0x6F12, 0x76C6);
	write_cmos_sensor(0x6F12, 0x7AC4);
	write_cmos_sensor(0x6F12, 0x7EC2);
	write_cmos_sensor(0x6F12, 0x9710);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x40F5);
	write_cmos_sensor(0x6F12, 0x9377);
	write_cmos_sensor(0x6F12, 0x8500);
	write_cmos_sensor(0x6F12, 0x2A84);
	write_cmos_sensor(0x6F12, 0x85C3);
	write_cmos_sensor(0x6F12, 0xB737);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x077C);
	write_cmos_sensor(0x6F12, 0x03D7);
	write_cmos_sensor(0x6F12, 0x6702);
	write_cmos_sensor(0x6F12, 0x0507);
	write_cmos_sensor(0x6F12, 0x2393);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0x9710);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x00ED);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x6F12, 0xD535);
	write_cmos_sensor(0x6F12, 0x1374);
	write_cmos_sensor(0x6F12, 0x0408);
	write_cmos_sensor(0x6F12, 0x11CC);
	write_cmos_sensor(0x6F12, 0xB737);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x077C);
	write_cmos_sensor(0x6F12, 0x03D7);
	write_cmos_sensor(0x6F12, 0x8705);
	write_cmos_sensor(0x6F12, 0x0507);
	write_cmos_sensor(0x6F12, 0x239C);
	write_cmos_sensor(0x6F12, 0xE704);
	write_cmos_sensor(0x6F12, 0x9710);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x6065);
	write_cmos_sensor(0x6F12, 0x9640);
	write_cmos_sensor(0x6F12, 0x8642);
	write_cmos_sensor(0x6F12, 0x7253);
	write_cmos_sensor(0x6F12, 0xE253);
	write_cmos_sensor(0x6F12, 0x5254);
	write_cmos_sensor(0x6F12, 0x4255);
	write_cmos_sensor(0x6F12, 0xB255);
	write_cmos_sensor(0x6F12, 0x2256);
	write_cmos_sensor(0x6F12, 0x9256);
	write_cmos_sensor(0x6F12, 0x0257);
	write_cmos_sensor(0x6F12, 0xF247);
	write_cmos_sensor(0x6F12, 0x6248);
	write_cmos_sensor(0x6F12, 0xD248);
	write_cmos_sensor(0x6F12, 0x424E);
	write_cmos_sensor(0x6F12, 0xB24E);
	write_cmos_sensor(0x6F12, 0x224F);
	write_cmos_sensor(0x6F12, 0x924F);
	write_cmos_sensor(0x6F12, 0x7370);
	write_cmos_sensor(0x6F12, 0x0430);
	write_cmos_sensor(0x6F12, 0x7390);
	write_cmos_sensor(0x6F12, 0x1434);
	write_cmos_sensor(0x6F12, 0x7310);
	write_cmos_sensor(0x6F12, 0x2434);
	write_cmos_sensor(0x6F12, 0x3644);
	write_cmos_sensor(0x6F12, 0xA644);
	write_cmos_sensor(0x6F12, 0x6161);
	write_cmos_sensor(0x6F12, 0x7300);
	write_cmos_sensor(0x6F12, 0x2030);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0xC369);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x83A4);
	write_cmos_sensor(0x6F12, 0xC700);
	write_cmos_sensor(0x6F12, 0x2A84);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0xA685);
	write_cmos_sensor(0x6F12, 0x1145);
	write_cmos_sensor(0x6F12, 0x9790);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x60AD);
	write_cmos_sensor(0x6F12, 0x2285);
	write_cmos_sensor(0x6F12, 0x97E0);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x40A9);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x87F5);
	write_cmos_sensor(0x6F12, 0x03C7);
	write_cmos_sensor(0x6F12, 0x0700);
	write_cmos_sensor(0x6F12, 0x8546);
	write_cmos_sensor(0x6F12, 0x6315);
	write_cmos_sensor(0x6F12, 0xD706);
	write_cmos_sensor(0x6F12, 0xB776);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x03A6);
	write_cmos_sensor(0x6F12, 0x468D);
	write_cmos_sensor(0x6F12, 0x8945);
	write_cmos_sensor(0x6F12, 0x9386);
	write_cmos_sensor(0x6F12, 0xA700);
	write_cmos_sensor(0x6F12, 0x630F);
	write_cmos_sensor(0x6F12, 0xB600);
	write_cmos_sensor(0x6F12, 0x9386);
	write_cmos_sensor(0x6F12, 0x2700);
	write_cmos_sensor(0x6F12, 0x630B);
	write_cmos_sensor(0x6F12, 0xE600);
	write_cmos_sensor(0x6F12, 0x3717);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x0356);
	write_cmos_sensor(0x6F12, 0x6738);
	write_cmos_sensor(0x6F12, 0x2D47);
	write_cmos_sensor(0x6F12, 0x6314);
	write_cmos_sensor(0x6F12, 0xE600);
	write_cmos_sensor(0x6F12, 0x9386);
	write_cmos_sensor(0x6F12, 0xA700);
	write_cmos_sensor(0x6F12, 0xB747);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x83A5);
	write_cmos_sensor(0x6F12, 0xC786);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x1305);
	write_cmos_sensor(0x6F12, 0x0404);
	write_cmos_sensor(0x6F12, 0x83C7);
	write_cmos_sensor(0x6F12, 0xC52F);
	write_cmos_sensor(0x6F12, 0x2148);
	write_cmos_sensor(0x6F12, 0x1D8E);
	write_cmos_sensor(0x6F12, 0x8147);
	write_cmos_sensor(0x6F12, 0x3387);
	write_cmos_sensor(0x6F12, 0xF500);
	write_cmos_sensor(0x6F12, 0xB388);
	write_cmos_sensor(0x6F12, 0xF600);
	write_cmos_sensor(0x6F12, 0x0317);
	write_cmos_sensor(0x6F12, 0xE73C);
	write_cmos_sensor(0x6F12, 0x8398);
	write_cmos_sensor(0x6F12, 0x0800);
	write_cmos_sensor(0x6F12, 0x8907);
	write_cmos_sensor(0x6F12, 0x1105);
	write_cmos_sensor(0x6F12, 0x4697);
	write_cmos_sensor(0x6F12, 0x3317);
	write_cmos_sensor(0x6F12, 0xC700);
	write_cmos_sensor(0x6F12, 0x232E);
	write_cmos_sensor(0x6F12, 0xE5FE);
	write_cmos_sensor(0x6F12, 0xE391);
	write_cmos_sensor(0x6F12, 0x07FF);
	write_cmos_sensor(0x6F12, 0x0546);
	write_cmos_sensor(0x6F12, 0xA685);
	write_cmos_sensor(0x6F12, 0x1145);
	write_cmos_sensor(0x6F12, 0x9790);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x60A4);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0x6700);
	write_cmos_sensor(0x6F12, 0x0361);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0xA35C);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x83A4);
	write_cmos_sensor(0x6F12, 0x8700);
	write_cmos_sensor(0x6F12, 0x4111);
	write_cmos_sensor(0x6F12, 0xAA89);
	write_cmos_sensor(0x6F12, 0x2E8A);
	write_cmos_sensor(0x6F12, 0xB28A);
	write_cmos_sensor(0x6F12, 0xA685);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x1145);
	write_cmos_sensor(0x6F12, 0x36C6);
	write_cmos_sensor(0x6F12, 0x9790);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x60A1);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x1387);
	write_cmos_sensor(0x6F12, 0x87F5);
	write_cmos_sensor(0x6F12, 0x0347);
	write_cmos_sensor(0x6F12, 0x2701);
	write_cmos_sensor(0x6F12, 0x1384);
	write_cmos_sensor(0x6F12, 0x87F5);
	write_cmos_sensor(0x6F12, 0xB246);
	write_cmos_sensor(0x6F12, 0x0149);
	write_cmos_sensor(0x6F12, 0x11CF);
	write_cmos_sensor(0x6F12, 0x3767);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x0357);
	write_cmos_sensor(0x6F12, 0x2777);
	write_cmos_sensor(0x6F12, 0xB777);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x07C7);
	write_cmos_sensor(0x6F12, 0x0E07);
	write_cmos_sensor(0x6F12, 0x03D9);
	write_cmos_sensor(0x6F12, 0x871C);
	write_cmos_sensor(0x6F12, 0x2394);
	write_cmos_sensor(0x6F12, 0xE71C);
	write_cmos_sensor(0x6F12, 0x5686);
	write_cmos_sensor(0x6F12, 0xD285);
	write_cmos_sensor(0x6F12, 0x4E85);
	write_cmos_sensor(0x6F12, 0x97E0);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x0088);
	write_cmos_sensor(0x6F12, 0x8347);
	write_cmos_sensor(0x6F12, 0x2401);
	write_cmos_sensor(0x6F12, 0x89C7);
	write_cmos_sensor(0x6F12, 0xB777);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x239C);
	write_cmos_sensor(0x6F12, 0x27E3);
	write_cmos_sensor(0x6F12, 0x0546);
	write_cmos_sensor(0x6F12, 0xA685);
	write_cmos_sensor(0x6F12, 0x1145);
	write_cmos_sensor(0x6F12, 0x9790);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0xC09B);
	write_cmos_sensor(0x6F12, 0x4101);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0x6700);
	write_cmos_sensor(0x6F12, 0xA357);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0x0353);
	write_cmos_sensor(0x6F12, 0x3784);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9307);
	write_cmos_sensor(0x6F12, 0x04F4);
	write_cmos_sensor(0x6F12, 0x83C7);
	write_cmos_sensor(0x6F12, 0x4701);
	write_cmos_sensor(0x6F12, 0x5D71);
	write_cmos_sensor(0x6F12, 0x2A89);
	write_cmos_sensor(0x6F12, 0x8DEF);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x03A4);
	write_cmos_sensor(0x6F12, 0x4700);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x1145);
	write_cmos_sensor(0x6F12, 0xA285);
	write_cmos_sensor(0x6F12, 0x9790);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x2098);
	write_cmos_sensor(0x6F12, 0x4A85);
	write_cmos_sensor(0x6F12, 0x97E0);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x400F);
	write_cmos_sensor(0x6F12, 0x0546);
	write_cmos_sensor(0x6F12, 0xA285);
	write_cmos_sensor(0x6F12, 0x1145);
	write_cmos_sensor(0x6F12, 0x9790);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0xA096);
	write_cmos_sensor(0x6F12, 0x6161);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0x6700);
	write_cmos_sensor(0x6F12, 0xE351);
	write_cmos_sensor(0x6F12, 0x1304);
	write_cmos_sensor(0x6F12, 0x04F4);
	write_cmos_sensor(0x6F12, 0x0347);
	write_cmos_sensor(0x6F12, 0x5401);
	write_cmos_sensor(0x6F12, 0x8547);
	write_cmos_sensor(0x6F12, 0x6310);
	write_cmos_sensor(0x6F12, 0xF716);
	write_cmos_sensor(0x6F12, 0xB785);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9384);
	write_cmos_sensor(0x6F12, 0x05F8);
	write_cmos_sensor(0x6F12, 0x4146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x05F8);
	write_cmos_sensor(0x6F12, 0x0A85);
	write_cmos_sensor(0x6F12, 0x9740);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x6058);
	write_cmos_sensor(0x6F12, 0x4146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x0401);
	write_cmos_sensor(0x6F12, 0x0808);
	write_cmos_sensor(0x6F12, 0x9740);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x6057);
	write_cmos_sensor(0x6F12, 0x4146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x0402);
	write_cmos_sensor(0x6F12, 0x0810);
	write_cmos_sensor(0x6F12, 0x9740);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x6056);
	write_cmos_sensor(0x6F12, 0x4146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x0403);
	write_cmos_sensor(0x6F12, 0x0818);
	write_cmos_sensor(0x6F12, 0x9740);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x6055);
	write_cmos_sensor(0x6F12, 0x1C40);
	write_cmos_sensor(0x6F12, 0xB7DB);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x014B);
	write_cmos_sensor(0x6F12, 0xBEC0);
	write_cmos_sensor(0x6F12, 0x5C40);
	write_cmos_sensor(0x6F12, 0x8149);
	write_cmos_sensor(0x6F12, 0x854A);
	write_cmos_sensor(0x6F12, 0xBEC2);
	write_cmos_sensor(0x6F12, 0x5C44);
	write_cmos_sensor(0x6F12, 0x096D);
	write_cmos_sensor(0x6F12, 0x370C);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0xBEC4);
	write_cmos_sensor(0x6F12, 0x1C48);
	write_cmos_sensor(0x6F12, 0x938B);
	write_cmos_sensor(0x6F12, 0x0B03);
	write_cmos_sensor(0x6F12, 0x930C);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x6F12, 0xBEC6);
	write_cmos_sensor(0x6F12, 0x0347);
	write_cmos_sensor(0x6F12, 0x7401);
	write_cmos_sensor(0x6F12, 0x8967);
	write_cmos_sensor(0x6F12, 0x631B);
	write_cmos_sensor(0x6F12, 0x5701);
	write_cmos_sensor(0x6F12, 0x93F7);
	write_cmos_sensor(0x6F12, 0xF900);
	write_cmos_sensor(0x6F12, 0x9808);
	write_cmos_sensor(0x6F12, 0xBA97);
	write_cmos_sensor(0x6F12, 0x83C7);
	write_cmos_sensor(0x6F12, 0x07FB);
	write_cmos_sensor(0x6F12, 0x8A07);
	write_cmos_sensor(0x6F12, 0xCA97);
	write_cmos_sensor(0x6F12, 0x9C43);
	write_cmos_sensor(0x6F12, 0x63DE);
	write_cmos_sensor(0x6F12, 0x3A01);
	write_cmos_sensor(0x6F12, 0x1387);
	write_cmos_sensor(0x6F12, 0x69FE);
	write_cmos_sensor(0x6F12, 0x63FA);
	write_cmos_sensor(0x6F12, 0xEA00);
	write_cmos_sensor(0x6F12, 0x1387);
	write_cmos_sensor(0x6F12, 0xA9FD);
	write_cmos_sensor(0x6F12, 0x63F6);
	write_cmos_sensor(0x6F12, 0xEA00);
	write_cmos_sensor(0x6F12, 0x1387);
	write_cmos_sensor(0x6F12, 0x49FC);
	write_cmos_sensor(0x6F12, 0x63E3);
	write_cmos_sensor(0x6F12, 0xEA0A);
	write_cmos_sensor(0x6F12, 0x131A);
	write_cmos_sensor(0x6F12, 0x1B00);
	write_cmos_sensor(0x6F12, 0x9808);
	write_cmos_sensor(0x6F12, 0x5297);
	write_cmos_sensor(0x6F12, 0x8354);
	write_cmos_sensor(0x6F12, 0x07FF);
	write_cmos_sensor(0x6F12, 0x0145);
	write_cmos_sensor(0x6F12, 0xB384);
	write_cmos_sensor(0x6F12, 0xF402);
	write_cmos_sensor(0x6F12, 0xB580);
	write_cmos_sensor(0x6F12, 0x637A);
	write_cmos_sensor(0x6F12, 0x9D00);
	write_cmos_sensor(0x6F12, 0x2685);
	write_cmos_sensor(0x6F12, 0x9780);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x6052);
	write_cmos_sensor(0x6F12, 0x4915);
	write_cmos_sensor(0x6F12, 0x1375);
	write_cmos_sensor(0x6F12, 0xF50F);
	write_cmos_sensor(0x6F12, 0x9C08);
	write_cmos_sensor(0x6F12, 0xD297);
	write_cmos_sensor(0x6F12, 0x03D7);
	write_cmos_sensor(0x6F12, 0x07FE);
	write_cmos_sensor(0x6F12, 0xB3D4);
	write_cmos_sensor(0x6F12, 0xA400);
	write_cmos_sensor(0x6F12, 0xC204);
	write_cmos_sensor(0x6F12, 0x6297);
	write_cmos_sensor(0x6F12, 0xC180);
	write_cmos_sensor(0x6F12, 0x2310);
	write_cmos_sensor(0x6F12, 0x9700);
	write_cmos_sensor(0x6F12, 0x03D7);
	write_cmos_sensor(0x6F12, 0x07FC);
	write_cmos_sensor(0x6F12, 0x83D7);
	write_cmos_sensor(0x6F12, 0x07FD);
	write_cmos_sensor(0x6F12, 0x050B);
	write_cmos_sensor(0x6F12, 0x6297);
	write_cmos_sensor(0x6F12, 0x8356);
	write_cmos_sensor(0x6F12, 0x0700);
	write_cmos_sensor(0x6F12, 0x3315);
	write_cmos_sensor(0x6F12, 0xF500);
	write_cmos_sensor(0x6F12, 0x558D);
	write_cmos_sensor(0x6F12, 0x4205);
	write_cmos_sensor(0x6F12, 0x4181);
	write_cmos_sensor(0x6F12, 0x2310);
	write_cmos_sensor(0x6F12, 0xA700);
	write_cmos_sensor(0x6F12, 0x8509);
	write_cmos_sensor(0x6F12, 0xE395);
	write_cmos_sensor(0x6F12, 0x99F7);
	write_cmos_sensor(0x6F12, 0x8D47);
	write_cmos_sensor(0x6F12, 0x37D4);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2319);
	write_cmos_sensor(0x6F12, 0xF40A);
	write_cmos_sensor(0x6F12, 0x2316);
	write_cmos_sensor(0x6F12, 0x040C);
	write_cmos_sensor(0x6F12, 0x9790);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x6093);
	write_cmos_sensor(0x6F12, 0xAA84);
	write_cmos_sensor(0x6F12, 0x9790);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x2092);
	write_cmos_sensor(0x6F12, 0x9307);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x6F12, 0x33D5);
	write_cmos_sensor(0x6F12, 0xA700);
	write_cmos_sensor(0x6F12, 0x9307);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x6F12, 0x1205);
	write_cmos_sensor(0x6F12, 0xB3D7);
	write_cmos_sensor(0x6F12, 0x9700);
	write_cmos_sensor(0x6F12, 0x1375);
	write_cmos_sensor(0x6F12, 0x0503);
	write_cmos_sensor(0x6F12, 0x8D8B);
	write_cmos_sensor(0x6F12, 0x5D8D);
	write_cmos_sensor(0x6F12, 0x2319);
	write_cmos_sensor(0x6F12, 0xA40C);
	write_cmos_sensor(0x6F12, 0x45B5);
	write_cmos_sensor(0x6F12, 0x1397);
	write_cmos_sensor(0x6F12, 0x1900);
	write_cmos_sensor(0x6F12, 0x9394);
	write_cmos_sensor(0x6F12, 0x0701);
	write_cmos_sensor(0x6F12, 0x5E97);
	write_cmos_sensor(0x6F12, 0xC180);
	write_cmos_sensor(0x6F12, 0x2310);
	write_cmos_sensor(0x6F12, 0x9700);
	write_cmos_sensor(0x6F12, 0x6DB7);
	write_cmos_sensor(0x6F12, 0x0347);
	write_cmos_sensor(0x6F12, 0x6401);
	write_cmos_sensor(0x6F12, 0xE315);
	write_cmos_sensor(0x6F12, 0xF7FA);
	write_cmos_sensor(0x6F12, 0xB784);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9384);
	write_cmos_sensor(0x6F12, 0x04F8);
	write_cmos_sensor(0x6F12, 0x4146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x0404);
	write_cmos_sensor(0x6F12, 0x0A85);
	write_cmos_sensor(0x6F12, 0x9740);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x2042);
	write_cmos_sensor(0x6F12, 0x4146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x0405);
	write_cmos_sensor(0x6F12, 0x0808);
	write_cmos_sensor(0x6F12, 0x9740);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x2041);
	write_cmos_sensor(0x6F12, 0x4146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x0406);
	write_cmos_sensor(0x6F12, 0x0810);
	write_cmos_sensor(0x6F12, 0x9740);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x2040);
	write_cmos_sensor(0x6F12, 0x4146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x0407);
	write_cmos_sensor(0x6F12, 0x0818);
	write_cmos_sensor(0x6F12, 0x9740);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x203F);
	write_cmos_sensor(0x6F12, 0x0357);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x8357);
	write_cmos_sensor(0x6F12, 0x2400);
	write_cmos_sensor(0x6F12, 0x37DB);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2310);
	write_cmos_sensor(0x6F12, 0xE104);
	write_cmos_sensor(0x6F12, 0x2311);
	write_cmos_sensor(0x6F12, 0xF104);
	write_cmos_sensor(0x6F12, 0x2312);
	write_cmos_sensor(0x6F12, 0xE104);
	write_cmos_sensor(0x6F12, 0x2313);
	write_cmos_sensor(0x6F12, 0xF104);
	write_cmos_sensor(0x6F12, 0x0357);
	write_cmos_sensor(0x6F12, 0x8400);
	write_cmos_sensor(0x6F12, 0x8357);
	write_cmos_sensor(0x6F12, 0xA400);
	write_cmos_sensor(0x6F12, 0x814A);
	write_cmos_sensor(0x6F12, 0x2314);
	write_cmos_sensor(0x6F12, 0xE104);
	write_cmos_sensor(0x6F12, 0x2315);
	write_cmos_sensor(0x6F12, 0xF104);
	write_cmos_sensor(0x6F12, 0x2316);
	write_cmos_sensor(0x6F12, 0xE104);
	write_cmos_sensor(0x6F12, 0x2317);
	write_cmos_sensor(0x6F12, 0xF104);
	write_cmos_sensor(0x6F12, 0x8149);
	write_cmos_sensor(0x6F12, 0x854B);
	write_cmos_sensor(0x6F12, 0x096D);
	write_cmos_sensor(0x6F12, 0x130B);
	write_cmos_sensor(0x6F12, 0x0B03);
	write_cmos_sensor(0x6F12, 0x370C);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x930C);
	write_cmos_sensor(0x6F12, 0x0004);
	write_cmos_sensor(0x6F12, 0x0347);
	write_cmos_sensor(0x6F12, 0x7401);
	write_cmos_sensor(0x6F12, 0x8967);
	write_cmos_sensor(0x6F12, 0x631B);
	write_cmos_sensor(0x6F12, 0x7701);
	write_cmos_sensor(0x6F12, 0x93F7);
	write_cmos_sensor(0x6F12, 0xF900);
	write_cmos_sensor(0x6F12, 0x9808);
	write_cmos_sensor(0x6F12, 0xBA97);
	write_cmos_sensor(0x6F12, 0x83C7);
	write_cmos_sensor(0x6F12, 0x07FB);
	write_cmos_sensor(0x6F12, 0x8A07);
	write_cmos_sensor(0x6F12, 0xCA97);
	write_cmos_sensor(0x6F12, 0x9C43);
	write_cmos_sensor(0x6F12, 0x63C4);
	write_cmos_sensor(0x6F12, 0x3B07);
	write_cmos_sensor(0x6F12, 0x139A);
	write_cmos_sensor(0x6F12, 0x1A00);
	write_cmos_sensor(0x6F12, 0x9808);
	write_cmos_sensor(0x6F12, 0x5297);
	write_cmos_sensor(0x6F12, 0x8354);
	write_cmos_sensor(0x6F12, 0x07FF);
	write_cmos_sensor(0x6F12, 0x0145);
	write_cmos_sensor(0x6F12, 0xB384);
	write_cmos_sensor(0x6F12, 0xF402);
	write_cmos_sensor(0x6F12, 0xB580);
	write_cmos_sensor(0x6F12, 0x637A);
	write_cmos_sensor(0x6F12, 0x9D00);
	write_cmos_sensor(0x6F12, 0x2685);
	write_cmos_sensor(0x6F12, 0x9780);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0xA03B);
	write_cmos_sensor(0x6F12, 0x4915);
	write_cmos_sensor(0x6F12, 0x1375);
	write_cmos_sensor(0x6F12, 0xF50F);
	write_cmos_sensor(0x6F12, 0x9C08);
	write_cmos_sensor(0x6F12, 0xD297);
	write_cmos_sensor(0x6F12, 0x03D7);
	write_cmos_sensor(0x6F12, 0x07FE);
	write_cmos_sensor(0x6F12, 0xB3D4);
	write_cmos_sensor(0x6F12, 0xA400);
	write_cmos_sensor(0x6F12, 0xC204);
	write_cmos_sensor(0x6F12, 0x6297);
	write_cmos_sensor(0x6F12, 0xC180);
	write_cmos_sensor(0x6F12, 0x2310);
	write_cmos_sensor(0x6F12, 0x9700);
	write_cmos_sensor(0x6F12, 0x03D7);
	write_cmos_sensor(0x6F12, 0x07FC);
	write_cmos_sensor(0x6F12, 0x83D7);
	write_cmos_sensor(0x6F12, 0x07FD);
	write_cmos_sensor(0x6F12, 0x850A);
	write_cmos_sensor(0x6F12, 0x6297);
	write_cmos_sensor(0x6F12, 0x8356);
	write_cmos_sensor(0x6F12, 0x0700);
	write_cmos_sensor(0x6F12, 0x3315);
	write_cmos_sensor(0x6F12, 0xF500);
	write_cmos_sensor(0x6F12, 0x558D);
	write_cmos_sensor(0x6F12, 0x4205);
	write_cmos_sensor(0x6F12, 0x4181);
	write_cmos_sensor(0x6F12, 0x2310);
	write_cmos_sensor(0x6F12, 0xA700);
	write_cmos_sensor(0x6F12, 0x8509);
	write_cmos_sensor(0x6F12, 0xE391);
	write_cmos_sensor(0x6F12, 0x99F9);
	write_cmos_sensor(0x6F12, 0x51BD);
	write_cmos_sensor(0x6F12, 0x1397);
	write_cmos_sensor(0x6F12, 0x1900);
	write_cmos_sensor(0x6F12, 0x9394);
	write_cmos_sensor(0x6F12, 0x0701);
	write_cmos_sensor(0x6F12, 0x5A97);
	write_cmos_sensor(0x6F12, 0xC180);
	write_cmos_sensor(0x6F12, 0x2310);
	write_cmos_sensor(0x6F12, 0x9700);
	write_cmos_sensor(0x6F12, 0xE5B7);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0xE326);
	write_cmos_sensor(0x6F12, 0xB737);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x83A7);
	write_cmos_sensor(0x6F12, 0x0761);
	write_cmos_sensor(0x6F12, 0xAA84);
	write_cmos_sensor(0x6F12, 0x2E89);
	write_cmos_sensor(0x6F12, 0x8297);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x03A4);
	write_cmos_sensor(0x6F12, 0x0700);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x1145);
	write_cmos_sensor(0x6F12, 0xA285);
	write_cmos_sensor(0x6F12, 0x9780);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0xC069);
	write_cmos_sensor(0x6F12, 0xCA85);
	write_cmos_sensor(0x6F12, 0x2685);
	write_cmos_sensor(0x6F12, 0x9730);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x20F5);
	write_cmos_sensor(0x6F12, 0x0546);
	write_cmos_sensor(0x6F12, 0xA285);
	write_cmos_sensor(0x6F12, 0x1145);
	write_cmos_sensor(0x6F12, 0x9780);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x2068);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0x6700);
	write_cmos_sensor(0x6F12, 0xC324);
	write_cmos_sensor(0x6F12, 0xB717);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x83C7);
	write_cmos_sensor(0x6F12, 0x0734);
	write_cmos_sensor(0x6F12, 0xEDCF);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0x6321);
	write_cmos_sensor(0x6F12, 0x9780);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x803E);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x1387);
	write_cmos_sensor(0x6F12, 0x87F5);
	write_cmos_sensor(0x6F12, 0x0347);
	write_cmos_sensor(0x6F12, 0xF701);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x87F5);
	write_cmos_sensor(0x6F12, 0x19EB);
	write_cmos_sensor(0x6F12, 0x37F7);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x8356);
	write_cmos_sensor(0x6F12, 0x6772);
	write_cmos_sensor(0x6F12, 0x2391);
	write_cmos_sensor(0x6F12, 0xD702);
	write_cmos_sensor(0x6F12, 0x0357);
	write_cmos_sensor(0x6F12, 0xA772);
	write_cmos_sensor(0x6F12, 0x2392);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0xB776);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x83C6);
	write_cmos_sensor(0x6F12, 0xB6F1);
	write_cmos_sensor(0x6F12, 0x0547);
	write_cmos_sensor(0x6F12, 0xA38F);
	write_cmos_sensor(0x6F12, 0xE700);
	write_cmos_sensor(0x6F12, 0x99C6);
	write_cmos_sensor(0x6F12, 0x83D6);
	write_cmos_sensor(0x6F12, 0x4701);
	write_cmos_sensor(0x6F12, 0x238F);
	write_cmos_sensor(0x6F12, 0xE700);
	write_cmos_sensor(0x6F12, 0x2380);
	write_cmos_sensor(0x6F12, 0xD702);
	write_cmos_sensor(0x6F12, 0x83C6);
	write_cmos_sensor(0x6F12, 0x0702);
	write_cmos_sensor(0x6F12, 0x03C7);
	write_cmos_sensor(0x6F12, 0xE701);
	write_cmos_sensor(0x6F12, 0xB9CE);
	write_cmos_sensor(0x6F12, 0x0DC3);
	write_cmos_sensor(0x6F12, 0x03D7);
	write_cmos_sensor(0x6F12, 0x6701);
	write_cmos_sensor(0x6F12, 0x0DCF);
	write_cmos_sensor(0x6F12, 0xB7F6);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2393);
	write_cmos_sensor(0x6F12, 0xE672);
	write_cmos_sensor(0x6F12, 0x03D7);
	write_cmos_sensor(0x6F12, 0x8701);
	write_cmos_sensor(0x6F12, 0x0DCF);
	write_cmos_sensor(0x6F12, 0xB7F6);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2395);
	write_cmos_sensor(0x6F12, 0xE672);
	write_cmos_sensor(0x6F12, 0x238F);
	write_cmos_sensor(0x6F12, 0x0700);
	write_cmos_sensor(0x6F12, 0x03C7);
	write_cmos_sensor(0x6F12, 0x0702);
	write_cmos_sensor(0x6F12, 0x7D17);
	write_cmos_sensor(0x6F12, 0x1377);
	write_cmos_sensor(0x6F12, 0xF70F);
	write_cmos_sensor(0x6F12, 0x2380);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0x01E7);
	write_cmos_sensor(0x6F12, 0x0547);
	write_cmos_sensor(0x6F12, 0x238F);
	write_cmos_sensor(0x6F12, 0xE700);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0x6700);
	write_cmos_sensor(0x6F12, 0x631A);
	write_cmos_sensor(0x6F12, 0x83D6);
	write_cmos_sensor(0x6F12, 0x2702);
	write_cmos_sensor(0x6F12, 0x37F7);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2313);
	write_cmos_sensor(0x6F12, 0xD772);
	write_cmos_sensor(0x6F12, 0xD1B7);
	write_cmos_sensor(0x6F12, 0x83D6);
	write_cmos_sensor(0x6F12, 0x4702);
	write_cmos_sensor(0x6F12, 0x37F7);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2315);
	write_cmos_sensor(0x6F12, 0xD772);
	write_cmos_sensor(0x6F12, 0xD1B7);
	write_cmos_sensor(0x6F12, 0x71DF);
	write_cmos_sensor(0x6F12, 0x03D7);
	write_cmos_sensor(0x6F12, 0xA701);
	write_cmos_sensor(0x6F12, 0x19CF);
	write_cmos_sensor(0x6F12, 0xB7F6);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2393);
	write_cmos_sensor(0x6F12, 0xE672);
	write_cmos_sensor(0x6F12, 0x03D7);
	write_cmos_sensor(0x6F12, 0xC701);
	write_cmos_sensor(0x6F12, 0x19CF);
	write_cmos_sensor(0x6F12, 0xB7F6);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2395);
	write_cmos_sensor(0x6F12, 0xE672);
	write_cmos_sensor(0x6F12, 0x238F);
	write_cmos_sensor(0x6F12, 0x0700);
	write_cmos_sensor(0x6F12, 0x6DBF);
	write_cmos_sensor(0x6F12, 0x83D6);
	write_cmos_sensor(0x6F12, 0x2702);
	write_cmos_sensor(0x6F12, 0x37F7);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2313);
	write_cmos_sensor(0x6F12, 0xD772);
	write_cmos_sensor(0x6F12, 0xC5B7);
	write_cmos_sensor(0x6F12, 0x83D6);
	write_cmos_sensor(0x6F12, 0x4702);
	write_cmos_sensor(0x6F12, 0x37F7);
	write_cmos_sensor(0x6F12, 0x0040);
	write_cmos_sensor(0x6F12, 0x2315);
	write_cmos_sensor(0x6F12, 0xD772);
	write_cmos_sensor(0x6F12, 0xC5B7);
	write_cmos_sensor(0x6F12, 0x8280);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0xC311);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0xA166);
	write_cmos_sensor(0x6F12, 0xB775);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9386);
	write_cmos_sensor(0x6F12, 0x76F7);
	write_cmos_sensor(0x6F12, 0x3777);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0xC701);
	write_cmos_sensor(0x6F12, 0x2946);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0xA57F);
	write_cmos_sensor(0x6F12, 0x3545);
	write_cmos_sensor(0x6F12, 0x2320);
	write_cmos_sensor(0x6F12, 0xF73C);
	write_cmos_sensor(0x6F12, 0x9710);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0xA01F);
	write_cmos_sensor(0x6F12, 0xB777);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0xB785);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x3755);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x3737);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x4774);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0xA58B);
	write_cmos_sensor(0x6F12, 0x1305);
	write_cmos_sensor(0x6F12, 0x0537);
	write_cmos_sensor(0x6F12, 0x232C);
	write_cmos_sensor(0x6F12, 0xF75E);
	write_cmos_sensor(0x6F12, 0x9710);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x4058);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x23A6);
	write_cmos_sensor(0x6F12, 0xA700);
	write_cmos_sensor(0x6F12, 0xB785);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x3765);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0xE59F);
	write_cmos_sensor(0x6F12, 0x1305);
	write_cmos_sensor(0x6F12, 0x45B2);
	write_cmos_sensor(0x6F12, 0x9710);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x2056);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x23A2);
	write_cmos_sensor(0x6F12, 0xA700);
	write_cmos_sensor(0x6F12, 0xB785);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x3755);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x2597);
	write_cmos_sensor(0x6F12, 0x1305);
	write_cmos_sensor(0x6F12, 0x0525);
	write_cmos_sensor(0x6F12, 0x9710);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x0054);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x23A4);
	write_cmos_sensor(0x6F12, 0xA700);
	write_cmos_sensor(0x6F12, 0xB775);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x3775);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x257B);
	write_cmos_sensor(0x6F12, 0x1305);
	write_cmos_sensor(0x6F12, 0x85D3);
	write_cmos_sensor(0x6F12, 0x9710);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0xE051);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x23A8);
	write_cmos_sensor(0x6F12, 0xA700);
	write_cmos_sensor(0x6F12, 0xB785);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x37B5);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x85CE);
	write_cmos_sensor(0x6F12, 0x1305);
	write_cmos_sensor(0x6F12, 0xA5C6);
	write_cmos_sensor(0x6F12, 0x9710);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0xC04F);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x23A0);
	write_cmos_sensor(0x6F12, 0xA700);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x3737);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x67D3);
	write_cmos_sensor(0x6F12, 0x2320);
	write_cmos_sensor(0x6F12, 0xF768);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0x6700);
	write_cmos_sensor(0x6F12, 0x4304);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0203);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0203);
	write_cmos_sensor(0x6F12, 0x0405);
	write_cmos_sensor(0x6F12, 0x0607);
	write_cmos_sensor(0x6F12, 0x0405);
	write_cmos_sensor(0x6F12, 0x0607);
	write_cmos_sensor(0x6F12, 0x10D0);
	write_cmos_sensor(0x6F12, 0x10D0);
	write_cmos_sensor(0x6F12, 0x1CD0);
	write_cmos_sensor(0x6F12, 0x1CD0);
	write_cmos_sensor(0x6F12, 0x22D0);
	write_cmos_sensor(0x6F12, 0x22D0);
	write_cmos_sensor(0x6F12, 0x2ED0);
	write_cmos_sensor(0x6F12, 0x2ED0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x0800);
	write_cmos_sensor(0x6F12, 0x0C00);
	write_cmos_sensor(0x6F12, 0x0800);
	write_cmos_sensor(0x6F12, 0x0C00);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x30D0);
	write_cmos_sensor(0x6F12, 0x32D0);
	write_cmos_sensor(0x6F12, 0x64D0);
	write_cmos_sensor(0x6F12, 0x66D0);
	write_cmos_sensor(0x6F12, 0x7CD0);
	write_cmos_sensor(0x6F12, 0x7ED0);
	write_cmos_sensor(0x6F12, 0xA8D0);
	write_cmos_sensor(0x6F12, 0xAAD0);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x6F12, 0x0405);
	write_cmos_sensor(0x6F12, 0x0405);
	write_cmos_sensor(0x6F12, 0x0405);
	write_cmos_sensor(0x6F12, 0x0405);
	write_cmos_sensor(0x6F12, 0x10D0);
	write_cmos_sensor(0x6F12, 0x10D0);
	write_cmos_sensor(0x6F12, 0x12D0);
	write_cmos_sensor(0x6F12, 0x12D0);
	write_cmos_sensor(0x6F12, 0x20D0);
	write_cmos_sensor(0x6F12, 0x20D0);
	write_cmos_sensor(0x6F12, 0x22D0);
	write_cmos_sensor(0x6F12, 0x22D0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x6F12, 0x30D0);
	write_cmos_sensor(0x6F12, 0x32D0);
	write_cmos_sensor(0x6F12, 0x38D0);
	write_cmos_sensor(0x6F12, 0x3AD0);
	write_cmos_sensor(0x6F12, 0x70D0);
	write_cmos_sensor(0x6F12, 0x72D0);
	write_cmos_sensor(0x6F12, 0x78D0);
	write_cmos_sensor(0x6F12, 0x7AD0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0xA3F3);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x1307);
	write_cmos_sensor(0x6F12, 0xC00E);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0xC701);
	write_cmos_sensor(0x6F12, 0xB785);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x3755);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x6F12, 0xBA97);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0x3777);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9385);
	write_cmos_sensor(0x6F12, 0x4507);
	write_cmos_sensor(0x6F12, 0x1305);
	write_cmos_sensor(0x6F12, 0xC504);
	write_cmos_sensor(0x6F12, 0x2320);
	write_cmos_sensor(0x6F12, 0xF73C);
	write_cmos_sensor(0x6F12, 0x9710);
	write_cmos_sensor(0x6F12, 0x00FC);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0x603C);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x23A0);
	write_cmos_sensor(0x6F12, 0xA710);
	write_cmos_sensor(0x6F12, 0x4928);
	write_cmos_sensor(0x6F12, 0xE177);
	write_cmos_sensor(0x6F12, 0x3747);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x5776);
	write_cmos_sensor(0x6F12, 0x2317);
	write_cmos_sensor(0x6F12, 0xF782);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0x6700);
	write_cmos_sensor(0x6F12, 0xE3F0);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0xE702);
	write_cmos_sensor(0x6F12, 0x83EC);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x83A4);
	write_cmos_sensor(0x6F12, 0x0710);
	write_cmos_sensor(0x6F12, 0xAA89);
	write_cmos_sensor(0x6F12, 0x2E8A);
	write_cmos_sensor(0x6F12, 0x0146);
	write_cmos_sensor(0x6F12, 0xA685);
	write_cmos_sensor(0x6F12, 0x1145);
	write_cmos_sensor(0x6F12, 0x9780);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0xA031);
	write_cmos_sensor(0x6F12, 0xB787);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x03C7);
	write_cmos_sensor(0x6F12, 0x4710);
	write_cmos_sensor(0x6F12, 0x3E84);
	write_cmos_sensor(0x6F12, 0x0149);
	write_cmos_sensor(0x6F12, 0x11CF);
	write_cmos_sensor(0x6F12, 0x3767);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x0357);
	write_cmos_sensor(0x6F12, 0x2777);
	write_cmos_sensor(0x6F12, 0xB777);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x07C7);
	write_cmos_sensor(0x6F12, 0x0E07);
	write_cmos_sensor(0x6F12, 0x03D9);
	write_cmos_sensor(0x6F12, 0x871C);
	write_cmos_sensor(0x6F12, 0x2394);
	write_cmos_sensor(0x6F12, 0xE71C);
	write_cmos_sensor(0x6F12, 0xD285);
	write_cmos_sensor(0x6F12, 0x4E85);
	write_cmos_sensor(0x6F12, 0x97D0);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0xA0F8);
	write_cmos_sensor(0x6F12, 0x8347);
	write_cmos_sensor(0x6F12, 0x4410);
	write_cmos_sensor(0x6F12, 0x89C7);
	write_cmos_sensor(0x6F12, 0xB777);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x239C);
	write_cmos_sensor(0x6F12, 0x27E3);
	write_cmos_sensor(0x6F12, 0x0546);
	write_cmos_sensor(0x6F12, 0xA685);
	write_cmos_sensor(0x6F12, 0x1145);
	write_cmos_sensor(0x6F12, 0x9780);
	write_cmos_sensor(0x6F12, 0xFFFB);
	write_cmos_sensor(0x6F12, 0xE780);
	write_cmos_sensor(0x6F12, 0xA02C);
	write_cmos_sensor(0x6F12, 0x1743);
	write_cmos_sensor(0x6F12, 0x01FC);
	write_cmos_sensor(0x6F12, 0x6700);
	write_cmos_sensor(0x6F12, 0xA3E8);
	write_cmos_sensor(0x6F12, 0xE177);
	write_cmos_sensor(0x6F12, 0x3747);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6F12, 0x9387);
	write_cmos_sensor(0x6F12, 0x5776);
	write_cmos_sensor(0x6F12, 0x2318);
	write_cmos_sensor(0x6F12, 0xF782);
	write_cmos_sensor(0x6F12, 0x8280);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x35CC);
	write_cmos_sensor(0x6F12, 0x1C80);
	write_cmos_sensor(0x6F12, 0x0024);
	write_cmos_sensor(0x6028, 0x2400);
	write_cmos_sensor(0x602A, 0x1354);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6F12, 0x7017);
	write_cmos_sensor(0x602A, 0x13B2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1236);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1A0A);
	write_cmos_sensor(0x6F12, 0x4C0A);
	write_cmos_sensor(0x602A, 0x2210);
	write_cmos_sensor(0x6F12, 0x3401);
	write_cmos_sensor(0x602A, 0x2176);
	write_cmos_sensor(0x6F12, 0x6400);
	write_cmos_sensor(0x602A, 0x222E);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x06B6);
	write_cmos_sensor(0x6F12, 0x0A00);
	write_cmos_sensor(0x602A, 0x06BC);
	write_cmos_sensor(0x6F12, 0x1001);
	write_cmos_sensor(0x602A, 0x2140);
	write_cmos_sensor(0x6F12, 0x0101);
	write_cmos_sensor(0x602A, 0x218E);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1A0E);
	write_cmos_sensor(0x6F12, 0x9600);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0xF44E, 0x0011);
	write_cmos_sensor(0xF44C, 0x0B0B);
	write_cmos_sensor(0xF44A, 0x0006);
	write_cmos_sensor(0x0118, 0x0002);
	write_cmos_sensor(0x011A, 0x0001);
}


static void preview_setting(void)
{
    LOG_INF("preview_setting start\n");
	write_cmos_sensor(0x6028,0x2400);
	write_cmos_sensor(0x602A,0x1A28);
	write_cmos_sensor(0x6F12,0x4C00);
	write_cmos_sensor(0x602A,0x065A);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x139E);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x139C);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x13A0);
	write_cmos_sensor(0x6F12,0x0A00);
	write_cmos_sensor(0x6F12,0x0120);
	write_cmos_sensor(0x602A,0x2072);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x1A64);
	write_cmos_sensor(0x6F12,0x0301);
	write_cmos_sensor(0x6F12,0xFF00);
	write_cmos_sensor(0x602A,0x19E6);
	write_cmos_sensor(0x6F12,0x0200);
	write_cmos_sensor(0x602A,0x1A30);
	write_cmos_sensor(0x6F12,0x3401);
	write_cmos_sensor(0x602A,0x19FC);
	write_cmos_sensor(0x6F12,0x0B00);
	write_cmos_sensor(0x602A,0x19F4);
	write_cmos_sensor(0x6F12,0x0606);
	write_cmos_sensor(0x602A,0x19F8);
	write_cmos_sensor(0x6F12,0x1010);
	write_cmos_sensor(0x602A,0x1B26);
	write_cmos_sensor(0x6F12,0x6F80);
	write_cmos_sensor(0x6F12,0xA060);
	write_cmos_sensor(0x602A,0x1A3C);
	write_cmos_sensor(0x6F12,0x6207);
	write_cmos_sensor(0x602A,0x1A48);
	write_cmos_sensor(0x6F12,0x6207);
	write_cmos_sensor(0x602A,0x1444);
	write_cmos_sensor(0x6F12,0x2000);
	write_cmos_sensor(0x6F12,0x2000);
	write_cmos_sensor(0x602A,0x144C);
	write_cmos_sensor(0x6F12,0x3F00);
	write_cmos_sensor(0x6F12,0x3F00);
	write_cmos_sensor(0x602A,0x7F6C);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x6F12,0x2F00);
	write_cmos_sensor(0x6F12,0xFA00);
	write_cmos_sensor(0x6F12,0x2400);
	write_cmos_sensor(0x6F12,0xE500);
	write_cmos_sensor(0x602A,0x0650);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x602A,0x0654);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x1A46);
	write_cmos_sensor(0x6F12,0xB000);
	write_cmos_sensor(0x602A,0x1A52);
	write_cmos_sensor(0x6F12,0xBF00);
	write_cmos_sensor(0x602A,0x0674);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x602A,0x0668);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x602A,0x0684);
	write_cmos_sensor(0x6F12,0x4001);
	write_cmos_sensor(0x602A,0x0688);
	write_cmos_sensor(0x6F12,0x4001);
	write_cmos_sensor(0x602A,0x147C);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x1480);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x19F6);
	write_cmos_sensor(0x6F12,0x0904);
	write_cmos_sensor(0x602A,0x0812);
	write_cmos_sensor(0x6F12,0x0010);
	write_cmos_sensor(0x602A,0x2148);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x2042);
	write_cmos_sensor(0x6F12,0x1A00);
	write_cmos_sensor(0x602A,0x0874);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x09C0);
	write_cmos_sensor(0x6F12,0x2008);
	write_cmos_sensor(0x602A,0x09C4);
	write_cmos_sensor(0x6F12,0x2000);
	write_cmos_sensor(0x602A,0x19FE);
	write_cmos_sensor(0x6F12,0x0E1C);
	write_cmos_sensor(0x602A,0x4D92);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x8104);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x4D94);
	write_cmos_sensor(0x6F12,0x0005);
	write_cmos_sensor(0x6F12,0x000A);
	write_cmos_sensor(0x6F12,0x0010);
	write_cmos_sensor(0x6F12,0x1510);
	write_cmos_sensor(0x6F12,0x000A);
	write_cmos_sensor(0x6F12,0x0040);
	write_cmos_sensor(0x6F12,0x1510);
	write_cmos_sensor(0x6F12,0x1510);
	write_cmos_sensor(0x602A,0x3570);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x3574);
	write_cmos_sensor(0x6F12,0x1304);
	write_cmos_sensor(0x602A,0x21E4);
	write_cmos_sensor(0x6F12,0x0400);
	write_cmos_sensor(0x602A,0x21EC);
	write_cmos_sensor(0x6F12,0x1D02);
	write_cmos_sensor(0x602A,0x2080);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x6F12,0xFF00);
	write_cmos_sensor(0x602A,0x2086);
	write_cmos_sensor(0x6F12,0x0001);
	write_cmos_sensor(0x602A,0x208E);
	write_cmos_sensor(0x6F12,0x14F4);
	write_cmos_sensor(0x602A,0x208A);
	write_cmos_sensor(0x6F12,0xD244);
	write_cmos_sensor(0x6F12,0xD244);
	write_cmos_sensor(0x602A,0x120E);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x212E);
	write_cmos_sensor(0x6F12,0x0200);
	write_cmos_sensor(0x602A,0x13AE);
	write_cmos_sensor(0x6F12,0x0101);
	write_cmos_sensor(0x602A,0x0718);
	write_cmos_sensor(0x6F12,0x0001);
	write_cmos_sensor(0x602A,0x0710);
	write_cmos_sensor(0x6F12,0x0002);
	write_cmos_sensor(0x6F12,0x0804);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x1B5C);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x0786);
	write_cmos_sensor(0x6F12,0x7701);
	write_cmos_sensor(0x602A,0x2022);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x602A,0x1360);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x1376);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x6F12,0x6038);
	write_cmos_sensor(0x6F12,0x7038);
	write_cmos_sensor(0x6F12,0x8038);
	write_cmos_sensor(0x602A,0x1386);
	write_cmos_sensor(0x6F12,0x0B00);
	write_cmos_sensor(0x602A,0x06FA);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x4A94);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x602A,0x0A76);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0AEE);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0B66);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0BDE);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0C56);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0CF2);
	write_cmos_sensor(0x6F12,0x0001);
	write_cmos_sensor(0x602A,0x0CF0);
	write_cmos_sensor(0x6F12,0x0101);
	write_cmos_sensor(0x602A,0x11B8);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x11F6);
	write_cmos_sensor(0x6F12,0x0020);
	write_cmos_sensor(0x602A,0x4A74);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0xD8FF);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0xD8FF);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6028,0x4000);
	write_cmos_sensor(0xF46A,0xAE80);
	write_cmos_sensor(0x0344,0x0000);
	write_cmos_sensor(0x0346,0x0000);
	write_cmos_sensor(0x0348,0x1FFF);
	write_cmos_sensor(0x034A,0x181F);
	write_cmos_sensor(0x034C,0x0FF0);
	write_cmos_sensor(0x034E,0x0C00);
	write_cmos_sensor(0x0350,0x0008);
	write_cmos_sensor(0x0352,0x0008);
	write_cmos_sensor(0x0900,0x0122);
	write_cmos_sensor(0x0380,0x0002);
	write_cmos_sensor(0x0382,0x0002);
	write_cmos_sensor(0x0384,0x0002);
	write_cmos_sensor(0x0386,0x0002);
	write_cmos_sensor(0x0110,0x1002);
	write_cmos_sensor(0x0114,0x0301);
	write_cmos_sensor(0x0116,0x3000);
	write_cmos_sensor(0x0136,0x1800);
	write_cmos_sensor(0x013E,0x0000);
	write_cmos_sensor(0x0300,0x0006);
	write_cmos_sensor(0x0302,0x0001);
	write_cmos_sensor(0x0304,0x0004);
	write_cmos_sensor(0x0306,0x008C);
	write_cmos_sensor(0x0308,0x0008);
	write_cmos_sensor(0x030A,0x0001);
	write_cmos_sensor(0x030C,0x0000);
	write_cmos_sensor(0x030E,0x0004);
	write_cmos_sensor(0x0310,0x0067);
	write_cmos_sensor(0x0312,0x0000);
	write_cmos_sensor(0x080E,0x0000);
	write_cmos_sensor(0x0340,0x0C54);
	write_cmos_sensor(0x0342,0x1716);
	write_cmos_sensor(0x0702,0x0000);
	write_cmos_sensor(0x0202,0x0100);
	write_cmos_sensor(0x0200,0x0100);
	write_cmos_sensor(0x0D00,0x0101);
	write_cmos_sensor(0x0D02,0x0101);
	write_cmos_sensor(0x0D04,0x0102);
	write_cmos_sensor(0x6226,0x0000);
}


static void hs_video_setting(void)
{

    LOG_INF("hs_video_setting RES_1280x720_120fps\n");
    write_cmos_sensor(0x6028, 0x2400);
    write_cmos_sensor(0x602A, 0x1A28);
    write_cmos_sensor(0x6F12, 0x4C00);
    write_cmos_sensor(0x602A, 0x065A);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x602A, 0x139E);
    write_cmos_sensor(0x6F12, 0x0300);
    write_cmos_sensor(0x602A, 0x139C);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x602A, 0x13A0);
    write_cmos_sensor(0x6F12, 0x0A00);
    write_cmos_sensor(0x6F12, 0x0020);
    write_cmos_sensor(0x602A, 0x2072);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x602A, 0x1A64);
    write_cmos_sensor(0x6F12, 0x0301);
    write_cmos_sensor(0x6F12, 0xFF00);
    write_cmos_sensor(0x602A, 0x19E6);
    write_cmos_sensor(0x6F12, 0x0201);
    write_cmos_sensor(0x602A, 0x1A30);
    write_cmos_sensor(0x6F12, 0x3401);
    write_cmos_sensor(0x602A, 0x19FC);
    write_cmos_sensor(0x6F12, 0x0B00);
    write_cmos_sensor(0x602A, 0x19F4);
    write_cmos_sensor(0x6F12, 0x0606);
    write_cmos_sensor(0x602A, 0x19F8);
    write_cmos_sensor(0x6F12, 0x1010);
    write_cmos_sensor(0x602A, 0x1B26);
    write_cmos_sensor(0x6F12, 0x6F80);
    write_cmos_sensor(0x6F12, 0xA020);
    write_cmos_sensor(0x602A, 0x1A3C);
    write_cmos_sensor(0x6F12, 0x5207);
    write_cmos_sensor(0x602A, 0x1A48);
    write_cmos_sensor(0x6F12, 0x5207);
    write_cmos_sensor(0x602A, 0x1444);
    write_cmos_sensor(0x6F12, 0x2100);
    write_cmos_sensor(0x6F12, 0x2100);
    write_cmos_sensor(0x602A, 0x144C);
    write_cmos_sensor(0x6F12, 0x4200);
    write_cmos_sensor(0x6F12, 0x4200);
    write_cmos_sensor(0x602A, 0x7F6C);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x3100);
    write_cmos_sensor(0x6F12, 0xF700);
    write_cmos_sensor(0x6F12, 0x2600);
    write_cmos_sensor(0x6F12, 0xE100);
    write_cmos_sensor(0x602A, 0x0650);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x602A, 0x0654);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x602A, 0x1A46);
    write_cmos_sensor(0x6F12, 0x8900);
    write_cmos_sensor(0x602A, 0x1A52);
    write_cmos_sensor(0x6F12, 0xBF00);
    write_cmos_sensor(0x602A, 0x0674);
    write_cmos_sensor(0x6F12, 0x0500);
    write_cmos_sensor(0x6F12, 0x0500);
    write_cmos_sensor(0x6F12, 0x0500);
    write_cmos_sensor(0x6F12, 0x0500);
    write_cmos_sensor(0x602A, 0x0668);
    write_cmos_sensor(0x6F12, 0x0800);
    write_cmos_sensor(0x6F12, 0x0800);
    write_cmos_sensor(0x6F12, 0x0800);
    write_cmos_sensor(0x6F12, 0x0800);
    write_cmos_sensor(0x602A, 0x0684);
    write_cmos_sensor(0x6F12, 0x4001);
    write_cmos_sensor(0x602A, 0x0688);
    write_cmos_sensor(0x6F12, 0x4001);
    write_cmos_sensor(0x602A, 0x147C);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x602A, 0x1480);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x602A, 0x19F6);
    write_cmos_sensor(0x6F12, 0x0904);
    write_cmos_sensor(0x602A, 0x0812);
    write_cmos_sensor(0x6F12, 0x0010);
    write_cmos_sensor(0x602A, 0x2148);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x602A, 0x2042);
    write_cmos_sensor(0x6F12, 0x1A00);
    write_cmos_sensor(0x602A, 0x0874);
    write_cmos_sensor(0x6F12, 0x1100);
    write_cmos_sensor(0x602A, 0x09C0);
    write_cmos_sensor(0x6F12, 0x1803);
    write_cmos_sensor(0x602A, 0x09C4);
    write_cmos_sensor(0x6F12, 0x1803);
    write_cmos_sensor(0x602A, 0x19FE);
    write_cmos_sensor(0x6F12, 0x0E1C);
    write_cmos_sensor(0x602A, 0x4D92);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x602A, 0x8104);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x602A, 0x4D94);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x602A, 0x3570);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x602A, 0x3574);
    write_cmos_sensor(0x6F12, 0x3801);
    write_cmos_sensor(0x602A, 0x21E4);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x602A, 0x21EC);
    write_cmos_sensor(0x6F12, 0x6801);
    write_cmos_sensor(0x602A, 0x2080);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0xFF01);
    write_cmos_sensor(0x602A, 0x2086);
    write_cmos_sensor(0x6F12, 0x0002);
    write_cmos_sensor(0x602A, 0x208E);
    write_cmos_sensor(0x6F12, 0x14F4);
    write_cmos_sensor(0x602A, 0x208A);
    write_cmos_sensor(0x6F12, 0xC244);
    write_cmos_sensor(0x6F12, 0xD244);
    write_cmos_sensor(0x602A, 0x120E);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x602A, 0x212E);
    write_cmos_sensor(0x6F12, 0x0A00);
    write_cmos_sensor(0x602A, 0x13AE);
    write_cmos_sensor(0x6F12, 0x0102);
    write_cmos_sensor(0x602A, 0x0718);
    write_cmos_sensor(0x6F12, 0x0005);
    write_cmos_sensor(0x602A, 0x0710);
    write_cmos_sensor(0x6F12, 0x0004);
    write_cmos_sensor(0x6F12, 0x0401);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x602A, 0x1B5C);
    write_cmos_sensor(0x6F12, 0x0300);
    write_cmos_sensor(0x602A, 0x0786);
    write_cmos_sensor(0x6F12, 0x7701);
    write_cmos_sensor(0x602A, 0x2022);
    write_cmos_sensor(0x6F12, 0x0101);
    write_cmos_sensor(0x6F12, 0x0101);
    write_cmos_sensor(0x602A, 0x1360);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x602A, 0x1376);
    write_cmos_sensor(0x6F12, 0x0200);
    write_cmos_sensor(0x6F12, 0x6038);
    write_cmos_sensor(0x6F12, 0x7038);
    write_cmos_sensor(0x6F12, 0x8038);
    write_cmos_sensor(0x602A, 0x1386);
    write_cmos_sensor(0x6F12, 0x0B00);
    write_cmos_sensor(0x602A, 0x06FA);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x602A, 0x4A94);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x602A, 0x0A76);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x602A, 0x0AEE);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x602A, 0x0B66);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x602A, 0x0BDE);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x602A, 0x0C56);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x602A, 0x0CF2);
    write_cmos_sensor(0x6F12, 0x0001);
    write_cmos_sensor(0x602A, 0x0CF0);
    write_cmos_sensor(0x6F12, 0x0101);
    write_cmos_sensor(0x602A, 0x11B8);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x602A, 0x11F6);
    write_cmos_sensor(0x6F12, 0x0010);
    write_cmos_sensor(0x602A, 0x4A74);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6028, 0x4000);
    write_cmos_sensor(0xF46A, 0xAE80);
    write_cmos_sensor(0x0344, 0x05F0);
    write_cmos_sensor(0x0346, 0x0660);
    write_cmos_sensor(0x0348, 0x1A0F);
    write_cmos_sensor(0x034A, 0x11BF);
    write_cmos_sensor(0x034C, 0x0500);
    write_cmos_sensor(0x034E, 0x02D0);
    write_cmos_sensor(0x0350, 0x0004);
    write_cmos_sensor(0x0352, 0x0004);
    write_cmos_sensor(0x0900, 0x0144);
    write_cmos_sensor(0x0380, 0x0002);
    write_cmos_sensor(0x0382, 0x0006);
    write_cmos_sensor(0x0384, 0x0002);
    write_cmos_sensor(0x0386, 0x0006);
    write_cmos_sensor(0x0110, 0x1002);
    write_cmos_sensor(0x0114, 0x0300);
    write_cmos_sensor(0x0116, 0x3000);
    write_cmos_sensor(0x0136, 0x1800);
    write_cmos_sensor(0x013E, 0x0000);
    write_cmos_sensor(0x0300, 0x0006);
    write_cmos_sensor(0x0302, 0x0001);
    write_cmos_sensor(0x0304, 0x0004);
    write_cmos_sensor(0x0306, 0x0096);
    write_cmos_sensor(0x0308, 0x0008);
    write_cmos_sensor(0x030A, 0x0001);
    write_cmos_sensor(0x030C, 0x0000);
    write_cmos_sensor(0x030E, 0x0003);
    write_cmos_sensor(0x0310, 0x0064);
    write_cmos_sensor(0x0312, 0x0000);
    write_cmos_sensor(0x080E, 0x0000);
    write_cmos_sensor(0x0340, 0x094C);
    write_cmos_sensor(0x0342, 0x0830);
    write_cmos_sensor(0x0702, 0x0000);
    write_cmos_sensor(0x0202, 0x0100);
    write_cmos_sensor(0x0200, 0x0100);
    write_cmos_sensor(0x0D00, 0x0101);
    write_cmos_sensor(0x0D02, 0x0001);
    write_cmos_sensor(0x0D04, 0x0102);
    write_cmos_sensor(0x6226, 0x0000);
}

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("start\n");
	write_cmos_sensor(0x6028,0x2400);
	write_cmos_sensor(0x602A,0x1A28);
	write_cmos_sensor(0x6F12,0x4C00);
	write_cmos_sensor(0x602A,0x065A);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x139E);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x139C);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x13A0);
	write_cmos_sensor(0x6F12,0x0A00);
	write_cmos_sensor(0x6F12,0x0120);
	write_cmos_sensor(0x602A,0x2072);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x1A64);
	write_cmos_sensor(0x6F12,0x0301);
	write_cmos_sensor(0x6F12,0xFF00);
	write_cmos_sensor(0x602A,0x19E6);
	write_cmos_sensor(0x6F12,0x0200);
	write_cmos_sensor(0x602A,0x1A30);
	write_cmos_sensor(0x6F12,0x3401);
	write_cmos_sensor(0x602A,0x19FC);
	write_cmos_sensor(0x6F12,0x0B00);
	write_cmos_sensor(0x602A,0x19F4);
	write_cmos_sensor(0x6F12,0x0606);
	write_cmos_sensor(0x602A,0x19F8);
	write_cmos_sensor(0x6F12,0x1010);
	write_cmos_sensor(0x602A,0x1B26);
	write_cmos_sensor(0x6F12,0x6F80);
	write_cmos_sensor(0x6F12,0xA060);
	write_cmos_sensor(0x602A,0x1A3C);
	write_cmos_sensor(0x6F12,0x6207);
	write_cmos_sensor(0x602A,0x1A48);
	write_cmos_sensor(0x6F12,0x6207);
	write_cmos_sensor(0x602A,0x1444);
	write_cmos_sensor(0x6F12,0x2000);
	write_cmos_sensor(0x6F12,0x2000);
	write_cmos_sensor(0x602A,0x144C);
	write_cmos_sensor(0x6F12,0x3F00);
	write_cmos_sensor(0x6F12,0x3F00);
	write_cmos_sensor(0x602A,0x7F6C);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x6F12,0x2F00);
	write_cmos_sensor(0x6F12,0xFA00);
	write_cmos_sensor(0x6F12,0x2400);
	write_cmos_sensor(0x6F12,0xE500);
	write_cmos_sensor(0x602A,0x0650);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x602A,0x0654);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x1A46);
	write_cmos_sensor(0x6F12,0xB000);
	write_cmos_sensor(0x602A,0x1A52);
	write_cmos_sensor(0x6F12,0xBF00);
	write_cmos_sensor(0x602A,0x0674);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x602A,0x0668);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x602A,0x0684);
	write_cmos_sensor(0x6F12,0x4001);
	write_cmos_sensor(0x602A,0x0688);
	write_cmos_sensor(0x6F12,0x4001);
	write_cmos_sensor(0x602A,0x147C);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x1480);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x19F6);
	write_cmos_sensor(0x6F12,0x0904);
	write_cmos_sensor(0x602A,0x0812);
	write_cmos_sensor(0x6F12,0x0010);
	write_cmos_sensor(0x602A,0x2148);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x2042);
	write_cmos_sensor(0x6F12,0x1A00);
	write_cmos_sensor(0x602A,0x0874);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x09C0);
	write_cmos_sensor(0x6F12,0x2008);
	write_cmos_sensor(0x602A,0x09C4);
	write_cmos_sensor(0x6F12,0x2000);
	write_cmos_sensor(0x602A,0x19FE);
	write_cmos_sensor(0x6F12,0x0E1C);
	write_cmos_sensor(0x602A,0x4D92);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x8104);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x4D94);
	write_cmos_sensor(0x6F12,0x0005);
	write_cmos_sensor(0x6F12,0x000A);
	write_cmos_sensor(0x6F12,0x0010);
	write_cmos_sensor(0x6F12,0x1510);
	write_cmos_sensor(0x6F12,0x000A);
	write_cmos_sensor(0x6F12,0x0040);
	write_cmos_sensor(0x6F12,0x1510);
	write_cmos_sensor(0x6F12,0x1510);
	write_cmos_sensor(0x602A,0x3570);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x3574);
	write_cmos_sensor(0x6F12,0x1304);
	write_cmos_sensor(0x602A,0x21E4);
	write_cmos_sensor(0x6F12,0x0400);
	write_cmos_sensor(0x602A,0x21EC);
	write_cmos_sensor(0x6F12,0x1D02);
	write_cmos_sensor(0x602A,0x2080);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x6F12,0xFF00);
	write_cmos_sensor(0x602A,0x2086);
	write_cmos_sensor(0x6F12,0x0001);
	write_cmos_sensor(0x602A,0x208E);
	write_cmos_sensor(0x6F12,0x14F4);
	write_cmos_sensor(0x602A,0x208A);
	write_cmos_sensor(0x6F12,0xD244);
	write_cmos_sensor(0x6F12,0xD244);
	write_cmos_sensor(0x602A,0x120E);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x212E);
	write_cmos_sensor(0x6F12,0x0200);
	write_cmos_sensor(0x602A,0x13AE);
	write_cmos_sensor(0x6F12,0x0101);
	write_cmos_sensor(0x602A,0x0718);
	write_cmos_sensor(0x6F12,0x0001);
	write_cmos_sensor(0x602A,0x0710);
	write_cmos_sensor(0x6F12,0x0002);
	write_cmos_sensor(0x6F12,0x0804);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x1B5C);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x0786);
	write_cmos_sensor(0x6F12,0x7701);
	write_cmos_sensor(0x602A,0x2022);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x602A,0x1360);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x1376);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x6F12,0x6038);
	write_cmos_sensor(0x6F12,0x7038);
	write_cmos_sensor(0x6F12,0x8038);
	write_cmos_sensor(0x602A,0x1386);
	write_cmos_sensor(0x6F12,0x0B00);
	write_cmos_sensor(0x602A,0x06FA);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x4A94);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x602A,0x0A76);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0AEE);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0B66);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0BDE);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0C56);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0CF2);
	write_cmos_sensor(0x6F12,0x0001);
	write_cmos_sensor(0x602A,0x0CF0);
	write_cmos_sensor(0x6F12,0x0101);
	write_cmos_sensor(0x602A,0x11B8);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x11F6);
	write_cmos_sensor(0x6F12,0x0020);
	write_cmos_sensor(0x602A,0x4A74);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0xD8FF);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0xD8FF);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6028,0x4000);
	write_cmos_sensor(0xF46A,0xAE80);
	write_cmos_sensor(0x0344,0x0000);
	write_cmos_sensor(0x0346,0x0000);
	write_cmos_sensor(0x0348,0x1FFF);
	write_cmos_sensor(0x034A,0x181F);
	write_cmos_sensor(0x034C,0x0FF0);
	write_cmos_sensor(0x034E,0x0C00);
	write_cmos_sensor(0x0350,0x0008);
	write_cmos_sensor(0x0352,0x0008);
	write_cmos_sensor(0x0900,0x0122);
	write_cmos_sensor(0x0380,0x0002);
	write_cmos_sensor(0x0382,0x0002);
	write_cmos_sensor(0x0384,0x0002);
	write_cmos_sensor(0x0386,0x0002);
	write_cmos_sensor(0x0110,0x1002);
	write_cmos_sensor(0x0114,0x0301);
	write_cmos_sensor(0x0116,0x3000);
	write_cmos_sensor(0x0136,0x1800);
	write_cmos_sensor(0x013E,0x0000);
	write_cmos_sensor(0x0300,0x0006);
	write_cmos_sensor(0x0302,0x0001);
	write_cmos_sensor(0x0304,0x0004);
	write_cmos_sensor(0x0306,0x008C);
	write_cmos_sensor(0x0308,0x0008);
	write_cmos_sensor(0x030A,0x0001);
	write_cmos_sensor(0x030C,0x0000);
	write_cmos_sensor(0x030E,0x0004);
	write_cmos_sensor(0x0310,0x0067);
	write_cmos_sensor(0x0312,0x0000);
	write_cmos_sensor(0x080E,0x0000);
	write_cmos_sensor(0x0340,0x0C54);
	write_cmos_sensor(0x0342,0x1716);
	write_cmos_sensor(0x0702,0x0000);
	write_cmos_sensor(0x0202,0x0100);
	write_cmos_sensor(0x0200,0x0100);
	write_cmos_sensor(0x0D00,0x0101);
	write_cmos_sensor(0x0D02,0x0101);
	write_cmos_sensor(0x0D04,0x0102);
	write_cmos_sensor(0x6226,0x0000);
}


static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("normal_video_setting start\n");
write_cmos_sensor(0x6028, 0x2400);
write_cmos_sensor(0x602A, 0x1A28);
write_cmos_sensor(0x6F12, 0x4C00);
write_cmos_sensor(0x602A, 0x065A);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x139E);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x139C);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x13A0);
write_cmos_sensor(0x6F12, 0x0A00);
write_cmos_sensor(0x6F12, 0x0120);
write_cmos_sensor(0x602A, 0x2072);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x1A64);
write_cmos_sensor(0x6F12, 0x0301);
write_cmos_sensor(0x6F12, 0xFF00);
write_cmos_sensor(0x602A, 0x19E6);
write_cmos_sensor(0x6F12, 0x0200);
write_cmos_sensor(0x602A, 0x1A30);
write_cmos_sensor(0x6F12, 0x3401);
write_cmos_sensor(0x602A, 0x19FC);
write_cmos_sensor(0x6F12, 0x0B00);
write_cmos_sensor(0x602A, 0x19F4);
write_cmos_sensor(0x6F12, 0x0606);
write_cmos_sensor(0x602A, 0x19F8);
write_cmos_sensor(0x6F12, 0x1010);
write_cmos_sensor(0x602A, 0x1B26);
write_cmos_sensor(0x6F12, 0x6F80);
write_cmos_sensor(0x6F12, 0xA060);
write_cmos_sensor(0x602A, 0x1A3C);
write_cmos_sensor(0x6F12, 0x6207);
write_cmos_sensor(0x602A, 0x1A48);
write_cmos_sensor(0x6F12, 0x6207);
write_cmos_sensor(0x602A, 0x1444);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x602A, 0x144C);
write_cmos_sensor(0x6F12, 0x3F00);
write_cmos_sensor(0x6F12, 0x3F00);
write_cmos_sensor(0x602A, 0x7F6C);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x6F12, 0x2F00);
write_cmos_sensor(0x6F12, 0xFA00);
write_cmos_sensor(0x6F12, 0x2400);
write_cmos_sensor(0x6F12, 0xE500);
write_cmos_sensor(0x602A, 0x0650);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x602A, 0x0654);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x1A46);
write_cmos_sensor(0x6F12, 0xB000);
write_cmos_sensor(0x602A, 0x1A52);
write_cmos_sensor(0x6F12, 0xBF00);
write_cmos_sensor(0x602A, 0x0674);
write_cmos_sensor(0x6F12, 0x0500);
write_cmos_sensor(0x6F12, 0x0500);
write_cmos_sensor(0x6F12, 0x0500);
write_cmos_sensor(0x6F12, 0x0500);
write_cmos_sensor(0x602A, 0x0668);
write_cmos_sensor(0x6F12, 0x0800);
write_cmos_sensor(0x6F12, 0x0800);
write_cmos_sensor(0x6F12, 0x0800);
write_cmos_sensor(0x6F12, 0x0800);
write_cmos_sensor(0x602A, 0x0684);
write_cmos_sensor(0x6F12, 0x4001);
write_cmos_sensor(0x602A, 0x0688);
write_cmos_sensor(0x6F12, 0x4001);
write_cmos_sensor(0x602A, 0x147C);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0x1480);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0x19F6);
write_cmos_sensor(0x6F12, 0x0904);
write_cmos_sensor(0x602A, 0x0812);
write_cmos_sensor(0x6F12, 0x0010);
write_cmos_sensor(0x602A, 0x2148);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x2042);
write_cmos_sensor(0x6F12, 0x1A00);
write_cmos_sensor(0x602A, 0x0874);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x09C0);
write_cmos_sensor(0x6F12, 0x2008);
write_cmos_sensor(0x602A, 0x09C4);
write_cmos_sensor(0x6F12, 0x2000);
write_cmos_sensor(0x602A, 0x19FE);
write_cmos_sensor(0x6F12, 0x0E1C);
write_cmos_sensor(0x602A, 0x4D92);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x8104);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x4D94);
write_cmos_sensor(0x6F12, 0x0005);
write_cmos_sensor(0x6F12, 0x000A);
write_cmos_sensor(0x6F12, 0x0010);
write_cmos_sensor(0x6F12, 0x1510);
write_cmos_sensor(0x6F12, 0x000A);
write_cmos_sensor(0x6F12, 0x0040);
write_cmos_sensor(0x6F12, 0x1510);
write_cmos_sensor(0x6F12, 0x1510);
write_cmos_sensor(0x602A, 0x3570);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x3574);
write_cmos_sensor(0x6F12, 0x4700);
write_cmos_sensor(0x602A, 0x21E4);
write_cmos_sensor(0x6F12, 0x0400);
write_cmos_sensor(0x602A, 0x21EC);
write_cmos_sensor(0x6F12, 0xC702);
write_cmos_sensor(0x602A, 0x2080);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x6F12, 0xFF00);
write_cmos_sensor(0x602A, 0x2086);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x602A, 0x208E);
write_cmos_sensor(0x6F12, 0x14F4);
write_cmos_sensor(0x602A, 0x208A);
write_cmos_sensor(0x6F12, 0xD244);
write_cmos_sensor(0x6F12, 0xD244);
write_cmos_sensor(0x602A, 0x120E);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0x212E);
write_cmos_sensor(0x6F12, 0x0200);
write_cmos_sensor(0x602A, 0x13AE);
write_cmos_sensor(0x6F12, 0x0101);
write_cmos_sensor(0x602A, 0x0718);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x602A, 0x0710);
write_cmos_sensor(0x6F12, 0x0002);
write_cmos_sensor(0x6F12, 0x0804);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x1B5C);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x0786);
write_cmos_sensor(0x6F12, 0x7701);
write_cmos_sensor(0x602A, 0x2022);
write_cmos_sensor(0x6F12, 0x0500);
write_cmos_sensor(0x6F12, 0x0500);
write_cmos_sensor(0x602A, 0x1360);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x1376);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x6F12, 0x6038);
write_cmos_sensor(0x6F12, 0x7038);
write_cmos_sensor(0x6F12, 0x8038);
write_cmos_sensor(0x602A, 0x1386);
write_cmos_sensor(0x6F12, 0x0B00);
write_cmos_sensor(0x602A, 0x06FA);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x602A, 0x4A94);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x6F12, 0x0600);
write_cmos_sensor(0x602A, 0x0A76);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0x0AEE);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0x0B66);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0x0BDE);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0x0C56);
write_cmos_sensor(0x6F12, 0x1000);
write_cmos_sensor(0x602A, 0x0CF2);
write_cmos_sensor(0x6F12, 0x0001);
write_cmos_sensor(0x602A, 0x0CF0);
write_cmos_sensor(0x6F12, 0x0101);
write_cmos_sensor(0x602A, 0x11B8);
write_cmos_sensor(0x6F12, 0x0100);
write_cmos_sensor(0x602A, 0x11F6);
write_cmos_sensor(0x6F12, 0x0020);
write_cmos_sensor(0x602A, 0x4A74);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0xD8FF);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0xD8FF);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6F12, 0x0000);
write_cmos_sensor(0x6028, 0x4000);
write_cmos_sensor(0xF46A, 0xAE80);
write_cmos_sensor(0x0344, 0x0000);
write_cmos_sensor(0x0346, 0x0308);
write_cmos_sensor(0x0348, 0x1FFF);
write_cmos_sensor(0x034A, 0x1517);
write_cmos_sensor(0x034C, 0x0FF0);
write_cmos_sensor(0x034E, 0x08F8);
write_cmos_sensor(0x0350, 0x0008);
write_cmos_sensor(0x0352, 0x0008);
write_cmos_sensor(0x0900, 0x0122);
write_cmos_sensor(0x0380, 0x0002);
write_cmos_sensor(0x0382, 0x0002);
write_cmos_sensor(0x0384, 0x0002);
write_cmos_sensor(0x0386, 0x0002);
write_cmos_sensor(0x0110, 0x1002);
write_cmos_sensor(0x0114, 0x0301);
write_cmos_sensor(0x0116, 0x3000);
write_cmos_sensor(0x0136, 0x1800);
write_cmos_sensor(0x013E, 0x0000);
write_cmos_sensor(0x0300, 0x0006);
write_cmos_sensor(0x0302, 0x0001);
write_cmos_sensor(0x0304, 0x0004);
write_cmos_sensor(0x0306, 0x008C);
write_cmos_sensor(0x0308, 0x0008);
write_cmos_sensor(0x030A, 0x0001);
write_cmos_sensor(0x030C, 0x0000);
write_cmos_sensor(0x030E, 0x0004);
write_cmos_sensor(0x0310, 0x006B);
write_cmos_sensor(0x0312, 0x0000);
write_cmos_sensor(0x080E, 0x0000);
write_cmos_sensor(0x0340, 0x0C54);
write_cmos_sensor(0x0342, 0x1716);
write_cmos_sensor(0x0702, 0x0000);
write_cmos_sensor(0x0202, 0x0100);
write_cmos_sensor(0x0200, 0x0100);
write_cmos_sensor(0x0D00, 0x0101);
write_cmos_sensor(0x0D02, 0x0101);
write_cmos_sensor(0x0D04, 0x0102);
write_cmos_sensor(0x6226, 0x0000);
}

static void slim_video_setting(void)
{

	LOG_INF("E\n");
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x6214, 0x7971);
	write_cmos_sensor(0x6218, 0x7150);
	write_cmos_sensor(0x0344, 0x0058);
	write_cmos_sensor(0x0346, 0x01AC);
	write_cmos_sensor(0x0348, 0x0F57);
	write_cmos_sensor(0x034A, 0x0A1B);
	write_cmos_sensor(0x034C, 0x0500);
	write_cmos_sensor(0x034E, 0x02D0);
	write_cmos_sensor(0x0350, 0x0000);
	write_cmos_sensor(0x0352, 0x0000);
	write_cmos_sensor(0x0340, 0x0330);
	write_cmos_sensor(0x0342, 0x13A0);
	write_cmos_sensor(0x0900, 0x0123);
	write_cmos_sensor(0x0380, 0x0001);
	write_cmos_sensor(0x0382, 0x0002);
	write_cmos_sensor(0x0384, 0x0001);
	write_cmos_sensor(0x0386, 0x0005);
	write_cmos_sensor(0x0404, 0x1000);
	write_cmos_sensor(0x0402, 0x1810);
	write_cmos_sensor(0x0136, 0x1800);
	write_cmos_sensor(0x0304, 0x0006);
	write_cmos_sensor(0x030C, 0x0000);
	write_cmos_sensor(0x0306, 0x00F6);
	write_cmos_sensor(0x0302, 0x0001);
	write_cmos_sensor(0x0300, 0x0008);
	write_cmos_sensor(0x030E, 0x0003);
	write_cmos_sensor(0x0312, 0x0002);
	write_cmos_sensor(0x0310, 0x005B);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1492);
	write_cmos_sensor(0x6F12, 0x0078);
	write_cmos_sensor(0x602A, 0x0E4E);
	write_cmos_sensor(0x6F12, 0xFFFF);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0118, 0x0104);
	write_cmos_sensor(0x021E, 0x0000);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x2126);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1168);
	write_cmos_sensor(0x6F12, 0x0020);
	write_cmos_sensor(0x602A, 0x2DB6);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1668);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x602A, 0x166A);
	write_cmos_sensor(0x6F12, 0xF0F0);
	write_cmos_sensor(0x602A, 0x118A);
	write_cmos_sensor(0x6F12, 0x0802);
	write_cmos_sensor(0x602A, 0x151E);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x217E);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1520);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x2522);
	write_cmos_sensor(0x6F12, 0x0804);
	write_cmos_sensor(0x602A, 0x2524);
	write_cmos_sensor(0x6F12, 0x0400);
	write_cmos_sensor(0x602A, 0x2568);
	write_cmos_sensor(0x6F12, 0x5500);
	write_cmos_sensor(0x602A, 0x2588);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x258C);
	write_cmos_sensor(0x6F12, 0x1111);
	write_cmos_sensor(0x602A, 0x25A6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x252C);
	write_cmos_sensor(0x6F12, 0x0601);
	write_cmos_sensor(0x602A, 0x252E);
	write_cmos_sensor(0x6F12, 0x0605);
	write_cmos_sensor(0x602A, 0x25A8);
	write_cmos_sensor(0x6F12, 0x1100);
	write_cmos_sensor(0x602A, 0x25AC);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x602A, 0x25B0);
	write_cmos_sensor(0x6F12, 0x1100);
	write_cmos_sensor(0x602A, 0x25B4);
	write_cmos_sensor(0x6F12, 0x0011);
	write_cmos_sensor(0x602A, 0x15A4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x602A, 0x15A6);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x602A, 0x15A8);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x602A, 0x15AA);
	write_cmos_sensor(0x6F12, 0x024D);
	write_cmos_sensor(0x602A, 0x15AC);
	write_cmos_sensor(0x6F12, 0x0151);
	write_cmos_sensor(0x602A, 0x15AE);
	write_cmos_sensor(0x6F12, 0x0555);
	write_cmos_sensor(0x602A, 0x15B0);
	write_cmos_sensor(0x6F12, 0x0659);
	write_cmos_sensor(0x602A, 0x15B2);
	write_cmos_sensor(0x6F12, 0x025D);
	write_cmos_sensor(0x602A, 0x15B4);
	write_cmos_sensor(0x6F12, 0x0161);
	write_cmos_sensor(0x602A, 0x15B6);
	write_cmos_sensor(0x6F12, 0x0565);
	write_cmos_sensor(0x602A, 0x15B8);
	write_cmos_sensor(0x6F12, 0x0669);
	write_cmos_sensor(0x602A, 0x15BA);
	write_cmos_sensor(0x6F12, 0x026D);
	write_cmos_sensor(0x602A, 0x15BC);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x602A, 0x15BE);
	write_cmos_sensor(0x6F12, 0x0575);
	write_cmos_sensor(0x602A, 0x15C0);
	write_cmos_sensor(0x6F12, 0x0679);
	write_cmos_sensor(0x602A, 0x15C2);
	write_cmos_sensor(0x6F12, 0x027D);
	write_cmos_sensor(0x602A, 0x15C4);
	write_cmos_sensor(0x6F12, 0x0141);
	write_cmos_sensor(0x602A, 0x15C6);
	write_cmos_sensor(0x6F12, 0x0545);
	write_cmos_sensor(0x602A, 0x15C8);
	write_cmos_sensor(0x6F12, 0x0649);
	write_cmos_sensor(0x602A, 0x15CA);
	write_cmos_sensor(0x6F12, 0x024D);
	write_cmos_sensor(0x602A, 0x15CC);
	write_cmos_sensor(0x6F12, 0x0151);
	write_cmos_sensor(0x602A, 0x15CE);
	write_cmos_sensor(0x6F12, 0x0555);
	write_cmos_sensor(0x602A, 0x15D0);
	write_cmos_sensor(0x6F12, 0x0659);
	write_cmos_sensor(0x602A, 0x15D2);
	write_cmos_sensor(0x6F12, 0x025D);
	write_cmos_sensor(0x602A, 0x15D4);
	write_cmos_sensor(0x6F12, 0x0161);
	write_cmos_sensor(0x602A, 0x15D6);
	write_cmos_sensor(0x6F12, 0x0565);
	write_cmos_sensor(0x602A, 0x15D8);
	write_cmos_sensor(0x6F12, 0x0669);
	write_cmos_sensor(0x602A, 0x15DA);
	write_cmos_sensor(0x6F12, 0x026D);
	write_cmos_sensor(0x602A, 0x15DC);
	write_cmos_sensor(0x6F12, 0x0171);
	write_cmos_sensor(0x602A, 0x15DE);
	write_cmos_sensor(0x6F12, 0x0575);
	write_cmos_sensor(0x602A, 0x15E0);
	write_cmos_sensor(0x6F12, 0x0679);
	write_cmos_sensor(0x602A, 0x15E2);
	write_cmos_sensor(0x6F12, 0x027D);
	write_cmos_sensor(0x602A, 0x1A50);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1A54);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0x0D00, 0x0101);
	write_cmos_sensor(0x0D02, 0x0001);
	write_cmos_sensor(0x0114, 0x0300);
	write_cmos_sensor(0xF486, 0x0000);
	write_cmos_sensor(0xF488, 0x0000);
	write_cmos_sensor(0xF48A, 0x0000);
	write_cmos_sensor(0xF48C, 0x0000);
	write_cmos_sensor(0xF48E, 0x0000);
	write_cmos_sensor(0xF490, 0x0000);
	write_cmos_sensor(0xF492, 0x0000);
	write_cmos_sensor(0xF494, 0x0000);
	write_cmos_sensor(0xF496, 0x0000);
	write_cmos_sensor(0xF498, 0x0000);
	write_cmos_sensor(0xF49A, 0x0000);
	write_cmos_sensor(0xF49C, 0x0000);
	write_cmos_sensor(0xF49E, 0x0000);
	write_cmos_sensor(0xF4A0, 0x0000);
	write_cmos_sensor(0xF4A2, 0x0000);
	write_cmos_sensor(0xF4A4, 0x0000);
	write_cmos_sensor(0xF4A6, 0x0000);
	write_cmos_sensor(0xF4A8, 0x0000);
	write_cmos_sensor(0xF4AA, 0x0000);
	write_cmos_sensor(0xF4AC, 0x0000);
	write_cmos_sensor(0xF4AE, 0x0000);
	write_cmos_sensor(0xF4B0, 0x0000);
	write_cmos_sensor(0xF4B2, 0x0000);
	write_cmos_sensor(0xF4B4, 0x0000);
	write_cmos_sensor(0xF4B6, 0x0000);
	write_cmos_sensor(0xF4B8, 0x0000);
	write_cmos_sensor(0xF4BA, 0x0000);
	write_cmos_sensor(0xF4BC, 0x0000);
	write_cmos_sensor(0xF4BE, 0x0000);
	write_cmos_sensor(0xF4C0, 0x0000);
	write_cmos_sensor(0xF4C2, 0x0000);
	write_cmos_sensor(0xF4C4, 0x0000);
	write_cmos_sensor(0x0202, 0x0010);
	write_cmos_sensor(0x0226, 0x0010);
	write_cmos_sensor(0x0204, 0x0020);
	write_cmos_sensor(0x0B06, 0x0101);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x107A);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x1074);
	write_cmos_sensor(0x6F12, 0x1D00);
	write_cmos_sensor(0x602A, 0x0E7C);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1120);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1122);
	write_cmos_sensor(0x6F12, 0x0028);
	write_cmos_sensor(0x602A, 0x1128);
	write_cmos_sensor(0x6F12, 0x0604);
	write_cmos_sensor(0x602A, 0x1AC0);
	write_cmos_sensor(0x6F12, 0x0200);
	write_cmos_sensor(0x602A, 0x1AC2);
	write_cmos_sensor(0x6F12, 0x0002);
	write_cmos_sensor(0x602A, 0x1494);
	write_cmos_sensor(0x6F12, 0x3D68);
	write_cmos_sensor(0x602A, 0x1498);
	write_cmos_sensor(0x6F12, 0xF10D);
	write_cmos_sensor(0x602A, 0x1488);
	write_cmos_sensor(0x6F12, 0x0F0F);
	write_cmos_sensor(0x602A, 0x148A);
	write_cmos_sensor(0x6F12, 0x170F);
	write_cmos_sensor(0x602A, 0x150E);
	write_cmos_sensor(0x6F12, 0x00C2);
	write_cmos_sensor(0x602A, 0x1510);
	write_cmos_sensor(0x6F12, 0xC0AF);
	write_cmos_sensor(0x602A, 0x1512);
	write_cmos_sensor(0x6F12, 0x0080);
	write_cmos_sensor(0x602A, 0x1486);
	write_cmos_sensor(0x6F12, 0x1430);
	write_cmos_sensor(0x602A, 0x1490);
	write_cmos_sensor(0x6F12, 0x4D09);
	write_cmos_sensor(0x602A, 0x149E);
	write_cmos_sensor(0x6F12, 0x01C4);
	write_cmos_sensor(0x602A, 0x11CC);
	write_cmos_sensor(0x6F12, 0x0008);
	write_cmos_sensor(0x602A, 0x11CE);
	write_cmos_sensor(0x6F12, 0x000B);
	write_cmos_sensor(0x602A, 0x11D0);
	write_cmos_sensor(0x6F12, 0x0003);
	write_cmos_sensor(0x602A, 0x11DA);
	write_cmos_sensor(0x6F12, 0x0012);
	write_cmos_sensor(0x602A, 0x11E6);
	write_cmos_sensor(0x6F12, 0x002A);
	write_cmos_sensor(0x602A, 0x125E);
	write_cmos_sensor(0x6F12, 0x0048);
	write_cmos_sensor(0x602A, 0x11F4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x11F8);
	write_cmos_sensor(0x6F12, 0x0016);
	write_cmos_sensor(0x6028, 0x4000);
	write_cmos_sensor(0xF444, 0x05BF);
	write_cmos_sensor(0xF44A, 0x0008);
	write_cmos_sensor(0xF44E, 0x0012);
	write_cmos_sensor(0xF46E, 0x6CC0);
	write_cmos_sensor(0xF470, 0x7809);
	write_cmos_sensor(0x6028, 0x2000);
	write_cmos_sensor(0x602A, 0x1CAA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CAE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CB8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBA);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBC);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CBE);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC0);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC2);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC4);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC6);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1CC8);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6000);
	write_cmos_sensor(0x6F12, 0x000F);
	write_cmos_sensor(0x602A, 0x6002);
	write_cmos_sensor(0x6F12, 0xFFFF);
	write_cmos_sensor(0x602A, 0x6004);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x6006);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6008);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x600E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6010);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6012);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6014);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6016);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6018);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x601E);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6020);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6022);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6024);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6026);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x6028);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602A);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x602C);
	write_cmos_sensor(0x6F12, 0x1000);
	write_cmos_sensor(0x602A, 0x1144);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1146);
	write_cmos_sensor(0x6F12, 0x1B00);
	write_cmos_sensor(0x602A, 0x1080);
	write_cmos_sensor(0x6F12, 0x0100);
	write_cmos_sensor(0x602A, 0x1084);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x108A);
	write_cmos_sensor(0x6F12, 0x00C0);
	write_cmos_sensor(0x602A, 0x1090);
	write_cmos_sensor(0x6F12, 0x0001);
	write_cmos_sensor(0x602A, 0x1092);
	write_cmos_sensor(0x6F12, 0x0000);
	write_cmos_sensor(0x602A, 0x1094);
	write_cmos_sensor(0x6F12, 0xA32E);
}

static kal_uint32 return_sensor_id(void)
{
	return ((read_cmos_sensor_8(0x0000) << 8) | read_cmos_sensor_8(0x0001));
}

static void custom2_setting(void)
{

	LOG_INF("E\n");
	write_cmos_sensor(0x6028,0x2400);
	write_cmos_sensor(0x602A,0x1A28);
	write_cmos_sensor(0x6F12,0x4C00);
	write_cmos_sensor(0x602A,0x065A);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x139E);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x139C);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x13A0);
	write_cmos_sensor(0x6F12,0x0A00);
	write_cmos_sensor(0x6F12,0x0120);
	write_cmos_sensor(0x602A,0x2072);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x1A64);
	write_cmos_sensor(0x6F12,0x0301);
	write_cmos_sensor(0x6F12,0xFF00);
	write_cmos_sensor(0x602A,0x19E6);
	write_cmos_sensor(0x6F12,0x0200);
	write_cmos_sensor(0x602A,0x1A30);
	write_cmos_sensor(0x6F12,0x3401);
	write_cmos_sensor(0x602A,0x19FC);
	write_cmos_sensor(0x6F12,0x0B00);
	write_cmos_sensor(0x602A,0x19F4);
	write_cmos_sensor(0x6F12,0x0606);
	write_cmos_sensor(0x602A,0x19F8);
	write_cmos_sensor(0x6F12,0x1010);
	write_cmos_sensor(0x602A,0x1B26);
	write_cmos_sensor(0x6F12,0x6F80);
	write_cmos_sensor(0x6F12,0xA060);
	write_cmos_sensor(0x602A,0x1A3C);
	write_cmos_sensor(0x6F12,0x6207);
	write_cmos_sensor(0x602A,0x1A48);
	write_cmos_sensor(0x6F12,0x6207);
	write_cmos_sensor(0x602A,0x1444);
	write_cmos_sensor(0x6F12,0x2000);
	write_cmos_sensor(0x6F12,0x2000);
	write_cmos_sensor(0x602A,0x144C);
	write_cmos_sensor(0x6F12,0x3F00);
	write_cmos_sensor(0x6F12,0x3F00);
	write_cmos_sensor(0x602A,0x7F6C);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x6F12,0x2F00);
	write_cmos_sensor(0x6F12,0xFA00);
	write_cmos_sensor(0x6F12,0x2400);
	write_cmos_sensor(0x6F12,0xE500);
	write_cmos_sensor(0x602A,0x0650);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x602A,0x0654);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x1A46);
	write_cmos_sensor(0x6F12,0xB000);
	write_cmos_sensor(0x602A,0x1A52);
	write_cmos_sensor(0x6F12,0xBF00);
	write_cmos_sensor(0x602A,0x0674);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x602A,0x0668);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x6F12,0x0800);
	write_cmos_sensor(0x602A,0x0684);
	write_cmos_sensor(0x6F12,0x4001);
	write_cmos_sensor(0x602A,0x0688);
	write_cmos_sensor(0x6F12,0x4001);
	write_cmos_sensor(0x602A,0x147C);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x1480);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x19F6);
	write_cmos_sensor(0x6F12,0x0904);
	write_cmos_sensor(0x602A,0x0812);
	write_cmos_sensor(0x6F12,0x0010);
	write_cmos_sensor(0x602A,0x2148);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x2042);
	write_cmos_sensor(0x6F12,0x1A00);
	write_cmos_sensor(0x602A,0x0874);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x09C0);
	write_cmos_sensor(0x6F12,0x2008);
	write_cmos_sensor(0x602A,0x09C4);
	write_cmos_sensor(0x6F12,0x2000);
	write_cmos_sensor(0x602A,0x19FE);
	write_cmos_sensor(0x6F12,0x0E1C);
	write_cmos_sensor(0x602A,0x4D92);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x8104);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x4D94);
	write_cmos_sensor(0x6F12,0x0005);
	write_cmos_sensor(0x6F12,0x000A);
	write_cmos_sensor(0x6F12,0x0010);
	write_cmos_sensor(0x6F12,0x1510);
	write_cmos_sensor(0x6F12,0x000A);
	write_cmos_sensor(0x6F12,0x0040);
	write_cmos_sensor(0x6F12,0x1510);
	write_cmos_sensor(0x6F12,0x1510);
	write_cmos_sensor(0x602A,0x3570);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x3574);
	write_cmos_sensor(0x6F12,0x1304);
	write_cmos_sensor(0x602A,0x21E4);
	write_cmos_sensor(0x6F12,0x0400);
	write_cmos_sensor(0x602A,0x21EC);
	write_cmos_sensor(0x6F12,0x1D02);
	write_cmos_sensor(0x602A,0x2080);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x6F12,0xFF00);
	write_cmos_sensor(0x602A,0x2086);
	write_cmos_sensor(0x6F12,0x0001);
	write_cmos_sensor(0x602A,0x208E);
	write_cmos_sensor(0x6F12,0x14F4);
	write_cmos_sensor(0x602A,0x208A);
	write_cmos_sensor(0x6F12,0xD244);
	write_cmos_sensor(0x6F12,0xD244);
	write_cmos_sensor(0x602A,0x120E);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x212E);
	write_cmos_sensor(0x6F12,0x0200);
	write_cmos_sensor(0x602A,0x13AE);
	write_cmos_sensor(0x6F12,0x0101);
	write_cmos_sensor(0x602A,0x0718);
	write_cmos_sensor(0x6F12,0x0001);
	write_cmos_sensor(0x602A,0x0710);
	write_cmos_sensor(0x6F12,0x0002);
	write_cmos_sensor(0x6F12,0x0804);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x1B5C);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x602A,0x0786);
	write_cmos_sensor(0x6F12,0x7701);
	write_cmos_sensor(0x602A,0x2022);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x6F12,0x0500);
	write_cmos_sensor(0x602A,0x1360);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x1376);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x6F12,0x6038);
	write_cmos_sensor(0x6F12,0x7038);
	write_cmos_sensor(0x6F12,0x8038);
	write_cmos_sensor(0x602A,0x1386);
	write_cmos_sensor(0x6F12,0x0B00);
	write_cmos_sensor(0x602A,0x06FA);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x4A94);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x6F12,0x0600);
	write_cmos_sensor(0x602A,0x0A76);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0AEE);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0B66);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0BDE);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0C56);
	write_cmos_sensor(0x6F12,0x1000);
	write_cmos_sensor(0x602A,0x0CF2);
	write_cmos_sensor(0x6F12,0x0001);
	write_cmos_sensor(0x602A,0x0CF0);
	write_cmos_sensor(0x6F12,0x0101);
	write_cmos_sensor(0x602A,0x11B8);
	write_cmos_sensor(0x6F12,0x0100);
	write_cmos_sensor(0x602A,0x11F6);
	write_cmos_sensor(0x6F12,0x0020);
	write_cmos_sensor(0x602A,0x4A74);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0xD8FF);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0xD8FF);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6F12,0x0000);
	write_cmos_sensor(0x6028,0x4000);
	write_cmos_sensor(0xF46A,0xAE80);
	write_cmos_sensor(0x0344,0x0000);
	write_cmos_sensor(0x0346,0x0000);
	write_cmos_sensor(0x0348,0x1FFF);
	write_cmos_sensor(0x034A,0x181F);
	write_cmos_sensor(0x034C,0x0FF0);
	write_cmos_sensor(0x034E,0x0C00);
	write_cmos_sensor(0x0350,0x0008);
	write_cmos_sensor(0x0352,0x0008);
	write_cmos_sensor(0x0900,0x0122);
	write_cmos_sensor(0x0380,0x0002);
	write_cmos_sensor(0x0382,0x0002);
	write_cmos_sensor(0x0384,0x0002);
	write_cmos_sensor(0x0386,0x0002);
	write_cmos_sensor(0x0110,0x1002);
	write_cmos_sensor(0x0114,0x0301);
	write_cmos_sensor(0x0116,0x3000);
	write_cmos_sensor(0x0136,0x1800);
	write_cmos_sensor(0x013E,0x0000);
	write_cmos_sensor(0x0300,0x0006);
	write_cmos_sensor(0x0302,0x0001);
	write_cmos_sensor(0x0304,0x0004);
	write_cmos_sensor(0x0306,0x008C);
	write_cmos_sensor(0x0308,0x0008);
	write_cmos_sensor(0x030A,0x0001);
	write_cmos_sensor(0x030C,0x0000);
	write_cmos_sensor(0x030E,0x0004);
	write_cmos_sensor(0x0310,0x0067);
	write_cmos_sensor(0x0312,0x0000);
	write_cmos_sensor(0x080E,0x0000);
	write_cmos_sensor(0x0340,0x0C54);
	write_cmos_sensor(0x0342,0x1716);
	write_cmos_sensor(0x0702,0x0000);
	write_cmos_sensor(0x0202,0x0100);
	write_cmos_sensor(0x0200,0x0100);
	write_cmos_sensor(0x0D00,0x0101);
	write_cmos_sensor(0x0D02,0x0101);
	write_cmos_sensor(0x0D04,0x0102);
	write_cmos_sensor(0x6226,0x0000);
}

/*************************************************************************
*FUNCTION
*  get_imgsensor_id
*
*DESCRIPTION
*  This function get the sensor ID
*
*PARAMETERS
*  *sensorID : return the sensor ID
*
*RETURNS
*  None
*
*GLOBALS AFFECTED
*
*************************************************************************/
extern int hbb_flag;
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
				*sensor_id = return_sensor_id();
if(*sensor_id == 0x38e1)
*sensor_id = imgsensor_info.sensor_id;
				if (*sensor_id == imgsensor_info.sensor_id) {
					pr_info
			("s5kjn1sp_ofilm i2c 0x%x, sid 0x%x\n",
			 imgsensor.i2c_write_id, *sensor_id);
					return ERROR_NONE;

				} else {
					pr_err
		("check id fail i2c 0x%x, sid: 0x%x\n",
					 imgsensor.i2c_write_id,
					 *sensor_id);
					*sensor_id = 0xFFFFFFFF;
				}
			LOG_INF
		("Read fail, i2cid: 0x%x, Rsid: 0x%x, info.sid:0x%x.\n",
			 imgsensor.i2c_write_id, *sensor_id,
			 imgsensor_info.sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		retry = 1;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {

		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}

/*************************************************************************
*FUNCTION
*  open
*
*DESCRIPTION
*  This function initialize the registers of CMOS sensor
*
*PARAMETERS
*  None
*
*RETURNS
*  None
*
*GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{

	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0;

	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
if(sensor_id == 0x38e1)
sensor_id = imgsensor_info.sensor_id;
			if (sensor_id == imgsensor_info.sensor_id) {
				LOG_INF
				("i2c write id: 0x%x, sensor id: 0x%x\n",
				 imgsensor.i2c_write_id, sensor_id);
				break;
			}
			LOG_INF
			("Read sensor id fail, id: 0x%x, sensor id: 0x%x\n",
			 imgsensor.i2c_write_id, sensor_id);
			retry--;
		} while (retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;

	sensor_init();
	spin_lock(&imgsensor_drv_lock);
	imgsensor.autoflicker_en = KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = KAL_FALSE;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}

/*************************************************************************
*FUNCTION
*  close
*
*DESCRIPTION
*
*
*PARAMETERS
*  None
*
*RETURNS
*  None
*
*GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	return ERROR_NONE;
}

/*************************************************************************
*FUNCTION
*preview
*
*DESCRIPTION
*  This function start the sensor preview.
*
*PARAMETERS
*  *image_window : address pointer of pixel numbers in one period of HSYNC
**sensor_config_data : address pointer of line numbers in one period of VSYNC
*
*RETURNS
*  None
*
*GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32
preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *
		image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("preview start\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;

	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}

/*************************************************************************
*FUNCTION
*  capture
*
*DESCRIPTION
*  This function setup the CMOS sensor in capture MY_OUTPUT mode
*
*PARAMETERS
*
*RETURNS
*  None
*
*GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32
capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *
		image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	int i;

	LOG_INF("capture start\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) {
		LOG_INF("capture30fps: use cap30FPS's setting: %d fps!\n",
				imgsensor.current_fps / 10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {

		LOG_INF("cap115fps: use cap1's setting: %d fps!\n",
				imgsensor.current_fps / 10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		LOG_INF
		("Warning:current_fps %d is not support, use cap1\n",
		 imgsensor.current_fps / 10);
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);
	capture_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);
	mdelay(10);

	for (i = 0; i < 10; i++) {
		LOG_INF("delay time = %d, the frame no = %d\n", i * 10,
				read_cmos_sensor(0x0005));
		mdelay(10);
	}


	return ERROR_NONE;
}

static kal_uint32
normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *
	 image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("normal_video start\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;

	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}

static kal_uint32
hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *
		 image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("hs_video start\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;

	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;

	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}

static kal_uint32
slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *
		   image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("slim_video start\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;

	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;

	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	set_mirror_flip(imgsensor.mirror);
	return ERROR_NONE;
}

static kal_uint32
Custom2(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *
		image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("Custom2 start\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CUSTOM2;
	imgsensor.pclk = imgsensor_info.custom2.pclk;

	imgsensor.line_length = imgsensor_info.custom2.linelength;
	imgsensor.frame_length = imgsensor_info.custom2.framelength;
	imgsensor.min_frame_length = imgsensor_info.custom2.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	custom2_setting();
	set_mirror_flip(IMAGE_NORMAL);
	return ERROR_NONE;
}

static kal_uint32
get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth =
		imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight =
		imgsensor_info.cap.grabwindow_height;
	sensor_resolution->SensorPreviewWidth =
		imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight =
		imgsensor_info.pre.grabwindow_height;
	sensor_resolution->SensorVideoWidth =
		imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight =
		imgsensor_info.normal_video.grabwindow_height;
	sensor_resolution->SensorHighSpeedVideoWidth =
		imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight =
		imgsensor_info.hs_video.grabwindow_height;
	sensor_resolution->SensorSlimVideoWidth =
		imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight =
		imgsensor_info.slim_video.grabwindow_height;
	sensor_resolution->SensorCustom2Width =
		imgsensor_info.custom2.grabwindow_width;
	sensor_resolution->SensorCustom2Height =
		imgsensor_info.custom2.grabwindow_height;
	return ERROR_NONE;
}

static kal_uint32
get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
		 MSDK_SENSOR_INFO_STRUCT *sensor_info,
		 MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_HIGH;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_HIGH;
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
	sensor_info->SensorInterruptDelayLines = 4;
	sensor_info->SensorResetActiveHigh = FALSE;
	sensor_info->SensorResetDelayCount = 5;
	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat =
		imgsensor_info.sensor_output_dataformat;
	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame =
		imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame =
		imgsensor_info.slim_video_delay_frame;
	sensor_info->Custom2DelayFrame = imgsensor_info.custom2_delay_frame;
	sensor_info->FrameTimeDelayFrame =
		imgsensor_info.frame_time_delay_frame;
	sensor_info->SensorMasterClockSwitch = 0;
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;
	sensor_info->AESensorGainDelayFrame =
		imgsensor_info.ae_sensor_gain_delay_frame;
	sensor_info->AEISPGainDelayFrame =
		imgsensor_info.ae_ispGain_delay_frame;
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3;
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2;
	sensor_info->SensorPixelClockCount = 3;
	sensor_info->SensorDataLatchCount = 2;
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;
	sensor_info->SensorHightSampling = 0;
	sensor_info->SensorPacketECCOrder = 1;
	sensor_info->PDAF_Support = 2;

	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.cap.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		sensor_info->SensorGrabStartX =
			imgsensor_info.normal_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.normal_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		sensor_info->SensorGrabStartX =
			imgsensor_info.slim_video.startx;
		sensor_info->SensorGrabStartY =
			imgsensor_info.slim_video.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		sensor_info->SensorGrabStartX = imgsensor_info.custom2.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.custom2.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.custom2.mipi_data_lp2hs_settle_dc;
		break;

	default:
		sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
		sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;
		sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount =
			imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
		break;
	}
	return ERROR_NONE;
}

static kal_uint32
control(enum MSDK_SCENARIO_ID_ENUM scenario_id,
		MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		preview(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		capture(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		normal_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		hs_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		slim_video(image_window, sensor_config_data);
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		Custom2(image_window, sensor_config_data);
		break;

	default:
		LOG_INF("Error ScenarioId setting");
		preview(image_window, sensor_config_data);
		return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}

static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);

	if (framerate == 0)

		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps, 1);
	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d\n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable)
		imgsensor.autoflicker_en = KAL_TRUE;
	else
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32
set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM
	  scenario_id, MUINT32 framerate)
{
	kal_uint32 frame_length;

	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		frame_length =
			imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length >
			 imgsensor_info.pre.framelength) ? (frame_length -
			imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);

		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		if (framerate == 0)
			return ERROR_NONE;
		frame_length =
			imgsensor_info.normal_video.pclk / framerate * 10 /
			imgsensor_info.normal_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
		(frame_length > imgsensor_info.normal_video.framelength)
		  ? (frame_length -
		   imgsensor_info.normal_video.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.normal_video.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		if (imgsensor.current_fps ==
			 imgsensor_info.cap1.max_framerate) {
			frame_length =
				imgsensor_info.cap1.pclk / framerate * 10 /
				imgsensor_info.cap1.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
			(frame_length > imgsensor_info.cap1.framelength)
			 ? (frame_length - imgsensor_info.cap1.framelength)
			  : 0;
			imgsensor.frame_length =
				imgsensor_info.cap1.framelength +
				imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		} else {
			if (imgsensor.current_fps !=
					imgsensor_info.cap.max_framerate)
				LOG_INF
		("current_fps %d is not support, so use cap' %d fps!\n",
				 framerate,
				 imgsensor_info.cap.max_framerate / 10);
			frame_length =
				imgsensor_info.cap.pclk / framerate * 10 /
				imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line =
				(frame_length > imgsensor_info.cap.framelength)
				 ? (frame_length -
				 imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length =
				imgsensor_info.cap.framelength +
				imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
		}
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		frame_length =
			imgsensor_info.hs_video.pclk / framerate * 10 /
			imgsensor_info.hs_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.hs_video.framelength)
		 ? (frame_length - imgsensor_info.hs_video.framelength)
		 : 0;
		imgsensor.frame_length =
			imgsensor_info.hs_video.framelength
			 + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		frame_length =
			imgsensor_info.slim_video.pclk / framerate * 10 /
			imgsensor_info.slim_video.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length > imgsensor_info.slim_video.framelength)
			  ? (frame_length -
			 imgsensor_info.slim_video.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.slim_video.framelength +
			imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		frame_length =
			imgsensor_info.custom2.pclk / framerate * 10 /
			imgsensor_info.custom2.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length >
			 imgsensor_info.custom2.framelength) ? (frame_length -
			 imgsensor_info.custom2.framelength) : 0;
		if (imgsensor.dummy_line < 0)
			imgsensor.dummy_line = 0;
		imgsensor.frame_length =
			imgsensor_info.custom2.framelength
			 + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		break;
	default:
		frame_length =
			imgsensor_info.pre.pclk / framerate * 10 /
			imgsensor_info.pre.linelength;
		spin_lock(&imgsensor_drv_lock);
		imgsensor.dummy_line =
			(frame_length >
			 imgsensor_info.pre.framelength) ? (frame_length -
			imgsensor_info.pre.framelength) : 0;
		imgsensor.frame_length =
			imgsensor_info.pre.framelength
			 + imgsensor.dummy_line;
		imgsensor.min_frame_length = imgsensor.frame_length;
		spin_unlock(&imgsensor_drv_lock);
		if (imgsensor.frame_length > imgsensor.shutter)
			set_dummy();
		LOG_INF("error scenario_id = %d, we use preview scenario\n",
				scenario_id);
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 get_default_framerate_by_scenario(enum
		MSDK_SCENARIO_ID_ENUM
		scenario_id,
		MUINT32 *framerate)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	switch (scenario_id) {
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		*framerate = imgsensor_info.pre.max_framerate;
		break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		*framerate = imgsensor_info.normal_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		*framerate = imgsensor_info.cap.max_framerate;
		break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		*framerate = imgsensor_info.hs_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
		*framerate = imgsensor_info.slim_video.max_framerate;
		break;
	case MSDK_SCENARIO_ID_CUSTOM2:
		*framerate = imgsensor_info.custom2.max_framerate;
		break;
	default:
		break;
	}
	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);
	if (enable) {

		write_cmos_sensor(0x0200, 0x0002);
		write_cmos_sensor(0x0202, 0x0002);
		write_cmos_sensor(0x0204, 0x0020);
		write_cmos_sensor(0x020E, 0x0100);
		write_cmos_sensor(0x0210, 0x0100);
		write_cmos_sensor(0x0212, 0x0100);
		write_cmos_sensor(0x0214, 0x0100);
		write_cmos_sensor(0x0600, 0x0002);
	} else {

		write_cmos_sensor(0x0600, 0x0000);
	}
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32
feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
				UINT8 *feature_para, UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16 = (UINT16 *) feature_para;
	UINT16 *feature_data_16 = (UINT16 *) feature_para;
	UINT32 *feature_return_para_32 = (UINT32 *) feature_para;
	UINT32 *feature_data_32 = (UINT32 *) feature_para;
	unsigned long long *feature_data = (unsigned long long *)feature_para;
	UINT32 fps = 0;

	struct SET_PD_BLOCK_INFO_T *PDAFinfo;
	struct SENSOR_VC_INFO_STRUCT *pvcinfo;
	struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;

	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data =
		(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

	pr_debug("feature_id = %d, len=%d\n", feature_id, *feature_para_len);
	switch (feature_id) {
	case SENSOR_FEATURE_GET_PERIOD:
		*feature_return_para_16++ = imgsensor.line_length;
		*feature_return_para_16 = imgsensor.frame_length;
		*feature_para_len = 4;
	break;
	case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
		pr_debug("imgsensor.pclk = %d,current_fps = %d\n",
				 imgsensor.pclk, imgsensor.current_fps);
		*feature_return_para_32 = imgsensor.pclk;
		*feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_ESHUTTER:
		set_shutter((kal_uint64)*feature_data);
	break;
	case SENSOR_FEATURE_SET_GAIN:
		set_gain((UINT16) *feature_data);
	break;
	case SENSOR_FEATURE_SET_FLASHLIGHT:
	break;
	case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
	break;
	case SENSOR_FEATURE_SET_REGISTER:
		write_cmos_sensor(sensor_reg_data->RegAddr,
						  sensor_reg_data->RegData);
	break;
	case SENSOR_FEATURE_GET_REGISTER:
		sensor_reg_data->RegData =
			read_cmos_sensor(sensor_reg_data->RegAddr);
	break;
	case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
		/*get the lens driver ID from EEPROM or
		 *just return LENS_DRIVER_ID_DO_NOT_CARE
		 */

		*feature_return_para_32 = LENS_DRIVER_ID_DO_NOT_CARE;
		*feature_para_len = 4;
	break;
	case SENSOR_FEATURE_SET_VIDEO_MODE:
		set_video_mode(*feature_data);
	break;
	case SENSOR_FEATURE_CHECK_SENSOR_ID:
		get_imgsensor_id(feature_return_para_32);
	break;
	case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
		set_auto_flicker_mode((BOOL) * feature_data_16,
			  *(feature_data_16 + 1));
	break;
	case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
		set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)
			  *feature_data, *(feature_data + 1));
	break;
	case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
		get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)
			  *(feature_data), (MUINT32 *) (uintptr_t) (*
			  (feature_data + 1)));
	break;

	case SENSOR_FEATURE_SET_TEST_PATTERN:
		set_test_pattern_mode((BOOL) * feature_data);
	break;

	case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
		*feature_return_para_32 = imgsensor_info.checksum_value;
		*feature_para_len = 4;
		break;

	case SENSOR_FEATURE_SET_FRAMERATE:
		pr_debug("current fps :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.current_fps = (UINT16) *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
	break;

	case SENSOR_FEATURE_SET_HDR:
		pr_debug("hdr enable :%d\n", *feature_data_32);
		spin_lock(&imgsensor_drv_lock);
		imgsensor.ihdr_mode = (UINT8) *feature_data_32;
		spin_unlock(&imgsensor_drv_lock);
	break;

	case SENSOR_FEATURE_GET_CROP_INFO:
		pr_debug("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n",
				 (UINT32) *feature_data_32);

		wininfo =
			(struct SENSOR_WINSIZE_INFO_STRUCT
			 *)(uintptr_t) (*(feature_data + 1));

	switch (*feature_data_32) {
	case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[1],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
	break;
	case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[2],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
	break;
	case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[3],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
	break;
	case MSDK_SCENARIO_ID_SLIM_VIDEO:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[4],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
	break;
	case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[5],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
	break;
	case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
	default:
			memcpy((void *)wininfo,
			(void *)&imgsensor_winsize_info[0],
			sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
	break;
	}
	break;
	case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
		/*pr_debug("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",
				 (UINT16) *feature_data,
				 (UINT16) *(feature_data + 1),
				 (UINT16) *(feature_data + 2));*/

		/*ihdr_write_shutter_gain((UINT16)*feature_data,
		 *(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
		 */
	break;
	case SENSOR_FEATURE_SET_AWB_GAIN:
	break;
	case SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME:
		set_shutter_frame_length((UINT16) (*feature_data),
			 (UINT16) (*(feature_data + 1)));
	break;
    case SENSOR_FEATURE_GET_PERIOD_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.cap.framelength << 16)
				+ imgsensor_info.cap.linelength;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.normal_video.framelength << 16)
				+ imgsensor_info.normal_video.linelength;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.hs_video.framelength << 16)
				+ imgsensor_info.hs_video.linelength;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.slim_video.framelength << 16)
				+ imgsensor_info.slim_video.linelength;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
			= (imgsensor_info.pre.framelength << 16)
				+ imgsensor_info.pre.linelength;
				break;
		}
		break;
	case SENSOR_FEATURE_SET_HDR_SHUTTER:
		pr_debug("SENSOR_FEATURE_SET_HDR_SHUTTER LE=%d, SE=%d\n",
				 (UINT16) *feature_data,
				 (UINT16) *(feature_data + 1));
		/*ihdr_write_shutter(
		 *(UINT16)*feature_data,(UINT16)*(feature_data+1));
		 */
	break;

    case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.cap.pclk;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.normal_video.pclk;
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.hs_video.pclk;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.slim_video.pclk;
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *)(uintptr_t)(*(feature_data + 1))
				= imgsensor_info.pre.pclk;
		        break;
		}
		break;

	case SENSOR_FEATURE_GET_PDAF_INFO:
		pr_debug("SENSOR_FEATURE_GET_PDAF_INFO scenarioId:%d\n",
				 (UINT16) *feature_data);
		PDAFinfo =(struct SET_PD_BLOCK_INFO_T*)(uintptr_t) (*(feature_data + 1));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)PDAFinfo, (void *)&imgsensor_pd_normal_video_info,
				sizeof(struct SET_PD_BLOCK_INFO_T));
			break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		default:
		break;
		}
	break;
	case SENSOR_FEATURE_GET_VC_INFO:
		LOG_INF("SENSOR_FEATURE_GET_VC_INFO %d\n",(UINT16) *feature_data);
		pvcinfo = (struct SENSOR_VC_INFO_STRUCT *) (uintptr_t) (*(feature_data + 1));
		switch (*feature_data_32) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[1],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[2],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[3],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			memcpy((void *)pvcinfo, (void *)&SENSOR_VC_INFO[0],
				sizeof(struct SENSOR_VC_INFO_STRUCT));
			break;
		}
		break;

	case SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY:
		pr_debug
		("SENSOR_FEATURE_GET_SENSOR_PDAF_CAPACITY scenarioId:%d\n",
		 (UINT16) *feature_data);

switch (*feature_data) {
		case MSDK_SCENARIO_ID_CUSTOM2:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 1;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) = 0;
			break;
		}
	break;
	case SENSOR_FEATURE_SET_PDAF:
		pr_debug("PDAF mode :%d\n", *feature_data_16);
		imgsensor.pdaf_mode = *feature_data_16;
	break;

	case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_SUSPEND\n");
		streaming_control(KAL_FALSE);
	break;
	case SENSOR_FEATURE_SET_STREAMING_RESUME:
		pr_debug("SENSOR_FEATURE_SET_STREAMING_RESUME, shutter:%llu\n",
				 *feature_data);
		if (*feature_data != 0)
			set_shutter(*feature_data);
		streaming_control(KAL_TRUE);
	break;
	case SENSOR_FEATURE_GET_PIXEL_RATE:
		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				(imgsensor_info.cap.pclk /
				 (imgsensor_info.cap.linelength - 80)) *
				imgsensor_info.cap.grabwindow_width;

		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				(imgsensor_info.normal_video.pclk /
				 (imgsensor_info.normal_video.linelength - 80))
				 *imgsensor_info.normal_video.grabwindow_width;

		break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				(imgsensor_info.hs_video.pclk /
				 (imgsensor_info.hs_video.linelength - 80)) *
				imgsensor_info.hs_video.grabwindow_width;

		break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				(imgsensor_info.slim_video.pclk /
				 (imgsensor_info.slim_video.linelength - 80)) *
				imgsensor_info.slim_video.grabwindow_width;

		break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				(imgsensor_info.slim_video.pclk /
				 (imgsensor_info.slim_video.linelength - 80)) *
				imgsensor_info.slim_video.grabwindow_width;

		break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				(imgsensor_info.pre.pclk /
				 (imgsensor_info.pre.linelength - 80)) *
				imgsensor_info.pre.grabwindow_width;
		break;
		}
	break;

	case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
		fps = (MUINT32) (*(feature_data + 2));

		switch (*feature_data) {
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				imgsensor_info.cap.mipi_pixel_rate;
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				imgsensor_info.normal_video.mipi_pixel_rate;
		break;
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				imgsensor_info.hs_video.mipi_pixel_rate;
		break;
		case MSDK_SCENARIO_ID_CUSTOM2:
			imgsensor_info.custom2.mipi_pixel_rate;
		break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				imgsensor_info.slim_video.mipi_pixel_rate;
		break;
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			*(MUINT32 *) (uintptr_t) (*(feature_data + 1)) =
				imgsensor_info.pre.mipi_pixel_rate;
		break;
		}

	break;

//cxc long exposure >
	case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
			*feature_return_para_32 =
				imgsensor.current_ae_effective_frame;
	break;
	case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
			memcpy(feature_return_para_32, &imgsensor.ae_frm_mode,
				sizeof(struct IMGSENSOR_AE_FRM_MODE));
	break;
//cxc long exposure <

	default:
	break;
	}
	return ERROR_NONE;
}

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5KJN1_OFILM_MAIN_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{

	if (pfFunc != NULL)
		*pfFunc = &sensor_func;
	return ERROR_NONE;
}
