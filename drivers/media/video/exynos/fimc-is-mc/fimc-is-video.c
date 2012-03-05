/*
 * Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is video functions
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <mach/videonode.h>
#include <media/exynos_mc.h>
#include <linux/cma.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/scatterlist.h>
#include <linux/videodev2.h>
#include <linux/videodev2_samsung.h>
#include <linux/v4l2-mediabus.h>

#include "fimc-is-core.h"
#include "fimc-is-helper.h"
#include "fimc-is-param.h"
#include "fimc-is-cmd.h"
#include "fimc-is-regs.h"
#include "fimc-is-err.h"
#include "fimc-is-misc.h"

static struct fimc_is_fmt fimc_is_formats[] = {
	 {
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.num_planes	= 1,
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,
	}, {
		.name		= "YUV 4:2:2 packed, CbYCrY",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
		.num_planes	= 1,
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,
	}, {
		.name		= "YUV 4:2:2 planar, Y/Cb/Cr",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.num_planes	= 1,
	}, {
		.name		= "YUV 4:2:0 planar, YCbCr",
		.pixelformat	= V4L2_PIX_FMT_YUV420,
		.num_planes	= 1,
	}, {
		.name		= "YUV 4:2:0 planar, YCbCr",
		.pixelformat	= V4L2_PIX_FMT_YVU420,
		.num_planes	= 1,
	}, {
		.name		= "YUV 4:2:0 planar, Y/CbCr",
		.pixelformat	= V4L2_PIX_FMT_NV12,
		.num_planes	= 1,
	}, {
		.name		= "YUV 4:2:0 planar, Y/CrCb",
		.pixelformat	= V4L2_PIX_FMT_NV21,
		.num_planes	= 1,
	}, {
		.name		= "YUV 4:2:0 non-contiguous 2-planar, Y/CbCr",
		.pixelformat	= V4L2_PIX_FMT_NV12M,
		.num_planes	= 2,
	}, {
		.name		= "YVU 4:2:0 non-contiguous 2-planar, Y/CrCb",
		.pixelformat	= V4L2_PIX_FMT_NV21M,
		.num_planes	= 2,
	}, {
		.name		= "YUV 4:2:0 non-contiguous 3-planar, Y/Cb/Cr",
		.pixelformat	= V4L2_PIX_FMT_YUV420M,
		.num_planes	= 3,
	}, {
		.name		= "YUV 4:2:0 non-contiguous 3-planar, Y/Cr/Cb",
		.pixelformat	= V4L2_PIX_FMT_YVU420M,
		.num_planes	= 3,
	},
};

static struct fimc_is_fmt *find_format(u32 *pixelformat, u32 *mbus_code, int index)
{
	struct fimc_is_fmt *fmt, *def_fmt = NULL;
	unsigned int i;

	if (index >= ARRAY_SIZE(fimc_is_formats))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(fimc_is_formats); ++i) {
		fmt = &fimc_is_formats[i];
		if (pixelformat && fmt->pixelformat == *pixelformat)
			return fmt;
		if (mbus_code && fmt->mbus_code == *mbus_code)
			return fmt;
		if (index == i)
			def_fmt = fmt;
	}
	return def_fmt;

}

static void set_plane_size(struct fimc_is_frame *frame, unsigned long sizes[])
{
	dbg(" ");
	switch(frame->format.pixelformat){
		case V4L2_PIX_FMT_YUYV:
			dbg("V4L2_PIX_FMT_YUYV(w:%d)(h:%d)\n", frame->width, frame->height);
			sizes[0] =  frame->width*frame->height*2;
			break;
		case V4L2_PIX_FMT_NV12M:
			dbg("V4L2_PIX_FMT_NV12M(w:%d)(h:%d)\n", frame->width, frame->height);
			sizes[0] =  frame->width*frame->height;
			sizes[1] =  frame->width*frame->height/2;
			break;
		case V4L2_PIX_FMT_YVU420M:
			dbg("V4L2_PIX_FMT_YVU420M(w:%d)(h:%d)\n", frame->width, frame->height);
			sizes[0] =  frame->width*frame->height;
			sizes[1] =  frame->width*frame->height/4;
			sizes[2] =  frame->width*frame->height/4;
			break;
	}
}

/*************************************************************************/
/* video file opertation										 */
/************************************************************************/

static int fimc_is_scalerc_video_open(struct file *file)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	file->private_data = &isp->video[FIMC_IS_VIDEO_NUM_SCALERC];
	isp->video[FIMC_IS_VIDEO_NUM_SCALERC].num_buf = 0;
	isp->video[FIMC_IS_VIDEO_NUM_SCALERC].buf_ref_cnt = 0;

	if (!test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state)) {
		isp->sensor_num = 1;

		fimc_is_load_fw(isp);

		set_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state);
		clear_bit(FIMC_IS_STATE_SENSOR_INITIALIZED, &isp->pipe_state);
	}

	clear_bit(FIMC_IS_STATE_SCALERC_STREAM_ON, &isp->pipe_state);
	return 0;

}

static int fimc_is_scalerc_video_close(struct file *file)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	int ret;

	dbg("%s\n", __func__);
	vb2_queue_release(&isp->video[FIMC_IS_VIDEO_NUM_SCALERC].vbq);

	if (!test_bit(FIMC_IS_STATE_SCALERP_STREAM_ON, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_SCALERC_STREAM_ON, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_3DNR_STREAM_ON, &isp->pipe_state) &&
		test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->power)){
		clear_bit(FIMC_IS_PWR_ST_POWEROFF, &isp->power);
		fimc_is_hw_subip_poweroff(isp);
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(FIMC_IS_PWR_ST_POWEROFF, &isp->power),
			FIMC_IS_SHUTDOWN_TIMEOUT_SENSOR);

		if (!ret) {
			err("wait timeout : %s\n", __func__);
			ret = -EINVAL;
		}

		dbg("stop flite & mipi (pos:%d) (port:%d)\n",
			isp->sensor.id_position,
			isp->pdata->sensor_info[isp->sensor.id_position]->flite_id);
		stop_fimc_lite(isp->pdata->sensor_info[isp->sensor.id_position]->flite_id);
		stop_mipi_csi(isp->pdata->sensor_info[isp->sensor.id_position]->csi_id);

		clear_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state);
		fimc_is_hw_a5_power(isp, 0);
	}
	return 0;

}

static unsigned int fimc_is_scalerc_video_poll(struct file *file,
				      struct poll_table_struct *wait)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_poll(&isp->video[FIMC_IS_VIDEO_NUM_SCALERC].vbq, file, wait);

}

static int fimc_is_scalerc_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_mmap(&isp->video[FIMC_IS_VIDEO_NUM_SCALERC].vbq, vma);

}

/*************************************************************************/
/* video ioctl operation						*/
/************************************************************************/

static int fimc_is_scalerc_video_querycap(struct file *file, void *fh,
						struct v4l2_capability *cap)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	strncpy(cap->driver, isp->pdev->name, sizeof(cap->driver) - 1);

	dbg("%s(devname : %s)\n", __func__, cap->driver);
	strncpy(cap->card, isp->pdev->name, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->version = KERNEL_VERSION(1, 0, 0);
	cap->capabilities = V4L2_CAP_STREAMING
					| V4L2_CAP_VIDEO_CAPTURE
					| V4L2_CAP_VIDEO_CAPTURE_MPLANE;

	return 0;
}

static int fimc_is_scalerc_video_enum_fmt_mplane(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerc_video_get_format_mplane(struct file *file, void *fh,
						struct v4l2_format *format)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerc_video_set_format_mplane(struct file *file, void *fh,
						struct v4l2_format *format)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix;
	struct fimc_is_fmt *frame;

	dbg("%s\n", __func__);

	pix = &format->fmt.pix_mp;
	frame = find_format(&pix->pixelformat, NULL, 0);

	if (!frame)
		return -EINVAL;

	isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame.format.pixelformat = frame->pixelformat;
	isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame.format.mbus_code = frame->mbus_code;
	isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame.format.num_planes = frame->num_planes;
	isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame.width = pix->width;
	isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame.height = pix->height;
	dbg("num_planes : %d\n", frame->num_planes);
	dbg("width : %d\n", pix->width);
	dbg("height : %d\n", pix->height);

	return 0;
}

static int fimc_is_scalerc_video_try_format_mplane(struct file *file, void *fh,
						struct v4l2_format *format)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerc_video_cropcap(struct file *file, void *fh,
						struct v4l2_cropcap *cropcap)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerc_video_get_crop(struct file *file, void *fh,
						struct v4l2_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerc_video_set_crop(struct file *file, void *fh,
						struct v4l2_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerc_video_reqbufs(struct file *file, void *priv,
						struct v4l2_requestbuffers *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s(buf->count : %d)\n", __func__, buf->count);

	ret = vb2_reqbufs(&video->vbq, buf);
	if (!ret)
		isp->video[FIMC_IS_VIDEO_NUM_SCALERC].num_buf = buf->count;

	if (buf->count == 0)
		isp->video[FIMC_IS_VIDEO_NUM_SCALERC].buf_ref_cnt = 0;
	dbg("%s(num_buf : %d)\n", __func__,
		isp->video[FIMC_IS_VIDEO_NUM_SCALERC].num_buf);

	return ret;
}

static int fimc_is_scalerc_video_querybuf(struct file *file, void *priv,
						struct v4l2_buffer *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;

	dbg("%s\n", __func__);
	ret = vb2_querybuf(&video->vbq, buf);

	return ret;
}

static int fimc_is_scalerc_video_qbuf(struct file *file, void *priv,
						struct v4l2_buffer *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;

	dbg("%s :: buf->index(%d)\n", __func__, buf->index);
	ret = vb2_qbuf(&video->vbq, buf);
	return ret;
}

static int fimc_is_scalerc_video_dqbuf(struct file *file, void *priv,
						struct v4l2_buffer *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;

	dbg("%s\n", __func__);
	ret = vb2_dqbuf(&video->vbq, buf, file->f_flags & O_NONBLOCK);
	return ret;
}

static int fimc_is_scalerc_video_streamon(struct file *file, void *priv,
						enum v4l2_buf_type type)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_streamon(&isp->video[FIMC_IS_VIDEO_NUM_SCALERC].vbq, type);
}

static int fimc_is_scalerc_video_streamoff(struct file *file, void *priv,
						enum v4l2_buf_type type)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_streamoff(&isp->video[FIMC_IS_VIDEO_NUM_SCALERC].vbq, type);
}

static int fimc_is_scalerc_video_enum_input(struct file *file, void *priv,
						struct v4l2_input *input)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	struct exynos5_fimc_is_sensor_info *sensor_info = isp->pdata->sensor_info[input->index];

	dbg("index(%d) sensor(%s)\n",  input->index, sensor_info->sensor_name);
	dbg("pos(%d) sensor_id(%d)\n", sensor_info->sensor_position, sensor_info->sensor_id);
	dbg("csi_id(%d) flite_id(%d)\n",  sensor_info->csi_id, sensor_info->flite_id);
	dbg("i2c_ch(%d)\n", sensor_info->i2c_channel);

	if (input->index >= FIMC_IS_MAX_CAMIF_CLIENTS)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;

	strncpy(input->name, sensor_info->sensor_name,
					FIMC_IS_MAX_SENSOR_NAME_LEN);
	return 0;
}

static int fimc_is_scalerc_video_g_input(struct file *file, void *priv,
						unsigned int *input)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerc_video_s_input(struct file *file, void *priv,
						unsigned int input)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	struct exynos5_fimc_is_sensor_info *sensor_info = isp->pdata->sensor_info[input];

	isp->sensor.id_position = input;
	isp->sensor.sensor_type = fimc_is_hw_get_sensor_type(sensor_info->sensor_id,
									sensor_info->flite_id);

	fimc_is_hw_set_default_size(isp, sensor_info->sensor_id);

	dbg("sensor info : pos(%d) type(%d)\n", input, isp->sensor.sensor_type);


	return 0;
}

const struct v4l2_file_operations fimc_is_scalerc_video_fops = {
	.owner		= THIS_MODULE,
	.open		= fimc_is_scalerc_video_open,
	.release	= fimc_is_scalerc_video_close,
	.poll		= fimc_is_scalerc_video_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= fimc_is_scalerc_video_mmap,
};

const struct v4l2_ioctl_ops fimc_is_scalerc_video_ioctl_ops = {
	.vidioc_querycap		= fimc_is_scalerc_video_querycap,
	.vidioc_enum_fmt_vid_cap_mplane	= fimc_is_scalerc_video_enum_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= fimc_is_scalerc_video_get_format_mplane,
	.vidioc_s_fmt_vid_cap_mplane	= fimc_is_scalerc_video_set_format_mplane,
	.vidioc_try_fmt_vid_cap_mplane	= fimc_is_scalerc_video_try_format_mplane,
	.vidioc_cropcap			= fimc_is_scalerc_video_cropcap,
	.vidioc_g_crop			= fimc_is_scalerc_video_get_crop,
	.vidioc_s_crop			= fimc_is_scalerc_video_set_crop,
	.vidioc_reqbufs			= fimc_is_scalerc_video_reqbufs,
	.vidioc_querybuf		= fimc_is_scalerc_video_querybuf,
	.vidioc_qbuf			= fimc_is_scalerc_video_qbuf,
	.vidioc_dqbuf			= fimc_is_scalerc_video_dqbuf,
	.vidioc_streamon		= fimc_is_scalerc_video_streamon,
	.vidioc_streamoff		= fimc_is_scalerc_video_streamoff,
	.vidioc_enum_input		= fimc_is_scalerc_video_enum_input,
	.vidioc_g_input			= fimc_is_scalerc_video_g_input,
	.vidioc_s_input			= fimc_is_scalerc_video_s_input,
};

static int fimc_is_scalerc_queue_setup(struct vb2_queue *vq,
			unsigned int *num_buffers,
			unsigned int *num_planes, unsigned long sizes[],
			void *allocators[])
{

	struct fimc_is_video_dev *video = vq->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	int i;


	*num_planes = isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame.format.num_planes;
	set_plane_size(&isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame, sizes);

	for (i = 0; i < *num_planes; i++)
		allocators[i] =  isp->alloc_ctx;

	dbg("%s(num_planes : %d)(size : %d)\n",
					__func__, (int)*num_planes, (int)sizes[0]);
	return 0;
}
static int fimc_is_scalerc_buffer_prepare(struct vb2_buffer *vb)
{
	dbg("--%s\n", __func__);
	return 0;
}


static inline void fimc_is_scalerc_lock(struct vb2_queue *vq)
{
	dbg("%s\n", __func__);
}

static inline void fimc_is_scalerc_unlock(struct vb2_queue *vq)
{
	dbg("%s\n", __func__);
}

static int fimc_is_scalerc_start_streaming(struct vb2_queue *q)
{
	struct fimc_is_video_dev *video = q->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	int ret;
	int i, j;
	int buf_index;

	dbg("%s(pipe_state : %d)\n", __func__, (int)isp->pipe_state);

	if (test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_SENSOR_INITIALIZED, &isp->pipe_state) ){

		fimc_is_init_set(isp, isp->sensor.sensor_type);

		if (test_bit(IS_ST_RUN, &isp->state)) {
			dbg("IS change mode\n");
			clear_bit(IS_ST_RUN, &isp->state);
			set_bit(IS_ST_CHANGE_MODE, &isp->state);
			fimc_is_hw_change_mode(isp, IS_MODE_PREVIEW_STILL);
			ret = wait_event_timeout(isp->irq_queue,
					test_bit(IS_ST_CHANGE_MODE_DONE,
					&isp->state),
					FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"Mode change timeout:%s\n", __func__);
				return -EBUSY;
			}
		}

		if (test_bit(IS_ST_CHANGE_MODE_DONE, &isp->state) &&
				!test_bit(IS_ST_STREAM_ON, &isp->state)) {
			dbg("IS Stream On");
			fimc_is_hw_set_stream(isp, 1);

			ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_ON, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"wait timeout : %s\n", __func__);
				while(1);
				return -EBUSY;
			}
			clear_bit(IS_ST_STREAM_ON, &isp->state);

		} else {
			dev_err(&isp->pdev->dev, "ON : not stream-on condition\n");
			return -EINVAL;
		}
		set_bit(FIMC_IS_STATE_SENSOR_INITIALIZED, &isp->pipe_state);
	}

	if (test_bit(FIMC_IS_STATE_SCALERC_BUFFER_PREPARED, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_SCALERC_STREAM_ON, &isp->pipe_state) &&
		test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state)) {

		/* buffer addr setting */
		for (i = 0; i < isp->video[FIMC_IS_VIDEO_NUM_SCALERC].num_buf; i++)
			for (j = 0; j < isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame.format.num_planes; j++) {
			buf_index = i*isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame.format.num_planes + j;
			dbg("(%d)set buf(%d:%d) = 0x%08x\n", buf_index, i, j,
				 isp->video[FIMC_IS_VIDEO_NUM_SCALERC].buf[i][j]);
			isp->is_p_region->shared[447+buf_index]
				= isp->video[FIMC_IS_VIDEO_NUM_SCALERC].buf[i][j];
		}

		dbg("buf_num:%d buf_plane:%d shared[447] : 0x%08x\n",
			isp->video[FIMC_IS_VIDEO_NUM_SCALERC].num_buf,
			isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame.format.num_planes,
			virt_to_phys(isp->mem.kvaddr_shared) + 447*sizeof(u32));

		clear_bit(IS_ST_STREAM_OFF, &isp->state);
		fimc_is_hw_set_stream(isp, 0); /*stream off */
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_OFF, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			if (!ret)
				err("s_power off failed!!\n");
			return -EBUSY;
		}

		IS_ISP_SET_PARAM_ADJUST_SHUTTER_TIME_MIN(isp, 0) ;
		IS_ISP_SET_PARAM_ADJUST_SHUTTER_TIME_MAX(isp, 33333) ;
		IS_SET_PARAM_BIT(isp, PARAM_ISP_ADJUST);
		IS_INC_PARAM_NUM(isp);


		IS_SCALERC_SET_PARAM_DMA_OUTPUT_CMD(isp,
			DMA_OUTPUT_COMMAND_ENABLE);
		IS_SCALERC_SET_PARAM_DMA_OUTPUT_BUFFERNUM(isp,
			isp->video[FIMC_IS_VIDEO_NUM_SCALERC].num_buf);
		IS_SCALERC_SET_PARAM_DMA_OUTPUT_BUFFERADDR(isp,
			(u32)isp->mem.dvaddr_shared + 447*sizeof(u32));

		IS_SET_PARAM_BIT(isp, PARAM_SCALERC_DMA_OUTPUT);
		IS_INC_PARAM_NUM(isp);

		fimc_is_mem_cache_clean((void *)isp->is_p_region,
			IS_PARAM_SIZE);

		isp->scenario_id = ISS_PREVIEW_STILL;
		set_bit(IS_ST_INIT_PREVIEW_STILL,	&isp->state);
		clear_bit(IS_ST_INIT_CAPTURE_STILL, &isp->state);
		clear_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state);
		fimc_is_hw_set_param(isp);
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}

		if (test_bit(IS_ST_RUN, &isp->state)) {
			dbg("IS change mode\n");
			clear_bit(IS_ST_RUN, &isp->state);
			set_bit(IS_ST_CHANGE_MODE, &isp->state);
			fimc_is_hw_change_mode(isp, IS_MODE_PREVIEW_STILL);
			ret = wait_event_timeout(isp->irq_queue,
				test_bit(IS_ST_CHANGE_MODE_DONE, &isp->state),
				FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"Mode change timeout:%s\n", __func__);
				return -EBUSY;
			}
		}

		if (test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state) &&
				!test_bit(IS_ST_STREAM_ON, &isp->state)) {
			dbg("IS Stream On");
			fimc_is_hw_set_stream(isp, 1);

			ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_ON, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"wait timeout : %s\n", __func__);
				return -EBUSY;
			}
			clear_bit(IS_ST_STREAM_ON, &isp->state);
		} else {
			dev_err(&isp->pdev->dev, "ON : not stream-on condition\n");
			return -EINVAL;
		}

		set_bit(FIMC_IS_STATE_SCALERC_STREAM_ON, &isp->pipe_state);
	}

	return 0;
}

static int fimc_is_scalerc_stop_streaming(struct vb2_queue *q)
{
	struct fimc_is_video_dev *video = q->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	int ret;



	clear_bit(IS_ST_STREAM_OFF, &isp->state);
	fimc_is_hw_set_stream(isp, 0); /*stream off */
	ret = wait_event_timeout(isp->irq_queue,
		test_bit(IS_ST_STREAM_OFF, &isp->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
	if (!ret) {
		dev_err(&isp->pdev->dev,
			"wait timeout : %s\n", __func__);
		if (!ret)
			err("s_power off failed!!\n");
		return -EBUSY;
	}

	IS_SCALERC_SET_PARAM_DMA_OUTPUT_CMD(isp,
		DMA_OUTPUT_COMMAND_DISABLE);
	IS_SCALERC_SET_PARAM_DMA_OUTPUT_BUFFERNUM(isp,
		0);
	IS_SCALERC_SET_PARAM_DMA_OUTPUT_BUFFERADDR(isp,
		0);

	IS_SET_PARAM_BIT(isp, PARAM_SCALERC_DMA_OUTPUT);
	IS_INC_PARAM_NUM(isp);

	fimc_is_mem_cache_clean((void *)isp->is_p_region,
		IS_PARAM_SIZE);

	isp->scenario_id = ISS_PREVIEW_STILL;
	set_bit(IS_ST_INIT_PREVIEW_STILL,	&isp->state);
	clear_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state);
	fimc_is_hw_set_param(isp);
	ret = wait_event_timeout(isp->irq_queue,
		test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
	if (!ret) {
		dev_err(&isp->pdev->dev,
			"wait timeout : %s\n", __func__);
		return -EBUSY;
	}

	if (test_bit(IS_ST_RUN, &isp->state)) {
		dbg("IS change mode\n");
		clear_bit(IS_ST_RUN, &isp->state);
		set_bit(IS_ST_CHANGE_MODE, &isp->state);
		fimc_is_hw_change_mode(isp, IS_MODE_PREVIEW_STILL);
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_CHANGE_MODE_DONE, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"Mode change timeout:%s\n", __func__);
			return -EBUSY;
		}
	}

	if (test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state) &&
			!test_bit(IS_ST_STREAM_ON, &isp->state)) {
		dbg("IS Stream On");
		fimc_is_hw_set_stream(isp, 1);

		ret = wait_event_timeout(isp->irq_queue,
		test_bit(IS_ST_STREAM_ON, &isp->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
		clear_bit(IS_ST_STREAM_ON, &isp->state);
	} else {
		dev_err(&isp->pdev->dev, "ON : not stream-on condition\n");
		return -EINVAL;
	}

	if (!test_bit(FIMC_IS_STATE_SCALERP_STREAM_ON, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_3DNR_STREAM_ON, &isp->pipe_state)) {
		clear_bit(IS_ST_STREAM_OFF, &isp->state);

		fimc_is_hw_set_stream(isp, 0);
		dbg("IS Stream Off");
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_OFF, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
	}

	clear_bit(IS_ST_RUN, &isp->state);
	clear_bit(IS_ST_STREAM_ON, &isp->state);
	clear_bit(FIMC_IS_STATE_SCALERC_BUFFER_PREPARED, &isp->pipe_state);
	clear_bit(FIMC_IS_STATE_SCALERC_STREAM_ON, &isp->pipe_state);

	return 0;
}

static void fimc_is_scalerc_buffer_queue(struct vb2_buffer *vb)
{
	struct fimc_is_video_dev *video = vb->vb2_queue->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	dma_addr_t kvaddr;
	unsigned int i;

	dbg("%s\n", __func__);
	isp->video[FIMC_IS_VIDEO_NUM_SCALERC].frame.format.num_planes = vb->num_planes;

	if (!test_bit(FIMC_IS_STATE_SCALERC_BUFFER_PREPARED, &isp->pipe_state)) {
		for (i = 0; i < vb->num_planes; i++) {
			isp->video[FIMC_IS_VIDEO_NUM_SCALERC].buf[vb->v4l2_buf.index][i]
					= isp->vb2->plane_addr(vb, i);
			kvaddr = isp->vb2->get_kvaddr(vb, i);
			dbg("index(%d)(%d) deviceVaddr(0x%08x)\n",
				vb->v4l2_buf.index, i,
				isp->video[FIMC_IS_VIDEO_NUM_SCALERC].buf[vb->v4l2_buf.index][i]);
			dbg("index(%d)(%d) KVaddr(0x%08x)\n", vb->v4l2_buf.index, i, kvaddr);
		}

		isp->video[FIMC_IS_VIDEO_NUM_SCALERC].buf_ref_cnt++;

		if (isp->video[FIMC_IS_VIDEO_NUM_SCALERC].num_buf
			== isp->video[FIMC_IS_VIDEO_NUM_SCALERC].buf_ref_cnt)
			set_bit(FIMC_IS_STATE_SCALERC_BUFFER_PREPARED, &isp->pipe_state);
	}

	if (!test_bit(FIMC_IS_STATE_SCALERC_STREAM_ON, &isp->pipe_state))
		fimc_is_scalerc_start_streaming(vb->vb2_queue);

	return;
}

const struct vb2_ops fimc_is_scalerc_qops = {
	.queue_setup		= fimc_is_scalerc_queue_setup,
	.buf_prepare		= fimc_is_scalerc_buffer_prepare,
	.buf_queue		= fimc_is_scalerc_buffer_queue,
	.wait_prepare		= fimc_is_scalerc_unlock,
	.wait_finish		= fimc_is_scalerc_lock,
	.start_streaming	= fimc_is_scalerc_start_streaming,
	.stop_streaming	= fimc_is_scalerc_stop_streaming,
};

/*************************************************************************/
/* video file opertation						 */
/************************************************************************/

static int fimc_is_scalerp_video_open(struct file *file)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	file->private_data = &isp->video[FIMC_IS_VIDEO_NUM_SCALERP];
	isp->video[FIMC_IS_VIDEO_NUM_SCALERP].num_buf = 0;
	isp->video[FIMC_IS_VIDEO_NUM_SCALERP].buf_ref_cnt = 0;

	if (!test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state)) {
		isp->sensor_num = 1;

		fimc_is_load_fw(isp);

		set_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state);
		clear_bit(FIMC_IS_STATE_SENSOR_INITIALIZED, &isp->pipe_state);
	}

	clear_bit(FIMC_IS_STATE_SCALERP_STREAM_ON, &isp->pipe_state);
	return 0;

}

static int fimc_is_scalerp_video_close(struct file *file)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	int ret;

	dbg("%s\n", __func__);
	vb2_queue_release(&isp->video[FIMC_IS_VIDEO_NUM_SCALERP].vbq);

	if (!test_bit(FIMC_IS_STATE_SCALERP_STREAM_ON, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_SCALERC_STREAM_ON, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_3DNR_STREAM_ON, &isp->pipe_state) &&
		test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->power)){
		clear_bit(FIMC_IS_PWR_ST_POWEROFF, &isp->power);
		fimc_is_hw_subip_poweroff(isp);
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(FIMC_IS_PWR_ST_POWEROFF, &isp->power),
			FIMC_IS_SHUTDOWN_TIMEOUT_SENSOR);

		if (!ret) {
			err("wait timeout : %s\n", __func__);
			ret = -EINVAL;
		}

		dbg("staop flite & mipi (pos:%d) (port:%d)\n",
			isp->sensor.id_position,
			isp->pdata->sensor_info[isp->sensor.id_position]->flite_id);
		stop_fimc_lite(isp->pdata->sensor_info[isp->sensor.id_position]->flite_id);
		stop_mipi_csi(isp->pdata->sensor_info[isp->sensor.id_position]->csi_id);

		clear_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state);
		fimc_is_hw_a5_power(isp, 0);
	}
	return 0;

}

static unsigned int fimc_is_scalerp_video_poll(struct file *file,
				      struct poll_table_struct *wait)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_poll(&isp->video[FIMC_IS_VIDEO_NUM_SCALERP].vbq, file, wait);

}

static int fimc_is_scalerp_video_mmap(struct file *file,
					struct vm_area_struct *vma)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_mmap(&isp->video[FIMC_IS_VIDEO_NUM_SCALERP].vbq, vma);

}

/*************************************************************************/
/* video ioctl operation						*/
/************************************************************************/

static int fimc_is_scalerp_video_querycap(struct file *file, void *fh,
						struct v4l2_capability *cap)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	strncpy(cap->driver, isp->pdev->name, sizeof(cap->driver) - 1);

	dbg("%s(devname : %s)\n", __func__, cap->driver);
	strncpy(cap->card, isp->pdev->name, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->version = KERNEL_VERSION(1, 0, 0);
	cap->capabilities = V4L2_CAP_STREAMING
					| V4L2_CAP_VIDEO_CAPTURE
					| V4L2_CAP_VIDEO_CAPTURE_MPLANE;

	return 0;
}

static int fimc_is_scalerp_video_enum_fmt_mplane(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerp_video_get_format_mplane(struct file *file, void *fh,
						struct v4l2_format *format)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerp_video_set_format_mplane(struct file *file, void *fh,
						struct v4l2_format *format)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix;
	struct fimc_is_fmt *frame;

	dbg("%s\n", __func__);

	pix = &format->fmt.pix_mp;
	frame = find_format(&pix->pixelformat, NULL, 0);

	if (!frame)
		return -EINVAL;

	isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame.format.pixelformat = frame->pixelformat;
	isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame.format.mbus_code = frame->mbus_code;
	isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame.format.num_planes = frame->num_planes;
	isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame.width = pix->width;
	isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame.height = pix->height;
	dbg("num_planes : %d\n", frame->num_planes);
	dbg("width : %d\n", pix->width);
	dbg("height : %d\n", pix->height);

	return 0;
}

static int fimc_is_scalerp_video_try_format_mplane(struct file *file, void *fh,
						struct v4l2_format *format)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerp_video_cropcap(struct file *file, void *fh,
						struct v4l2_cropcap *cropcap)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerp_video_get_crop(struct file *file, void *fh,
						struct v4l2_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerp_video_set_crop(struct file *file, void *fh,
						struct v4l2_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerp_video_reqbufs(struct file *file, void *priv,
						struct v4l2_requestbuffers *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s(buf->count : %d)\n", __func__, buf->count);
	ret = vb2_reqbufs(&video->vbq, buf);
	if (!ret)
		isp->video[FIMC_IS_VIDEO_NUM_SCALERP].num_buf = buf->count;

	if (buf->count == 0)
		isp->video[FIMC_IS_VIDEO_NUM_SCALERP].buf_ref_cnt = 0;

	dbg("%s(num_buf | %d)\n", __func__,
		isp->video[FIMC_IS_VIDEO_NUM_SCALERP].num_buf);

	return ret;
}

static int fimc_is_scalerp_video_querybuf(struct file *file, void *priv,
						struct v4l2_buffer *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;

	dbg("%s\n", __func__);
	ret = vb2_querybuf(&video->vbq, buf);

	return ret;
}

static int fimc_is_scalerp_video_qbuf(struct file *file, void *priv,
						struct v4l2_buffer *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;

	dbg("%s :: buf->index(%d)\n", __func__, buf->index);
	ret = vb2_qbuf(&video->vbq, buf);
	return ret;
}

static int fimc_is_scalerp_video_dqbuf(struct file *file, void *priv,
						struct v4l2_buffer *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;

	ret = vb2_dqbuf(&video->vbq, buf, file->f_flags & O_NONBLOCK);
	dbg("%s :: buf->index((0x%08x)\n", __func__, buf->index);
	return ret;
}

static int fimc_is_scalerp_video_streamon(struct file *file, void *priv,
						enum v4l2_buf_type type)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_streamon(&isp->video[FIMC_IS_VIDEO_NUM_SCALERP].vbq, type);
}

static int fimc_is_scalerp_video_streamoff(struct file *file, void *priv,
						enum v4l2_buf_type type)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_streamoff(&isp->video[FIMC_IS_VIDEO_NUM_SCALERP].vbq, type);
}

static int fimc_is_scalerp_video_enum_input(struct file *file, void *priv,
						struct v4l2_input *input)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	struct exynos5_fimc_is_sensor_info *sensor_info = isp->pdata->sensor_info[input->index];

	dbg("index(%d) sensor(%s)\n",  input->index, sensor_info->sensor_name);
	dbg("pos(%d) sensor_id(%d)\n", sensor_info->sensor_position, sensor_info->sensor_id);
	dbg("csi_id(%d) flite_id(%d)\n",  sensor_info->csi_id, sensor_info->flite_id);
	dbg("i2c_ch(%d)\n", sensor_info->i2c_channel);

	if (input->index >= FIMC_IS_MAX_CAMIF_CLIENTS)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;

	strncpy(input->name, sensor_info->sensor_name,
					FIMC_IS_MAX_SENSOR_NAME_LEN);
	return 0;
}

static int fimc_is_scalerp_video_g_input(struct file *file, void *priv,
						unsigned int *input)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_scalerp_video_s_input(struct file *file, void *priv,
					unsigned int input)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	struct exynos5_fimc_is_sensor_info *sensor_info = isp->pdata->sensor_info[input];

	isp->sensor.id_position = input;
	isp->sensor.sensor_type = fimc_is_hw_get_sensor_type(sensor_info->sensor_id,
									sensor_info->flite_id);

	fimc_is_hw_set_default_size(isp, sensor_info->sensor_id);

	dbg("sensor info : pos(%d) type(%d)\n", input, isp->sensor.sensor_type);


	return 0;
}
static int fimc_is_scalerp_video_g_ctrl(struct file *file, void *priv,
					struct v4l2_control *ctrl)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	switch (ctrl->id){
		case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
			if (!is_af_use(isp))
				ctrl->value = 0x02;
			else
				ctrl->value = isp->af.af_lock_state;
			break;
		default:
			break;
	}
	return 0;
}

static int fimc_is_scalerp_video_s_ctrl(struct file *file, void *priv,
					struct v4l2_control *ctrl)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	int ret;

	dbg("%s\n", __func__);
	switch (ctrl->id){
		case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
			ret = fimc_is_v4l2_af_start_stop(isp, ctrl->value);
			break;

		default:
			break;
	}
	return 0;
}

const struct v4l2_file_operations fimc_is_scalerp_video_fops = {
	.owner		= THIS_MODULE,
	.open		= fimc_is_scalerp_video_open,
	.release	= fimc_is_scalerp_video_close,
	.poll		= fimc_is_scalerp_video_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= fimc_is_scalerp_video_mmap,
};

const struct v4l2_ioctl_ops fimc_is_scalerp_video_ioctl_ops = {
	.vidioc_querycap		= fimc_is_scalerp_video_querycap,
	.vidioc_enum_fmt_vid_cap_mplane	= fimc_is_scalerp_video_enum_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= fimc_is_scalerp_video_get_format_mplane,
	.vidioc_s_fmt_vid_cap_mplane	= fimc_is_scalerp_video_set_format_mplane,
	.vidioc_try_fmt_vid_cap_mplane	= fimc_is_scalerp_video_try_format_mplane,
	.vidioc_cropcap			= fimc_is_scalerp_video_cropcap,
	.vidioc_g_crop			= fimc_is_scalerp_video_get_crop,
	.vidioc_s_crop			= fimc_is_scalerp_video_set_crop,
	.vidioc_reqbufs			= fimc_is_scalerp_video_reqbufs,
	.vidioc_querybuf		= fimc_is_scalerp_video_querybuf,
	.vidioc_qbuf			= fimc_is_scalerp_video_qbuf,
	.vidioc_dqbuf			= fimc_is_scalerp_video_dqbuf,
	.vidioc_streamon		= fimc_is_scalerp_video_streamon,
	.vidioc_streamoff		= fimc_is_scalerp_video_streamoff,
	.vidioc_enum_input		= fimc_is_scalerp_video_enum_input,
	.vidioc_g_input			= fimc_is_scalerp_video_g_input,
	.vidioc_s_input			= fimc_is_scalerp_video_s_input,
	.vidioc_g_ctrl			= fimc_is_scalerp_video_g_ctrl,
	.vidioc_s_ctrl			= fimc_is_scalerp_video_s_ctrl,
};

static int fimc_is_scalerp_queue_setup(struct vb2_queue *vq,
			unsigned int *num_buffers, unsigned int *num_planes,
			unsigned long sizes[],
			void *allocators[])
{

	struct fimc_is_video_dev *video = vq->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	int i;


	*num_planes = isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame.format.num_planes;
	set_plane_size(&isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame, sizes);

	for (i = 0; i < *num_planes; i++)
		allocators[i] =  isp->alloc_ctx;

	dbg("%s(num_planes : %d)(size : %d)\n",
					__func__, (int)*num_planes, (int)sizes[0]);

	return 0;
}
static int fimc_is_scalerp_buffer_prepare(struct vb2_buffer *vb)
{
	dbg("--%s\n", __func__);
	return 0;
}


static inline void fimc_is_scalerp_lock(struct vb2_queue *vq)
{
	dbg("%s\n", __func__);
}

static inline void fimc_is_scalerp_unlock(struct vb2_queue *vq)
{
	dbg("%s\n", __func__);
}

static int fimc_is_scalerp_start_streaming(struct vb2_queue *q)
{
	struct fimc_is_video_dev *video = q->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	int ret;
	int i, j;
	int buf_index;

	dbg("%s(pipe_state : %d)\n", __func__, (int)isp->pipe_state);

	if (test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_SENSOR_INITIALIZED, &isp->pipe_state) ){

		fimc_is_init_set(isp, isp->sensor.sensor_type);

		if (test_bit(IS_ST_RUN, &isp->state)) {
			dbg("IS change mode\n");
			clear_bit(IS_ST_RUN, &isp->state);
			set_bit(IS_ST_CHANGE_MODE, &isp->state);
			fimc_is_hw_change_mode(isp, IS_MODE_PREVIEW_STILL);
			ret = wait_event_timeout(isp->irq_queue,
					test_bit(IS_ST_CHANGE_MODE_DONE,
					&isp->state),
					FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"Mode change timeout:%s\n", __func__);
				return -EBUSY;
			}
		}

		if (test_bit(IS_ST_CHANGE_MODE_DONE, &isp->state) &&
				!test_bit(IS_ST_STREAM_ON, &isp->state)) {
			dbg("IS Stream On");
			fimc_is_hw_set_stream(isp, 1);

			ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_ON, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"wait timeout : %s\n", __func__);
				while(1);
				return -EBUSY;
			}
			clear_bit(IS_ST_STREAM_ON, &isp->state);

		} else {
			dev_err(&isp->pdev->dev, "ON : not stream-on condition\n");
			return -EINVAL;
		}
		set_bit(FIMC_IS_STATE_SENSOR_INITIALIZED, &isp->pipe_state);
	}

	if (test_bit(FIMC_IS_STATE_SCALERP_BUFFER_PREPARED, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_SCALERP_STREAM_ON, &isp->pipe_state) &&
		test_bit(FIMC_IS_STATE_SENSOR_INITIALIZED, &isp->pipe_state)) {
		/* frame rate setting */
		IS_ISP_SET_PARAM_ADJUST_SHUTTER_TIME_MIN(isp, 0) ;
		IS_ISP_SET_PARAM_ADJUST_SHUTTER_TIME_MAX(isp, 33333) ;
		IS_SET_PARAM_BIT(isp, PARAM_ISP_ADJUST);
		IS_INC_PARAM_NUM(isp);

		/* buffer addr setting */
		for (i = 0; i < isp->video[FIMC_IS_VIDEO_NUM_SCALERP].num_buf; i++)
			for (j = 0; j < isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame.format.num_planes; j++) {
			buf_index = i*isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame.format.num_planes + j;
			dbg("(%d)set buf(%d:%d) = 0x%08x\n", buf_index, i, j,
				 isp->video[FIMC_IS_VIDEO_NUM_SCALERP].buf[i][j]);
			isp->is_p_region->shared[400+buf_index]
				= isp->video[FIMC_IS_VIDEO_NUM_SCALERP].buf[i][j];
		}

		dbg("buf_num:%d buf_plane:%d shared[400] : 0x%08x\n",
			isp->video[FIMC_IS_VIDEO_NUM_SCALERP].num_buf,
			isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame.format.num_planes,
			virt_to_phys(isp->mem.kvaddr_shared) + 400*sizeof(u32));

		IS_SCALERP_SET_PARAM_DMA_OUTPUT_CMD(isp,
			DMA_OUTPUT_COMMAND_ENABLE);
		IS_SCALERP_SET_PARAM_DMA_OUTPUT_BUFFERNUM(isp,
			isp->video[FIMC_IS_VIDEO_NUM_SCALERP].num_buf);
		IS_SCALERP_SET_PARAM_DMA_OUTPUT_BUFFERADDR(isp,
			(u32)isp->mem.dvaddr_shared + 400*sizeof(u32));
		IS_SET_PARAM_BIT(isp, PARAM_SCALERP_DMA_OUTPUT);
		IS_INC_PARAM_NUM(isp);

		fimc_is_mem_cache_clean((void *)isp->is_p_region,
			IS_PARAM_SIZE);

		clear_bit(IS_ST_STREAM_OFF, &isp->state);
		fimc_is_hw_set_stream(isp, 0); /*stream off */
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_OFF, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			if (!ret)
				err("s_power off failed!!\n");
			return -EBUSY;
		}

		isp->scenario_id = ISS_PREVIEW_STILL;
		set_bit(IS_ST_INIT_PREVIEW_STILL,	&isp->state);
		clear_bit(IS_ST_INIT_CAPTURE_STILL, &isp->state);
		clear_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state);
		fimc_is_hw_set_param(isp);
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}

		if (test_bit(IS_ST_RUN, &isp->state)) {
			dbg("IS change mode\n");
			clear_bit(IS_ST_RUN, &isp->state);
			set_bit(IS_ST_CHANGE_MODE, &isp->state);
			fimc_is_hw_change_mode(isp, IS_MODE_PREVIEW_STILL);
			ret = wait_event_timeout(isp->irq_queue,
					test_bit(IS_ST_CHANGE_MODE_DONE, &isp->state),
					FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"Mode change timeout:%s\n", __func__);
				return -EBUSY;
			}
		}

		if (test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state) &&
				!test_bit(IS_ST_STREAM_ON, &isp->state)) {
			dbg("IS Stream On");
			fimc_is_hw_set_stream(isp, 1);

			ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_ON, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"wait timeout : %s\n", __func__);
				return -EBUSY;
			}
			clear_bit(IS_ST_STREAM_ON, &isp->state);

		} else {
			dev_err(&isp->pdev->dev, "ON : not stream-on condition\n");
			return -EINVAL;
		}
		set_bit(FIMC_IS_STATE_SCALERP_STREAM_ON, &isp->pipe_state);
	}

	return 0;
}

static int fimc_is_scalerp_stop_streaming(struct vb2_queue *q)
{
	struct fimc_is_video_dev *video = q->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	int ret;

	clear_bit(IS_ST_STREAM_OFF, &isp->state);
	fimc_is_hw_set_stream(isp, 0); /*stream off */
	ret = wait_event_timeout(isp->irq_queue,
		test_bit(IS_ST_STREAM_OFF, &isp->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
	if (!ret) {
		dev_err(&isp->pdev->dev,
			"wait timeout : %s\n", __func__);
		if (!ret)
			err("s_power off failed!!\n");
		return -EBUSY;
	}

	IS_SCALERP_SET_PARAM_DMA_OUTPUT_CMD(isp,
		DMA_OUTPUT_COMMAND_DISABLE);
	IS_SCALERP_SET_PARAM_DMA_OUTPUT_BUFFERNUM(isp,
		0);
	IS_SCALERP_SET_PARAM_DMA_OUTPUT_BUFFERADDR(isp,
		0);

	IS_SET_PARAM_BIT(isp, PARAM_SCALERP_DMA_OUTPUT);
	IS_INC_PARAM_NUM(isp);

	fimc_is_mem_cache_clean((void *)isp->is_p_region,
		IS_PARAM_SIZE);

	isp->scenario_id = ISS_PREVIEW_STILL;
	set_bit(IS_ST_INIT_PREVIEW_STILL,	&isp->state);
	clear_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state);
	fimc_is_hw_set_param(isp);
	ret = wait_event_timeout(isp->irq_queue,
		test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
	if (!ret) {
		dev_err(&isp->pdev->dev,
			"wait timeout : %s\n", __func__);
		return -EBUSY;
	}

	if (test_bit(IS_ST_RUN, &isp->state)) {
		dbg("IS change mode\n");
		clear_bit(IS_ST_RUN, &isp->state);
		set_bit(IS_ST_CHANGE_MODE, &isp->state);
		fimc_is_hw_change_mode(isp, IS_MODE_PREVIEW_STILL);
		ret = wait_event_timeout(isp->irq_queue,
				test_bit(IS_ST_CHANGE_MODE_DONE, &isp->state),
				FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"Mode change timeout:%s\n", __func__);
			return -EBUSY;
		}
	}

	if (test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state) &&
			!test_bit(IS_ST_STREAM_ON, &isp->state)) {
		dbg("IS Stream On");
		fimc_is_hw_set_stream(isp, 1);

		ret = wait_event_timeout(isp->irq_queue,
		test_bit(IS_ST_STREAM_ON, &isp->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
		clear_bit(IS_ST_STREAM_ON, &isp->state);
	} else {
		dev_err(&isp->pdev->dev, "ON : not stream-on condition\n");
		return -EINVAL;
	}

	if (!test_bit(FIMC_IS_STATE_SCALERC_STREAM_ON, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_3DNR_STREAM_ON, &isp->pipe_state)) {
		clear_bit(IS_ST_STREAM_OFF, &isp->state);
		dbg("IS Stream Off");
		fimc_is_hw_set_stream(isp, 0);
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_OFF, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
	}
	clear_bit(IS_ST_RUN, &isp->state);
	clear_bit(IS_ST_STREAM_ON, &isp->state);
	clear_bit(FIMC_IS_STATE_SCALERP_BUFFER_PREPARED, &isp->pipe_state);
	clear_bit(FIMC_IS_STATE_SCALERP_STREAM_ON, &isp->pipe_state);

	return 0;
}

static void fimc_is_scalerp_buffer_queue(struct vb2_buffer *vb)
{
	struct fimc_is_video_dev *video = vb->vb2_queue->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	dma_addr_t kvaddr;
	unsigned int i;

	dbg("%s\n", __func__);
	isp->video[FIMC_IS_VIDEO_NUM_SCALERP].frame.format.num_planes = vb->num_planes;

	if (!test_bit(FIMC_IS_STATE_SCALERP_BUFFER_PREPARED, &isp->pipe_state)) {
		for (i = 0; i < vb->num_planes; i++) {
			isp->video[FIMC_IS_VIDEO_NUM_SCALERP].buf[vb->v4l2_buf.index][i]
				= isp->vb2->plane_addr(vb, i);
			kvaddr = isp->vb2->get_kvaddr(vb, i);
			dbg("index(%d)(%d) deviceVaddr(0x%08x)\n", vb->v4l2_buf.index, i,
				isp->video[FIMC_IS_VIDEO_NUM_SCALERP].buf[vb->v4l2_buf.index][i]);
			dbg("index(%d)(%d) KVaddr(0x%08x)\n", vb->v4l2_buf.index, i, kvaddr);
		}

		isp->video[FIMC_IS_VIDEO_NUM_SCALERP].buf_ref_cnt++;

		if (isp->video[FIMC_IS_VIDEO_NUM_SCALERP].num_buf
			== isp->video[FIMC_IS_VIDEO_NUM_SCALERP].buf_ref_cnt)
			set_bit(FIMC_IS_STATE_SCALERP_BUFFER_PREPARED, &isp->pipe_state);
	}

	if (!test_bit(FIMC_IS_STATE_SCALERP_STREAM_ON, &isp->pipe_state))
		fimc_is_scalerp_start_streaming(vb->vb2_queue);

	return;
}

const struct vb2_ops fimc_is_scalerp_qops = {
	.queue_setup		= fimc_is_scalerp_queue_setup,
	.buf_prepare		= fimc_is_scalerp_buffer_prepare,
	.buf_queue		= fimc_is_scalerp_buffer_queue,
	.wait_prepare		= fimc_is_scalerp_unlock,
	.wait_finish		= fimc_is_scalerp_lock,
	.start_streaming	= fimc_is_scalerp_start_streaming,
	.stop_streaming	= fimc_is_scalerp_stop_streaming,
};


/*************************************************************************/
/* video file opertation						 */
/************************************************************************/

static int fimc_is_3dnr_video_open(struct file *file)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	file->private_data = &isp->video[FIMC_IS_VIDEO_NUM_3DNR];
	isp->video[FIMC_IS_VIDEO_NUM_3DNR].num_buf = 0;
	isp->video[FIMC_IS_VIDEO_NUM_3DNR].buf_ref_cnt = 0;

	if (!test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state)) {
		isp->sensor_num = 1;

		fimc_is_load_fw(isp);

		set_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state);
		clear_bit(FIMC_IS_STATE_SENSOR_INITIALIZED, &isp->pipe_state);
	}

	clear_bit(FIMC_IS_STATE_3DNR_STREAM_ON, &isp->pipe_state);
	return 0;

}

static int fimc_is_3dnr_video_close(struct file *file)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	int ret;

	dbg("%s\n", __func__);
	vb2_queue_release(&isp->video[FIMC_IS_VIDEO_NUM_3DNR].vbq);

	if (!test_bit(FIMC_IS_STATE_SCALERP_STREAM_ON, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_SCALERC_STREAM_ON, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_3DNR_STREAM_ON, &isp->pipe_state) &&
		test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->power)){
		clear_bit(FIMC_IS_PWR_ST_POWEROFF, &isp->power);
		fimc_is_hw_subip_poweroff(isp);
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(FIMC_IS_PWR_ST_POWEROFF, &isp->power),
			FIMC_IS_SHUTDOWN_TIMEOUT_SENSOR);

		if (!ret) {
			err("wait timeout : %s\n", __func__);
			ret = -EINVAL;
		}

		dbg("staop flite & mipi (pos:%d) (port:%d)\n",
			isp->sensor.id_position,
			isp->pdata->sensor_info[isp->sensor.id_position]->flite_id);
		stop_fimc_lite(isp->pdata->sensor_info[isp->sensor.id_position]->flite_id);
		stop_mipi_csi(isp->pdata->sensor_info[isp->sensor.id_position]->csi_id);

		clear_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state);
		fimc_is_hw_a5_power(isp, 0);
	}
	return 0;

}

static unsigned int fimc_is_3dnr_video_poll(struct file *file,
				      struct poll_table_struct *wait)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_poll(&isp->video[FIMC_IS_VIDEO_NUM_3DNR].vbq, file, wait);

}

static int fimc_is_3dnr_video_mmap(struct file *file,
					struct vm_area_struct *vma)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_mmap(&isp->video[FIMC_IS_VIDEO_NUM_3DNR].vbq, vma);

}

/*************************************************************************/
/* video ioctl operation						*/
/************************************************************************/

static int fimc_is_3dnr_video_querycap(struct file *file, void *fh,
					struct v4l2_capability *cap)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	strncpy(cap->driver, isp->pdev->name, sizeof(cap->driver) - 1);

	dbg("%s(devname : %s)\n", __func__, cap->driver);
	strncpy(cap->card, isp->pdev->name, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->version = KERNEL_VERSION(1, 0, 0);
	cap->capabilities = V4L2_CAP_STREAMING
				| V4L2_CAP_VIDEO_CAPTURE
				| V4L2_CAP_VIDEO_CAPTURE_MPLANE;

	return 0;
}

static int fimc_is_3dnr_video_enum_fmt_mplane(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_3dnr_video_get_format_mplane(struct file *file, void *fh,
						struct v4l2_format *format)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_3dnr_video_set_format_mplane(struct file *file, void *fh,
						struct v4l2_format *format)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix;
	struct fimc_is_fmt *frame;

	dbg("%s\n", __func__);

	pix = &format->fmt.pix_mp;
	frame = find_format(&pix->pixelformat, NULL, 0);

	if (!frame)
		return -EINVAL;

	isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame.format.pixelformat = frame->pixelformat;
	isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame.format.mbus_code = frame->mbus_code;
	isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame.format.num_planes = frame->num_planes;
	isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame.width = pix->width;
	isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame.height = pix->height;
	dbg("num_planes : %d\n", frame->num_planes);
	dbg("width : %d\n", pix->width);
	dbg("height : %d\n", pix->height);

	return 0;
}

static int fimc_is_3dnr_video_try_format_mplane(struct file *file, void *fh,
						struct v4l2_format *format)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_3dnr_video_cropcap(struct file *file, void *fh,
						struct v4l2_cropcap *cropcap)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_3dnr_video_get_crop(struct file *file, void *fh,
						struct v4l2_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_3dnr_video_set_crop(struct file *file, void *fh,
						struct v4l2_crop *crop)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_3dnr_video_reqbufs(struct file *file, void *priv,
						struct v4l2_requestbuffers *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	ret = vb2_reqbufs(&video->vbq, buf);
	if (!ret)
		isp->video[FIMC_IS_VIDEO_NUM_3DNR].num_buf = buf->count;

	if (buf->count == 0)
		isp->video[FIMC_IS_VIDEO_NUM_3DNR].buf_ref_cnt = 0;

	dbg("%s(num_buf | %d)\n", __func__,
		isp->video[FIMC_IS_VIDEO_NUM_3DNR].num_buf);

	return ret;
}

static int fimc_is_3dnr_video_querybuf(struct file *file, void *priv,
						struct v4l2_buffer *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;

	dbg("%s\n", __func__);
	ret = vb2_querybuf(&video->vbq, buf);

	return ret;
}

static int fimc_is_3dnr_video_qbuf(struct file *file, void *priv,
						struct v4l2_buffer *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;

	dbg("%s :: buf->index(%d)\n", __func__, buf->index);
	ret = vb2_qbuf(&video->vbq, buf);
	return ret;
}

static int fimc_is_3dnr_video_dqbuf(struct file *file, void *priv,
						struct v4l2_buffer *buf)
{
	int ret;
	struct fimc_is_video_dev *video = file->private_data;

	dbg("%s\n", __func__);
	ret = vb2_dqbuf(&video->vbq, buf, file->f_flags & O_NONBLOCK);
	return ret;
}

static int fimc_is_3dnr_video_streamon(struct file *file, void *priv,
						enum v4l2_buf_type type)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_streamon(&isp->video[FIMC_IS_VIDEO_NUM_3DNR].vbq, type);
}

static int fimc_is_3dnr_video_streamoff(struct file *file, void *priv,
						enum v4l2_buf_type type)
{
	struct fimc_is_dev *isp = video_drvdata(file);

	dbg("%s\n", __func__);
	return vb2_streamoff(&isp->video[FIMC_IS_VIDEO_NUM_3DNR].vbq, type);
}

static int fimc_is_3dnr_video_enum_input(struct file *file, void *priv,
						struct v4l2_input *input)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	struct exynos5_fimc_is_sensor_info *sensor_info = isp->pdata->sensor_info[input->index];

	dbg("index(%d) sensor(%s)\n",  input->index, sensor_info->sensor_name);
	dbg("pos(%d) sensor_id(%d)\n", sensor_info->sensor_position, sensor_info->sensor_id);
	dbg("csi_id(%d) flite_id(%d)\n",  sensor_info->csi_id, sensor_info->flite_id);
	dbg("i2c_ch(%d)\n", sensor_info->i2c_channel);

	if (input->index >= FIMC_IS_MAX_CAMIF_CLIENTS)
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;

	strncpy(input->name, sensor_info->sensor_name,
					FIMC_IS_MAX_SENSOR_NAME_LEN);
	return 0;
}

static int fimc_is_3dnr_video_g_input(struct file *file, void *priv,
						unsigned int *input)
{
	dbg("%s\n", __func__);
	return 0;
}

static int fimc_is_3dnr_video_s_input(struct file *file, void *priv,
						unsigned int input)
{
	struct fimc_is_dev *isp = video_drvdata(file);
	struct exynos5_fimc_is_sensor_info *sensor_info = isp->pdata->sensor_info[input];

	isp->sensor.id_position = input;
	isp->sensor.sensor_type = fimc_is_hw_get_sensor_type(sensor_info->sensor_id,
									sensor_info->flite_id);

	fimc_is_hw_set_default_size(isp, sensor_info->sensor_id);

	dbg("%s sensor info : pos(%d) type(%d)\n", __func__, input, isp->sensor.sensor_type);


	return 0;
}

const struct v4l2_file_operations fimc_is_3dnr_video_fops = {
	.owner		= THIS_MODULE,
	.open		= fimc_is_3dnr_video_open,
	.release	= fimc_is_3dnr_video_close,
	.poll		= fimc_is_3dnr_video_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= fimc_is_3dnr_video_mmap,
};

const struct v4l2_ioctl_ops fimc_is_3dnr_video_ioctl_ops = {
	.vidioc_querycap		= fimc_is_3dnr_video_querycap,
	.vidioc_enum_fmt_vid_cap_mplane	= fimc_is_3dnr_video_enum_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= fimc_is_3dnr_video_get_format_mplane,
	.vidioc_s_fmt_vid_cap_mplane	= fimc_is_3dnr_video_set_format_mplane,
	.vidioc_try_fmt_vid_cap_mplane	= fimc_is_3dnr_video_try_format_mplane,
	.vidioc_cropcap			= fimc_is_3dnr_video_cropcap,
	.vidioc_g_crop			= fimc_is_3dnr_video_get_crop,
	.vidioc_s_crop			= fimc_is_3dnr_video_set_crop,
	.vidioc_reqbufs			= fimc_is_3dnr_video_reqbufs,
	.vidioc_querybuf		= fimc_is_3dnr_video_querybuf,
	.vidioc_qbuf			= fimc_is_3dnr_video_qbuf,
	.vidioc_dqbuf			= fimc_is_3dnr_video_dqbuf,
	.vidioc_streamon		= fimc_is_3dnr_video_streamon,
	.vidioc_streamoff		= fimc_is_3dnr_video_streamoff,
	.vidioc_enum_input		= fimc_is_3dnr_video_enum_input,
	.vidioc_g_input			= fimc_is_3dnr_video_g_input,
	.vidioc_s_input			= fimc_is_3dnr_video_s_input,
};

static int fimc_is_3dnr_queue_setup(struct vb2_queue *vq,
			unsigned int *num_buffers, unsigned int *num_planes,
			unsigned long sizes[],
			void *allocators[])
{

	struct fimc_is_video_dev *video = vq->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	int i;

	*num_planes = isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame.format.num_planes;
	set_plane_size(&isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame, sizes);

	for (i = 0; i < *num_planes; i++)
		allocators[i] =  isp->alloc_ctx;

	dbg("%s(num_planes : %d)(size : %d)\n",
					__func__, (int)*num_planes, (int)sizes[0]);

	return 0;
}

static int fimc_is_3dnr_buffer_prepare(struct vb2_buffer *vb)
{
	dbg("--%s\n", __func__);
	return 0;
}

static inline void fimc_is_3dnr_lock(struct vb2_queue *vq)
{
	dbg("%s\n", __func__);
}

static inline void fimc_is_3dnr_unlock(struct vb2_queue *vq)
{
	dbg("%s\n", __func__);
}

static int fimc_is_3dnr_start_streaming(struct vb2_queue *q)
{
	struct fimc_is_video_dev *video = q->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	int ret;
	int i, j;
	int buf_index;

	dbg("%s(pipe_state : %d)\n", __func__, (int)isp->pipe_state);

	if (test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_SENSOR_INITIALIZED, &isp->pipe_state) ){

		fimc_is_init_set(isp, isp->sensor.sensor_type);

		if (test_bit(IS_ST_RUN, &isp->state)) {
			dbg("IS change mode\n");
			clear_bit(IS_ST_RUN, &isp->state);
			set_bit(IS_ST_CHANGE_MODE, &isp->state);
			fimc_is_hw_change_mode(isp, IS_MODE_PREVIEW_STILL);
			ret = wait_event_timeout(isp->irq_queue,
					test_bit(IS_ST_CHANGE_MODE_DONE,
					&isp->state),
					FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"Mode change timeout:%s\n", __func__);
				return -EBUSY;
			}
		}

		if (test_bit(IS_ST_CHANGE_MODE_DONE, &isp->state) &&
				!test_bit(IS_ST_STREAM_ON, &isp->state)) {
			dbg("IS Stream On");
			fimc_is_hw_set_stream(isp, 1);

			ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_ON, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"wait timeout : %s\n", __func__);
				while(1);
				return -EBUSY;
			}
			clear_bit(IS_ST_STREAM_ON, &isp->state);

		} else {
			dev_err(&isp->pdev->dev, "ON : not stream-on condition\n");
			return -EINVAL;
		}
		set_bit(FIMC_IS_STATE_SENSOR_INITIALIZED, &isp->pipe_state);
	}

	if (test_bit(FIMC_IS_STATE_3DNR_BUFFER_PREPARED, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_3DNR_STREAM_ON, &isp->pipe_state) &&
		test_bit(FIMC_IS_STATE_FW_DOWNLOADED, &isp->pipe_state)) {

		/* buffer addr setting */
		for (i = 0; i < isp->video[FIMC_IS_VIDEO_NUM_3DNR].num_buf; i++)
			for (j = 0; j < isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame.format.num_planes; j++) {
			buf_index = i*isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame.format.num_planes + j;
			dbg("(%d)set buf(%d:%d) = 0x%08x\n", buf_index, i, j,
				 isp->video[FIMC_IS_VIDEO_NUM_3DNR].buf[i][j]);
			isp->is_p_region->shared[350+buf_index]
				= isp->video[FIMC_IS_VIDEO_NUM_3DNR].buf[i][j];
		}

		dbg("buf_num:%d buf_plane:%d shared[350] : 0x%08x\n",
			isp->video[FIMC_IS_VIDEO_NUM_3DNR].num_buf,
			isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame.format.num_planes,
			virt_to_phys(isp->mem.kvaddr_shared) + 350*sizeof(u32));

		clear_bit(IS_ST_STREAM_OFF, &isp->state);
		fimc_is_hw_set_stream(isp, 0); /*stream off */
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_OFF, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			if (!ret)
				err("s_power off failed!!\n");
			return -EBUSY;
		}
		IS_ISP_SET_PARAM_ADJUST_SHUTTER_TIME_MIN(isp, 0) ;
		IS_ISP_SET_PARAM_ADJUST_SHUTTER_TIME_MAX(isp, 33333) ;
		IS_SET_PARAM_BIT(isp, PARAM_ISP_ADJUST);
		IS_INC_PARAM_NUM(isp);

		IS_TDNR_SET_PARAM_DMA_OUTPUT_CMD(isp,
			DMA_OUTPUT_COMMAND_ENABLE);
		IS_TDNR_SET_PARAM_DMA_OUTPUT_BUFFERNUM(isp,
			isp->video[FIMC_IS_VIDEO_NUM_3DNR].num_buf);
		IS_TDNR_SET_PARAM_DMA_OUTPUT_BUFFERADDR(isp,
			(u32)isp->mem.dvaddr_shared + 350*sizeof(u32));

		IS_SET_PARAM_BIT(isp, PARAM_TDNR_DMA_OUTPUT);
		IS_INC_PARAM_NUM(isp);

		fimc_is_mem_cache_clean((void *)isp->is_p_region,
			IS_PARAM_SIZE);

		isp->scenario_id = ISS_PREVIEW_STILL;
		set_bit(IS_ST_INIT_PREVIEW_STILL,	&isp->state);
		clear_bit(IS_ST_INIT_CAPTURE_STILL, &isp->state);
		clear_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state);
		fimc_is_hw_set_param(isp);
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}

		if (test_bit(IS_ST_RUN, &isp->state)) {
			dbg("IS change mode\n");
			clear_bit(IS_ST_RUN, &isp->state);
			set_bit(IS_ST_CHANGE_MODE, &isp->state);
			fimc_is_hw_change_mode(isp, IS_MODE_PREVIEW_STILL);
			ret = wait_event_timeout(isp->irq_queue,
					test_bit(IS_ST_CHANGE_MODE_DONE, &isp->state),
					FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"Mode change timeout:%s\n", __func__);
				return -EBUSY;
			}
		}

		if (test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state) &&
				!test_bit(IS_ST_STREAM_ON, &isp->state)) {
			dbg("IS Stream On");
			fimc_is_hw_set_stream(isp, 1);

			ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_ON, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
			if (!ret) {
				dev_err(&isp->pdev->dev,
					"wait timeout : %s\n", __func__);
				return -EBUSY;
			}
			clear_bit(IS_ST_STREAM_ON, &isp->state);
		} else {
			dev_err(&isp->pdev->dev, "ON : not stream-on condition\n");
			return -EINVAL;
		}

		set_bit(FIMC_IS_STATE_3DNR_STREAM_ON, &isp->pipe_state);
	}

	return 0;
}

static int fimc_is_3dnr_stop_streaming(struct vb2_queue *q)
{
	struct fimc_is_video_dev *video = q->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	int ret;



	clear_bit(IS_ST_STREAM_OFF, &isp->state);
	fimc_is_hw_set_stream(isp, 0); /*stream off */
	ret = wait_event_timeout(isp->irq_queue,
		test_bit(IS_ST_STREAM_OFF, &isp->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
	if (!ret) {
		dev_err(&isp->pdev->dev,
			"wait timeout : %s\n", __func__);
		if (!ret)
			err("s_power off failed!!\n");
		return -EBUSY;
	}

	IS_TDNR_SET_PARAM_DMA_OUTPUT_CMD(isp,
		DMA_OUTPUT_COMMAND_DISABLE);
	IS_TDNR_SET_PARAM_DMA_OUTPUT_BUFFERNUM(isp,
		0);
	IS_TDNR_SET_PARAM_DMA_OUTPUT_BUFFERADDR(isp,
		0);

	IS_SET_PARAM_BIT(isp, PARAM_TDNR_DMA_OUTPUT);
	IS_INC_PARAM_NUM(isp);

	fimc_is_mem_cache_clean((void *)isp->is_p_region,
		IS_PARAM_SIZE);

	isp->scenario_id = ISS_PREVIEW_STILL;
	set_bit(IS_ST_INIT_PREVIEW_STILL,	&isp->state);
	clear_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state);
	fimc_is_hw_set_param(isp);
	ret = wait_event_timeout(isp->irq_queue,
		test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
	if (!ret) {
		dev_err(&isp->pdev->dev,
			"wait timeout : %s\n", __func__);
		return -EBUSY;
	}

	if (test_bit(IS_ST_RUN, &isp->state)) {
		dbg("IS change mode\n");
		clear_bit(IS_ST_RUN, &isp->state);
		set_bit(IS_ST_CHANGE_MODE, &isp->state);
		fimc_is_hw_change_mode(isp, IS_MODE_PREVIEW_STILL);
		ret = wait_event_timeout(isp->irq_queue,
				test_bit(IS_ST_CHANGE_MODE_DONE, &isp->state),
				FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"Mode change timeout:%s\n", __func__);
			return -EBUSY;
		}
	}

	if (test_bit(IS_ST_INIT_PREVIEW_VIDEO, &isp->state) &&
			!test_bit(IS_ST_STREAM_ON, &isp->state)) {
		dbg("IS Stream On");
		fimc_is_hw_set_stream(isp, 1);

		ret = wait_event_timeout(isp->irq_queue,
		test_bit(IS_ST_STREAM_ON, &isp->state),
		FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
		clear_bit(IS_ST_STREAM_ON, &isp->state);
	} else {
		dev_err(&isp->pdev->dev, "ON : not stream-on condition\n");
		return -EINVAL;
	}

	if (!test_bit(FIMC_IS_STATE_SCALERC_STREAM_ON, &isp->pipe_state) &&
		!test_bit(FIMC_IS_STATE_SCALERP_STREAM_ON, &isp->pipe_state)) {
		clear_bit(IS_ST_STREAM_OFF, &isp->state);
		fimc_is_hw_set_stream(isp, 0);
		dbg("IS Stream Off");
		ret = wait_event_timeout(isp->irq_queue,
			test_bit(IS_ST_STREAM_OFF, &isp->state),
			FIMC_IS_SHUTDOWN_TIMEOUT);
		if (!ret) {
			dev_err(&isp->pdev->dev,
				"wait timeout : %s\n", __func__);
			return -EBUSY;
		}
	}

	clear_bit(IS_ST_RUN, &isp->state);
	clear_bit(IS_ST_STREAM_ON, &isp->state);
	clear_bit(FIMC_IS_STATE_3DNR_BUFFER_PREPARED, &isp->pipe_state);
	clear_bit(FIMC_IS_STATE_3DNR_STREAM_ON, &isp->pipe_state);

	return 0;
}

static void fimc_is_3dnr_buffer_queue(struct vb2_buffer *vb)
{
	struct fimc_is_video_dev *video = vb->vb2_queue->drv_priv;
	struct fimc_is_dev	*isp = video->dev;
	dma_addr_t kvaddr;
	unsigned int i;

	dbg("%s\n", __func__);
	isp->video[FIMC_IS_VIDEO_NUM_3DNR].frame.format.num_planes = vb->num_planes;

	if (!test_bit(FIMC_IS_STATE_3DNR_BUFFER_PREPARED, &isp->pipe_state)) {
		for (i = 0; i < vb->num_planes; i++) {
			isp->video[FIMC_IS_VIDEO_NUM_3DNR].buf[vb->v4l2_buf.index][i]
				= isp->vb2->plane_addr(vb, i);
			kvaddr = isp->vb2->get_kvaddr(vb, i);
			dbg("index(%d)(%d) deviceVaddr(0x%08x)\n", vb->v4l2_buf.index, i,
				isp->video[FIMC_IS_VIDEO_NUM_3DNR].buf[vb->v4l2_buf.index][i]);
			dbg("index(%d)(%d) KVaddr(0x%08x)\n",
				vb->v4l2_buf.index, i, kvaddr);
		}

		isp->video[FIMC_IS_VIDEO_NUM_3DNR].buf_ref_cnt++;

		if (isp->video[FIMC_IS_VIDEO_NUM_3DNR].num_buf
			== isp->video[FIMC_IS_VIDEO_NUM_3DNR].buf_ref_cnt)
			set_bit(FIMC_IS_STATE_3DNR_BUFFER_PREPARED, &isp->pipe_state);
	}

	if (!test_bit(FIMC_IS_STATE_3DNR_STREAM_ON, &isp->pipe_state))
		fimc_is_3dnr_start_streaming(vb->vb2_queue);

	return;
}

const struct vb2_ops fimc_is_3dnr_qops = {
	.queue_setup		= fimc_is_3dnr_queue_setup,
	.buf_prepare		= fimc_is_3dnr_buffer_prepare,
	.buf_queue		= fimc_is_3dnr_buffer_queue,
	.wait_prepare		= fimc_is_3dnr_unlock,
	.wait_finish		= fimc_is_3dnr_lock,
	.start_streaming	= fimc_is_3dnr_start_streaming,
	.stop_streaming	= fimc_is_3dnr_stop_streaming,
};

