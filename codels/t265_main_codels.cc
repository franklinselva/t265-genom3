/*
 * Copyright (c) 2019 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                              Martin Jacquet - February 2020
 */
#include "act265.h"

#include "t265_c_types.h"

#include "codels.hpp"

#include <err.h>
#include <cmath>
#include <iostream>

/* --- Task main -------------------------------------------------------- */


/** Codel t265_main_start of task main.
 *
 * Triggered by t265_start.
 * Yields to t265_poll.
 */
genom_event
t265_main_start(t265_ids *ids, const t265_extrinsics *extrinsics,
                const t265_frame *frame, const genom_context self)
{
    *extrinsics->data(self) = {0,0,0,0,0,0};
    extrinsics->write(self);

    ids->pipe = new or_camera_pipe();
    ids->undist = new t265_undist_s();
    ids->info.started = false;
    ids->info.compression_rate = -1;
    ids->info.frequency = 30;

    frame->open("raw", self);
    frame->open("compressed", self);

    (void)genom_sequence_reserve(&(frame->data("raw", self)->pixels), 0);
    (void)genom_sequence_reserve(&(frame->data("compressed", self)->pixels), 0);

    return t265_poll;
}


/** Codel t265_main_poll of task main.
 *
 * Triggered by t265_poll.
 * Yields to t265_pause_poll, t265_pub.
 */
genom_event
t265_main_poll(bool started, or_camera_pipe **pipe,
               const genom_context self)
{
    if (!started)
        return t265_pause_poll;

    try {
        (*pipe)->data = (*pipe)->pipe.wait_for_frames(15000);
    }
    catch (rs2::error e) {
        warnx("%s\n", e.what());
        return t265_pause_poll;
    }

    return t265_pub;
}


/** Codel t265_main_pub of task main.
 *
 * Triggered by t265_pub.
 * Yields to t265_poll.
 */
genom_event
t265_main_pub(int16_t compression_rate, const or_camera_pipe *pipe,
              uint16_t cam_id, const t265_undist_s *undist,
              const t265_frame *frame, const genom_context self)
{
    or_sensor_frame* rfdata = frame->data("raw", self);
    or_sensor_frame* cfdata = frame->data("compressed", self);

    video_frame rsframe = pipe->data.get_fisheye_frame(cam_id);
    const uint16_t w = rsframe.get_width();
    const uint16_t h = rsframe.get_height();
    const double ms = rsframe.get_timestamp();

    Mat cvframe = Mat(
        Size(w, h),
        CV_8UC1,
        (void*) rsframe.get_data(),
        Mat::AUTO_STEP
    );

    remap(cvframe, cvframe, undist->m1, undist->m2, INTER_LINEAR);

    const uint16_t s = cvframe.size().height;

    if (s*s != rfdata->pixels._maximum)
    {

        if (genom_sequence_reserve(&(rfdata->pixels), s*s)  == -1) {
            t265_e_mem_detail d;
            snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
            warnx("%s", d.what);
            return t265_e_mem(&d,self);
        }
        rfdata->pixels._length = s*s;
        rfdata->height = s;
        rfdata->width = s;
        rfdata->bpp = 1;
        rfdata->compressed = false;
    }

    memcpy(rfdata->pixels._buffer, cvframe.data, rfdata->pixels._length);
    rfdata->ts.sec = floor(ms/1000);
    rfdata->ts.nsec = (ms - (double)rfdata->ts.sec*1000) * 1e6;

    if (compression_rate != -1)
    {
        std::vector<int32_t> compression_params;
        compression_params.push_back(IMWRITE_JPEG_QUALITY);
        compression_params.push_back(compression_rate);

        std::vector<uint8_t> buf;
        imencode(".jpg", cvframe, buf, compression_params);

        if (buf.size() > cfdata->pixels._maximum)
            if (genom_sequence_reserve(&(cfdata->pixels), buf.size())  == -1) {
                t265_e_mem_detail d;
                snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
                warnx("%s", d.what);
                return t265_e_mem(&d,self);
            }
        cfdata->pixels._length = buf.size();
        cfdata->height = rfdata->height;
        cfdata->width = rfdata->width;
        cfdata->bpp = 1;
        cfdata->compressed = true;

        memcpy(cfdata->pixels._buffer, buf.data(), buf.size());
        cfdata->ts = rfdata->ts;
    }

    frame->write("raw", self);
    frame->write("compressed", self);

    return t265_poll;
}


/* --- Activity connect ------------------------------------------------- */

/** Codel t265_connect of activity connect.
 *
 * Triggered by t265_start.
 * Yields to t265_ether.
 * Throws t265_e_rs, t265_e_io.
 */
genom_event
t265_connect(const char serial[32], uint16_t id, uint16_t size,
             float fov, uint16_t *cam_id, or_camera_pipe **pipe,
             bool *started, const t265_intrinsics *intrinsics,
             t265_undist_s **undist, const genom_context self)
{
    if (*started)
    {
        t265_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "already connected to device, disconnect() first");
        warnx("%s", d.what);
        return t265_e_io(&d,self);
    }
    else if (id != 1 && id != 2)
    {
        t265_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "invalid camera id: 1 for left, 2 for right");
        warnx("%s", d.what);
        return t265_e_io(&d,self);
    }
    else
    {
        *cam_id = id;
        rs2_intrinsics intrinsics_rs2;

        config cfg;
        if (!strcmp(serial,"\0") || !strcmp(serial,"0"))
            cfg.enable_device("");
        else
            cfg.enable_device(serial);

        cfg.enable_stream(RS2_STREAM_FISHEYE, 1, 848, 800, RS2_FORMAT_Y8, 30);
        cfg.enable_stream(RS2_STREAM_FISHEYE, 2, 848, 800, RS2_FORMAT_Y8, 30);

        try {
            // Start streaming
            pipeline_profile pipe_profile = (*pipe)->pipe.start(cfg);
            video_stream_profile stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, id).as<video_stream_profile>();
            intrinsics_rs2 = stream.get_intrinsics();
        } catch (rs2::error& e) {
            t265_e_rs_detail d;
            snprintf(d.what, sizeof(d.what), "%s", e.what());
            warnx("rs error: %s", d.what);
            return t265_e_rs(&d,self);
        }

        // Init local calib structure
        Mat K = Mat::zeros(3, 3, CV_32F);
        K.at<float>(0,0) = intrinsics_rs2.fx;
        K.at<float>(1,1) = intrinsics_rs2.fy;
        K.at<float>(0,2) = intrinsics_rs2.ppx;
        K.at<float>(1,2) = intrinsics_rs2.ppy;
        K.at<float>(0,1) = 0;
        K.at<float>(2,2) = 1;
        Mat D = (Mat_<float>(4,1) <<
            intrinsics_rs2.coeffs[0],
            intrinsics_rs2.coeffs[1],
            intrinsics_rs2.coeffs[2],
            intrinsics_rs2.coeffs[3]
        );

        // Compute desired calibration matrix
        float f_px = size/2 /tan(fov/2);
        float c = size/2;
        Mat P = (Mat_<float>(3,3) <<
            f_px,    0, c,
               0, f_px, c,
               0,    0, 1
        );

        // Compute undistortion maps and store in ids
        fisheye::initUndistortRectifyMap(K, D, Mat::eye(3,3, CV_32F), P, Size(size,size), CV_16SC2, (*undist)->m1, (*undist)->m2);

        // Publish intrinsincs with 'fake distortion' (=0) since the image is undistorted before publishing
        or_sensor_intrinsics* intr_data = intrinsics->data(self);
        *intr_data = {
            .calib = {
                f_px,
                f_px,
                c,
                c,
                0,
            },
            .disto = {0, 0, 0, 0, 0},
        };
        intrinsics->write(self);

        // Init boolean
        *started = true;
    }

    warnx("connected to T265 device (fisheye camera %i)", id);
    return t265_ether;
}


/* --- Activity disconnect ---------------------------------------------- */

/** Codel t265_disconnect of activity disconnect.
 *
 * Triggered by t265_start.
 * Yields to t265_ether.
 * Throws t265_e_rs.
 */
genom_event
t265_disconnect(or_camera_pipe **pipe, bool *started,
                const genom_context self)
{
    try {
        (*pipe)->pipe.stop();
    } catch (rs2::error& e) {
        t265_e_rs_detail d;
        snprintf(d.what, sizeof(d.what), "%s", e.what());
        warnx("rs error: %s", d.what);
        return t265_e_rs(&d,self);
    }
    *started = false;

    warnx("disconnected from device");
    return t265_ether;
}


/* --- Activity set_extrinsics ------------------------------------------ */

/** Codel t265_set_extrinsics of activity set_extrinsics.
 *
 * Triggered by t265_start.
 * Yields to t265_ether.
 */
genom_event
t265_set_extrinsics(const sequence6_float *ext_values,
                    const t265_extrinsics *extrinsics,
                    const genom_context self)
{
    *extrinsics->data(self) = {
        ext_values->_buffer[0],
        ext_values->_buffer[1],
        ext_values->_buffer[2],
        ext_values->_buffer[3],
        ext_values->_buffer[4],
        ext_values->_buffer[5],
    };
    extrinsics->write(self);

    warnx("new extrinsic calibration");
    return t265_ether;
}
