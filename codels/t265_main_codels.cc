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

/* --- Task main -------------------------------------------------------- */


/** Codel t265_main_start of task main.
 *
 * Triggered by t265_start.
 * Yields to t265_poll.
 */
genom_event
t265_main_start(t265_ids *ids, const t265_extrinsics *extrinsics,
                const genom_context self)
{
    extrinsics->write(self);
    ids->pipe = new or_camera_pipe();
    ids->info.started = false;

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

    (*pipe)->data = (*pipe)->pipe.wait_for_frames();

    return t265_pub;
}


/** Codel t265_main_pub of task main.
 *
 * Triggered by t265_pub.
 * Yields to t265_poll.
 */
genom_event
t265_main_pub(const or_camera_pipe *pipe, uint16_t cam_id,
              const t265_frame *frame, const genom_context self)
{
    or_sensor_frame* fdata = frame->data(self);

    video_frame rsframe = pipe->data.get_fisheye_frame(cam_id);
    const uint16_t w = rsframe.get_width();
    const uint16_t h = rsframe.get_height();

    if (h*w > fdata->pixels._length)
    {
        if (genom_sequence_reserve(&(fdata->pixels), h*w)  == -1) {
            t265_e_mem_detail d;
            snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
            printf("t265: %s\n", d.what);
            return t265_e_mem(&d,self);
        }
        fdata->pixels._length = h*w;
        fdata->height = h;
        fdata->width = w;
        fdata->bpp = 1;
    }

    fdata->pixels._buffer = (uint8_t*) rsframe.get_data();

    double ms = rsframe.get_timestamp();
    fdata->ts.sec = floor(ms/1000);
    fdata->ts.nsec = (ms - fdata->ts.sec*1000) * 1e6;

    frame->write(self);

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
t265_connect(uint16_t id, uint16_t *cam_id, or_camera_pipe **pipe,
             bool *started, const t265_intrinsics *intrinsics,
             const genom_context self)
{
    if (*started)
    {
        t265_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "already connected to gazebo, disconnect() first");
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

        try {
            // Start streaming
            pipeline_profile pipe_profile = (*pipe)->pipe.start();
            video_stream_profile stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE, id).as<video_stream_profile>();
            intrinsics_rs2 = stream.get_intrinsics();
        } catch (rs2::error& e) {
            t265_e_rs_detail d;
            snprintf(d.what, sizeof(d.what), "%s", e.what());
            warnx("rs error: %s", d.what);
            return t265_e_rs(&d,self);
        }

        or_sensor_intrinsics* intr_data = intrinsics->data(self);
        *intr_data = {
            .calib = {
                intrinsics_rs2.fx,
                intrinsics_rs2.fy,
                intrinsics_rs2.ppx,
                intrinsics_rs2.ppy,
                0,
            },
            .disto = {
                intrinsics_rs2.coeffs[0],
                intrinsics_rs2.coeffs[1],
                intrinsics_rs2.coeffs[2],
                intrinsics_rs2.coeffs[3],
                intrinsics_rs2.coeffs[4],
            }
        };
        intrinsics->write(self);

        // Init boolean
        *started = true;
    }
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
