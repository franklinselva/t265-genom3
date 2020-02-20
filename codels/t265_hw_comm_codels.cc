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
 *
 */
#include "act265.h"

#include "t265_c_types.h"

#include "codels.hpp"

/* --- Task hw_comm ----------------------------------------------------- */


/** Codel t265_comm_start of task hw_comm.
 *
 * Triggered by t265_start.
 * Yields to t265_poll.
 * Throws t265_e_hw.
 */
genom_event
t265_comm_start(t265_ids *ids, const t265_extrinsics *extrinsics,
                const genom_context self)
{
    *extrinsics->data(self) = {0,0,0, 0,0,0};
    extrinsics->write(self);
    ids->pipe = new t265_pipe_s();
    ids->started = false;

    return t265_poll;
}


/** Codel t265_comm_poll of task hw_comm.
 *
 * Triggered by t265_poll.
 * Yields to t265_pause_poll, t265_read.
 * Throws t265_e_hw.
 */
genom_event
t265_comm_poll(t265_pipe_s **pipe, const genom_context self)
{
    // Loop in poll until pipe is initialized (through connect activity)
    if (!(*pipe)->init)
        return t265_pause_poll;

    // Wait for next set of frames from the camera
    (*pipe)->data = (*pipe)->pipe.wait_for_frames();

    return t265_read;
}


/** Codel t265_comm_read of task hw_comm.
 *
 * Triggered by t265_read.
 * Yields to t265_poll.
 * Throws t265_e_hw.
 */
genom_event
t265_comm_read(const t265_pipe_s *pipe, t265_RSdata_s **data,
               bool *started, const genom_context self)
{
    // Read Image
    (*data)->bw = pipe->data.get_fisheye_frame();
    (*started) = true;
    return t265_poll;
}


/* --- Activity connect ------------------------------------------------- */

/** Codel t265_connect of activity connect.
 *
 * Triggered by t265_start.
 * Yields to t265_ether.
 * Throws t265_e_hw.
 */
genom_event
t265_connect(t265_ids *ids, const t265_intrinsics *intrinsics,
             const genom_context self)
{
    std::cout << "t265: initializing connection to hardware... ";

    // Test if device already connected
    if (ids->pipe->init) {
        std::cout << "error" << std::endl;
        t265_e_hw_detail d;
        snprintf(d.what, sizeof(d.what), "device already connected");
        printf("t265: %s\n", d.what);
        return t265_e_hw(&d,self);
    }

rs2::context ctx;

    // Start streaming
    rs2::pipeline_profile pipe_profile = ids->pipe->pipe.start();

    // Set configuration as written in the .json calibration file
    rs2::device dev = pipe_profile.get_device();

    // Get intrinsics
    rs2::video_stream_profile stream = pipe_profile.get_stream(RS2_STREAM_FISHEYE).as<rs2::video_stream_profile>();
    const uint h = stream.height();
    const uint w = stream.width();

    rs2_intrinsics intrinsics_rs2 = stream.get_intrinsics();
    float k[5] = { intrinsics_rs2.fx,
                   intrinsics_rs2.fy,
                   intrinsics_rs2.ppx,
                   intrinsics_rs2.ppy,
                   0 };
    float* d = intrinsics_rs2.coeffs;

    // Initialize intrinsics and publish
    intrinsics->data(self)->calib._length = 5;
    intrinsics->data(self)->disto._length = 5;
    for (int i=0; i<5; i++)
    {
        intrinsics->data(self)->calib._buffer[i] = k[i];
        intrinsics->data(self)->disto._buffer[i] = d[i];
    }
    intrinsics->write(self);

    // Initialize sequence for frame
    ids->bw.height = h;
    ids->bw.width = w;
    ids->bw.bpp = 1;
    if (genom_sequence_reserve(&(ids->bw.pixels), h*w)  == -1) {
        t265_e_mem_detail d;
        snprintf(d.what, sizeof(d.what), "unable to allocate bw frame memory");
        printf("t265: %s\n", d.what);
        return t265_e_mem(&d,self);
    }
    ids->bw.pixels._length = h*w;

    // Init boolean
    ids->pipe->init = true;
    ids->data = new t265_RSdata_s();

    std::cout << "done" << std::endl;

    return t265_ether;
}


/* --- Activity set_extrinsics ------------------------------------------ */

/** Codel t265_set_extrinsics of activity set_extrinsics.
 *
 * Triggered by t265_start.
 * Yields to t265_ether.
 * Throws t265_e_hw.
 */
genom_event
t265_set_extrinsics(const sequence6_double *ext_values,
                    const t265_extrinsics *extrinsics,
                    const genom_context self)
{
    std::cout << "t265: new extrinsics calibration: ";
    t265_extrinsics_s ext;
    ext = {ext_values->_buffer[0],
           ext_values->_buffer[1],
           ext_values->_buffer[2],
           ext_values->_buffer[3],
           ext_values->_buffer[4],
           ext_values->_buffer[5]};

    *extrinsics->data(self) = ext;
    extrinsics->write(self);
    std::cout << extrinsics->data(self)->tx << " " <<
                 extrinsics->data(self)->ty << " " <<
                 extrinsics->data(self)->tz << " " <<
                 extrinsics->data(self)->roll << " " <<
                 extrinsics->data(self)->pitch << " " <<
                 extrinsics->data(self)->yaw << std::endl;

    return t265_ether;
}
