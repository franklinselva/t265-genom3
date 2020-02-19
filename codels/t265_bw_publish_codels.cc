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
#include "opencv2/opencv.hpp"

using namespace cv;

/* --- Global variables ------------------------------------------------- */
bool display = false;
bool record = false;
VideoWriter recorder;

/* --- Task bw_publish -------------------------------------------------- */


/** Codel t265_bw_start of task bw_publish.
 *
 * Triggered by t265_start.
 * Yields to t265_pause_start, t265_pub.
 * Throws t265_e_hw.
 */
genom_event
t265_bw_start(bool started, const genom_context self)
{
    // Loop in start until data is initialized, then yield to poll
    if (started)
        return t265_pub;
    else
        return t265_pause_start;
}


/** Codel t265_bw_pub of task bw_publish.
 *
 * Triggered by t265_pub.
 * Yields to t265_visu.
 * Throws t265_e_hw.
 */
genom_event
t265_bw_pub(const t265_RSdata_s *data, t265_frame_s *bw,
            const t265_bw_out *bw_out, const genom_context self)
{
    rs2::video_frame video_data = data->bw;
    const uint16_t w = video_data.get_width();
    const uint16_t h = video_data.get_height();

    if (w*h*bw->bpp == bw->pixels._length)
    {
        bw->pixels._buffer = (uint8_t*) video_data.get_data();

        // Create timestamp
        uint32_t ms = data->bw.get_timestamp();
        uint32_t s = floor(ms/1000);
        uint64_t ns = (ms - s*1000) * 1e6;
        bw->ts.sec = s;
        bw->ts.nsec = ns;
    }
    else
    {
        t265_e_hw_detail d;
        snprintf(d.what, sizeof(d.what), "frame size incorrect");
        printf("t265: %s\n", d.what);
        return t265_e_hw(&d,self);
    }
    // Update port data
    *(bw_out->data(self)) = *bw;
    bw_out->write(self);

    return t265_visu;
}


/** Codel t265_bw_visu of task bw_publish.
 *
 * Triggered by t265_visu.
 * Yields to t265_pause_pub.
 * Throws t265_e_hw.
 */
genom_event
t265_bw_visu(const t265_frame_s *bw, const genom_context self)
{
    Mat cvBW(Size(bw->width, bw->height), CV_8UC1, (void*)bw->pixels._buffer, Mat::AUTO_STEP);

    if (display)
    {
        imshow("T265 BW", cvBW);
        waitKey(1);
    }
    if (record)
    {
        Mat cvRec;
        cvtColor(cvBW, cvRec, COLOR_GRAY2BGR);
        recorder.write(cvRec);
    }
    return t265_pause_pub;
}


/* --- Activity display ------------------------------------------------- */

/** Codel t265_disp_start of activity display.
 *
 * Triggered by t265_start.
 * Yields to t265_ether.
 * Throws t265_e_hw.
 */
genom_event
t265_disp_start(const genom_context self)
{
    display = true;
    return t265_ether;
}


/* --- Activity stop_display -------------------------------------------- */

/** Codel t265_disp_stop of activity stop_display.
 *
 * Triggered by t265_start.
 * Yields to t265_ether.
 * Throws t265_e_hw.
 */
genom_event
t265_disp_stop(const genom_context self)
{
    display = false;
    destroyAllWindows();
    return t265_ether;
}


/* --- Activity record -------------------------------------------------- */

/** Codel t265_rec_start of activity record.
 *
 * Triggered by t265_start.
 * Yields to t265_ether.
 * Throws t265_e_hw.
 */
genom_event
t265_rec_start(const char path[64], const t265_frame_s *bw,
               const genom_context self)
{
    char path_bw[64];
    snprintf(path_bw, sizeof(path_bw), "%s/bw.avi", path);
    recorder = VideoWriter(path_bw,CV_FOURCC('M','J','P','G'), 30, Size(bw->width,bw->height));
    record = true;
    return t265_ether;
}


/* --- Activity stop_record --------------------------------------------- */

/** Codel t265_rec_stop of activity stop_record.
 *
 * Triggered by t265_start.
 * Yields to t265_ether.
 * Throws t265_e_hw.
 */
genom_event
t265_rec_stop(const genom_context self)
{
    recorder.release();
    record = false;
    return t265_ether;
}
