#
# Copyright (c) 2019 LAAS/CNRS
# All rights reserved.
#
# Redistribution  and  use  in  source  and binary  forms,  with  or  without
# modification, are permitted provided that the following conditions are met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
# THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
# WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
# MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
# ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
# WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
# IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#                                              Martin Jacquet - February 2020
#
#
lib_LTLIBRARIES = libt265_codels.la

libt265_codels_la_SOURCES  =	t265_c_types.h
libt265_codels_la_SOURCES +=	t265_codels.cc
libt265_codels_la_SOURCES +=	t265_main_codels.cc

libt265_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libt265_codels_la_LIBADD   =	$(requires_LIBS)
libt265_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libt265_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libt265_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	t265_c_types.h
CLEANFILES=	${BUILT_SOURCES}
t265_c_types.h: ${top_srcdir}/t265.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/t265_c_types.h.d
