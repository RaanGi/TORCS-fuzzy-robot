##############################################################################
#
#    file                 : Makefile
#    created              : dom jul 26 13:46:50 CEST 2020
#    copyright            : (C) 2002 Marcos Arroyo Ruiz
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = RanGi
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp driver.cpp fuzzycontrol.cpp cardata.cpp opponent.cpp spline.cpp pit.cpp strategy.cpp learn.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml logo.rgb
SHIPSUBDIRS = 0 1 2 3 4 5 6 7 8 9

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-RanGi_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-RanGi_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
