ROBOT       = RanGi
SHIPDIR     = drivers/${ROBOT}/2
SHIP        = default.xml
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-bt_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-bt_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))
include ${MAKE_DEFAULT}
