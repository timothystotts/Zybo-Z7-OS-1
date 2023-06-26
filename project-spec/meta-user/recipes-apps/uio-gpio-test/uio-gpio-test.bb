#
# This file is the uio-gpio-test recipe.
#

SUMMARY = "Simple uio-gpio-test application"
SECTION = "PETALINUX/apps"
LICENSE = "GPL-2.0"
LIC_FILES_CHKSUM = "file://COPYING;md5=393a5ca445f6965873eca0259a17f833"

SRC_URI = " \
		file://uio-gpio-test.c \
		file://uio-gpio-pwm-test.c \
		file://Makefile \
		file://COPYING \
		  "

S = "${WORKDIR}"

RDEPENDS:${PN} = "libuio-tools"
DEPENDS = "libuio"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 uio-gpio-test ${D}${bindir}
	     install -m 0755 uio-gpio-pwm-test ${D}${bindir}
}
