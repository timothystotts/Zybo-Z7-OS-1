#
# This file is the uio-gpio-test recipe.
#

SUMMARY = "Simple uio-gpio-test application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = " \
		file://uio-gpio-test.c \
		file://uio-gpio-pwm-test.c \
		file://Makefile \
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

