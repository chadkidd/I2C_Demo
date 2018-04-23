AC_PREREQ(2.61)

# package version number (not shared library version)
# odd micro numbers indicate in-progress development
# even micro numbers indicate released versions
m4_define(I2C_Demo_version_major, 0)
m4_define(I2C_Demo_version_minor, 1)
m4_define(I2C_Demo_version_micro, 0)

m4_define([I2C_Demo_version],
          [I2C_Demo_version_major.I2C_Demo_version_minor.I2C_Demo_version_micro])
m4_define([I2C_Demo_api_version],
          [I2C_Demo_version_major.I2C_Demo_version_minor])

AC_INIT(I2C_Demo, I2C_Demo_version)
AM_INIT_AUTOMAKE(I2C_Demo, I2C_Demo_version)
AC_CONFIG_HEADERS(config.h)

# Checks for programs.
AC_PROG_LIBTOOL
AM_PROG_CC_C_O
AC_PROG_INSTALL

AC_SUBST(I2C_Demo_CFLAGS)
AC_SUBST(I2C_Demo_LIBS)

AC_OUTPUT([
Makefile
src/Makefile
])

echo ""
echo "   I2C_Demo $VERSION"
echo "   ====================="
echo ""
echo "   To build the project, run \"make\""
echo ""
