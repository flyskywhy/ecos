#===============================================================================
#
#    makefile
#
#    hal/mn10300/stdeval1/src
#
#===============================================================================
#####COPYRIGHTBEGIN####
#
# -------------------------------------------
# The contents of this file are subject to the Cygnus eCos Public License
# Version 1.0 (the "License"); you may not use this file except in
# compliance with the License.  You may obtain a copy of the License at
# http://sourceware.cygnus.com/ecos
# 
# Software distributed under the License is distributed on an "AS IS"
# basis, WITHOUT WARRANTY OF ANY KIND, either express or implied.  See the
# License for the specific language governing rights and limitations under
# the License.
# 
# The Original Code is eCos - Embedded Cygnus Operating System, released
# September 30, 1998.
# 
# The Initial Developer of the Original Code is Cygnus.  Portions created
# by Cygnus are Copyright (C) 1998,1999 Cygnus Solutions.  All Rights Reserved.
# -------------------------------------------
#
#####COPYRIGHTEND####
#===============================================================================


PACKAGE       := hal_mn10300_stdeval1
include ../../../../../pkgconf/pkgconf.mak

LIBRARY       := libtarget.a
COMPILE       := hal_diag.c stdeval1.S plf_stub.c
OTHER_OBJS    :=
OTHER_TARGETS :=
OTHER_CLEAN   :=
OTHER_DEPS    :=

include $(COMPONENT_REPOSITORY)/pkgconf/makrules.src


