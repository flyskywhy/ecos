#===============================================================================
#
#    makefile
#
#    compat/uitron/tests
#
#===============================================================================
#####COPYRIGHTBEGIN####
#                                                                          
# -------------------------------------------                              
# The contents of this file are subject to the Red Hat eCos Public License 
# Version 1.1 (the "License"); you may not use this file except in         
# compliance with the License.  You may obtain a copy of the License at    
# http://www.redhat.com/                                                   
#                                                                          
# Software distributed under the License is distributed on an "AS IS"      
# basis, WITHOUT WARRANTY OF ANY KIND, either express or implied.  See the 
# License for the specific language governing rights and limitations under 
# the License.                                                             
#                                                                          
# The Original Code is eCos - Embedded Configurable Operating System,      
# released September 30, 1998.                                             
#                                                                          
# The Initial Developer of the Original Code is Red Hat.                   
# Portions created by Red Hat are                                          
# Copyright (C) 1998, 1999, 2000 Red Hat, Inc.                             
# All Rights Reserved.                                                     
# -------------------------------------------                              
#                                                                          
#####COPYRIGHTEND####
#===============================================================================

PACKAGE       := uitron
include ../../../../pkgconf/pkgconf.mak

TESTS	:= test1   test2   test3   test4   test5   \
           test6   test7   test8   test9           \
           testcxx testcx2 testcx3 testcx4 testcx5 \
           testcx6 testcx7 testcx8 testcx9         \
	   testintr


include $(COMPONENT_REPOSITORY)/pkgconf/makrules.tst




