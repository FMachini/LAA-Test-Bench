##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
##	Christian Kirches et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  examples/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   3.2
##	Date:      2007-2017
##

include ../make.mk

##
##	flags
##
#ARMADILHO FLAGS
ARMA_INCLUDE_FLAG = -I ../include
OPT = -O2
EXTRA_OPT = -fwhole-program
LIB_FLAGS_ARMA = -lblas -llapack -larmadillo
CPPFLAGS_ARMA = $(ARMA_INCLUDE_FLAG) $(OPT) $(EXTRA_OPT) 


IFLAGS      =  -I. \
               -I${IDIR}


QPOASES_EXES = ${BINDIR}/teste${EXE}


QPOASES_INCLUDE = \
	-I. ${IDIR}/qpOASES.cpp \
	-I. ${IDIR}/qpOASES/LapackBlasReplacement.cpp \
	-I. ${IDIR}/qpOASES/SQProblem.cpp \
	-I. ${IDIR}/qpOASES/QProblem.cpp \
	-I. ${IDIR}/qpOASES/Flipper.cpp \
	-I. ${IDIR}/qpOASES/QProblemB.cpp \
	-I. ${IDIR}/qpOASES/Bounds.hpp \
	-I. ${IDIR}/qpOASES/Constraints.cpp \
	-I. ${IDIR}/qpOASES/SubjectTo.cpp \
	-I. ${IDIR}/qpOASES/Indexlist.cpp \
	-I. ${IDIR}/qpOASES/Utils.cpp \
	-I. ${IDIR}/qpOASES/Constants.cpp \
	-I. ${IDIR}/qpOASES/Types.cpp \
	-I. ${IDIR}/qpOASES/Options.cpp \
	-I. ${IDIR}/qpOASES/Matrices.cpp \
	-I. ${IDIR}/qpOASES/MessageHandling.cpp \
	-I. ${IDIR}/qpOASES/UnitTesting.cpp
##
##	targets
##

all: ${QPOASES_EXES}

${BINDIR}/teste${EXE}: teste.${OBJEXT} ${LINK_DEPENDS}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} ${CPPFLAGS_ARMA} $<  ${QPOASES_INCLUDE} ${LIB_FLAGS_ARMA} ${QPOASES_LINK} ${LINK_LIBRARIES}

clean:
	@${ECHO} "Cleaning up (examples)"
	@${RM} -f *.${OBJEXT} ${QPOASES_EXES}

clobber: clean


${LINK_DEPENDS}:
	@cd ..; ${MAKE} -s src

%.${OBJEXT}: %.cpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<


##
##	end of file
##
