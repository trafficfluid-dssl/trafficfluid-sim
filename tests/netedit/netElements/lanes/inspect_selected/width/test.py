#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2020 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    test.py
# @author  Pablo Alvarez Lopez
# @date    203-11-25

# import common functions for netedit tests
import os
import sys

testRoot = os.path.join(os.environ.get('SUMO_HOME', '.'), 'tests')
neteditTestRoot = os.path.join(
    os.environ.get('TEXTTEST_HOME', testRoot), 'netedit')
sys.path.append(neteditTestRoot)
import neteditTestFunctions as netedit  # noqa

# Open netedit
neteditProcess, referencePosition = netedit.setupAndStart(neteditTestRoot)

# recompute
netedit.rebuildNetwork()

# toogle select lanes
netedit.changeEditMode('4')

# go to select mode
netedit.selectMode()

# select first lane
netedit.leftClick(referencePosition, 250, 165)

# select second lane
netedit.leftClick(referencePosition, 250, 105)

# go to inspect mode
netedit.inspectMode()

# inspect lane
netedit.leftClick(referencePosition, 250, 105)

# Change parameter 3 with a non valid value (dummy)
netedit.modifyAttribute(5, "dummyWidth", True)

# Change parameter 3 with a non valid value (empty)
netedit.modifyAttribute(5, "", True)

# Change parameter 3 with a non valid value (negative)
netedit.modifyAttribute(5, "-2", True)

# Change parameter 3 with a valid value (default)
netedit.modifyAttribute(5, "default", True)

# Change parameter 3 with a valid value (default)
netedit.modifyAttribute(5, "4", True)

# recompute
netedit.rebuildNetwork()

# Check undos
netedit.undo(referencePosition, 1)

# recompute
netedit.rebuildNetwork()

# check redos
netedit.redo(referencePosition, 1)

# save additionals
netedit.saveAdditionals(referencePosition)

# save network
netedit.saveNetwork(referencePosition)

# quit netedit
netedit.quit(neteditProcess)
