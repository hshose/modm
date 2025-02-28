/*
 * Copyright (c) 2020, Erik Henriksson
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <unittest/testsuite.hpp>

/// @ingroup modm_test_test_architecture
class FiberTest : public unittest::TestSuite
{
public:
	void
	setUp();

	void
	testOneFiber();

	void
	testTwoFibers();

	void
	testYieldFromSubroutine();

	void
	testPollFor();

	void
	testPollUntil();

	void
	testSleepFor();

	void
	testSleepUntil();

	void
	testStopToken();
};
