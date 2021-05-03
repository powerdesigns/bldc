/*
	Copyright 2020 Martin Paz mpaz@paltatech.com

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef BUILT_IN_TEST_H_
#define BUILT_IN_TEST_H_

//
typedef enum {
	BIT_IDLE = 0,
	BIT_RUN,
	BIT_STOP
} BIT_state_t;

// Functions
void built_in_test_init(void);
int BIT_pulse_test(int total_pulse, float pulse_dwell_us, float pulse_off_us);
FlagStatus BIT_get_state(void);

#endif /* BUILT_IN_TEST_H_ */
