# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


def interpolate_function(value, start_x, start_y, end_x, end_y, ascending=None):
    if end_x - start_x == 0:
        if (ascending and value < start_x) or (not ascending and value > start_x):
            return start_y
        elif (ascending and value > start_x) or (not ascending and value < start_x):
            return end_y
        else:
            return start_y + (end_y - start_y) / 2
    slope = (end_y - start_y) / (end_x - start_x)
    return slope * (value - start_x) + start_y


def interpolate_lookup_table(value, table):
    if not table:
        return value

    # Interpolate
    for i in range(int(len(table) / 3) - 1):
        if (value < table[i * 3 + 1] and value >= table[(i + 1) * 3 + 1]) or \
                (value >= table[i * 3 + 1] and value < table[(i + 1) * 3 + 1]):
            return interpolate_function(
                value,
                table[i * 3 + 1],
                table[i * 3],
                table[(i + 1) * 3 + 1],
                table[(i + 1) * 3]
            )

    # Extrapolate (we assume that the table is sorted, order is irrelevant)
    ascending = (table[1] < table[len(table) - 1*3 + 1])
    if (ascending and value >= table[1]) or (not ascending and value < table[1]):
        return interpolate_function(
            value,
            table[len(table) - 2 * 3 + 1],
            table[len(table) - 2 * 3 + 0],
            table[len(table) - 1 * 3 + 1],
            table[len(table) - 1 * 3 + 0],
            ascending
        )
    else:
        return interpolate_function(
            value,
            table[1],
            table[0],
            table[3 + 1],
            table[3],
            ascending
        )
