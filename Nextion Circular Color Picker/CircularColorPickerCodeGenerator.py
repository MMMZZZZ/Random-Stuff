# Code Generator for "Circular Color Picker for Nextion"
# (c) Max Zuidberg 2022
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.


import argparse
from pathlib import Path
from math import tan, pi


# Note: because of the various { } in the code I did not use f-string
# which otherwise would have been my preferred choice for this task.
template = """
// Circular Color Picker for Nextion (c) Max Zuidberg 2022
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
// 
// Put this code in the touch press or release
// event of the pic component with the color wheel.
// Requires the two variables field.val and ring.val
// 
// sya0 = x, sya1 = sya1
// Note the usage of the hidden sya0, sya1 variables 
// within event code as local, temporary variable is fine. 
sya0=tch0
sya1=tch1
// 
// Convert absolute coordinates to coordinates relative to
// the color wheel center.
// sys0 = x_center, sys1 = y_center
sys0=WHEEL_PIC.w/2
sys0+=WHEEL_PIC.x
sys1=WHEEL_PIC.h/2
sys1+=WHEEL_PIC.y
sya0-=sys0
sya1-=sys1
// 
// Determine ring
ring.val=0
// sys0 = r^2 = x^2 + y^2
sys0=sya0*sya0
sys1=sya1*sya1
sys0+=sys1
// repeat for all rings
RING_SELECTING_CODE
// 
// Determine quadrant (0-3). Note: pixel y coords are inverted
// compared to mathematical y coords. But we want math. quadrants.
sya1*=-1
sys2=0
if(sya1<0)
{
  sys2+=2
}
sys0=sya0*sya1
if(sys0<0)
{
  sys2+=1
  // In this case we also want to swap x and y otherwise the 
  // atan(abs(x/y)) (calculated below) gives values running
  // "the wrong way" (cw instead of ccw).
  sys1=sya1
  sya1=sya0
  sya0=sys1
}
// 
field.val=sys2*FIELDS_PER_RING_QUADRANT
// 
// x,y sign is not required anymore
if(sya0<0)
{
  sya0*=-1
}
if(sya1<0)
{
  sya1*=-1
}
// 
// Determine field in ring quadrant
// Factor 100000 chosen more or less arbitrarily. 
// sys0 = 100000 * tan_a = 100000 * y / x
sys0=100000*sya1
sys0/=sya0
// repeat for all fields
FIELD_SELECTING_CODE
"""

ring_template = """
if(sys0>=NTH_RING_LIMIT)
{
  ring.val++
}
"""

field_template = """
if(sys0>=NTH_FIELD_LIMIT)
{
  field.val++
}
"""


def codegen(pic_name, pic_size, rings, fields_per_ring):
    if fields_per_ring % 4:
        raise ValueError("fields_per_ring must be a multiple of 4.")
    fields_per_ring_quadrant = fields_per_ring // 4

    ring_selecting_code = ""
    ring_size = pic_size / rings / 2
    for r in range(1, rings):
        r = int((r * ring_size) ** 2)
        ring_selecting_code += ring_template.replace("NTH_RING_LIMIT", str(r))

    field_selecting_code = ""
    field_size = 2 * pi / fields_per_ring
    for f in range(1, fields_per_ring_quadrant):
        f = tan(f * field_size)
        f = int(f * 100000)
        field_selecting_code += field_template.replace("NTH_FIELD_LIMIT", str(f))

    to_replace = {
        "WHEEL_PIC": str(pic_name),
        "FIELDS_PER_RING_QUADRANT": str(fields_per_ring_quadrant),
        "RING_SELECTING_CODE": str(ring_selecting_code),
        "FIELD_SELECTING_CODE": str(field_selecting_code),
    }

    code = template
    for k, v in to_replace.items():
        code = code.replace(k, v)
    return code


if __name__ == "__main__":
    desc = """Code Generator for "Circular Color Picker for Nextion"
              (c) Max Zuidberg 2022
              This Source Code Form is subject to the terms of the Mozilla Public
              License, v. 2.0. If a copy of the MPL was not distributed with this
              file, You can obtain one at http://mozilla.org/MPL/2.0/."""
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument("-p", "--pic-name", type=str, required=True,
                        help="Name of the picture component with the color wheel.")
    parser.add_argument("-s", "--pic-size", type=int, required=True,
                        help="Size of the color wheel (diameter)")
    parser.add_argument("-r", "--rings", type=int, required=True,
                        help="Number of rings in the color wheel.")
    parser.add_argument("-f", "--fields", type=int, required=True,
                        help="Number of color fields per ring. Note: atm this must be a "
                             "multiple of 4. Also make sure that a horizontal line would "
                             "line up with the border between two fields (Instead of one "
                             "field being symmetrical around the horizontal axis).")
    parser.add_argument("-o", "--output", type=str,
                        help="Optional output file for the generated code. If omitted, "
                             "the code will be written to the console. ")

    args = parser.parse_args()
    code = codegen(pic_name=args.pic_name, pic_size=args.pic_size, rings=args.rings, fields_per_ring=args.fields)

    if args.output:
        args.output = Path(args.output)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, "w") as f:
            f.write(code)
    else:
        print(code)
