# Circular Color Picker for Nextion

... including a code generator.

![Screenshot](Screenshot.png)

## Index

* [Description](#description)
	* [General](#general)
	* [The Code (Generator)](#the-code-generator)
	* [The Nextion Project](#the-nextion-project)
* [Usage](#usage)
	* [Code Generator](#code-generator)
	* [Color Wheel](#color-wheel)
* [How it works](#how-it-works)
* [Final Thoughts](#final-thoughts)
* [License](#license)

## Description

### General

This project implements a circular color picker on Nextion. Usually you'd need trigonometric functions for this (which Nextion doesn't allow you to use unless you implement them yourself). This project however circumvents them. The "cost" is that you cannot have a continuous color wheel but only one with discrete color fields. Still pretty cool. 

### The Code (Generator)

The code generator works as command line tool. You provide it the properties of your color wheel (size, number of colors, ... - see [Usage](#usage) below) and it generates the corresponding Nextion code. 

The generated code only converts touch presses to polar coordinates or in simple terms: the color `ring` and the color `field` within that ring. It is up to you how you convert these values into an actual color. I can think of the following options:

* Use my hsv2rgb code from my rectangular HSV color picker. In most cases color wheels give you h (hue) and s (saturation) while fixing v (value) at maximum. See the [demo project](#the-nextion-project).
* Have a long (maybe auto-generated) if-else chain that determines the color
* Use an array. Nextion doesn't allow users to define their own arrays so you need to be creative.
	* One option is to use a string (yes this is no joke). It is of course a pretty slow solution but at least it doesn't consume any user RAM. In pseudo code:
		```
		s = "light_red,light_green,light_blue;red,green,blue;dark_red,dark_green,dark_blue"
		// Split by ring and select the right one
		s = s.split(";")[ring.val]
		// Split by color within the ring and select final color field
		s = s.split(",")[field.val]
		color = int(s)
		```
	* The other option is to save the values in variables on the page. These can then be accessed using the page's component array `b[id]`. For let's say a color wheel with 4 rings and 12 colors each you'd have to create 48 variables (tip: use copy paste to get 1, 2, 4, 8, ... variables quickly). Assuming the first one is named `va0` the code would look somewhat like the following. The downside is obviously the RAM consumption. 
		```
		// calculate array index. Assuming the color fields are 
		// numerated ring by ring. Colors per ring: 12
		sys0=ring.val*12
		sys0+=field.val
		// add offset since the beginning of the array is not at
		// id = 0 (0 is always the id of the page itself)
		sys0+=va0.id
		color.val=b[sys0].val
		```

### The Nextion Project

The Nextion HMI file of this project contains a fully working demo with a color wheel having 8 rings and 24 fields per ring (definitely nothing you'd want to code by hand). The touch press code of the color wheel picture component contains the auto-generated code, determining the `ring` and `field` values. The touch release event adjusts those values such that they can be processed by the hsv2rgb code. That code has been directly copied from the [Nextion HSV Demo](/Nextion%20HSV%20Test/) project.

## Usage

### Code Generator

The [code generator](CircularColorPickerCodeGenerator.py) has been written in python 3.9. No additional modules required. You can run the script from the command line using 
```python CircularColorPickerCodeGenerator.py --help```
The integrated help should explain all required arguments. Don't worry, only a few properties of your desired color wheel are required.

For those who don't want to run a python script I included a [compiled version for windows](CircularColorPickerCodeGenerator.exe). Works the same way:
```CircularColorPickerCodeGenerator.exe --help```

You can choose if you want the code printed to the console or saved as (text) file. 

### Color Wheel

The code uses some tricks to determine the color field. One of them requires that the number of colors per ring is a multiple of 4. It's also necessary that the horizontal axis lines up with one of the borders between color fields. 

Bad (color field is _on_ the horizontal axis):

![Color Wheel unaligned](Color%20Wheel%20unaligned.png)

Good (horizontal axis is between the color fields):

![Color Wheel aligned](Color%20Wheel%20aligned.png)

## How it works

### The Problem
The big issue to solve is how to get the polar coordinates, meaning, how to get the radius and the angle of the touch position relative to the center of the color wheel (or in other words, the ring and the field within the ring). Usually you'd write something like this: 

```
x -= x_center
y -= y_center
radius = sqrt(x*x + y*y)
angle  = atan(y / x)
```

However, Nextion doesn't allow you to use common math functions unless you write them yourself. While this is possible it's a pain in the ass and likely results in very, very slow code. 

### The Solution 

Having discrete color fields instead of a continuous spectrum allows for one important simplification: we don't need to know the exact value of `radius` and `angle`. Only if they're between certain limits. Assuming each ring has a thickness of 10 pixels. Then the second ring would go from `radius = 10` to `radius = 20`, meaning `radius` squared goes from `100` to `400` respectively. And that in turns means that we don't need the square root anymore:

```
r_squared = x*x + y*y
if (r_squared < 100)
{
	ring = 0
}
else if (r_squared < 400)
{
	ring = 1
}
...
```

So instead of having to use the square root at runtime, we need to square the limits (`100`, `400`, ...) when writing the code. 

The same works for the angle. Here, too, we don't need to actually convert `y / x` to `angle` but can directly check if `y / x` is within a certain range. 

### One more thing...

Acutally one more issue. In fact, the code posted [above](#the-problem) doesn't work because `tan(alpha) == tan(alpha + 180deg)`. Meaning, it would only work on half the wheel. In the other half it would point again to the first half. So we need another trick to figure out in which half we are.

I'm going a bit further by determining the quadrant by checking the sign of the relative `x` and `y` coordinates (in the first quadrant `x` and `y` are positive, in the third both are negative f.ex.). Then, within each quadrant I can use the method described above to determine the actual angle relative to the quadrant. Add both together and voila, 360 degrees covered. This is also the reason why you need a multiple of 4 for the number of field per ring; they must lign up with the limits of the quadrants.

### Automation

While the tricks described here do make things a lot easier, it's still a lot of work to write the code - especially since you have to calculate all the squared radius values and all the tangents of the angles. That's why I wrote the python script. It automates this process and generates the code for you for an arbitrary number of color fields. It's basically just copying a template and adding the numbers, so not much to see there. 

## Final Thoughts

This was a cool project with some out of the box thinking but let's step back for a second... We need to write _code generators_ and use hacks to make Nextion usable? The actual solution could be so damn simple: let me read the color of the pixel the user touched. Boom. One could have arbitrarily shaped color fields and always get the exact color the user touched with _no effort_. But no, that would be too simple. The next best solution, calculating the values, could be rather easy again. But nooo. Not with Nextion. Math functions? Nope. Heck you can't even use `()` in your Nextion math. 

As so often it becomes a huge pain in the ass because Nextion is insanely limited. And I mean that litterally. If you can't come up with scripts, hacks and workarounds you're risking to loose sanity. That shouldn't be as normal as it is for Nextion users...

Or, to quote their forum admin ([Source](https://nextion.tech/forums/topic/where-are-the-solidworks-models-for-the-displays/) requires login to see; otherwise it shows you an error 404): 

> The Nextion itself is a **very quality device** at a very economical price.

Amen.

## License

Copyright 2022 Max Zuidberg. This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

> MPL is a copyleft license that is easy to comply with. You must make the source code for any of your changes available under MPL, but you can combine the MPL software with proprietary code, as long as you keep the MPL code in separate files. Version 2.0 is, by default, compatible with LGPL and GPL version 2 or greater. You can distribute binaries under a proprietary license, as long as you make the source available under MPL.

(Source: https://tldrlegal.com/license/mozilla-public-license-2.0-(mpl-2) )

