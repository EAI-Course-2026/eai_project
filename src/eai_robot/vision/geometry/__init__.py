"""Where a target is and how large it looks, independent of what it is.

`targets.py` is the representation the servo loops consume; `scale.py` turns its
apparent size into a distance. Both are pure numpy: no camera, no model, no I/O,
which is what lets the fixed rule's geometry be tested without hardware.
"""
