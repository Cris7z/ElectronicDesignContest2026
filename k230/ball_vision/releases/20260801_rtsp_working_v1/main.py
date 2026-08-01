"""K230 boot wrapper: preserve the actual startup exception on the SD card."""

import sys

_ERROR_PATH = "/sdcard/main_error.txt"

try:
    import os
    try:
        os.remove(_ERROR_PATH)
    except OSError:
        pass
except BaseException:
    pass

try:
    import ball_app
except BaseException as error:
    try:
        with open(_ERROR_PATH, "w") as handle:
            handle.write("K230 main.py startup failed:\n")
            if hasattr(sys, "print_exception"):
                sys.print_exception(error, handle)
            else:
                handle.write(repr(error))
                handle.write("\n")
    except BaseException:
        pass
    raise
