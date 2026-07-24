# https://gist.github.com/vladignatyev/06860ec2040cb497f0f3
# MIT Licensed
# Copyright (c) 2016 Vladimir Ignatev
import sys


def progress(count: int, total: int, status: str = "") -> None:
    bar_len = 60
    filled_len = round(bar_len * count / float(total))

    percents = round(100.0 * count / float(total), 1)
    bar = "=" * filled_len + "-" * (bar_len - filled_len)

    sys.stdout.write(f"[{bar}] {percents}% ...{status}\r")
    sys.stdout.flush()
