# https://gist.github.com/vladignatyev/06860ec2040cb497f0f3
# MIT Licensed
# Copyright (c) 2016 Vladimir Ignatev
import sys


def progress(count: int, total: int, status: str = '') -> None:
    bar_len = 60
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    bar = '=' * filled_len + '-' * (bar_len - filled_len)

    sys.stdout.write('[%s] %s%s ...%s\r' % (bar, percents, '%', status))
    sys.stdout.flush()
