#!/usr/bin/env python3
"""Initialize a private Mac layout once; preserve topics, colours and history."""
import os
import re
import sys


def initialize(source, destination):
    with open(source, encoding='utf-8') as input_file:
        text = input_file.read()
    start = text.find('\nWindow Geometry:\n')
    if start < 0:
        raise ValueError('Missing Window Geometry')
    before, geometry = text[:start], text[start:]
    geometry = re.sub(r'^  (Width|Height|X|Y):.*$',
                      lambda m: '  %s: %s' % (m[1], {'Width': 1380, 'Height': 820, 'X': 20, 'Y': 20}[m[1]]),
                      geometry, flags=re.M)
    # The opaque Qt state contains Ubuntu dock sizes larger than the virtual screen.
    geometry = re.sub(r'^  QMainWindow State:.*\n', '', geometry, flags=re.M)
    os.makedirs(os.path.dirname(destination), exist_ok=True)
    try:
        with open(destination, 'x', encoding='utf-8') as output:
            output.write(before + geometry)
    except FileExistsError:
        pass


if __name__ == '__main__':
    initialize(*sys.argv[1:])
