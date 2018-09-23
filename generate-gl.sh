#!/bin/bash

rm -r lib/gl
python -m glad --generator=c --out-path=lib/gl
