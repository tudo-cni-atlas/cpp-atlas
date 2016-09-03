#!/bin/bash
sudo socat -d -d PTY,link="/dev/ttyATLAS",raw,echo=0 tcp:localhost:8800,nodelay&
