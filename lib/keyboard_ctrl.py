import sys
import termios
import time
from termios import (BRKINT, CS8, CSIZE, ECHO, ICANON, ICRNL, IEXTEN, INPCK,
                     ISTRIP, IXON, PARENB, VMIN, VTIME)
from typing import NoReturn

# Indexes for termios list.
IFLAG = 0
OFLAG = 1
CFLAG = 2
LFLAG = 3
ISPEED = 4
OSPEED = 5
CC = 6

STDIN_FD = sys.stdin.fileno()

def getch() -> str:
  old_settings = termios.tcgetattr(STDIN_FD)
  try:
    # set
    mode = old_settings.copy()
    mode[IFLAG] &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON)
    #mode[OFLAG] &= ~(OPOST)
    mode[CFLAG] &= ~(CSIZE | PARENB)
    mode[CFLAG] |= CS8
    mode[LFLAG] &= ~(ECHO | ICANON | IEXTEN)
    mode[CC][VMIN] = 1
    mode[CC][VTIME] = 0
    termios.tcsetattr(STDIN_FD, termios.TCSAFLUSH, mode)

    ch = sys.stdin.read(1)
  finally:
    termios.tcsetattr(STDIN_FD, termios.TCSADRAIN, old_settings)
  return ch

def keyboard_poll_thread(q: 'Queue[str]'):
  while True:
    c = getch()
    # print("got %s" % c)
    if c == '1':
      q.put("cruise_up")
    elif c == '2':
      q.put("cruise_down")
    elif c == '3':
      q.put("cruise_cancel")
    elif c == 'w':
      q.put("throttle_%f" % 1.0)
    elif c == 'a':
      q.put("steer_%f" % 0.15)
    elif c == 's':
      q.put("brake_%f" % 1.0)
    elif c == 'd':
      q.put("steer_%f" % -0.15)
    elif c == 'q':
      q.put("quit")
      break

def test(q: 'Queue[str]') -> NoReturn:
  while True:
    print([q.get_nowait() for _ in range(q.qsize())] or None)
    time.sleep(0.25)

if __name__ == '__main__':
  from multiprocessing import Process, Queue
  q: Queue[str] = Queue()
  p = Process(target=test, args=(q,))
  p.daemon = True
  p.start()

  keyboard_poll_thread(q)
