#!/usr/bin/env python3
import zmq
from typing import NoReturn

import cereal.messaging as messaging
from common.logging_extra import SwagLogFileFormatter
from system.swaglog import get_file_handler


def main() -> NoReturn:
  log_handler = get_file_handler()
  log_handler.setFormatter(SwagLogFileFormatter(None))
  log_level = 20  # logging.INFO

  ctx = zmq.Context().instance()
  sock = ctx.socket(zmq.PULL)
  sock.bind("ipc:///tmp/logmessage")

  # and we publish them
  log_message_sock = messaging.pub_sock('logMessage')
  error_log_message_sock = messaging.pub_sock('errorLogMessage')

  while True:
    dat = b''.join(sock.recv_multipart())
    level = dat[0]
    record = dat[1:].decode("utf-8")
    if level >= log_level:
      log_handler.emit(record)

    # then we publish them
    msg = messaging.new_message()
    msg.logMessage = record
    log_message_sock.send(msg.to_bytes())

    if level >= 40:  # logging.ERROR
      msg = messaging.new_message()
      msg.errorLogMessage = record
      error_log_message_sock.send(msg.to_bytes())


if __name__ == "__main__":
  main()
