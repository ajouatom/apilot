# Replay

## Replay driving data

`replay` replays all the messages logged while running openpilot.

```bash
# Log in via browser to have access to non-public routes
python tools/lib/auth.py

# Start a replay
tools/replay/replay <route-name>

# Example:
# tools/replay/replay '4cf7a6ad03080c90|2021-09-29--13-46-36'
# or use --demo to replay the default demo route:
# tools/replay/replay --demo

# watch the replay with the normal openpilot UI
cd selfdrive/ui && ./ui

# or try out a debug visualizer:
python replay/ui.py
```

## usage

``` bash
$ tools/replay/replay -h
Usage: tools/replay/replay [options] route
Mock openpilot components by publishing logged messages.

Options:
  -h, --help             Displays this help.
  -a, --allow <allow>    whitelist of services to send
  -b, --block <block>    blacklist of services to send
  -s, --start <seconds>  start from <seconds>
  --demo                 use a demo route instead of providing your own
  --dcam                 load driver camera
  --ecam                 load wide road camera

Arguments:
  route                  the drive to replay. find your drives at
                         connect.comma.ai
```

## watch3

watch all three cameras simultaneously from your comma three routes with watch3

simply replay a route using the `--dcam` and `--ecam` flags:

```bash
# start a replay
cd tools/replay && ./replay --demo --dcam --ecam

# then start watch3
cd selfdrive/ui && ./watch3
```

![](https://i.imgur.com/IeaOdAb.png)

## Stream CAN messages to your device

Replay CAN messages as they were recorded using a [panda jungle](https://comma.ai/shop/products/panda-jungle). The jungle has 6x OBD-C ports for connecting all your comma devices. Check out the [jungle repo](https://github.com/commaai/panda_jungle) for more info.

`can_replay.py` is a convenient script for when any CAN data will do.

In order to replay specific route:
```bash
MOCK=1 selfdrive/boardd/tests/boardd_old.py

# In another terminal:
tools/replay/replay <route-name>
```
