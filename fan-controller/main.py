#!/usr/bin/env python3

import sys
import os

from pid_fan_controller import PID_fan_controller


DRY_RUN = os.getenv('DRY_RUN')
CONFIG_FILE = os.getenv('CONFIG_FILE')


def main():
    """
    Main function to run the PID fan controller.
    """
    if not CONFIG_FILE:
        print("Please set the CONFIG_FILE environment variable.", file=sys.stderr)
        sys.exit(1)

    config_file = str(CONFIG_FILE)
    dry_run = True if DRY_RUN else False

    controller = PID_fan_controller(config_file)

    controller.override_fan_auto_control(True, dry_run)
    try:
        controller.run_loop(dry_run)
    finally:
        controller.override_fan_auto_control(False, dry_run)


if __name__ == "__main__":
    main()
