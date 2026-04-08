#!/usr/bin/env python3

import json
import logging
import sys
import os
import signal
import paho.mqtt.client as mqtt

from pid_fan_controller import PID_fan_controller


DRY_RUN = os.getenv('DRY_RUN')
CONFIG_FILE = os.getenv('CONFIG_FILE')
MQTT_HOST = os.getenv('MQTT_HOST')

MQTT_PORT = 1883
MQTT_KEEPALIVE = 30
MQTT_STATE_TOPIC = "tvaux/fans/state"


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
)


stopped = False


def stop(*_args):
    global stopped
    stopped = True


def mqtt_client():
    if not MQTT_HOST:
        logging.info("MQTT disabled, MQTT_HOST is not set")
        return None

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=MQTT_KEEPALIVE)
    client.loop_start()
    logging.info("publishing fan state to %s via MQTT host %s", MQTT_STATE_TOPIC, MQTT_HOST)
    return client


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
    mclient = mqtt_client()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    controller.override_fan_auto_control(True, dry_run)
    try:
        while not stopped:
            state = controller.step(dry_run)
            if mclient:
                mclient.publish(MQTT_STATE_TOPIC, json.dumps(state), qos=1, retain=True)
    finally:
        if mclient:
            mclient.loop_stop()
            mclient.disconnect()
        controller.override_fan_auto_control(False, dry_run)


if __name__ == "__main__":
    main()
