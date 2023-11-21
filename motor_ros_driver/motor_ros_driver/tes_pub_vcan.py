#!/usr/bin/env python

"""
Shows how to receive messages via polling.
"""

import can
from can.bus import BusState
import cantools


def pub_v_can():
    """Receives all messages and prints them to the console until Ctrl+C is pressed."""

    # this uses the default configuration (for example from environment variables, or a
    # config file) see https://python-can.readthedocs.io/en/stable/configuration.html
    bus = can.Bus(interface='socketcan', channel='vcan0', bitrate=500000)  
    can_db = cantools.database.load_file('can_parser.dbc')

    while True:
        print('send')
         # set to read-only, only supported on some interfaces
        motor_elec_out ={'Ua': 0.0, 'Ub': 0.0, 'current': 0.0, 'electrical_angle': 0.0}
        data = can_db.encode_message(4,motor_elec_out)
        msg = can.Message(arbitration_id=4, data=data, is_extended_id=False)

        try:
            bus.send(msg)

        except KeyboardInterrupt:
            print('err')


if __name__ == "__main__":
    pub_v_can()