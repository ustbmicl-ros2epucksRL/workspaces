#!/usr/bin/env python3
"""Ensure diffdrive_controller is loaded and active for a robot swarm."""

import argparse
import time

import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)


class ControllerActivator(Node):
    def __init__(self):
        super().__init__('diffdrive_controller_activator')

    def _call(self, client, request, timeout_sec=5.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return None
        try:
            return future.result()
        except Exception:
            return None

    def _controller_clients(self, robot_name):
        prefix = f'/{robot_name}/controller_manager'
        return (
            self.create_client(ListControllers, f'{prefix}/list_controllers'),
            self.create_client(LoadController, f'{prefix}/load_controller'),
            self.create_client(ConfigureController, f'{prefix}/configure_controller'),
            self.create_client(SwitchController, f'{prefix}/switch_controller'),
        )

    def controller_states(self, robot_name):
        list_client = self.create_client(
            ListControllers,
            f'/{robot_name}/controller_manager/list_controllers',
        )
        response = self._call(list_client, ListControllers.Request())
        if response is None:
            return None
        return {controller.name: controller.state for controller in response.controller}

    def ensure_active_once(self, robot_name):
        (
            list_client,
            load_client,
            configure_client,
            switch_client,
        ) = self._controller_clients(robot_name)
        response = self._call(list_client, ListControllers.Request())
        if response is None:
            return False, 'list_unavailable'

        states = {controller.name: controller.state for controller in response.controller}
        if states.get('diffdrive_controller') == 'active':
            return True, 'already_active'

        if 'diffdrive_controller' not in states:
            load_request = LoadController.Request()
            load_request.name = 'diffdrive_controller'
            self._call(load_client, load_request)
            states = self.controller_states(robot_name) or {}

        if states.get('diffdrive_controller') == 'unconfigured':
            configure_request = ConfigureController.Request()
            configure_request.name = 'diffdrive_controller'
            self._call(configure_client, configure_request, timeout_sec=7.0)

        switch_request = SwitchController.Request()
        switch_request.activate_controllers = ['diffdrive_controller']
        switch_request.deactivate_controllers = []
        switch_request.strictness = SwitchController.Request.BEST_EFFORT
        switch_request.activate_asap = True
        switch_request.timeout.sec = 5
        switch_request.timeout.nanosec = 0
        self._call(switch_client, switch_request, timeout_sec=7.0)

        states = self.controller_states(robot_name)
        if states is None:
            return False, 'post_switch_list_unavailable'
        return states.get('diffdrive_controller') == 'active', states.get('diffdrive_controller', 'missing')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-count', type=int, default=10)
    parser.add_argument('--timeout-seconds', type=float, default=90.0)
    parser.add_argument('--settle-seconds', type=float, default=2.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = ControllerActivator()
    deadline = time.monotonic() + args.timeout_seconds
    last_status = {}
    try:
        while time.monotonic() < deadline:
            all_active = True
            for index in range(args.robot_count):
                robot_name = f'robot_{index}'
                active, status = node.ensure_active_once(robot_name)
                last_status[robot_name] = status
                all_active = all_active and active
            if all_active:
                if args.settle_seconds > 0:
                    time.sleep(args.settle_seconds)
                print('all_diffdrive_active')
                return 0
            time.sleep(1.0)
        print('diffdrive_not_active ' + repr(last_status))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
