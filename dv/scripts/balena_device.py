#!/usr/bin/env python3

import os, sys, time
from pprint import pprint
import requests

from balena import Balena

import logging, coloredlogs

coloredlogs.install(
    level='DEBUG',
    # fmt="%(asctime)s %(hostname)s %(name)s[%(process)d] %(levelname)s %(message)s"
    fmt="%(name)s[%(process)d] %(levelname)s %(message)s",
    level_styles={
        'info': {
            'color': 'green',
            'bold': True,
        },
        'debug': {
            'color': 'white'
        },
        'warn': {
            'color': 'yellow',
        },
        'error': {
            'color': 'red',
            'bold': True,
        },
    },
)
logger = logging.getLogger(__name__)


class Device():
    """ Balena updater class to handle balena SDK and API interactions."""

    def __init__(self):
        self.auth_token = os.environ.get("BALENA_API_KEY")
        self.device_uuid = os.environ.get("BALENA_DEVICE_UUID")
        self.application_uuid = os.environ.get("BALENA_APP_UUID")
        self.supervisor_address = os.environ.get("BALENA_SUPERVISOR_ADDRESS")
        self.supervisor_api_key = os.environ.get("BALENA_SUPERVISOR_API_KEY")
        self.update_lock_file_path = os.environ.get("BALENA_APP_LOCK_PATH")
        self.service_name = os.environ.get("BALENA_SERVICE_NAME")

        self.name_of_counter_to_force_update_restart = "COUNTER_TO_FORCE_UPDATE_RESTART"
        self.current_release_hash = None
        self.available_release_hash = None

        self.balena = balena = Balena()

    def authenticate(self) -> bool:
        """ Authenticate with the balena API if not already authenticated.

            Returns:
                bool: True if the authentication was successful, False otherwise.
        """

        if self.balena.auth.is_logged_in():
            return True

        if not self.auth_token:
            raise Exception("BALENA_API_KEY environment variable not set")
        if not self.device_uuid:
            raise Exception("BALENA_DEVICE_UUID environment variable not set")

        try:
            self.balena.auth.login_with_token(self.auth_token)
        except:
            return False

        if not self.balena.auth.is_logged_in():
            return False

        return True

    def get_supervisor_state_status(self):
        url = f"{self.supervisor_address}/v2/state/status"
        headers = {"Authorization": f"Bearer {self.supervisor_api_key}"}
        r = requests.get(url, headers=headers)
        return r.json()

    def wait_for_supervisor_state(self, targets_super_state="success", target_app_state="applied"):
        super_state = self.get_supervisor_state_status()
        current_super_status = super_state["status"]
        current_app_state = super_state["appState"]
        logger.debug(f"Waiting for {current_super_status} ==> {targets_super_state} and {current_app_state} ==> {target_app_state}")

        while current_super_status != targets_super_state and current_app_state != target_app_state:
            time.sleep(1)
            logger.debug(f"Waiting for {current_super_status} ==> {targets_super_state} and {current_app_state} ==> {target_app_state}")
            super_state = self.get_supervisor_state_status()
            current_super_status = super_state["status"]
            current_app_state = super_state["appState"]

    def wait_for_service_status(self, service_name, target_service_status="Running"):
        logger.debug(f"Waiting for {service_name}[{target_service_status}]")
        # my_app_id = self.get_self_app_id()
        service = self.get_service(service_name)
        # current_service_app_id = service["status"]
        current_service_status = service["appId"]

        while current_service_status != target_service_status:
            time.sleep(1)
            service = self.get_service(service_name)
            current_service_status = service["appId"]
            logger.debug(f"Waiting for {service_name}: {current_service_status} ==> {target_service_status}")

    def try_stop_service(self, service_name):

        service = self.get_service(service_name)

        if service["status"] == "Running":
            self.balena.models.device.stop_service(None, service["imageId"])
        else:
            logger.warning(f"{service_name} found but it is not running")

        service_status = self.get_service(service_name)["status"]
        while service_status != "exited":
            logger.debug(f"{service_name} {service_status}")
            service_status = self.get_service(service_name)["status"]
            time.sleep(1)

        service_status = self.get_service(service_name)["status"]
        logger.info(f"{service_name} {service_status}")

    def try_start_service(self, service_name):
        status = self.get_supervisor_state_status()
        for container in status["containers"]:
            if container["serviceName"] == service_name:
                if container["status"] != "Running":
                    self.balena.models.device.start_service(None, container["imageId"])
                    break
        else:
            logger.info(f"Image for {service_name} is not found")
            logger.debug(status)

        service_status = self.get_service(service_name)["status"]
        while service_status != "Running":
            logger.debug(f"{service_name} {service_status}")
            service_status = self.get_service(service_name)["status"]
            time.sleep(1)

        service_status = self.get_service(service_name)["status"]
        logger.info(f"{service_name} {service_status}")

    def get_self_app_id(self):
        service = self.get_service(self.service_name)
        return service["appId"]

    def get_service(self, service_name):
        status = self.get_supervisor_state_status()
        res = [container for container in status["containers"] if container["serviceName"] == service_name]

        if len(res) == 1:
            return res[0]

        timeout = 30

        for n in range(timeout):
            logger.debug(f"Waiting for a service {service_name} to appeare in super status {n}")
            time.sleep(1)
            status = self.get_supervisor_state_status()
            res = [container for container in status["containers"] if container["serviceName"] == service_name]
            if len(res) == 1:
                return res[0]

        logger.error(f"Service {service_name} was not found in super state for {timeout} seconds")
        return None

    def leave_me_alone(self):
        status = self.get_supervisor_state_status()
        running_services_names = [container["serviceName"] for container in status["containers"] if
                                  container["serviceName"] != self.service_name and container["status"] == "Running"]

        for running_services_name in running_services_names:
            self.try_stop_service(running_services_name)

        return running_services_names


if __name__ == '__main__':
    pass
    # device = Device()
    # print(device.authenticate())
    # pprint(device.get_supervisor_state_status())
    #
    # # device.try_stop_service("mavlink-router")
    # device.try_start_service("mavlink-router")
    #
    # time.sleep(1)
    #
    # stopped_service_names = device.leave_me_alone()
    # print("hehee, I'm the only here")
    #
    # time.sleep(5)
    #
    # for service_name in stopped_service_names:
    #     device.try_start_service(service_name)
