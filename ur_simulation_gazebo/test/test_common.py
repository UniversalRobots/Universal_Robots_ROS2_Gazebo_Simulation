# Copyright 2024, FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import logging
import time

import rclpy
from rclpy.action import ActionClient

from controller_manager_msgs.srv import ListControllers

TIMEOUT_WAIT_SERVICE = 10
TIMEOUT_WAIT_SERVICE_INITIAL = 120
TIMEOUT_WAIT_ACTION = 20


def _wait_for_service(node, srv_name, srv_type, timeout):
    client = node.create_client(srv_type, srv_name)

    logging.info("Waiting for service '%s' with timeout %fs...", srv_name, timeout)
    if client.wait_for_service(timeout) is False:
        raise Exception(f"Could not reach service '{srv_name}' within timeout of {timeout}")
    logging.info("  Successfully connected to service '%s'", srv_name)

    return client


def _wait_for_action(node, action_name, action_type, timeout):
    client = ActionClient(node, action_type, action_name)

    logging.info("Waiting for action server '%s' with timeout %fs...", action_name, timeout)
    if client.wait_for_server(timeout) is False:
        raise Exception(
            f"Could not reach action server '{action_name}' within timeout of {timeout}"
        )

    logging.info("  Successfully connected to action server '%s'", action_name)
    return client


def wait_for_controller(
    node, controller_name, timeout, cm_list_service="/controller_manager/list_controllers"
):
    logging.info("Waiting for controller '%s' with timeout %fs...", controller_name, timeout)
    client = _wait_for_service(node, cm_list_service, ListControllers, TIMEOUT_WAIT_SERVICE)
    controller_active = False
    start_time = node.get_clock().now()
    while not controller_active and (
        node.get_clock().now() - start_time < rclpy.time.Duration(seconds=timeout)
    ):
        result = _call_service(node, client, ListControllers.Request())
        for controller in result.controller:
            if controller.name == controller_name:
                controller_active = controller.state == "active"
                if controller_active:
                    logging.info("Controller '%s' is active.", controller_name)
                    return True
        time.sleep(1)
    raise Exception(
        f"Could not find active controller '{controller_name}' within timeout of {timeout}"
    )


def _call_service(node, client, request):
    logging.info("Calling service client '%s' with request '%s'", client.srv_name, request)
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        logging.info("  Received result: %s", future.result())
        return future.result()

    raise Exception(f"Error while calling service '{client.srv_name}': {future.exception()}")


class _ServiceInterface:
    def __init__(
        self, node, initial_timeout=TIMEOUT_WAIT_SERVICE_INITIAL, timeout=TIMEOUT_WAIT_SERVICE
    ):
        self.__node = node

        self.__service_clients = {
            srv_name: (
                _wait_for_service(self.__node, srv_name, srv_type, initial_timeout),
                srv_type,
            )
            for srv_name, srv_type in self.__initial_services.items()
        }
        self.__service_clients.update(
            {
                srv_name: (_wait_for_service(self.__node, srv_name, srv_type, timeout), srv_type)
                for srv_name, srv_type in self.__services.items()
            }
        )

    def __init_subclass__(mcs, namespace="", initial_services={}, services={}, **kwargs):
        super().__init_subclass__(**kwargs)

        mcs.__initial_services = {
            namespace + "/" + srv_name: srv_type for srv_name, srv_type in initial_services.items()
        }
        mcs.__services = {
            namespace + "/" + srv_name: srv_type for srv_name, srv_type in services.items()
        }

        for srv_name, srv_type in list(initial_services.items()) + list(services.items()):
            full_srv_name = namespace + "/" + srv_name

            setattr(
                mcs,
                srv_name,
                lambda s, full_srv_name=full_srv_name, *args, **kwargs: _call_service(
                    s.__node,
                    s.__service_clients[full_srv_name][0],
                    s.__service_clients[full_srv_name][1].Request(*args, **kwargs),
                ),
            )


class ActionInterface:
    def __init__(self, node, action_name, action_type, timeout=TIMEOUT_WAIT_ACTION):
        self.__node = node

        self.__action_name = action_name
        self.__action_type = action_type
        self.__action_client = _wait_for_action(node, action_name, action_type, timeout)

    def send_goal(self, *args, **kwargs):
        goal = self.__action_type.Goal(*args, **kwargs)

        logging.info("Sending goal to action server '%s': %s", self.__action_name, goal)
        future = self.__action_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self.__node, future)

        if future.result() is not None:
            logging.info("  Received result: %s", future.result())
            return future.result()
        pass

    def get_result(self, goal_handle, timeout):
        future_res = goal_handle.get_result_async()

        logging.info(
            "Waiting for action result from '%s' with timeout %fs", self.__action_name, timeout
        )
        rclpy.spin_until_future_complete(self.__node, future_res, timeout_sec=timeout)

        if future_res.result() is not None:
            logging.info("  Received result: %s", future_res.result().result)
            return future_res.result().result
        else:
            raise Exception(
                f"Exception while calling action '{self.__action_name}': {future_res.exception()}"
            )
