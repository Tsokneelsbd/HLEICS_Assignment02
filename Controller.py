import helics as h
import logging
import numpy as np
from pynlcontrol import BasicUtils
import matplotlib.pyplot as plt
import sys
import pandas as pd

logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler())
logger.setLevel(logging.DEBUG)


def destroy_federate(fed):
    grantedtime = h.helicsFederateRequestTime(fed, h.HELICS_TIME_MAXTIME)
    status = h.helicsFederateDisconnect(fed)
    h.helicsFederateFree(fed)
    h.helicsCloseLibrary()
    logger.info("Federate finalized")


if __name__ == "__main__":
    K = float(sys.argv[1])
    fed = h.helicsCreateValueFederateFromConfig("Control.json")
    federate_name = h.helicsFederateGetName(fed)
    logger.info(f"Created federate {federate_name}")

    sub_count = h.helicsFederateGetInputCount(fed)
    logger.debug(f"Number of subscriptions: {sub_count}")
    pub_count = h.helicsFederateGetPublicationCount(fed)
    logger.debug(f"Number of publications: {pub_count}")

    subid = {}
    for i in range(0, sub_count):
        subid[i] = h.helicsFederateGetInputByIndex(fed, i)
        sub_name = h.helicsSubscriptionGetTarget(subid[i])
        logger.debug(f"Registered subscription---> {sub_name}")

    pubid = {}
    for i in range(0, pub_count):
        pubid[i] = h.helicsFederateGetPublicationByIndex(fed, i)
        pub_name = h.helicsPublicationGetName(pubid[i])
        logger.debug(f"Registered publication---> {pub_name}")

    h.helicsFederateEnterExecutingMode(fed)
    logger.info("Entered HELICS execution mode")

    total_time = 50
    update_interval = h.helicsFederateGetTimeProperty(
        fed, h.HELICS_PROPERTY_TIME_PERIOD)
    granted_time = 0.0

    time_sim = []
    x_sim = []
    u_sim = []


    while granted_time < total_time:
        requested_time = granted_time + update_interval

        logger.debug(f"\tRequesting time {requested_time}")
        granted_time = h.helicsFederateRequestTime(fed, requested_time)
        logger.debug(f"\tGranted time: {granted_time}")

        x = h.helicsInputGetDouble(subid[0])
        logger.debug(f"\tReceived state signal {x:.2f} at {granted_time}")

        u = -K*x

        h.helicsPublicationPublishDouble(pubid[0], u)
        logger.debug(f"\tPublished u: {u:.2f} at {granted_time}\n")

        time_sim.append(granted_time)
        x_sim.append(x)
        u_sim.append(u)

    destroy_federate(fed)
