import helics as h
import logging
import numpy as np
from pynlcontrol import BasicUtils
import matplotlib.pyplot as plt
import sympy as sym
import pandas as pd
import sys

logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler())
logger.setLevel(logging.DEBUG)


## state -space representation of isolated microgrid system including the frequency control loops (primary and secondary)
class System:
    def Fc(self, x, u):
        x1 = x[0, 0] #delta (rotor angle)
        x2 = x[1, 0] # frequency (change of frequency)
        x3 = x[2, 0] # rate of change of frequency (change of frequency)
        Ki = 2.0 # secondary coltroller gain
        Rp = 0.05 # primary controller roop-regulation cofficient
        Tg = 0.2 #governer time constant 
        M = 4.0 #Inertia constant
        D = 1.5 #Damping constant
        
        #State-space equation of islanded microgrid
        return np.array(
            [[x2,
              x3,
              -Ki/(M*Tg)*x1-(D/(M*Tg)+1/(Rp*M*Tg))*x2-(D/M+1/Tg)*x3-1/(M*Tg)*u
              ]]
        ).T


## RK4 solver  (4th-order)
    def solve(self, x, u, Ts):
        return BasicUtils.Integrate(self.Fc, 'rk4', 0.02, x, u)

## Del_P_e  (Change of load)
def LoadGen(t):
    return 0.0 if t <= 5 else 0.1


def destroy_federate(fed):
    grantedtime = h.helicsFederateRequestTime(fed, h.HELICS_TIME_MAXTIME)
    status = h.helicsFederateDisconnect(fed)
    h.helicsFederateFree(fed)
    h.helicsCloseLibrary()
    logger.info("Federate finalized")


if __name__ == "__main__":
    system = System()

    fed = h.helicsCreateValueFederateFromConfig("SysEqn.json")
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
    logger.info("\nEntered HELICS execution mode\m")

    total_time = 50
    update_interval = h.helicsFederateGetTimeProperty(
        fed, h.helics_property_time_period)
    logger.debug(f"Update interval: {update_interval}")

    x = np.array([[0.0, 0.0, 0.0]]).T  # Initial states for d

    time_sim = []
    x_sim = []
    u_sim = []

    initial_time = 0.0
    logger.debug(f"Requesting initial time {initial_time}")
    granted_time = h.helicsFederateRequestTime(fed, initial_time)
    logger.debug(f"Granted initial interval: {granted_time}")

    h.helicsPublicationPublishDouble(pubid[0], x[1, 0])
    logger.debug(f"\tPublished state: {x[1,0]:.2f} at {granted_time}")

    while granted_time < total_time:
        requested_time = granted_time + update_interval

        logger.debug(f"\tRequesting time {requested_time}")
        granted_time = h.helicsFederateRequestTime(fed, requested_time)
        logger.debug(f"\tGranted time: {granted_time}")

        # if (granted_time % 0.1).is_integer():
        u = h.helicsInputGetDouble(subid[0])
        logger.debug(f"\tReceived control signal {u:.2f}  at {granted_time}")

        logger.debug(f"\tUpdate interval: {update_interval}")
        Load = LoadGen(granted_time)
        logger.debug(f"\t Load at time {granted_time}: {Load}")
        x = system.solve(x, Load - u, update_interval)

        logger.debug(
            f"\tPublished state signal{x[1, 0]: .2f}  at {granted_time}\n")

        h.helicsPublicationPublishDouble(pubid[0], x[1, 0])

        time_sim.append(granted_time)
        x_sim.append(x[1, 0])
        u_sim.append(u)

    logger.debug(f"t: {time_sim}")

    destroy_federate(fed)

df = pd.DataFrame(data={'time': time_sim, 'x': x_sim, 'u': u_sim})

K = float(sys.argv[1])
df.to_csv(f'data_fin{K:.0f}.csv', sep=',')
