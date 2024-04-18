from RoboSapiensAdaptivePlatform.utils.nodes import Node_generic

def configuration():
    print("Configuration triggered")

sm = Node_generic()




sm.enter()

sm.raise_ra_penter_configuration_mode()
print(sm.is_state_active(sm.State.node_operational_r1configuration_mode))
x = 10