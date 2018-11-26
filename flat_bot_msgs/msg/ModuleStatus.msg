#--------------------------------------
#
# FlatBot ModuleStatus message
#
# represents the current state of an
# assembly module and all of its
# connectors 
#
#--------------------------------------

# header defines current timestamp and module_id
Header header

# seconds since this module last booted up
float32 uptime

# current bus voltage (volts)
float32 bus_voltage

flat_bot_msgs/ConnectorStatus connector_states
