# Fast DDS hangs while creating the full Rokae client set on this robot PC.
# Respect an explicit operator choice; otherwise use the verified RMW.
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi
