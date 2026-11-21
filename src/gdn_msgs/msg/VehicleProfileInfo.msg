# The vehicle profile a GDN run used (R2.5a): the run log's second message, right after
# RunInfo, when the node was given one (-p vehicle_profile:=...). gdn_replay applies it.
string name
string path                          # as given to the node
string fingerprint                   # 12 hex digits (gdn/vehicle_profile.hpp)
string text                          # the profile file as read
