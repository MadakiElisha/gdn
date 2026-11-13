# First message of every GDN run log (R1.11): what is needed to replay the run.
builtin_interfaces/Time start_time   # wall-clock time the node started
string gdn_version                   # git describe of the build ("-dirty": uncommitted changes)
string config_name
string config_path
string config_fingerprint            # 12 hex digits (gdn/filter_config.hpp)
string config_overrides              # "key=value, ..." against the file, or empty
string config_text                   # every value as run; gdn_replay loads it (--dump-config format)
string map_path
string map_fingerprint               # FNV-1a of the map file's bytes, 16 hex digits
float64 origin_lat                   # local frame origin, deg
float64 origin_lon
string trn_model                     # agl | ideal
uint32 state_size                    # length of the state layout (gdn::kStateSize)
