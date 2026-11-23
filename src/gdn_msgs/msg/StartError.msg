# A start error GDN was given (R2.6a, a scenario's start_error; R1.7's in the Monte Carlo):
# once its heading is aligned it moves its position estimate by this much and is told
# sigma per axis. In the run log right after RunInfo (and the vehicle profile), so
# gdn_replay applies it too.
float64 north_m
float64 east_m
float64 sigma_m                      # max(distance, 10 m), as mc_campaign --start-error
