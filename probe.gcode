; dual speed probe macro

; set parameters
#<fast_rate>=160
#<slow_rate>=80
#<probe_dist>=100
#<probe_offset>=10
#<retract_height>=5

G38.2 G91 Z[-#<probe_dist>] F#<fast_rate> ; probe fast
G0 Z3  ; retract a little
G38.2 G91 Z[-#<probe_dist>] F#<slow_rate>; probe slowly
#<wco_z_touch>=#5063 ; save the z touch WCO location
G0 Z[#<retract_height>+#<probe_offset>] ; retract
G10 L2 P0 Z[#<wco_z_touch>+#<probe_offset>]
G91 G0 Z[#<retract_height>+#<probe_offset>]