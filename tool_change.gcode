;-------------------------------
; FluidNC ATC Tool Change Macro
; Designed for a 5-tool turret:
; - 1mm per tool position
; - 0.05 mm backwards for backlash/locking
;
; Assumptions:
;   * Machine starts with tool 1 in position.
;   * Tool change is called with: M6 T<target_tool> (where <target_tool> is 1..5)
;   * The current tool is stored in a variable called #<current_tool>
;-------------------------------

; Ensure current tool variable is defined – if not, initialize to 1
;#<current_tool> = [#<current_tool> ?: 1]

; Capture the desired tool from the M6 command.
; FluidNC automatically makes the tool number available in parameter T.
#<target_tool> = #T

; Safety check: if the target is the same as the current tool, do nothing.
o1 if [#<target_tool> EQ #<current_tool>]
  (Tool already in position - no change required)
  M30  ; End program
o1 endif

; Calculate the tool difference (delta)
#<delta> = [#<target_tool> - #<current_tool>]

; If delta is negative, add 5 so that the turret always rotates forward.
o2 if [#<delta> LT 0]
  #<delta> = [#<delta> + 5]
o2 endif

; Calculate the total steps to move: each tool increment = 40 steps.
#<steps> = [#<delta> * 1]

; Move the turret (A axis) forward the calculated steps.
; Use external IO to set direction of turret
M62 P0
G1 A[#<steps> + 0.1] F75

; Apply the 2 step backlash compensation (move 2 steps backwards).
M63 P0
G1 A-0.076 F75

; Update the current tool variable to the new tool.
#<current_tool> = #<target_tool>

M30  ; End of program