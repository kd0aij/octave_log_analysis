function segments = doSegment2(sl_dur, data)
# assumes POS and ATT are already in workspace

segments = segment_maneuvers2(sl_dur, 0, data)
