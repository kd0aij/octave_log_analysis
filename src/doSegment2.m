function segments = doSegment2(sl_dur, ATT, GPS, POS)
# assumes POS and ATT are already in workspace

segments = segment_maneuvers(sl_dur, 0, ATT, GPS, POS)
