from RTBox import RTBox

test_box = RTBox()
test_box.prep()
print 'Clock ratio: ' + str(test_box.get_clock_ratio())
print 't_diff: ' + str(test_box.get_t_diff())
[events, event_times] = test_box.read(secs=5.0)
print events
print event_times
test_box.close()
print 'End run'