SWITCH_UP=1
SWITCH_DOWN=3

sw_map = (
( 4, 2, SWITCH_UP),
( 12, 2, SWITCH_DOWN),
( 16, 17, SWITCH_UP),
( 17, 16, SWITCH_UP),
( 18, 15, SWITCH_UP),
( 19, 14, SWITCH_UP),
( 20, 13, SWITCH_UP),
( 21, 12, SWITCH_UP),
( 22, 10, SWITCH_UP),
( 23, 11, SWITCH_UP),
( 24, 17, SWITCH_DOWN),
( 25, 16, SWITCH_DOWN),
( 26, 15, SWITCH_DOWN),
( 27, 14, SWITCH_DOWN),
( 28, 13, SWITCH_DOWN),
( 29, 12, SWITCH_DOWN),
( 30, 10, SWITCH_DOWN),
( 31, 11, SWITCH_DOWN),
( 32, 7, SWITCH_UP),
( 33, 6, SWITCH_UP),
( 34, 5, SWITCH_UP),
( 35, 4, SWITCH_UP),
( 36, 3, SWITCH_UP),
( 37, 8, SWITCH_UP),
( 38, 1, SWITCH_UP),
( 39, 9, SWITCH_UP),
( 40, 7, SWITCH_DOWN),
( 41, 6, SWITCH_DOWN),
( 42, 5, SWITCH_DOWN),
( 43, 4, SWITCH_DOWN),
( 44, 3, SWITCH_DOWN),
( 45, 8, SWITCH_DOWN),
( 46, 1, SWITCH_DOWN),
( 47, 9, SWITCH_DOWN),
)

for sw_code, sw_n, sw_pos in sw_map:
    if sw_pos == SWITCH_UP:
        vn = 'this->swbits_ups'
        mn = 'SET_SW_UP'
    else:
        vn = 'this->swbits_downs'
        mn = 'SET_SW_DOWN'
        
    print "case {sw_code}: {mn}({sw_n}); break; ".format(**locals())