### Verdict
| move | base 0.8 N·m | 1.5 N·m | 2.5 N·m | 4.0 N·m +mass +arm | 4.0 N·m (light, extra) | 4.0 N·m +mass, kp60 (extra) |
|---|---|---|---|---|---|---|
| stand | PASS | PASS | PASS | PASS | PASS | PASS |
| sit | PASS/ctrl | PASS/ctrl | PASS/ctrl | PASS/ctrl | PASS/ctrl | PASS/ctrl |
| lay | PASS | PASS | PASS | PASS | PASS | PASS |
| tgrab | PASS | PASS | PASS | PASS | PASS | PASS |
| walk | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| happy | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| stairs | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| couch | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| hop | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| jump | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| flip | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| pounce | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| gallop | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| sweep | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |

### Residual peak (% body weight, guided run; <35 = PASS w/ controller)
| move | base 0.8 N·m | 1.5 N·m | 2.5 N·m | 4.0 N·m +mass +arm | 4.0 N·m (light, extra) | 4.0 N·m +mass, kp60 (extra) |
|---|---|---|---|---|---|---|
| stand | 4 | 4 | 4 | 2 | 4 | 2 |
| sit | 21 | 21 | 21 | 21 | 21 | 23 |
| lay | 83 | 83 | 83 | 82 | 83 | 82 |
| tgrab | 6 | 6 | 6 | 5 | 6 | 4 |
| walk | 80 | 80 | 80 | 81 | 80 | 81 |
| happy | 80 | 80 | 80 | 81 | 80 | 79 |
| stairs | 73 | 129 | 302 | 198 | 309 | 211 |
| couch | 198 | 207 | 208 | 176 | 208 | 182 |
| hop | 228 | 215 | 215 | 213 | 215 | 215 |
| jump | 301 | 299 | 299 | 289 | 299 | 286 |
| flip | 334 | 335 | 335 | 334 | 335 | 322 |
| pounce | 192 | 194 | 194 | 182 | 194 | 185 |
| gallop | 364 | 794 | 810 | 402 | 773 | 325 |
| sweep | 435 | 662 | 584 | 378 | 789 | 382 |

### Open-loop (motors alone): path error mm / fell
| move | base 0.8 N·m | 1.5 N·m | 2.5 N·m | 4.0 N·m +mass +arm | 4.0 N·m (light, extra) | 4.0 N·m +mass, kp60 (extra) |
|---|---|---|---|---|---|---|
| stand | 8 | 8 | 8 | 5 | 8 | 5 |
| sit | 76 | 76 | 76 | 80 | 76 | 80 |
| lay | 47 | 47 | 47 | 47 | 47 | 46 |
| tgrab | 15 | 15 | 15 | 12 | 15 | 11 |
| walk | 1185 fell | 1186 fell | 1189 fell | 1187 fell | 1189 fell | 1183 fell |
| happy | 823 fell | 902 fell | 916 fell | 912 fell | 916 fell | 932 fell |
| stairs | 952 | 870 | 870 | 804 | 870 | 909 |
| couch | 1017 | 1229 fell | 997 | 1066 | 1025 | 1070 |
| hop | 381 | 380 | 381 | 394 | 381 | 393 |
| jump | 920 | 888 | 886 | 896 | 989 | 891 |
| flip | 408 | 406 | 406 | 394 | 406 | 391 |
| pounce | 410 | 356 | 356 | 411 | 356 | 379 |
| gallop | 9164 fell | 8977 fell | 8783 fell | 9023 fell | 8719 fell | 8534 fell |
| sweep | 13518 fell | 12919 fell | 12947 fell | 12762 fell | 13420 fell | 13072 fell |

### Saturated motors in the guided run (peak ≥ 0.98 × limit; % = share of samples at the limit, shown when ≥ 0.5 %)
| move | base 0.8 N·m | 1.5 N·m | 2.5 N·m | 4.0 N·m +mass +arm | 4.0 N·m (light, extra) | 4.0 N·m +mass, kp60 (extra) |
|---|---|---|---|---|---|---|
| stand | none (max FR_dist 0.29×) | none (max neck_lift 0.19×) | none (max neck_lift 0.19×) | none (max neck_lift 0.19×) | none (max neck_lift 0.19×) | none (max neck_lift 0.18×) |
| sit | none (max spine_drum_rear 0.56×) | none (max spine_drum_rear 0.56×) | none (max spine_drum_rear 0.56×) | none (max spine_drum_rear 0.56×) | none (max spine_drum_rear 0.56×) | none (max spine_drum_rear 0.56×) |
| lay | spine_twist (32%) | spine_twist (32%) | spine_twist (32%) | spine_twist (56%) | spine_twist (32%) | spine_twist (59%) |
| tgrab | none (max FR_dist 0.29×) | none (max FL_abd 0.27×) | none (max FL_abd 0.27×) | none (max FL_abd 0.25×) | none (max FL_abd 0.27×) | none (max FL_abd 0.25×) |
| walk | spine_twist (9%), FR_knee, FR_dist | spine_twist (9%), FR_knee, FR_dist | spine_twist (9%), FR_knee, FR_dist | spine_twist (17%), FR_knee | spine_twist (9%), FR_knee | spine_twist (18%), FR_knee, FR_dist |
| happy | spine_twist (30%), FL_dist (2%), FR_dist (1%) | spine_twist (30%), FR_knee, FR_dist | spine_twist (30%), FR_knee, FR_dist | spine_twist (31%), FR_knee, FR_dist | spine_twist (30%), FR_knee, FR_dist | spine_twist (32%), FR_knee, FR_dist |
| stairs | spine_twist (25%), RL_dist (2%), FR_dist (2%) | spine_twist (25%), FL_knee (1%), spine_drum_front (1%) | spine_twist (20%), FR_knee (6%), spine_drum_front (6%) | spine_twist (29%), FL_knee (8%), FR_knee (4%) | spine_twist (21%), spine_drum_front (7%), FR_knee (6%) | spine_twist (31%), FL_knee (9%), FR_knee (5%) |
| couch | spine_drum_rear (70%), spine_drum_front (36%), neck_lift (7%) | spine_drum_rear (70%), spine_drum_front (34%), neck_lift (7%) | spine_drum_rear (70%), spine_drum_front (34%), neck_lift (7%) | spine_drum_rear (71%), spine_drum_front (32%), neck_lift (6%) | spine_drum_rear (70%), spine_drum_front (34%), neck_lift (7%) | spine_drum_rear (71%), spine_drum_front (31%), neck_lift (6%) |
| hop | spine_drum_rear (48%), spine_drum_front (38%), neck_lift (6%) | spine_drum_rear (48%), spine_drum_front (37%), neck_lift (5%) | spine_drum_rear (48%), spine_drum_front (36%), neck_lift (5%) | spine_drum_rear (51%), spine_drum_front (48%), neck_lift (5%) | spine_drum_rear (48%), spine_drum_front (36%), neck_lift (5%) | spine_drum_rear (51%), spine_drum_front (49%), neck_lift (5%) |
| jump | spine_drum_rear (35%), spine_drum_front (18%), FL_dist (9%) | spine_drum_rear (23%), spine_drum_front (21%), neck_lift (3%) | spine_drum_rear (39%), spine_drum_front (13%), neck_lift (4%) | spine_drum_rear (36%), spine_drum_front (14%), neck_lift (5%) | spine_drum_rear (39%), spine_drum_front (13%), neck_lift (4%) | spine_drum_rear (36%), spine_drum_front (14%), neck_lift (4%) |
| flip | spine_drum_front (49%), spine_drum_rear (28%), neck_lift (13%) | spine_drum_front (45%), spine_drum_rear (28%), neck_lift (14%) | spine_drum_front (45%), spine_drum_rear (28%), neck_lift (14%) | spine_drum_front (62%), spine_drum_rear (29%), neck_lift (14%) | spine_drum_front (45%), spine_drum_rear (28%), neck_lift (14%) | spine_drum_rear (65%), spine_drum_front (63%), neck_lift (15%) |
| pounce | spine_drum_rear (60%), spine_drum_front (58%), spine_twist (10%) | spine_drum_rear (60%), spine_drum_front (58%), spine_twist (10%) | spine_drum_rear (60%), spine_drum_front (58%), spine_twist (10%) | spine_drum_rear (60%), spine_drum_front (58%), spine_twist (10%) | spine_drum_rear (60%), spine_drum_front (58%), spine_twist (10%) | spine_drum_rear (60%), spine_drum_front (58%), spine_twist (10%) |
| gallop | spine_drum_front (92%), spine_drum_rear (68%), RL_dist (62%) | spine_drum_front (91%), spine_drum_rear (69%), spine_twist (50%) | spine_drum_front (90%), spine_drum_rear (69%), spine_twist (51%) | spine_drum_front (93%), spine_drum_rear (75%), spine_twist (54%) | spine_drum_front (90%), spine_drum_rear (67%), spine_twist (52%) | spine_drum_front (93%), spine_drum_rear (81%), spine_twist (61%) |
| sweep | spine_drum_front (91%), spine_drum_rear (70%), RL_dist (65%) | spine_drum_front (89%), spine_drum_rear (70%), spine_twist (58%) | spine_drum_front (88%), spine_drum_rear (71%), spine_twist (57%) | spine_drum_front (92%), spine_drum_rear (79%), spine_twist (61%) | spine_drum_front (85%), spine_drum_rear (67%), spine_twist (56%) | spine_drum_front (93%), spine_drum_rear (77%), spine_twist (58%) |

### Count of saturated actuators per move (guided run)
| move | base 0.8 N·m | 1.5 N·m | 2.5 N·m | 4.0 N·m +mass +arm | 4.0 N·m (light, extra) | 4.0 N·m +mass, kp60 (extra) |
|---|---|---|---|---|---|---|
| stand | 0 | 0 | 0 | 0 | 0 | 0 |
| sit | 0 | 0 | 0 | 0 | 0 | 0 |
| lay | 1 | 1 | 1 | 1 | 1 | 1 |
| tgrab | 0 | 0 | 0 | 0 | 0 | 0 |
| walk | 3 | 3 | 3 | 2 | 2 | 3 |
| happy | 6 | 4 | 4 | 4 | 4 | 4 |
| stairs | 8 | 11 | 10 | 6 | 10 | 7 |
| couch | 15 | 13 | 11 | 10 | 11 | 10 |
| hop | 9 | 9 | 9 | 7 | 7 | 7 |
| jump | 9 | 7 | 9 | 7 | 7 | 9 |
| flip | 10 | 6 | 6 | 6 | 6 | 6 |
| pounce | 6 | 4 | 4 | 4 | 4 | 4 |
| gallop | 19 | 22 | 19 | 22 | 22 | 21 |
| sweep | 21 | 21 | 22 | 21 | 22 | 21 |

No run went NaN.
