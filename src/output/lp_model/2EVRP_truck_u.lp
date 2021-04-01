\ Model 2E-VRP
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 y[0,0] + 0 y[1,1] + 167.6305461424021 y[1,2] + 47.16990566028301 y[1,3]
   + 158.113883008419 y[1,4] + 167.6305461424021 y[2,1] + 0 y[2,2]
   + 205.7304061144099 y[2,3] + 106.3014581273465 y[2,4]
   + 47.16990566028301 y[3,1] + 205.7304061144099 y[3,2] + 0 y[3,3]
   + 205.2437575177379 y[3,4] + 158.113883008419 y[4,1]
   + 106.3014581273465 y[4,2] + 205.2437575177379 y[4,3] + 0 y[4,4]
   + 50 Constant
Subject To
 trucks3[1]: - y[1,2] - y[1,3] - y[1,4] + y[2,1] + y[3,1] + y[4,1] = 0
 trucks3[2]: y[1,2] - y[2,1] - y[2,3] - y[2,4] + y[3,2] + y[4,2] = 0
 trucks3[3]: y[1,3] + y[2,3] - y[3,1] - y[3,2] - y[3,4] + y[4,3] = 0
 trucks3[4]: y[1,4] + y[2,4] + y[3,4] - y[4,1] - y[4,2] - y[4,3] = 0
 if_truck_then_hub[1]: - o[0] + y[2,1] + y[3,1] + y[4,1] = 0
 if_truck_then_hub[2]: - o[1] + y[1,2] + y[3,2] + y[4,2] = 0
 if_truck_then_hub[3]: - o[2] + y[1,3] + y[2,3] + y[4,3] = 0
 if_truck_then_hub[4]: - o[3] + y[1,4] + y[2,4] + y[3,4] = 0
 start_from_depot: u[0] = 1
 lowest_hub_order[1]: u[1] >= 2
 lowest_hub_order[2]: u[2] >= 2
 lowest_hub_order[3]: u[3] >= 2
 lowest_hub_order[4]: u[4] >= 2
 lowest_hub_order_2[1]: u[1] <= 5
 lowest_hub_order_2[2]: u[2] <= 5
 lowest_hub_order_2[3]: u[3] <= 5
 lowest_hub_order_2[4]: u[4] <= 5
 respect_order[2,3]: 4 y[2,3] + u[2] - u[3] <= 3
 respect_order[2,4]: 4 y[2,4] + u[2] - u[4] <= 3
 respect_order[3,2]: 4 y[3,2] - u[2] + u[3] <= 3
 respect_order[3,4]: 4 y[3,4] + u[3] - u[4] <= 3
 respect_order[4,2]: 4 y[4,2] - u[2] + u[4] <= 3
 respect_order[4,3]: 4 y[4,3] - u[3] + u[4] <= 3
 no_hub_no_u[1]: - 5 o[0] + u[1] <= 0
 no_hub_no_u[2]: - 5 o[1] + u[2] <= 0
 no_hub_no_u[3]: - 5 o[2] + u[3] <= 0
 no_hub_no_u[4]: - 5 o[3] + u[4] <= 0
 constraint_17: o[0] + o[1] + o[2] + o[3] = 4
 no_subtours_3[0,1]: y[0,1] + y[1,0] <= 1
 no_subtours_3[0,2]: y[0,2] + y[2,0] <= 1
 no_subtours_3[0,3]: y[0,3] + y[3,0] <= 1
 no_subtours_3[0,4]: y[0,4] + y[4,0] <= 1
 no_subtours_3[1,0]: y[0,1] + y[1,0] <= 1
 no_subtours_3[1,2]: y[1,2] + y[2,1] <= 1
 no_subtours_3[1,3]: y[1,3] + y[3,1] <= 1
 no_subtours_3[1,4]: y[1,4] + y[4,1] <= 1
 no_subtours_3[2,0]: y[0,2] + y[2,0] <= 1
 no_subtours_3[2,1]: y[1,2] + y[2,1] <= 1
 no_subtours_3[2,3]: y[2,3] + y[3,2] <= 1
 no_subtours_3[2,4]: y[2,4] + y[4,2] <= 1
 no_subtours_3[3,0]: y[0,3] + y[3,0] <= 1
 no_subtours_3[3,1]: y[1,3] + y[3,1] <= 1
 no_subtours_3[3,2]: y[2,3] + y[3,2] <= 1
 no_subtours_3[3,4]: y[3,4] + y[4,3] <= 1
 no_subtours_3[4,0]: y[0,4] + y[4,0] <= 1
 no_subtours_3[4,1]: y[1,4] + y[4,1] <= 1
 no_subtours_3[4,2]: y[2,4] + y[4,2] <= 1
 no_subtours_3[4,3]: y[3,4] + y[4,3] <= 1
 no_subtours_1[0]: y[1,0] + y[2,0] + y[3,0] + y[4,0] <= 1
 no_subtours_1[1]: y[0,1] + y[2,1] + y[3,1] + y[4,1] <= 1
 no_subtours_1[2]: y[0,2] + y[1,2] + y[3,2] + y[4,2] <= 1
 no_subtours_1[3]: y[0,3] + y[1,3] + y[2,3] + y[4,3] <= 1
 no_subtours_1[4]: y[0,4] + y[1,4] + y[2,4] + y[3,4] <= 1
 no_subtours_2[0]: y[0,1] + y[0,2] + y[0,3] + y[0,4] <= 1
 no_subtours_2[1]: y[1,0] + y[1,2] + y[1,3] + y[1,4] <= 1
 no_subtours_2[2]: y[2,0] + y[2,1] + y[2,3] + y[2,4] <= 1
 no_subtours_2[3]: y[3,0] + y[3,1] + y[3,2] + y[3,4] <= 1
 no_subtours_2[4]: y[4,0] + y[4,1] + y[4,2] + y[4,3] <= 1
Bounds
 1 <= u[0] <= 5
 1 <= u[1] <= 5
 1 <= u[2] <= 5
 1 <= u[3] <= 5
 1 <= u[4] <= 5
 Constant = 1
Binaries
 o[0] o[1] o[2] o[3] y[0,0] y[0,1] y[0,2] y[0,3] y[0,4] y[1,0] y[1,1]
 y[1,2] y[1,3] y[1,4] y[2,0] y[2,1] y[2,2] y[2,3] y[2,4] y[3,0] y[3,1]
 y[3,2] y[3,3] y[3,4] y[4,0] y[4,1] y[4,2] y[4,3] y[4,4]
Generals
 u[0] u[1] u[2] u[3] u[4]
End
