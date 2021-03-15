Instances of the LV-SAV

This instances set includes different instances with:

	4 different number of customers: 15, 30, 50, 100. 
	3 different combinations of number of satellites: 3 satellites, 4 satellites, 5 satellites.
	4 different categories: Ca, Cb, Cc, Cd

Each instance is represented by a notation which consists of the associated category name, an index, number of satellites and number of customers.
For example, "Ca3,3,15" denotes the third instance of the category "Ca", with 3 satellites and 15 customers.

Each text file contains:

	For each customer: x coordinate, y coordinate, start of time window, end of time window, demand size, service time
	For each satellite: x coordinate, y coordinate
	For depot: x coordinate, y coordinate

Other setting for all instances are as follows:
	Depot time window [0,450]
	LV capacity=200 
	SAV capacity=50
	Number of SAVs an LV can carry=4 - should we leave it open? 
