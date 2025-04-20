class Auctioneer:
    def __init__(self, tugs):
        self.tugs_available = tugs  # The tugs that are available to fulfill a new task (refreshed every timestep)
        self.prices = []
        self.price_combinations = []
        self.pairings = []

    def tug_availability(self,tugs):   # Check for tugs that are available to receive a task, based on their status
        tugs_available = []

        for tug in tugs:
            if tug.status == 'idle' or tug.status == 'to_depot':
                tugs_available.append(tug)

        self.tugs_available = tugs_available

    def ask_price(self, tasks, nodes_dict, heuristics, t, depots):
        """Inquire each tug about its price for a certain task"""
        self.prices = []
        self.price_combinations = []
        
        # Only consider tasks that are not already being executed
        active_tasks = [task for task in tasks if task.flight_id not in [tug.current_task for tug in self.tugs_available if tug.current_task is not None]]
        
        for task in active_tasks:
            for tug in self.tugs_available:
                # Skip if tug is already assigned to a task
                if tug.status not in ['idle', 'to_depot']:
                    continue
                    
                price, suitable = tug.bidders_value(task, nodes_dict, heuristics, t, depots)
                if suitable:
                    self.prices.append(price)
                    self.price_combinations.append([task, tug])

    def decision(self, dep_depot, arr_depot, START_NODES):
        self.pairings = []
        tasks_served = []
        tugs_served = []
        
        # Sort price combinations by price (highest first)
        sorted_bids = sorted(zip(self.prices, self.price_combinations), key=lambda x: x[0], reverse=True)

        
        # Go through sorted bids
        for price, combo in sorted_bids:
            task, tug = combo
            
            # Skip if task or tug already allocated
            if task in tasks_served or tug in tugs_served:
                continue
                
            # Try to assign the task
            if tug.assign_task(task, dep_depot, arr_depot, START_NODES):
                self.pairings.append([task, tug])
                tasks_served.append(task)
                tugs_served.append(tug)
                
                # Remove from depots
                if task in dep_depot.tasks:
                    dep_depot.tasks.remove(task)
                else:
                    arr_depot.tasks.remove(task)
                    
                if tug in dep_depot.tugs:
                    dep_depot.tugs.remove(tug)
                elif tug in arr_depot.tugs:
                    arr_depot.tugs.remove(tug)