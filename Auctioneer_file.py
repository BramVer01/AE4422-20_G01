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

    def ask_price(self,tasks,nodes_dict,heuristics,t,depots):   # Inquire each tug about its price for a certain task
        self.prices = []
        self.price_combinations = []

        for task in tasks:
            for tug in self.tugs_available:
                price,suitable = tug.bidders_value(task,nodes_dict,heuristics,t,depots)
                if suitable:
                    self.prices.append(price)
                    self.price_combinations.append([task,tug])

    def decision(self, dep_depot, arr_depot):
        self.pairings = []
        tasks_served = []
        tugs_served = []
        
        for i in range(len(self.prices)):
            max_price_idx = self.prices.index(max(self.prices))
            task = self.price_combinations[max_price_idx][0]
            tug = self.price_combinations[max_price_idx][1]
            
            if task not in tasks_served and tug not in tugs_served:
                # Attempt to assign the task
                if tug.assign_task(task):  # Returns True if assignment successful
                    self.pairings.append([task, tug])
                    tasks_served.append(task)
                    tugs_served.append(tug)
                    
                    # Remove the task and tug from their respective depots
                    if task in dep_depot.tasks:
                        dep_depot.tasks.remove(task)
                    else:
                        arr_depot.tasks.remove(task)
                        
                    if tug in dep_depot.tugs:
                        dep_depot.tugs.remove(tug)
                    elif tug in arr_depot.tugs:
                        arr_depot.tugs.remove(tug)
                
            del self.prices[max_price_idx]
            del self.price_combinations[max_price_idx]