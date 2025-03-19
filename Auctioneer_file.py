class Auctioneer:
    def __init__(self, tugs):
        self.tugs_available = tugs  # The tugs that are available to fulfill a new task (refreshed every timestep)
        self.prices = []
        self.price_combinations = []
        self.pairings = []

    def tug_availability(self,tugs):   # Check for tugs that are available to receive a task, based on their status
        tugs_available = []

        for tug in tugs:
            if tug.status == 'idle' or tug.status == 'to_depot':   # VOEG NOG 'to_depot' TOE ZODRA TUGS NIET MEER NAAR DE DEPOT HOEVEN TE RIJDEN!!!
                tugs_available.append(tug)

        self.tugs_available = tugs_available

    def ask_price(self,tasks,nodes_dict,heuristics,t):   # Inquire each tug about its price for a certain task
        self.prices = []
        self.price_combinations = []

        for task in tasks:
            for tug in self.tugs_available:
                price = tug.bidders_value(task,nodes_dict,heuristics,t)
                self.prices.append(price)
                self.price_combinations.append([task,tug])

    def decision(self,dep_depot,arr_depot):   # Decide the pairings task/tug based on highest prices (i.e. highest priority and proximity)
        self.pairings = []
        tasks_served = []
        tugs_served = []

        for i in range(len(self.prices)):
            max_price_idx = self.prices.index(max(self.prices))
            if self.price_combinations[max_price_idx][0] not in tasks_served and self.price_combinations[max_price_idx][1] not in tugs_served:
                self.pairings.append([self.price_combinations[max_price_idx][0],self.price_combinations[max_price_idx][1]])
                tasks_served.append(self.price_combinations[max_price_idx][0])
                tugs_served.append(self.price_combinations[max_price_idx][1])
            del self.prices[max_price_idx]
            del self.price_combinations[max_price_idx]

        for pair in self.pairings:
            if pair[0] in dep_depot.tasks:
                dep_depot.tasks.remove(pair[0])
            else:
                arr_depot.tasks.remove(pair[0])

            if pair[1] in dep_depot.tugs:
                dep_depot.tugs.remove(pair[1])
            elif pair[1] in arr_depot.tugs:
                arr_depot.tugs.remove(pair[1])

            pair[1].assign_task(pair[0])