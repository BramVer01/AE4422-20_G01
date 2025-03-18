class Auctioneer:
    def __init__(self, tugs):
        self.tugs_available = tugs  # The tugs that are available to fulfill a new task (refreshed every timestep)

    def tug_availability(self,tugs):   # Check for tugs that are available to receive a task, based on their status
        tugs_available = []

        for tug in tugs:
            if tug.status == 'idle' or tug.status == 'to_depot':
                tugs_available.append(tug)

        self.tugs_available = tugs_available

    def ask_price(self,tugs_available,tasks,nodes_dict,heuristics,t):   # Inquire each tug about its price for a certain task
        prices = []
        price_combinations = []
        for task in tasks:
            for tug in tugs_available:
                price = tug.bidders_value(task,nodes_dict,heuristics,t)
                prices.append(price)
                price_combinations.append([task,tug])

        return prices, price_combinations

    def decision(self,prices,price_combinations):   # Decide the pairings task/tug based on highest prices (i.e. highest priority and proximity)
        pairings = []
        tasks_served = []
        tugs_served = []

        for i in range(len(prices)):
            min_price = max(prices)
            min_price_idx = prices.index(min_price)
            if price_combinations[min_price_idx][0] not in tasks_served and price_combinations[min_price_idx][1] not in tugs_served:
                pairings.append([price_combinations[min_price_idx][0],price_combinations[min_price_idx][1]])
                del prices[min_price_idx]

        return pairings