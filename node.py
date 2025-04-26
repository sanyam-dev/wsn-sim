import math
from numpy.random import normal

class node:
	"""
	initialises the node

	parameters:
	-	x coordinate of node
	-	y coordinate of node
	-	node id
	"""
	def	__init__(self, x, y, id,processing_param, queue_param) -> None:
		self.x = x
		self.y = y
		self.id = id
		self.critical_energy = 0
		self.current_energy = 0
		self.initial_energy = 0
		self.energy_parameter = 500*10**(-9) #nJ/bit
		self.free_space_energy_parameter = 25*10**(-12)	#pJ/bit/m^2
		self.multihop_energy_paramter = 0.065*10**(-12)	#pJ/bit/m^2
		self.d0 = math.sqrt(self.free_space_energy_parameter/self.multihop_energy_paramter)
		self.processing_param=processing_param
		self.queue_param=queue_param
		self.packets = 0
		self.packets_this_rnd=0
		
	def	dist(self, Node)-> float:
		"""
		distance of node from reference node

		parameters:
		-	Reference Node
		"""
		x1 = self.x - Node.x
		y1 = self.y - Node.y
		return round(math.sqrt((x1)**2 + (y1)**2), 2)

	def node_energy_setup(self, initial_energy, critical_energy):
		"""
		sets up energy of node

		parameters:
		-	initial energy
		-	critical energy
		"""
		self.initial_energy = initial_energy
		self.current_energy = initial_energy
		self.critical_energy = critical_energy
		return

	def is_functional(self)	-> bool:
		return self.current_energy > self.critical_energy or self.id == 0

	def energy_for_transmission(self, k, d):
		"""
		returns energy required for transmission

		parameters:
		-	k: packet length
		-	d: distance of transmission
		"""
		if(d < self.d0):
			return k*(self.energy_parameter + self.free_space_energy_parameter*d**2)
		else:
			return k*(self.energy_parameter + self.multihop_energy_paramter*d**4)

	def energy_for_reception(self, k):
		"""
		returns energy required for reception

		parameters:
		-	k: packet length
		"""
		return k*self.energy_parameter

	def setup_for_leach(self):
		"""
		void function that setup's LEACH protocole
		"""
		self.role = 0 #	not a head
		self.times_elected = 0
		self.last_head_rnd = -1e5
		self.clusterID = 0
		self.dist_to_head = 1e12
		self.children_clusters = list()

	def transmit(self, k, d, Node)->int:
		"""
			Simulates transmission of data from self node to node
			passed in arguement function

			parameters:
			-	packet length
			-	distance
			-	Node
		"""

		if(self.current_energy  <= self.critical_energy or
     		Node.current_energy <= Node.critical_energy):
			return -1

		et = self.energy_for_transmission(k, d)
		er = Node.energy_for_reception(k)

		if(et > self.current_energy or er > Node.current_energy):
			return -1

		self.current_energy -= et
		Node.current_energy -= er
		return 1

	def set_packets(self, number_of_packets):
		self.packets += number_of_packets
		self.packets_this_rnd = number_of_packets
		return number_of_packets

	def dropped_packets(self):
		return math.floor(normal(self.packets*0.07, 2))
	
	