import math
from numpy.random import normal
from message import Message, TransmissionMetrics
import time

class node:
	"""
	initialises the node

	parameters:
	-	x coordinate of node
	-	y coordinate of node
	-	node id
	"""
	def __init__(self, x, y, id, processing_param, queue_param) -> None:
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
		self.message_queue = []
		self.messages_generated = 0
		self.messages_sent = []
		
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

	def create_message(self, data: bytes, size: int) -> Message:
		"""
		Creates a new message with this node as the generator
		
		parameters:
		- data: message content in bytes
		- size: size of the message in bits
		"""
		message_id = self.messages_generated
		self.messages_generated += 1
		message = Message(message_id, data, self, size)
		self.message_queue.append(message)
		return message

	def transmit(self, k, d, Node) -> int:
		"""
		Simulates transmission of data from self node to node
		passed in argument function

		parameters:
		- k: packet length
		- d: distance
		- Node: receiving node
		"""
		if(self.current_energy <= self.critical_energy or
		   Node.current_energy <= Node.critical_energy):
			return -1

		et = self.energy_for_transmission(k, d)
		er = Node.energy_for_reception(k)

		if(et > self.current_energy or er > Node.current_energy):
			return -1

		start_time = time.time()
		self.current_energy -= et
		Node.current_energy -= er
		
		# Calculate transmission latency based on processing and queue parameters
		latency = (self.processing_param * k) + (self.queue_param * len(self.message_queue))
		
		# If there's a message to transmit from the queue, record its metrics
		if self.message_queue:
			message = self.message_queue[0]
			message.record_transmission(self, Node, latency, et + er)
			Node.message_queue.append(message)
			self.message_queue.pop(0)
			self.messages_sent.append(message.message_id)
		
		return 1

	def get_message_stats(self):
		"""Returns statistics about message handling for this node"""
		return {
			'messages_generated': self.messages_generated,
			'messages_in_queue': len(self.message_queue),
			'messages_sent': len(self.messages_sent),
			'current_queue_delay': self.queue_param * len(self.message_queue)
		}

	def get_node_stats(self):
		"""
		Returns comprehensive statistics about the node's current state
		including energy levels, location, message stats, and packet info
		"""
		message_stats = self.get_message_stats()
		return {
			'node_id': self.id,
			'location': {
				'x': self.x,
				'y': self.y
			},
			'energy': {
				'current': self.current_energy,
				'initial': self.initial_energy,
				'critical': self.critical_energy,
				'remaining_percentage': (self.current_energy / self.initial_energy * 100) if self.initial_energy > 0 else 0
			},
			'packets': {
				'total': self.packets,
				'current_round': self.packets_this_rnd,
				'dropped': self.dropped_packets()
			},
			'messages': message_stats,
			'parameters': {
				'processing': self.processing_param,
				'queue': self.queue_param
			}
		}

	def set_packets(self, number_of_packets):
		self.packets += number_of_packets
		self.packets_this_rnd = number_of_packets
		return number_of_packets

	def dropped_packets(self):
		return math.floor(normal(self.packets*0.07, 2))

