import network
from node import node
from message import Message
from typing import List, Dict
import time

class Simulator:
		def __init__(self):
				self.network = None
				self.simulation_time = 0
				self.messages = []
				self.stats = {
						'total_messages': 0,
						'total_energy_consumed': 0,
						'total_latency': 0,
						'successful_transmissions': 0,
						'failed_transmissions': 0
				}

		def initialize_network(self, nodes: List[node]):
				"""Initialize the network with a list of nodes"""
				self.network = nodes

		def step(self, duration: float):
				"""Simulate network for a specified duration"""
				self.simulation_time += duration
				# Process message queues for each node
				for node in self.network:
						if node.message_queue and node.is_functional():
								self._process_node_messages(node)

		def _process_node_messages(self, source_node: node):
				"""Process messages in a node's queue"""
				if not source_node.message_queue:
						return
				
				message = source_node.message_queue[0]
				# Find nearest functional node
				target_node = self._find_nearest_functional_node(source_node)
				if target_node:
						distance = source_node.dist(target_node)
						result = source_node.transmit(message.size, distance, target_node)
						if result == 1:
								self.stats['successful_transmissions'] += 1
						else:
								self.stats['failed_transmissions'] += 1

		def _find_nearest_functional_node(self, source_node: node) -> node:
				"""Find the nearest functional node to transmit to"""
				nearest_node = None
				min_distance = float('inf')
				
				for node in self.network:
						if node != source_node and node.is_functional():
								distance = source_node.dist(node)
								if distance < min_distance:
										min_distance = distance
										nearest_node = node
				
				return nearest_node

		def get_simulation_stats(self) -> Dict:
				"""Get comprehensive simulation statistics"""
				node_stats = []
				for node in self.network:
						node_stats.append(node.get_node_stats())
						
				message_stats = []
				for node in self.network:
						for message in node.message_queue:
								message_stats.append(message.get_metrics_summary())
				
				return {
						'simulation_time': self.simulation_time,
						'network_stats': self.stats,
						'node_stats': node_stats,
						'message_stats': message_stats
				}