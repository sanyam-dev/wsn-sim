from node import node
import random
from math import sqrt
import matplotlib.pyplot as plt
import heapq as pq
import networkx as nx
import numpy as np
import os
from typing import List, Dict
from message import Message

class network:
    def __init__(self, area_length=100, area_width=100, nodes=0, base_x=0, base_y=0):
        self.area_length = area_length
        self.area_width = area_width
        self.nodes = nodes
        self.base_x = base_x
        self.base_y = base_y
        self.node_list = []
        self.sink = None
        self.node_map = {}
        self.adjacency_list = {}
        self.graph = None
        self.position = None
        self.distances = None
        self.energy_consumed = 0.0
        self.initial_energy = 0.0
        self.critical_energy = 0.0
        self.network_formed = False

    def add_node(self, node: node) -> None:
        """Add a node to the network"""
        self.node_list.append(node)
        self.node_map[node.id] = node
        self.adjacency_list[node.id] = []
        if node.is_sink:
            self.sink = node

    def create_random_network(self, num_nodes: int, processing_param: float, queue_param: float,
                            initial_energy: float = 1.0, critical_energy: float = 0.1) -> None:
        """Create a random network topology"""
        self.initial_energy = initial_energy
        self.critical_energy = critical_energy
        for i in range(num_nodes):
            new_node = node(i, random.uniform(0, self.area_length), random.uniform(0, self.area_width),
                        processing_param, queue_param, initial_energy)
            self.add_node(new_node)
        self.network_formed = True

    def findShortestPath(self, source_node: node):
        """Find shortest path using Dijkstra's algorithm"""
        unvisited_nodes = list(self.node_list)
        shortest_path = {}
        previous_nodes = {}
        max_value = float('inf')
        for node in unvisited_nodes:
            shortest_path[node] = max_value
        shortest_path[source_node] = 0

        while unvisited_nodes:
            min_node = None
            for node in unvisited_nodes:
                if min_node is None:
                    min_node = node
                elif shortest_path[node] < shortest_path[min_node]:
                    min_node = node

            for child_node in self.adjacency_list[min_node.id]:
                if child_node in unvisited_nodes:
                    if shortest_path[min_node] + 1 < shortest_path[child_node]:
                        shortest_path[child_node] = shortest_path[min_node] + 1
                        previous_nodes[child_node] = min_node

            unvisited_nodes.remove(min_node)

        return self.reconstruct_path(source_node, previous_nodes)

    def reconstruct_path(self, source_node: node, previous_nodes: Dict):
        """Reconstruct the path from source to sink"""
        path = []
        current_node = self.sink
        while current_node != source_node:
            path.append(current_node)
            current_node = previous_nodes[current_node]
        path.append(source_node)
        path.reverse()
        return path

    def latency(self, source_node: node, next_node: node) -> float:
        """Calculate latency based on distance and processing time"""
        distance = source_node.dist(next_node)
        # Use packet_length from network parameters instead of message size
        processing_time = next_node.processing_param * self.packet_length if hasattr(self, 'packet_length') else 0
        return distance + processing_time

    def get_network_stats(self) -> Dict:
        """Get basic network statistics"""
        stats = {
            'total_nodes': len(self.node_list),
            'sink_node': self.sink.id if self.sink else None,
            'area_length': self.area_length,
            'area_width': self.area_width,
            'network_formed': self.network_formed
        }
        return stats

    def set_parameters(self, dist_para, len_of_packets, transmission_rate, speed_of_transmission, radio_distance):
        """Set network parameters"""
        self.distribution_parameters = dist_para
        self.packet_length = len_of_packets  # bits
        self.transmission_rate = transmission_rate  # kbps
        self.transmission_speed = speed_of_transmission  # bps
        self.radio_distance = radio_distance  # m
        self.get_graph()
        self.apl = self.get_apl(self.graph)
        self.acc = self.get_acc(self.graph)

    def get_graph(self):
        """Generate adjacency matrix based on radio distance"""
        graph = [[0 for _ in range(self.nodes + 1)] for _ in range(self.nodes + 1)]
        for i in range(self.nodes + 1):
            x = self.node_map[i]
            for j in range(i+1, self.nodes + 1):
                y = self.node_map[j]
                if sqrt((x.x - y.x)**2 + (y.y-x.y)**2) <= self.radio_distance:
                    graph[i][j] = 1
                    graph[j][i] = 1
        self.graph = graph
        return graph

    def set_nxg(self):
        """Create NetworkX graph representation"""
        G = nx.Graph()
        mp = self.node_map
        # Add nodes
        for i in range(1, self.nodes + 1):
            G.add_node(i, pos=(mp[i].x, mp[i].y))
        G.add_node(0, pos=(0, 0))
        
        # Add edges
        for i in range(self.nodes + 1):
            for j in range(self.nodes + 1):
                if i == j:
                    continue
                elif self.graph[i][j] == 1 and not G.has_edge(i, j):
                    n1 = mp[i]
                    n2 = mp[j]
                    e = n1.energy_for_transmission(self.packet_length, n1.dist(n2))
                    color = 'red' if n1.dist(n2) > self.radio_distance else 'black'
                    G.add_edge(i, j, color=color, weight=e)
        
        self.apl = round(nx.average_shortest_path_length(G), 3)
        self.acc = round(nx.average_clustering(G), 3)
        self.nxg = G
        return G

    def show_network(self):
        """Display network topology"""
        x, y = [], []
        for Node in self.node_list:
            if Node.current_energy >= Node.critical_energy:
                x.append(Node.x)
                y.append(Node.y)
        
        plt.figure(figsize=(10, 10))
        plt.xlim(-1, self.area_length)
        plt.ylim(-1, self.area_width)
        plt.scatter(x, y, marker='x', label='Nodes')
        plt.scatter(self.base_x, self.base_y, c="red", marker='o', label='Base Station')
        plt.legend()
        plt.grid(True)
        plt.title('Network Topology')
        plt.show()

    def show_graph(self):
        """Display network graph with NetworkX"""
        if not hasattr(self, 'nxg'):
            self.set_nxg()
        
        G = self.nxg
        pos = nx.get_node_attributes(G, 'pos')
        edges = G.edges()
        node_colors = ['red' if node == 0 else 'blue' for node in G]
        edge_colors = [G[u][v]['color'] for u, v in edges]
        
        plt.figure(figsize=(12, 8))
        nx.draw(G, pos, node_color=node_colors, node_size=60,
                edge_color=edge_colors, with_labels=True, font_color="green")
        plt.title('Network Graph Visualization')
        plt.show()

    def save_network(self, path):
        """Save network configuration and state"""
        x_pos = [self.node_map[i].x for i in range(self.nodes + 1)]
        y_pos = [self.node_map[i].y for i in range(self.nodes + 1)]
        
        if not hasattr(self, 'nxg'):
            self.set_nxg()
        
        edges = list(self.nxg.edges())
        edge_colors = [self.nxg[u][v]['color'] for u, v in edges]
        
        params = {
            "area_x": self.area_length,
            "area_y": self.area_width,
            "number_of_nodes": self.nodes,
            "base_x": self.base_x,
            "base_y": self.base_y,
            "node_initial_energy": self.initial_energy,
            "node_critical_energy": self.critical_energy,
            "dist_para": getattr(self, 'distribution_parameters', None),
            "len_of_packets": getattr(self, 'packet_length', None),
            "transmission_rate": getattr(self, 'transmission_rate', None),
            "speed_of_transmission": getattr(self, 'transmission_speed', None),
            "radio_distance": getattr(self, 'radio_distance', None)
        }
        
        net_data = {
            'x': x_pos,
            'y': y_pos,
            'graph_data': {
                'edges': edges,
                'edges_color': edge_colors
            },
            'params': params
        }
        
        try:
            np.save(f"{path}_network_data.npy", net_data)
            print(f"Network data saved @ {path}")
        except Exception as e:
            print(f"Failed to save: {str(e)}")

    def load_network_topology(self, path):
        """Load network topology from file"""
        try:
            network_data = np.load(path, allow_pickle=True).item()
            x = network_data['x']
            y = network_data['y']
            graph_data = network_data['graph_data']
            params = network_data['params']
            
            # Update network parameters
            self.nodes = params['number_of_nodes']
            self.area_length = params['area_x']
            self.area_width = params['area_y']
            self.base_x = params['base_x']
            self.base_y = params['base_y']
            
            # Create sink node
            sink = node(self.base_x, self.base_y, 0, 0, 0)
            sink.node_energy_setup(5*1e9, -1*1e9)
            self.sink = sink
            self.node_map[0] = sink
            
            # Create other nodes
            for i in range(1, self.nodes + 1):
                new_node = node(x[i], y[i], i, 0, 0)
                new_node.node_energy_setup(params['node_initial_energy'], 
                                         params['node_critical_energy'])
                self.node_list.append(new_node)
                self.node_map[i] = new_node
            
            # Set network parameters
            self.set_parameters(
                params['dist_para'],
                params['len_of_packets'],
                params['transmission_rate'],
                params['speed_of_transmission'],
                params['radio_distance']
            )
            
            return x, y, graph_data
            
        except Exception as e:
            print(f"Error loading network topology: {str(e)}")
            return None, None, None
            
    def get_acc(self, adj_matrix):
        """Calculate average clustering coefficient"""
        num_nodes = len(adj_matrix)
        total_clustering_coefficient = 0

        for i in range(num_nodes):
            neighbors = [j for j in range(num_nodes) if adj_matrix[i][j] == 1]
            num_neighbors = len(neighbors)

            if num_neighbors > 1:
                num_edges_between_neighbors = sum(
                    1 for j in range(num_neighbors)
                    for k in range(j+1, num_neighbors)
                    if adj_matrix[neighbors[j]][neighbors[k]] == 1
                )
                clustering_coefficient = 2 * num_edges_between_neighbors / (num_neighbors * (num_neighbors - 1))
                total_clustering_coefficient += clustering_coefficient

        average_coefficient = total_clustering_coefficient / num_nodes
        return round(average_coefficient, 3)

    def get_apl(self, adj_matrix):
        """Calculate average path length using Floyd-Warshall algorithm"""
        num_nodes = len(adj_matrix)
        dist_matrix = [row[:] for row in adj_matrix]
        
        # Initialize distance matrix
        for i in range(num_nodes):
            dist_matrix[i][i] = 0
            for j in range(num_nodes):
                if i != j and dist_matrix[i][j] == 0:
                    dist_matrix[i][j] = float('inf')
        
        # Floyd-Warshall algorithm
        for k in range(num_nodes):
            for i in range(num_nodes):
                for j in range(num_nodes):
                    if dist_matrix[i][k] != float('inf') and dist_matrix[k][j] != float('inf'):
                        dist_matrix[i][j] = min(dist_matrix[i][j],
                                              dist_matrix[i][k] + dist_matrix[k][j])
        
        # Calculate average path length
        total_path_length = sum(dist_matrix[i][j] 
                              for i in range(num_nodes) 
                              for j in range(num_nodes)
                              if i != j and dist_matrix[i][j] != float('inf'))
        
        pairs = num_nodes * (num_nodes - 1)
        average_length = total_path_length / pairs if pairs > 0 else 0
        return round(average_length, 3)

class Network(network):
    """Network class with message handling capabilities"""
    
    def __init__(self, area_length, area_width, nodes, base_x, base_y) -> None:
        super().__init__(area_length, area_width, nodes, base_x, base_y)
        self.messages_processed = 0
        self.total_network_latency = 0.0
        self.message_stats = {
            'total_messages': 0,
            'delivered_messages': 0,
            'dropped_messages': 0
        }

    def save_network(self, path):
        """Enhanced network saving with message statistics"""
        data = super().save_network(path)
        message_stats = {
            'messages_processed': self.messages_processed,
            'total_latency': self.total_network_latency,
            'message_stats': self.message_stats
        }
        try:
            np.save(f"{path}_message_stats.npy", message_stats)
        except Exception as e:
            print(f"Failed to save message stats: {str(e)}")

    def load_network_topology(self, path):
        """Enhanced network loading with message statistics"""
        x, y, graph_data = super().load_network_topology(path)
        try:
            message_stats = np.load(f"{path.rsplit('_', 1)[0]}_message_stats.npy", allow_pickle=True).item()
            self.messages_processed = message_stats.get('messages_processed', 0)
            self.total_network_latency = message_stats.get('total_latency', 0.0)
            self.message_stats = message_stats.get('message_stats', {
                'total_messages': 0,
                'delivered_messages': 0,
                'dropped_messages': 0
            })
        except Exception:
            pass  # Message stats might not exist for older networks
        return x, y, graph_data

    def process_network_messages(self):
        """Process all messages in the network using base class path finding"""
        processed_count = 0
        for node in self.node_list:
            if node.is_functional() and node.message_queue:
                path = self.findShortestPath(node)
                if path and len(path) > 0:
                    if self._process_message(node, path[0]):
                        processed_count += 1
        return processed_count

    def _process_message(self, source_node: node, next_node_id: int) -> bool:
        """Process a single message using inherited network parameters"""
        if not source_node.message_queue:
            return False

        message = source_node.message_queue[0]
        next_node = self.node_map[next_node_id]
        distance = source_node.dist(next_node)

        result = source_node.transmit(message.size, distance, next_node)
        if result == 1:
            latency = self.latency(source_node, next_node)
            self.total_network_latency += latency
            self.messages_processed += 1
            self.message_stats['delivered_messages'] += 1
            return True
        else:
            self.message_stats['dropped_messages'] += 1
            return False