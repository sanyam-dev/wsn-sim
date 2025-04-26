from dataclasses import dataclass, field
from typing import List, Dict
import time
from node import node

@dataclass
class TransmissionMetrics:
    latency: float
    energy_consumed: float
    sender_id: int
    receiver_id: int
    timestamp: float = field(default_factory=time.time)

class Message:
    def __init__(self, message_id: int, data: bytes, generator_node: node, size: int):
        self.message_id = message_id
        self.data = data
        self.size = size  # in bits
        self.generator_node = generator_node
        self.transmission_history: List[TransmissionMetrics] = []
        self.total_transmissions = 0
        self.total_energy_consumed = 0.0
        self.total_latency = 0.0
        
    def record_transmission(self, sender: node, receiver: node, latency: float, energy: float) -> None:
        """Record a transmission of this message between nodes"""
        metrics = TransmissionMetrics(
            latency=latency,
            energy_consumed=energy,
            sender_id=sender.id,
            receiver_id=receiver.id
        )
        self.transmission_history.append(metrics)
        self.total_transmissions += 1
        self.total_energy_consumed += energy
        self.total_latency += latency

    def get_transmission_path(self) -> List[int]:
        """Returns ordered list of node IDs this message has traversed"""
        path = [self.generator_node.id]
        for metric in self.transmission_history:
            if metric.receiver_id not in path:
                path.append(metric.receiver_id)
        return path

    def get_metrics_summary(self) -> Dict:
        """Returns summary of message transmission metrics"""
        return {
            'message_id': self.message_id,
            'size': self.size,
            'total_transmissions': self.total_transmissions,
            'total_energy_consumed': self.total_energy_consumed,
            'total_latency': self.total_latency,
            'generator_node': self.generator_node.id,
            'path': self.get_transmission_path()
        }